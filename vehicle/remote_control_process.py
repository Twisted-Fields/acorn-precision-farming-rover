"""
*********************************************************************
                     This file is part of:
                       The Acorn Project
             https://wwww.twistedfields.com/research
*********************************************************************
Copyright (c) 2019-2021 Taylor Alexander, Twisted Fields LLC
Copyright (c) 2021 The Acorn Project contributors (cf. AUTHORS.md).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*********************************************************************
"""
import corner_actuator
import time
import math
from steering import calculate_steering, compare_steering_values, steering_to_numpy
import zmq
import pickle
import gps_tools
import numpy as np
import spline_lib
import os
import datetime
from motors import _STATE_ENABLED, STATE_DISCONNECTED
import gps
import coloredlogs
import subprocess
from enum import Enum
from multiprocessing import shared_memory, resource_tracker
import random
from joystick import Joystick
from utils import AppendFIFO, clamp

# This file gets imported by server but we should only import GPIO on raspi.
if "arm" in os.uname().machine:
    import board
    import busio
    from adafruit_mcp230xx.mcp23017 import MCP23017

COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

ACCELERATION_COUNTS_SEC = 0.5

_RESUME_MOTION_WARNING_TIME_SEC = 4
# _RESUME_MOTION_WARNING_TIME_SEC = -1

_SEC_IN_ONE_MINUTE = 60

_MAXIMUM_ALLOWED_DISTANCE_METERS = 3.5
_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES = 120  # 20
_VOLTAGE_CUTOFF = 20
_VOLTAGE_RESUME_MANUAL_CONTROL = 35

_GPS_ERROR_RETRIES = 3

CONTROL_STARTUP = "Initializing..."
CONTROL_GPS_STARTUP = "Waiting for GPS fix."
CONTROL_ONLINE = "Online and awaiting commands."
CONTROL_AUTONOMY = "Autonomy operating."
CONTROL_AUTONOMY_PAUSE = "Autonomy paused with temporary error."
CONTROL_LOW_VOLTAGE = "Low voltage Pause."
CONTROL_AUTONOMY_ERROR_DISTANCE = "Autonomy failed - too far from path."
CONTROL_AUTONOMY_ERROR_ANGLE = "Autonomy failed - path angle too great."
CONTROL_AUTONOMY_ERROR_RTK_AGE = "Autonomy failed - rtk base data too old."
CONTROL_AUTONOMY_ERROR_SOLUTION_AGE = "Autonomy failed - gps solution too old."
CONTROL_OVERRIDE = "Remote control override."
CONTROL_SERVER_ERROR = "Server communication error."
CONTROL_MOTOR_ERROR = "Motor error detected."
CONTROL_NO_STEERING_SOLUTION = "No steering solution possible."

GPS_PRINT_INTERVAL = 10

_NUM_GPS_SUBSAMPLES = 10

_LOOP_RATE = 10

_ERROR_RATE_AVERAGING_COUNT = 3

_ALLOWED_RTK_AGE_SEC = 20.0
_ALLOWED_SOLUTION_AGE_SEC = 1.0

_ALLOWED_MOTOR_SEND_LAPSE_SEC = 5

SERVER_COMMUNICATION_DELAY_LIMIT_SEC = 10
_SERVER_DELAY_RECONNECT_WIFI_SECONDS = 120

_BEGIN_AUTONOMY_SPEED_RAMP_SEC = 3.0

_PATH_END_PAUSE_SEC = 5.0

_SLOW_POLLING_SLEEP_S = 0.5

_MILLISECONDS_PER_SECOND = 1000
_POLL_MILLISECONDS = 100
_FAST_POLL_MILLISECONDS = 20
_VERY_FAST_POLL_MILLISECONDS = 5

_ERROR_SKIP_RATE = 40

_DISENGAGEMENT_RETRY_DELAY_MINUTES = 1
_DISENGAGEMENT_RETRY_DELAY_SEC = _DISENGAGEMENT_RETRY_DELAY_MINUTES * _SEC_IN_ONE_MINUTE

_STEERING_ANGLE_LIMIT_AUTONOMY_DEGREES = 180
_STEERING_ANGLE_LIMIT_DEGREES = 120

_DEFAULT_MAXIMUM_VELOCITY = 0.4


def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel - last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel - last_vel
    else:
        increment = math.copysign(ACCELERATION_COUNTS_SEC,
                                  unfiltered_vel - last_vel) * period_s
    return last_vel + increment


class Direction(Enum):
    FORWARD = 1
    BACKWARD = 2
    EITHER = 3


class PathControlValues():
    def __init__(self, angular_p, lateral_p, angular_d, lateral_d):
        self.angular_p = angular_p
        self.lateral_p = lateral_p
        self.angular_d = angular_d
        self.lateral_d = lateral_d


class NavigationParameters():
    def __init__(self, travel_speed, path_following_direction,
                 vehicle_travel_direction, loop_path):
        self.travel_speed = travel_speed
        self.path_following_direction = path_following_direction
        self.vehicle_travel_direction = vehicle_travel_direction
        self.loop_path = loop_path


class PathSection():
    def __init__(self,
                 points,
                 control_values,
                 navigation_parameters,
                 max_dist=0,
                 max_angle=0,
                 end_dist=0,
                 end_angle=0):
        self.points: list = points
        self.spline: spline_lib.GpsSpline = None
        self.maximum_allowed_distance_meters = max_dist
        self.maximum_allowed_angle_error_degrees = max_angle
        self.control_values = control_values
        self.end_distance_m = end_dist
        self.end_angle_degrees = end_angle
        self.navigation_parameters: NavigationParameters = navigation_parameters


class EnergySegment():
    def __init__(self, sequence_num, start_gps, end_gps, distance_sum,
                 total_watt_seconds, avg_watts, per_motor_total_watt_seconds,
                 per_motor_watt_average, subsampled_points, autonomy_operating,
                 wifi_ap_name, wifi_signal_strength):
        self.sequence_num = sequence_num
        self.time_stamp = end_gps.time_stamp
        self.start_gps = start_gps
        self.end_gps = end_gps
        self.duration = end_gps.time_stamp - start_gps.time_stamp
        self.distance_sum = distance_sum
        self.meters_per_second = distance_sum / self.duration
        self.watt_seconds_per_meter = total_watt_seconds / distance_sum
        self.height_change = end_gps.height_m - start_gps.height_m
        self.avg_watts = avg_watts
        self.per_motor_total_watt_seconds = per_motor_total_watt_seconds
        self.per_motor_watt_average = per_motor_watt_average
        self.subsampled_points = subsampled_points
        self.autonomy_operating = autonomy_operating
        self.wifi_ap_name = wifi_ap_name
        self.wifi_signal_strength = wifi_signal_strength


class RemoteControl():
    def __init__(self,
                 remote_to_main_lock,
                 main_to_remote_lock,
                 remote_to_main_string,
                 main_to_remote_string,
                 logging,
                 logging_details,
                 simulated_hardware=False):
        self.joy = Joystick(simulated_hardware)
        self.simulated_hardware = simulated_hardware
        self.motor_socket = None
        self.robot_object = None
        self.next_point_heading = -180
        self.activate_autonomy = False
        self.autonomy_velocity = 0
        self.resume_motion_timer = 0
        self.remote_to_main_lock = remote_to_main_lock
        self.main_to_remote_lock = main_to_remote_lock
        self.remote_to_main_string = remote_to_main_string
        self.main_to_remote_string = main_to_remote_string
        self.logger = logging.getLogger('main.remote')
        _LOGGER_FORMAT_STRING, _LOGGER_DATE_FORMAT, _LOGGER_LEVEL = logging_details
        coloredlogs.install(fmt=_LOGGER_FORMAT_STRING,
                            datefmt=_LOGGER_DATE_FORMAT,
                            level=_LOGGER_LEVEL,
                            logger=self.logger)
        self.gps = gps.GPS(self.logger, self.simulated_hardware)
        self.default_navigation_parameters = NavigationParameters(travel_speed=0.0,
                                                                  path_following_direction=Direction.EITHER,
                                                                  vehicle_travel_direction=Direction.EITHER,
                                                                  loop_path=True)
        self.default_path_control_vals = PathControlValues(angular_p=0.9,
                                                           lateral_p=-0.25,
                                                           angular_d=0.3,
                                                           lateral_d=-0.05)
        self.nav_path = PathSection(points=[],
                                    control_values=self.default_path_control_vals,
                                    navigation_parameters=self.default_navigation_parameters,
                                    max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                                    max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                                    end_dist=1.0,
                                    end_angle=30)
        self.maximum_velocity = _DEFAULT_MAXIMUM_VELOCITY
        self.nav_path_list = []
        self.nav_path_index = 0
        self.gps_path = []
        self.load_path_time = time.time()
        self.loaded_path_name = ""
        self.autonomy_hold = True
        self.control_state = CONTROL_STARTUP
        self.motor_state = STATE_DISCONNECTED
        self.driving_direction = 1.0
        self.gps_path_lateral_error = 0
        self.gps_path_angular_error = 0
        self.gps_error_update_time = 0
        self.gps_angle_error_rate_averaging_list = []
        self.gps_lateral_error_rate_averaging_list = []
        self.last_autonomy_steer_cmd = 0
        self.last_autonomy_strafe_cmd = 0
        self.solution_age_averaging_list = []
        self.voltage_average = 0
        self.disengagement_time = time.time()

        self.power_consumption_list = []
        self.avg_watts_per_meter = 0
        self.watt_hours_per_meter = 0
        self.total_watts = 0
        self.voltages = []
        self.bus_currents = []
        self.last_energy_segment = None
        self.temperatures = []
        self.last_calculated_steering = calculate_steering(0, 0, 0, _STEERING_ANGLE_LIMIT_DEGREES)
        self.last_wifi_restart_time = time.time()
        self.last_vel_cmd = 0
        self.vel_cmd = 0

        self.loop_count = -1

    def run_setup(self):
        if self.simulated_hardware:

            class FakeAlarm():
                def __init__(self):
                    self.value = 0

            self.alarm1 = FakeAlarm()
            self.alarm2 = FakeAlarm()
            self.alarm3 = FakeAlarm()
        else:
            i2c = busio.I2C(board.SCL, board.SDA)
            mcp = MCP23017(i2c)  # , address=0x20)  # MCP23017
            self.alarm1 = mcp.get_pin(0)
            self.alarm2 = mcp.get_pin(1)
            self.alarm3 = mcp.get_pin(2)

            self.alarm1.switch_to_output(value=False)
            self.alarm2.switch_to_output(value=False)
            self.alarm3.switch_to_output(value=False)
        self.connect_to_motors()
        self.joy.connect()

    def connect_to_motors(self, port=5590):
        context = zmq.Context()
        #  Socket to talk to motor control process
        self.motor_socket = context.socket(zmq.REQ)
        self.motor_socket.connect("tcp://localhost:{}".format(port))
        self.motor_socket.setsockopt(zmq.LINGER, 50)
        self.motor_send_okay = True
        self.motor_last_send_time = time.time()

    def close_motor_socket(self):
        self.motor_socket.close()
        del(self.motor_socket)
        self.motor_socket = None

    def load_path(self,
                  path,
                  simulation_teleport=False,
                  generate_spline=False):
        """
        TODO: doc
        """
        if isinstance(path, PathSection):
            self.nav_path = path
            if len(path.points) > 2 and generate_spline:
                self.nav_path.spline = spline_lib.GpsSpline(path.points,
                                                            smooth_factor=10,
                                                            num_points=1000)
                self.nav_path.points = self.nav_path.spline.points
            elif len(path.points) == 2:
                self.nav_path.points = tuple(gps_tools.check_point(p) for p in path.points)
        else:
            # Legacy paths
            nav_spline = spline_lib.GpsSpline(path,
                                              smooth_factor=10,
                                              num_points=1000)
            self.nav_path = PathSection(points=nav_spline.points,
                                        control_values=self.default_path_control_vals,
                                        navigation_parameters=self.default_navigation_parameters,
                                        max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                                        max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                                        end_dist=1.0,
                                        end_angle=45)
            self.nav_path.spline = nav_spline
        self.loaded_path_name = self.robot_object.loaded_path_name
        if self.simulated_hardware and simulation_teleport:
            # Place simulated robot at start of path.
            start_index = 0 if len(self.nav_path.points) == 2 else int(len(self.nav_path.spline.points) / 2) - 5
            initial_heading = gps_tools.get_heading(
                self.nav_path.points[start_index], self.nav_path.points[start_index + 1])
            self.gps.update_simulated_sample(self.nav_path.points[start_index].lat,
                                             self.nav_path.points[start_index].lon,
                                             initial_heading + 30)

        # Set initial nav_direction.
        direction = self.nav_path.navigation_parameters.path_following_direction
        if direction == Direction.EITHER:
            dist_start = gps_tools.get_distance(self.gps.last_sample(), self.nav_path.points[0])
            dist_end = gps_tools.get_distance(
                self.gps.last_sample(), self.nav_path.points[len(self.nav_path.points) - 1])
            # This may be overridden farther down.
            self.nav_direction = 1 if dist_start < dist_end else -1
        elif direction == Direction.FORWARD:
            self.nav_direction = 1
        elif direction == Direction.BACKWARD:
            self.nav_direction = -1

    def run_loop(self):
        self.reloaded_path = True

        # set up shared memory for GUI four wheel steeing debugger (sim only).
        steering_tmp = steering_to_numpy(self.last_calculated_steering)
        try:
            self.steering_debug_shared_memory = shared_memory.SharedMemory(name='acorn_steering_debug')
            self.logger.info("Connected to existing shared memory acorn_steering_debug")
        except FileNotFoundError:
            self.steering_debug_shared_memory = shared_memory.SharedMemory(
                name="acorn_steering_debug",
                create=True,
                size=steering_tmp.nbytes)
            self.logger.info("Created shared memory acorn_steering_debug")
        # Untrack the resource so it does not get destroyed. This allows the
        # steering debug window to stay open.
        resource_tracker.unregister(self.steering_debug_shared_memory._name, 'shared_memory')
        self.steering_debug = np.ndarray(steering_tmp.shape,
                                         dtype=steering_tmp.dtype,
                                         buffer=self.steering_debug_shared_memory.buf)
        self.steering_debug[:] = steering_tmp[:]

        try:
            while True:
                self.loop_count += 1
                self.run_once()
        except KeyboardInterrupt:
            pass

    def run_once(self):
        tick_time = time.time()

        # Get real or simulated GPS data.
        if self.simulated_hardware:
            if self.loop_count % GPS_PRINT_INTERVAL == 0:
                self.gps.dump()
            s = self.gps.last_sample()
            self.gps.update_simulated_sample(s.lat, s.lon, s.azimuth_degrees)
        else:
            self.gps.update_from_device(print_gps=self.loop_count % GPS_PRINT_INTERVAL == 0,
                                        retries=_GPS_ERROR_RETRIES,)

        debug_time = time.time()
        recieved_robot_object = None
        time1 = time.time() - debug_time
        try:
            # Read robot object from shared memory.
            # Object is sent by main process.
            with self.main_to_remote_lock:
                recieved_robot_object = pickle.loads(self.main_to_remote_string["value"])
            time2 = time.time() - debug_time
        except Exception as e:
            self.logger.error("Exception reading remote string.")
            raise(e)

        # TODO: switch to isinstance() without introducing circular dependency.
        if str(type(recieved_robot_object)) != "<class '__main__.Robot'>":
            self.logger.info("Waiting for valid robot object before running remote control code.")
            time.sleep(_SLOW_POLLING_SLEEP_S)
            return

        self.robot_object = recieved_robot_object
        self.logger.debug("Remote received new robot object.")
        time3 = time.time() - debug_time
        if not self.nav_path.points or self.loaded_path_name != self.robot_object.loaded_path_name:
            if self.robot_object.loaded_path and self.gps.last_sample() is not None:
                self.reload_path()
        self.activate_autonomy = self.robot_object.activate_autonomy
        # self.autonomy_velocity = self.robot_object.autonomy_velocity
        # Autonomy is disabled until robot is ready or if joystick is used.
        if self.autonomy_hold:
            self.activate_autonomy = False
        # Reset disabled autonomy if autonomy is turned off in the command.
        if self.robot_object.clear_autonomy_hold:
            self.logger.debug("clearing autonomy hold.")
            self.autonomy_hold = False
            # Reset disengagement timer.
            self.disengagement_time = time.time() - _DISENGAGEMENT_RETRY_DELAY_SEC

        time4 = time.time() - debug_time

        #    print("begin robot calc")

        debug_points = (None, None, None, None)
        gps_path_angle_error = 0
        absolute_path_distance = math.inf
        time5 = 0
        sample = self.gps.last_sample()
        if self.robot_object is not None and sample is not None:
            vehicle_front = gps_tools.project_point(sample, sample.azimuth_degrees, 1.0)
            vehicle_rear = gps_tools.project_point(sample, sample.azimuth_degrees, -1.0)

            time5 = time.time() - debug_time
            (projected_path_tangent_point, closest_path_point,
             absolute_path_distance, drive_reverse, calculated_rotation, calculated_strafe) = self.steering_calc()

            # These extra points can be displayed in the web UI.
            # TODO: That's commented out in the server code. Resolve?
            debug_points = (vehicle_front, vehicle_rear, projected_path_tangent_point, closest_path_point)

        time6 = time.time() - debug_time

        self.joy.update_value()
        if self.joy.activated():
            self.logger.info("DISABLED AUTONOMY becase joystick is activated: {}".format(self.joy))
            self.autonomy_hold = True
            self.activate_autonomy = False
            self.control_state = CONTROL_OVERRIDE

        (user_web_page_plot_steer_cmd, user_web_page_plot_strafe_cmd,
         strafe_d, steer_d, strafe_p, steer_p,
         autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd
         ) = self.calc_commands_for_autonomy(calculated_rotation, calculated_strafe, drive_reverse)

        # In the following list the order for when we set control state
        # matters
        # TODO: clarify? What does this note mean?

        time7 = time.time() - debug_time

        error_messages, fatal_error, zero_output = self.safty_checks(absolute_path_distance, gps_path_angle_error)

        if self.loop_count % _ERROR_SKIP_RATE == 0:
            for error in error_messages:
                self.logger.error(error)

        self.maybe_restart_wifi()

        if zero_output:
            if self.activate_autonomy and time.time() - self.disengagement_time > _DISENGAGEMENT_RETRY_DELAY_SEC:
                self.disengagement_time = time.time()
                self.logger.error("Disengaging Autonomy.")
                self.write_errors(error_messages)
                if fatal_error:
                    self.autonomy_hold = True
                    self.activate_autonomy = False
        elif not self.activate_autonomy:
            self.resume_motion_timer = time.time()
            self.control_state = CONTROL_ONLINE

        time8 = time.time() - debug_time

        # Activate a brief pause at the end of a track.
        if time.time() - self.load_path_time < _PATH_END_PAUSE_SEC and not zero_output:
            zero_output = True
            # Don't use the motion timer here as it reactivates alarm.
            if self.loop_count % 10 == 0:
                self.logger.info("Taking a short delay at the end of the path so zeroing out autonomy commands.")

        if time.time() - self.disengagement_time < _DISENGAGEMENT_RETRY_DELAY_SEC:
            zero_output = True
            self.resume_motion_timer = time.time()
            if self.loop_count % 10 == 0:
                self.logger.info(
                    "Disengaged for {:.1f} more seconds so zeroing out autonomy commands."
                    .format(_DISENGAGEMENT_RETRY_DELAY_SEC - (time.time() - self.disengagement_time)))

        if (self.activate_autonomy and not zero_output and
                time.time() - self.resume_motion_timer < _RESUME_MOTION_WARNING_TIME_SEC):
            zero_output = True
            self.alarm1.value = True
            self.alarm2.value = False
            self.alarm3.value = True
        else:
            self.alarm1.value = False
            self.alarm2.value = False
            self.alarm3.value = False

        # Determine final drive commands.
        vel_cmd, steer_cmd, strafe_cmd = self.calc_drive_commands(
            autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output)
        # Slow Vel down by 50% when steering is at max.
        self.vel_cmd = vel_cmd * 1.0 / (1.0 + abs(steer_cmd))

        time8b = time.time() - debug_time
        # Update master on latest calculations.
        send_data = (self.gps.last_sample(), self.nav_path.points,
                     self.next_point_heading, debug_points,
                     self.control_state, self.motor_state,
                     self.autonomy_hold, self.gps_path_lateral_error,
                     self.gps_path_angular_error,
                     self.gps_path_lateral_error_rate(),
                     self.gps_path_angular_error_rate(),
                     strafe_p, steer_p, strafe_d, steer_d,
                     user_web_page_plot_steer_cmd,
                     user_web_page_plot_strafe_cmd,
                     self.gps.is_dual_fix(),
                     self.voltage_average,
                     self.last_energy_segment,
                     self.temperatures)
        with self.remote_to_main_lock:
            self.remote_to_main_string["value"] = pickle.dumps(send_data)

        period = time.time() - tick_time
        self.last_energy_segment = None

        time9 = time.time() - debug_time
        self.vel_cmd = self.vel_cmd * 0.6  # Fixed factor reduction

        # Perform acceleration on self.vel_cmd value.
        profiled_vel = get_profiled_velocity(self.last_vel_cmd, self.vel_cmd,
                                             period)
        self.logger.debug("self.last_vel_cmd {}, self.vel_cmd {}, profiled_vel {}".format(
            self.last_vel_cmd, self.vel_cmd, profiled_vel))
        self.vel_cmd = profiled_vel
        self.last_vel_cmd = self.vel_cmd
        tick_time = time.time()

        steering_limit = _STEERING_ANGLE_LIMIT_DEGREES
        if len(self.nav_path.points) == 2 and self.activate_autonomy:
            steering_limit = _STEERING_ANGLE_LIMIT_AUTONOMY_DEGREES
        self.logger.debug("Final values: Steer {}, Vel {}, Strafe {}".format(
            steer_cmd, self.vel_cmd, strafe_cmd))
        calc = calculate_steering(steer_cmd, self.vel_cmd, strafe_cmd, steering_limit)
        self.logger.debug("Calculated 4ws values: Steer {}, Vel {}".format(
            calc, self.vel_cmd))

        steering_okay, steering_error_string = compare_steering_values(
            self.last_calculated_steering, calc)
        if not steering_okay:
            self.logger.error("{}".format(steering_error_string))
            self.logger.error("Final values: Steer {}, Vel {}, Strafe {}".format(
                steer_cmd, self.vel_cmd, strafe_cmd))
            self.logger.error("old 4ws values: Steer {}".format(
                self.last_calculated_steering))
            self.logger.error("new 4ws values: Steer {}".format(calc))
            # if self.activate_autonomy:
            # TODO: the following line is a hack for testing. TLA 9/30/2021
            # self.load_path_time = time.time() - _PATH_END_PAUSE_SEC
            # calc["front_left"] = (calc["front_left"][0], 0.0)
            # calc["front_right"] = (calc["front_right"][0], 0.0)
            # calc["rear_left"] = (calc["rear_left"][0], 0.0)
            # calc["rear_right"] = (calc["rear_right"][0], 0.0)

        # calc["front_left"] = (0.6, 0.0)
        # calc["front_right"] = (0.0, 0.0)
        # calc["rear_left"] = (0.0, 0.0)
        # calc["rear_right"] = (0.0, 0.0)

        self.last_calculated_steering = calc

        # Send calculated values to shared memory, copying in values
        # rather than replacing the object.
        self.steering_debug[:] = steering_to_numpy(calc)[:]

        # if not steering_okay:
        #     sys.exit()

        # If the robot is simulated, estimate movement odometry.
        if self.simulated_hardware:
            if abs(steer_cmd) > 0.001 or abs(self.vel_cmd) > 0.001 or abs(strafe_cmd) > 0.001:
                self.simulate_next_gps_sample(steer_cmd, strafe_cmd)
        if not self.motor_socket:
            self.logger.error("Connect to motor control socket")
            self.connect_to_motors()

        time10 = self.print_power_consumption(calc, debug_time)

        if self.simulated_hardware:
            time.sleep(_POLL_MILLISECONDS / _MILLISECONDS_PER_SECOND)
            # time.sleep(0.1)
        self.logger.debug(
            "Took {} sec to get here. {} {} {} {} {} {} {} {} {} {} {} {}"
            .format(time.time() - debug_time, time1, time2, time3,
                    time4, time5, time6, time7, time8, time8b, time9,
                    time10, self.robot_object.wifi_ap_name))

    def reload_path(self):
        if isinstance(self.robot_object.loaded_path[0], PathSection):
            self.nav_path_list = self.robot_object.loaded_path
            if self.simulated_hardware:
                self.nav_path_index = 0
            else:
                closest_row_index = 0
                min_distance = math.inf
                for index, row_path in enumerate(self.nav_path_list):
                    for point in row_path.points:
                        dist = gps_tools.get_distance(self.gps.last_sample(), point)
                        if dist < min_distance:
                            min_distance = dist
                            closest_row_index = index
                self.logger.info(f"Loading path. "
                                 f"List length {len(self.nav_path_list)}, "
                                 f"Closest path index {closest_row_index}, "
                                 f"min_distance {min_distance}")
                self.nav_path_index = closest_row_index

            self.load_path(self.nav_path_list[self.nav_path_index],
                           simulation_teleport=True, generate_spline=True)
            self.reloaded_path = True

        else:
            self.load_path(self.robot_object.loaded_path,
                           simulation_teleport=True, generate_spline=True)
            self.reloaded_path = True
        self.load_path_time = time.time()

    def steering_calc(self):
        """
        Begin steering calculation.
        TODO: Break this out to a separate module.
        """

        if not self.nav_path.points:
            return (gps_tools.GpsPoint(0, 0),  # projected_path_tangent_point,
                    None,  # closest_path_point,
                    math.inf,  # absolute_path_distance,
                    None,  # drive_reverse,
                    None,  # calculated_rotation,
                    None)  # calculated_strafe)

        closest_path_point, path_point_heading = self.calc_closest_path_point_and_path_point_heading()
        absolute_path_distance = gps_tools.get_distance(self.gps.last_sample(), closest_path_point)
        calculated_rotation = path_point_heading - self.gps.last_sample().azimuth_degrees
        # Truncate values to between 0 and 360
        calculated_rotation %= 360
        # Set value to +/- 180
        if calculated_rotation > 180:
            calculated_rotation -= 360
        self.logger.debug("calculated_rotation: {}, DISTANCE: {}".format(
            calculated_rotation, abs(absolute_path_distance)))
        self.logger.debug("robot heading {}, path heading {}".format(
            self.gps.last_sample().azimuth_degrees, path_point_heading))

        projected_path_tangent_point = gps_tools.project_point(closest_path_point, path_point_heading, 3.0)
        gps_lateral_distance_error = gps_tools.get_approx_distance_point_from_line(
            self.gps.last_sample(), closest_path_point, projected_path_tangent_point)
        calculated_strafe = gps_lateral_distance_error

        _MAXIMUM_ROTATION_ERROR_DEGREES = 140

        vehicle_dir = self.nav_path.navigation_parameters.vehicle_travel_direction
        path_dir = self.nav_path.navigation_parameters.path_following_direction
        drive_solution_okay = (vehicle_dir == Direction.EITHER or
                               vehicle_dir == path_dir and abs(calculated_rotation) <= _MAXIMUM_ROTATION_ERROR_DEGREES)
        self.driving_direction = -1 if vehicle_dir == Direction.BACKWARD else 1
        if vehicle_dir == Direction.EITHER and (abs(calculated_rotation) > 90 or self.nav_direction == -1):
            self.driving_direction = -1

        if vehicle_dir == Direction.EITHER:
            if abs(calculated_rotation) > 90:
                calculated_rotation -= math.copysign(180, calculated_rotation)
            calculated_strafe *= self.nav_direction
        elif vehicle_dir == Direction.FORWARD:
            if (abs(calculated_rotation) > _MAXIMUM_ROTATION_ERROR_DEGREES and
                    (path_dir in (Direction.EITHER, Direction.BACKWARD))):
                calculated_rotation -= math.copysign(180, calculated_rotation)
                calculated_strafe *= -1
                self.nav_direction = -1
        elif vehicle_dir == Direction.BACKWARD:
            if abs(calculated_rotation) > _MAXIMUM_ROTATION_ERROR_DEGREES:
                if (path_dir in (Direction.EITHER, Direction.FORWARD)):
                    calculated_rotation -= math.copysign(180, calculated_rotation)
                    # calculated_strafe *= -1
                    self.nav_direction = 1
            elif path_dir == Direction.BACKWARD:
                calculated_strafe *= -1

        drive_reverse = 1.0
        if (len(self.nav_path.points) == 2 and path_dir == Direction.EITHER):
            if abs(calculated_rotation) > 20:
                original = calculated_strafe
                if abs(calculated_rotation) > 40:
                    calculated_strafe = 0
                    self.gps_lateral_error_rate_averaging_list = []
                else:
                    calculated_strafe *= (40 - abs(calculated_rotation)) / 20.0
                self.logger.debug("Reduced strafe from {}, to: {}".format(original, calculated_strafe))
            else:
                vehicle_position = (self.gps.last_sample().lat, self.gps.last_sample().lon)
                drive_reverse = gps_tools.determine_point_move_sign(
                    self.nav_path.points, vehicle_position)
                # Figure out if we're close to aligned with the
                # target point and reduce forward or reverse
                # velocity if so.
                closest_pt_on_line = gps_tools.find_closest_pt_on_line(
                    self.nav_path.points[0], self.nav_path.points[1],
                    vehicle_position)
                dist_along_line = gps_tools.get_distance(self.nav_path.points[0], closest_pt_on_line)
                self.logger.debug("dist_along_line {}".format(dist_along_line))
                if gps_lateral_distance_error > 1.0:
                    # Reduce forward/reverse direction command
                    # if we are far from the line but also
                    # aligned to the target point.
                    if dist_along_line < 1.0:
                        drive_reverse = 0
                    elif dist_along_line < 2.0:
                        drive_reverse *= 0.1
                    elif dist_along_line < 4.0:
                        drive_reverse *= 0.25

            self.logger.debug("rotation {}, strafe: {} direction {}, drive_reverse {}".
                              format(calculated_rotation, calculated_strafe,
                                     self.driving_direction, drive_reverse))

        if not drive_solution_okay:
            self.autonomy_hold = True
            self.activate_autonomy = False
            self.control_state = CONTROL_NO_STEERING_SOLUTION
            self.logger.error("Could not find drive solution. Disabling autonomy.")
            self.logger.error("calculated_rotation: {}, vehicle_travel_direction {}, path_following_direction {}"
                              .format(calculated_rotation, self.nav_path.
                                      navigation_parameters.vehicle_travel_direction,
                                      self.nav_path.navigation_parameters.
                                      path_following_direction))

        self.logger.debug(
            f"calculated_rotation: {calculated_rotation}, "
            f"vehicle_travel_direction {self.nav_path.navigation_parameters.vehicle_travel_direction}, "
            f"path_following_direction {self.nav_path.navigation_parameters.path_following_direction}, "
            f"self.nav_direction {self.nav_direction}, "
            f"self.driving_direction {self.driving_direction}")

        gps_path_angle_error = calculated_rotation
        # Accumulate a list of error values for angular and
        # lateral error. This allows averaging of errors
        # and also determination of their rate of change.
        time_delta = time.time() - self.gps_error_update_time
        self.gps_error_update_time = time.time()
        if not self.reloaded_path:
            AppendFIFO(self.gps_lateral_error_rate_averaging_list,
                       (gps_lateral_distance_error - self.gps_path_lateral_error) / time_delta,
                       _ERROR_RATE_AVERAGING_COUNT)
            AppendFIFO(self.gps_angle_error_rate_averaging_list,
                       (gps_path_angle_error - self.gps_path_angular_error) / time_delta,
                       _ERROR_RATE_AVERAGING_COUNT)
            self.logger.debug(
                "gps_lateral_distance_error: {}, self.gps_path_lateral_error: {}"
                .format(gps_lateral_distance_error, self.gps_path_lateral_error))
        self.gps_path_angular_error = gps_path_angle_error
        self.gps_path_lateral_error = gps_lateral_distance_error

        # Check end conditions.
        pt = self.nav_path.points[0] if self.nav_direction == -1 else self.nav_path.points[-1]
        end_distance = gps_tools.get_distance(self.gps.last_sample(), pt)
        if abs(calculated_rotation) < self.nav_path.end_angle_degrees and end_distance < self.nav_path.end_distance_m:
            self.logger.info("MET END CONDITIONS {} {}".format(calculated_rotation, absolute_path_distance))
            if self.nav_path.navigation_parameters.loop_path:
                self.load_path(self.nav_path, simulation_teleport=False, generate_spline=False)
            else:
                self.nav_path_index += 1
                if self.nav_path_index < len(self.nav_path_list):
                    self.load_path(self.nav_path_list[self.nav_path_index],
                                   simulation_teleport=False, generate_spline=True)
                else:
                    # self.nav_path_list = []
                    self.nav_path_index = 0
                    self.load_path(self.nav_path_list[self.nav_path_index],
                                   simulation_teleport=True, generate_spline=True)
                    # self.control_state = CONTROL_ONLINE
                    # self.autonomy_hold = True
                    # self.activate_autonomy = False

            self.load_path_time = time.time()
            self.gps_lateral_error_rate_averaging_list = []
            self.gps_angle_error_rate_averaging_list = []
            self.reloaded_path = True
            return (projected_path_tangent_point, closest_path_point, absolute_path_distance,
                    drive_reverse, calculated_rotation, calculated_strafe)

        self.reloaded_path = False
        self.logger.debug("self.gps_path_lateral_error_rate {}, {} / {}".format(
            self.gps_path_lateral_error_rate(),
            sum(self.gps_lateral_error_rate_averaging_list),
            len(self.gps_lateral_error_rate_averaging_list)))

        self.next_point_heading = calculated_rotation
        return (projected_path_tangent_point, closest_path_point, absolute_path_distance,
                drive_reverse, calculated_rotation, calculated_strafe)

    def calc_closest_path_point_and_path_point_heading(self):
        path_point_heading = None
        if len(self.nav_path.points) == 2:
            p0, p1 = self.nav_path.points
            if self.nav_path.navigation_parameters.vehicle_travel_direction == Direction.EITHER:
                closest_path_point = gps_tools.check_point(p0)
            else:
                if self.nav_direction == -1:
                    closest_path_point = gps_tools.check_point(p0)
                elif self.nav_direction == 1:
                    closest_path_point = gps_tools.check_point(p1)
            path_point_heading = gps_tools.get_heading(p0, p1)
        else:
            closest_u = self.nav_path.spline.closestUOnSpline(self.gps.last_sample())
            closest_path_point = self.nav_path.spline.coordAtU(closest_u)
            # Heading specified at this point on the path.
            path_point_heading = math.degrees(self.nav_path.spline.slopeRadiansAtU(closest_u))
        return closest_path_point, path_point_heading

    def calc_commands_for_autonomy(self, calculated_rotation, calculated_strafe, drive_reverse):
        strafe_d = 0
        steer_d = 0
        strafe_p = 0
        steer_p = 0
        user_web_page_plot_steer_cmd = 0
        user_web_page_plot_strafe_cmd = 0
        autonomy_vel_cmd = 0.0
        autonomy_steer_cmd = 0.0
        autonomy_strafe_cmd = 0.0
        strafe_multiplier = 1.0

        # Calculate driving commands for autonomy.
        if (self.next_point_heading != -180 and self.activate_autonomy and
                calculated_rotation is not None and calculated_strafe is not None):

            # calculated_rotation *= 2.0
            # calculated_strafe *= -0.35
            # steer_d = self.gps_path_angular_error_rate() * 0.8
            # strafe_d = self.gps_path_lateral_error_rate * -0.2

            steer_p = calculated_rotation * self.nav_path.control_values.angular_p
            strafe_p = calculated_strafe * self.nav_path.control_values.lateral_p
            steer_d = self.gps_path_angular_error_rate() * self.nav_path.control_values.angular_d
            strafe_d = self.gps_path_lateral_error_rate() * self.nav_path.control_values.lateral_d

            self.logger.debug("strafe_p {}, strafe_d {}, steer_p {} steer_d {}".format(
                strafe_p, strafe_d, strafe_d, steer_d))

            steer_command_value = steer_p + steer_d
            strafe_command_value = strafe_p + strafe_d

            _STEER_LIMIT = 45
            unfiltered_steer_cmd = clamp(steer_command_value, -_STEER_LIMIT, _STEER_LIMIT)
            unfiltered_steer_cmd /= 45.0
            _STRAFE_LIMIT = 0.25
            unfiltered_strafe_cmd = clamp(strafe_command_value, -_STRAFE_LIMIT, _STRAFE_LIMIT)
            unfiltered_strafe_cmd *= self.driving_direction * strafe_multiplier

            autonomy_steer_diff = unfiltered_steer_cmd - self.last_autonomy_steer_cmd
            autonomy_strafe_diff = unfiltered_strafe_cmd - self.last_autonomy_strafe_cmd

            self.logger.debug("diffs: {}, {}".format(
                autonomy_steer_diff * _LOOP_RATE, autonomy_strafe_diff * _LOOP_RATE))

            # Rate of change clamping
            steer_rate = 4.0 / _LOOP_RATE
            strafe_rate = 40.0 / _LOOP_RATE
            autonomy_steer_cmd = clamp(unfiltered_steer_cmd,
                                       self.last_autonomy_steer_cmd - steer_rate,
                                       self.last_autonomy_steer_cmd + steer_rate)
            autonomy_steer_cmd *= self.driving_direction
            autonomy_strafe_cmd = clamp(unfiltered_strafe_cmd,
                                        self.last_autonomy_strafe_cmd - strafe_rate,
                                        self.last_autonomy_strafe_cmd + strafe_rate)
            if abs(autonomy_steer_cmd) > 0.8:
                # Maxed out steer and strafe can result in strafe only
                # due to steering limits, so reduce strafe with maxed
                # steering. TODO: linear taper
                autonomy_strafe_cmd *= 0.2

            self.last_autonomy_steer_cmd = autonomy_steer_cmd
            self.last_autonomy_strafe_cmd = autonomy_strafe_cmd
            user_web_page_plot_steer_cmd = autonomy_steer_cmd * self.driving_direction
            user_web_page_plot_strafe_cmd = autonomy_strafe_cmd * self.driving_direction

            if 0.0 <= self.nav_path.navigation_parameters.travel_speed <= self.maximum_velocity:
                autonomy_vel_cmd = (self.nav_path.navigation_parameters.travel_speed
                                    * self.driving_direction * drive_reverse)
                self.logger.debug("Travel speed: {}".format(self.nav_path.navigation_parameters.travel_speed))
            else:
                self.logger.error("Invalid travel speed specified! Got {}. Maximum allowed is {}"
                                  .format(self.nav_path.navigation_parameters.travel_speed, self.maximum_velocity))
                autonomy_vel_cmd = 0.0
            self.logger.debug(
                "self.autonomy_velocity {}, self.driving_direction {}, drive_reverse {} , autonomy_vel_cmd {}"
                .format(self.autonomy_velocity, self.driving_direction,
                        drive_reverse, autonomy_vel_cmd))

            self.joy.clear_steer()  # ensures that vel goes to zero when autonomy disabled
            logger_string = (f"steer_cmd: {autonomy_steer_cmd:.2f}, "
                             f"strafe_cmd: {autonomy_strafe_cmd:.2f}, "
                             f"vel_cmd: {self.vel_cmd:.2f}, "
                             f"calculated_rotation: {calculated_rotation:.2f}, "
                             f"calculated_strafe: {calculated_strafe:.2f}")
            if self.loop_count % 10 == 0:
                self.logger.info(logger_string)
            else:
                self.logger.debug(logger_string)
        return (user_web_page_plot_steer_cmd, user_web_page_plot_strafe_cmd,
                strafe_d, steer_d, strafe_p, steer_p,
                autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd)

    def safty_checks(self, absolute_path_distance, gps_path_angle_error):
        # Begin Safety Checks
        error_messages = []
        fatal_error = False
        zero_output = False

        if self.motor_state != _STATE_ENABLED:
            error_messages.append("Motor error so zeroing out autonomy commands.")
            zero_output = True
            self.control_state = CONTROL_MOTOR_ERROR
            self.resume_motion_timer = time.time()

        if self.voltage_average < _VOLTAGE_CUTOFF:
            fatal_error = True
            error_messages.append("Voltage low so zeroing out autonomy commands.")
            zero_output = True
            self.control_state = CONTROL_LOW_VOLTAGE
            self.resume_motion_timer = time.time()

        if not self.gps.is_dual_fix():
            error_messages.append("No GPS fix so zeroing out autonomy commands.")
            zero_output = True
            self.control_state = CONTROL_GPS_STARTUP
            self.resume_motion_timer = time.time()
        elif abs(absolute_path_distance) > self.nav_path.maximum_allowed_distance_meters:
            # Distance from path exceeds allowed limit.
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = CONTROL_AUTONOMY_ERROR_DISTANCE
            error_messages.append(
                "GPS distance {} meters too far from path so zeroing out autonomy commands."
                .format(abs(absolute_path_distance)))
        elif abs(gps_path_angle_error) > self.nav_path.maximum_allowed_angle_error_degrees:
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = CONTROL_AUTONOMY_ERROR_ANGLE
            error_messages.append(
                "GPS path angle {} exceeds allowed limit {} so zeroing out autonomy commands."
                .format(
                    abs(gps_path_angle_error),
                    self.nav_path.maximum_allowed_angle_error_degrees))
        elif self.gps.last_sample().rtk_age > _ALLOWED_RTK_AGE_SEC:
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = CONTROL_AUTONOMY_ERROR_RTK_AGE
            error_messages.append("RTK base station data too old so zeroing out autonomy commands.")
        elif time.time() - self.gps.last_sample().time_stamp > _ALLOWED_SOLUTION_AGE_SEC:
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = CONTROL_AUTONOMY_ERROR_SOLUTION_AGE
            error_messages.append("RTK solution too old so zeroing out autonomy commands.")

        if time.time() - self.robot_object.last_server_communication_stamp > SERVER_COMMUNICATION_DELAY_LIMIT_SEC:
            zero_output = True
            self.resume_motion_timer = time.time()
            error_messages.append(
                f"Server communication error so zeroing out autonomy commands. "
                f"Last stamp age {time.time() - self.robot_object.last_server_communication_stamp} "
                f"exceeds allowed age of {SERVER_COMMUNICATION_DELAY_LIMIT_SEC} seconds. "
                f"AP name: {self.robot_object.wifi_ap_name}, "
                f"Signal Strength {self.robot_object.wifi_strength} dbm\r\n")

        return error_messages, fatal_error, zero_output

    def maybe_restart_wifi(self):
        if (time.time() > self.last_wifi_restart_time + 500 and
            time.time() - self.robot_object.last_server_communication_stamp > _SERVER_DELAY_RECONNECT_WIFI_SECONDS and
                self.robot_object.last_server_communication_stamp > 0):
            self.logger.error(
                "Last Wifi signal strength: {} dbm\r\n".format(
                    self.robot_object.wifi_strength))
            self.logger.error("Last Wifi AP associated: {}\r\n".format(
                self.robot_object.wifi_ap_name))
            self.logger.error("Restarting wlan1...")
            try:
                subprocess.check_call("ifconfig wlan1 down",
                                      shell=True)
                subprocess.check_call("ifconfig wlan1 up", shell=True)
            except BaseException:
                pass
            self.last_wifi_restart_time = time.time()
            self.logger.error("Restarted wlan1.")

    def write_errors(self, error_messages):
        # Ensure we always print errors if we are deactivating autonomy.
        if self.loop_count % _ERROR_SKIP_RATE != 0:
            for error in error_messages:
                self.logger.error(error)
        self.logger.error("Last Wifi signal strength: {} dbm\r\n".format(
            self.robot_object.wifi_strength))
        self.logger.error("Last Wifi AP associated: {}\r\n".format(
            self.robot_object.wifi_ap_name))
        self.logger.error("Last CPU Temp: {}\r\n".format(
            self.robot_object.cpu_temperature_c))

        with open("error_log.txt", 'a+') as file1:
            file1.write("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n")
            file1.write("Disegagement Log\r\n")
            file1.write(datetime.datetime.now().strftime(
                "%a %b %d, %I:%M:%S %p\r\n"))
            file1.write("Last Wifi signal strength: {} dbm\r\n".format(
                self.robot_object.wifi_strength))
            file1.write("Last Wifi AP associated: {}\r\n".format(
                self.robot_object.wifi_ap_name))
            file1.write("Last CPU Temp: {}\r\n".format(
                self.robot_object.cpu_temperature_c))
            if self.gps.last_good_sample() is not None:
                file1.write("Last known GPS location: {}, {}\r\n".
                            format(self.gps.last_good_sample().lat,
                                   self.gps.last_good_sample().lon))
            else:
                file1.write("No valid GPS location recorded.")
            error_count = 1
            for error in error_messages:
                file1.write("Error {}: {}\r\n".format(error_count, error))
                error_count += 1
            file1.write("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n")

    def calc_drive_commands(self, autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output):
        vel_cmd = 0.0
        steer_cmd = 0.0
        strafe_cmd = 0
        if self.activate_autonomy:
            autonomy_time_elapsed = time.time() - self.load_path_time - _PATH_END_PAUSE_SEC
            if autonomy_time_elapsed < _BEGIN_AUTONOMY_SPEED_RAMP_SEC:
                autonomy_vel_cmd *= autonomy_time_elapsed / _BEGIN_AUTONOMY_SPEED_RAMP_SEC
                print(autonomy_time_elapsed)
                # autonomy_strafe_cmd = 0
            self.control_state = CONTROL_AUTONOMY
            if zero_output:
                self.control_state = CONTROL_AUTONOMY_PAUSE
            else:
                vel_cmd = autonomy_vel_cmd
                steer_cmd = autonomy_steer_cmd
                strafe_cmd = autonomy_strafe_cmd
        else:
            if self.voltage_average < _VOLTAGE_CUTOFF:
                if self.loop_count % _ERROR_SKIP_RATE == 0:
                    self.logger.error(f"LOW VOLTAGE PAUSE. Voltage average: {self.voltage_average:.2f}")
            else:
                vel_cmd = self.joy.throttle
                steer_cmd = self.joy.steer
                if math.fabs(self.joy.strafe) < 0.1:
                    strafe_cmd = 0
                else:
                    strafe_cmd = math.copysign(math.fabs(self.joy.strafe) - 0.1, self.joy.strafe)
        return vel_cmd, steer_cmd, strafe_cmd

    def simulate_next_gps_sample(self, steer_cmd, strafe_cmd):
        steering_multiplier = 0.4
        vel_multiplier = 0.5
        steering_multiplier = 0.4
        vel_multiplier = 0.5
        strafe_multiplier = 1.0
        new_heading_degrees = self.gps.last_sample().azimuth_degrees + \
            steer_cmd * 45.0 * steering_multiplier * self.driving_direction
        new_heading_degrees %= 360
        next_point = gps_tools.project_point(self.gps.last_sample(),
                                             new_heading_degrees,
                                             self.vel_cmd * vel_multiplier)
        # Calculate translation for strafe, which is movement 90 degrees from heading.
        next_point = gps_tools.project_point(next_point,
                                             new_heading_degrees + 90,
                                             strafe_cmd * strafe_multiplier)
        rand_dist = random.uniform(-0.05, 0.05)
        rand_angle = random.uniform(0, 360.0)
        next_point = gps_tools.project_point(next_point, rand_angle, rand_dist)
        new_heading_degrees += random.uniform(-3.0, 3.0)
        self.gps.update_simulated_sample(next_point.lat, next_point.lon, new_heading_degrees)

    def print_power_consumption(self, calc, debug_time):
        # Try to send final drive commands to motor process.
        try:
            if self.motor_send_okay:
                # print("send_calc")
                self.motor_socket.send_pyobj(pickle.dumps(calc),
                                             flags=zmq.NOBLOCK)
                self.motor_send_okay = False
                self.motor_last_send_time = time.time()

            time10 = time.time() - debug_time
            # else:
            #     print("NOT OKAY TO SEND")
            while self.motor_socket.poll(timeout=_VERY_FAST_POLL_MILLISECONDS):
                # print("poll motor message")
                motor_message = pickle.loads(self.motor_socket.recv_pyobj())
                self.motor_send_okay = True
                # print("motor_message: {}".format(motor_message))
                try:
                    self.motor_state, self.voltages, self.bus_currents, self.temperatures = motor_message
                    self.total_watts = 0
                    if len(self.voltages) > 0:
                        self.voltage_average = sum(self.voltages) / len(self.voltages)
                    for volt, current in zip(self.voltages,
                                             self.bus_currents):
                        self.total_watts += volt * current
                    # print("Drawing {} Watts.".format(int(self.total_watts)))
                except Exception as e:
                    self.logger.error("Error reading motor state message.")
                    self.logger.error(e)
                    self.motor_state = STATE_DISCONNECTED
        except zmq.error.Again:
            self.logger.error("Remote server unreachable.")
        except zmq.error.ZMQError:
            self.logger.error("ZMQ error with motor command socket. Resetting.")
            self.motor_state = STATE_DISCONNECTED
            self.close_motor_socket()

        # If we have a GPS fix, update power consumption metrics.
        if self.gps.is_dual_fix():
            # Calculate power consumption metrics in 1 meter segments
            self.power_consumption_list.append(
                (self.gps.last_sample(), self.total_watts,
                 self.voltages, self.bus_currents))
            oldest_power_sample_gps = self.power_consumption_list[0][0]
            distance = gps_tools.get_distance(oldest_power_sample_gps,
                                              self.gps.last_sample())
            if distance > 1.0:
                total_watt_seconds = 0
                watt_average = self.power_consumption_list[0][1]
                distance_sum = 0
                last_sample_gps = None
                motor_total_watt_seconds = [0, 0, 0, 0]
                motor_watt_average = [0, 0, 0, 0]
                list_subsamples = []
                sample_collector_index = 0
                for sample_num in range(1, len(self.power_consumption_list)):
                    sample1 = self.power_consumption_list[sample_num - 1]
                    sample2 = self.power_consumption_list[sample_num]
                    if sample1 is None or sample2 is None:
                        continue
                    sample_distance = gps_tools.get_distance(sample1[0], sample2[0])
                    sample_duration = sample2[0].time_stamp - sample1[0].time_stamp
                    sample_avg_watts = (sample1[1] + sample2[1]) / 2.0
                    if sample_num > sample_collector_index:
                        list_subsamples.append(sample1[0])
                        if len(self.power_consumption_list) > _NUM_GPS_SUBSAMPLES:
                            sample_collector_index += int(len(self.power_consumption_list)/_NUM_GPS_SUBSAMPLES)
                        else:
                            sample_collector_index += 1
                    # print(sample1[2])
                    try:
                        for idx in range(len(sample1[2])):
                            motor_watt_average[idx] = (sample1[2][idx] * sample1[3][idx] +
                                                       sample2[2][idx] * sample2[3][idx]) * 0.5
                            motor_total_watt_seconds[idx] = motor_watt_average[idx] * sample_duration
                    except Exception as e:
                        self.logger.error(e)
                        self.logger.error(sample1[2])
                        self.logger.error(sample1[3])
                    watt_average += sample2[1]
                    watt_seconds = sample_avg_watts * sample_duration
                    total_watt_seconds += watt_seconds
                    distance_sum += sample_distance
                    last_sample_gps = sample2[0]

                avg_watts = (watt_average) / \
                    len(self.power_consumption_list)
                list_subsamples.append(last_sample_gps)

                self.last_energy_segment = EnergySegment(
                    self.loop_count, oldest_power_sample_gps,
                    last_sample_gps, distance_sum, total_watt_seconds,
                    avg_watts, motor_total_watt_seconds,
                    motor_watt_average, list_subsamples,
                    self.activate_autonomy,
                    self.robot_object.wifi_ap_name,
                    self.robot_object.wifi_strength)

                # Reset power consumption list.
                self.power_consumption_list = []
                self.logger.info(f"Avg watts {self.last_energy_segment.avg_watts:.1f}, "
                                 f"watt seconds per meter: {self.last_energy_segment.watt_seconds_per_meter:.1f}, "
                                 f"meters per second: {self.last_energy_segment.meters_per_second:.2f}, "
                                 f"height change {self.last_energy_segment.height_change:.2f}")
        return time10

    def gps_path_angular_error_rate(self):
        if len(self.gps_angle_error_rate_averaging_list) == 0:
            return 0
        return sum(self.gps_angle_error_rate_averaging_list) / len(self.gps_angle_error_rate_averaging_list)

    def gps_path_lateral_error_rate(self):
        if len(self.gps_lateral_error_rate_averaging_list) == 0:
            return 0
        return sum(self.gps_lateral_error_rate_averaging_list) / len(self.gps_lateral_error_rate_averaging_list)


def run_control(remote_to_main_lock,
                main_to_remote_lock,
                remote_to_main_string,
                main_to_remote_string,
                logging,
                logging_details,
                simulated_hardware=False):
    remote_control = RemoteControl(remote_to_main_lock, main_to_remote_lock,
                                   remote_to_main_string,
                                   main_to_remote_string, logging,
                                   logging_details, simulated_hardware)
    remote_control.run_setup()
    remote_control.run_loop()


if __name__ == "__main__":
    run_control()
