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
import time
import math
import pickle
import gps_tools
import numpy as np
import spline_lib
import os
import datetime
import gps
import subprocess
from enum import Enum
import random
from multiprocessing import shared_memory
import datetime

from joystick import Joystick, JoystickSBUS
from steering import calculate_steering, compare_steering_values, steering_to_numpy, recalculate_steering_values, steering_calc
from utils import AppendFIFO, clamp
import corner_actuator_can as corner_actuator
import model
from model import Direction
import utils
import display_i2c as display

BOARD_VERSION = 2


# This file gets imported by server but we should only import GPIO on raspi.
if os.uname().machine in ['armv7l','aarch64']:
    import board
    import busio
    from adafruit_mcp230xx.mcp23017 import MCP23017
    from adafruit_mcp230xx.mcp23016 import MCP23016

# COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

IGNORE_SERVER_COMMUNICATIONS_ERRORS = True

ACCELERATION_COUNTS_SEC = 0.5

_RESUME_MOTION_WARNING_TIME_SEC = 4
# _RESUME_MOTION_WARNING_TIME_SEC = -1

_SEC_IN_ONE_MINUTE = 60

_DEFAULT_TRAVEL_SPEED = 0.2
_DEFAULT_PATH_END_DISTANCE_METERS = 1
_DEFAULT_PATH_END_ANGLE_DEGREES=45
_DEFAULT_ANGULAR_P=1.5
_DEFAULT_LATERAL_P=0.25
_DEFAULT_ANGULAR_D=0.3
_DEFAULT_LATERAL_D=0.05
_DEFAULT_MAXIMUM_VELOCITY = 0.4

# _DEFAULT_TRAVEL_SPEED = 0.7
# _DEFAULT_PATH_END_DISTANCE_METERS = 1
# _DEFAULT_PATH_END_ANGLE_DEGREES=45
# _DEFAULT_ANGULAR_P=1
# _DEFAULT_LATERAL_P=0.18
# _DEFAULT_ANGULAR_D=0.2
# _DEFAULT_LATERAL_D=0.02
# _DEFAULT_MAXIMUM_VELOCITY = 0.75

_CLOSED_LOOP_ENDS_DISTANCE_METERS = 2.0

_MAXIMUM_ALLOWED_DISTANCE_METERS = 2.5
_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES = 120  # 20
_VOLTAGE_CUTOFF = 25
_VOLTAGE_RESUME_MANUAL_CONTROL = 35

_GPS_ERROR_RETRIES = 3


_NUM_GPS_SUBSAMPLES = 10


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
_VERY_FAST_POLL_MILLISECONDS = 1

_SIMULATED_LOOP_RATE_HZ = 30

_ERROR_SKIP_RATE = 60
GPS_PRINT_INTERVAL = 30
_DEFAULT_LOGGING_SKIP_RATE = 200

_DISENGAGEMENT_RETRY_DELAY_MINUTES = 1
_DISENGAGEMENT_RETRY_DELAY_SEC = _DISENGAGEMENT_RETRY_DELAY_MINUTES * _SEC_IN_ONE_MINUTE

_USE_SBUS_JOYSTICK = True

_DEBUG_STEERING = False
_RUN_PROFILER = False

SYSTEM_HAS_GPS = False



_ENCODER_LOG_PATH = "/home/acorn/encoder_logs/"
_FUSION_LOG_CUTOFF = 1000

def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel - last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel - last_vel
    else:
        increment = math.copysign(ACCELERATION_COUNTS_SEC,
                                  unfiltered_vel - last_vel) * period_s
    return last_vel + increment



class PathControlValues():
    def __init__(self, angular_p, lateral_p, angular_d, lateral_d):
        self.angular_p = angular_p
        self.lateral_p = lateral_p
        self.angular_d = angular_d
        self.lateral_d = lateral_d

    def __str__(self):
        return f"Angular P:{self.angular_p}, Lateral P:{self.lateral_p}, Angular D:{self.angular_d}, Lateral D:{self.lateral_d}"

class AccessoryParameters():
    def __init__(self, accessory_type):
        self.accessory_type = accessory_type

class CameraParameters(AccessoryParameters):
    def __init__(self):
        super().__init__(model.AccessoryType.CAMERA)

class LawnmowerParameters(AccessoryParameters):
    def __init__(self, height_percent, power_percent):
        self.height_percent = height_percent
        self.power_percent = power_percent
        super().__init__(model.AccessoryType.MOWER)


class NavigationParameters():
    def __init__(self, travel_speed, path_following_direction,
                 vehicle_travel_direction, repeat_path):
        self.travel_speed = travel_speed
        self.path_following_direction = path_following_direction
        self.vehicle_travel_direction = vehicle_travel_direction
        self.repeat_path = repeat_path
    def __str__(self):
        return f"Travel Speed:{self.travel_speed}, Path Following Direction:{self.path_following_direction}, Vehicle Travel Direction:{self.vehicle_travel_direction}, Repeat Path:{self.repeat_path}"

class PathSection():
    def __init__(self,
                 points,
                 control_values,
                 navigation_parameters,
                 max_dist=0,
                 max_angle=0,
                 end_dist=0,
                 end_angle=0,
                 closed_loop=False,
                 reset_steering=False,
                 strafe_priority_distance_m=1.0):
        self.points: list = points
        self.spline: spline_lib.GpsSpline = None
        self.maximum_allowed_distance_meters = max_dist
        self.maximum_allowed_angle_error_degrees = max_angle
        self.control_values = control_values
        self.end_distance_m = end_dist
        self.end_angle_degrees = end_angle
        self.navigation_parameters: NavigationParameters = navigation_parameters
        self.closed_loop = closed_loop
        self.accessory_parameters: AccessoryParameters = None
        self.reset_steering = reset_steering  # if true, set steering to zero at start
        self.strafe_priority_distance_m = strafe_priority_distance_m

    def __str__(self):
        str = (f"PathSection is {gps_tools.calc_path_length(self.points):.2f} meters long with {len(self.points)} points.\n")
        str += f"Control Values: {self.control_values}\n"
        str += f"Num points:{len(self.points)}, Max Dist:{self.maximum_allowed_distance_meters}, Max Angle:{self.maximum_allowed_angle_error_degrees}, End Dist:{self.end_distance_m}, End Angle:{self.end_angle_degrees}\n"
        str += f"Navigation Parameters: {self.navigation_parameters}"
        return str



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
        if isinstance(self.duration, datetime.timedelta):
            self.duration = self.duration.total_seconds()
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
                 stop_signal,
                 remote_to_main_lock,
                 main_to_remote_lock,
                 remote_to_main_string,
                 main_to_remote_string,
                 logging,
                 debug,
                 simulated_hardware=False):
        self.stop_signal = stop_signal
        self.remote_to_main_lock = remote_to_main_lock
        self.main_to_remote_lock = main_to_remote_lock
        self.remote_to_main_string = remote_to_main_string
        self.main_to_remote_string = main_to_remote_string
        self.simulated_hardware = simulated_hardware
        self.robot_object = None
        self.next_point_heading = -180
        self.activate_autonomy = False
        self.autonomy_velocity = 0
        self.resume_motion_timer = 0
        self.reloaded_path = False
        self.logger = logging.getLogger('main.remote')
        utils.config_logging(self.logger, debug)
        if _USE_SBUS_JOYSTICK and not simulated_hardware:
            self.joy = JoystickSBUS(self.logger, simulated_hardware)
        else:
            self.joy = Joystick(simulated_hardware)
        fake_gps = False
        if simulated_hardware or SYSTEM_HAS_GPS == False:
            fake_gps = True
        self.gps = gps.GPS(self.logger, fake_gps)
        self.default_navigation_parameters = NavigationParameters(travel_speed=_DEFAULT_TRAVEL_SPEED,
                                                                  path_following_direction=Direction.EITHER,
                                                                  vehicle_travel_direction=Direction.EITHER,
                                                                  repeat_path=True)
        self.default_path_control_vals = PathControlValues(angular_p=_DEFAULT_ANGULAR_P,
                                                           lateral_p=_DEFAULT_LATERAL_P,
                                                           angular_d=_DEFAULT_ANGULAR_D,
                                                           lateral_d=_DEFAULT_LATERAL_D)


        self.nav_path = PathSection(points=[],
                                    control_values=self.default_path_control_vals,
                                    navigation_parameters=self.default_navigation_parameters,
                                    max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                                    max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                                    end_dist=_DEFAULT_PATH_END_DISTANCE_METERS,
                                    end_angle=_DEFAULT_PATH_END_ANGLE_DEGREES)
        self.maximum_velocity = _DEFAULT_MAXIMUM_VELOCITY
        self.nav_path_list = []
        self.nav_path_index = 0
        self.gps_path = []
        self.load_path_time = time.time()
        self.loaded_path_name = ""
        self.last_closest_path_u = 0
        self.autonomy_hold = True
        self.control_state = model.CONTROL_STARTUP
        self.motor_state = model.MOTOR_DISCONNECTED
        self.motor_encoders = None
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
        self.last_calculated_steering = calculate_steering(0, 0, 0)
        self.last_wifi_restart_time = time.time()
        self.last_vel_cmd = 0
        self.vel_cmd = 0
        self.tick_time = time.time()
        self.loop_count = -1
        self.fusion_data_collector = []
        if simulated_hardware:
            self.display = display.DisplaySimulated()
        else:
            self.display = display.Display()
        self.display.display_voltage(0.0)
        self.debug_file_name = "debug_file.txt"
        self.reset_steering_timer = time.time()
        self.motor_comms_last_checksum = 0
        self.last_motor_comms_checksum_change = time.time()

    def run_setup(self):
        if True or self.simulated_hardware:
            class FakeAlarm():
                def __init__(self):
                    self.value = 0
            self.alarm1 = FakeAlarm()
            self.alarm2 = FakeAlarm()
            self.alarm3 = FakeAlarm()
        else:
            i2c = busio.I2C(board.SCL, board.SDA)
            if BOARD_VERSION==1:
                mcp = MCP23017(i2c)  # , address=0x20)  # MCP23017
                self.alarm1 = mcp.get_pin(0)
                self.alarm2 = mcp.get_pin(1)
                self.alarm3 = mcp.get_pin(2)
            elif BOARD_VERSION==2:
                mcp = MCP23016(i2c, address=0x20)
                self.alarm1 = mcp.get_pin(8)
                self.alarm2 = mcp.get_pin(9)
                self.alarm3 = mcp.get_pin(12)

            self.alarm1.switch_to_output(value=False)
            self.alarm2.switch_to_output(value=False)
            self.alarm3.switch_to_output(value=False)
        self.joy.connect()
        self.setup_motor_shared_memory()

    def setup_motor_shared_memory(self):
        motor_output_name = 'motor_output_sharedmem'
        while not os.path.exists('/dev/shm/' + motor_output_name):
            self.logger.warn(f"Waiting for shared memory {motor_output_name}")
            time.sleep(0.5)
        self.motor_out_shm = shared_memory.SharedMemory(name=motor_output_name)
        self.logger.info(f"Connected to existing shared memory {motor_output_name}")
        self.motor_output = np.ndarray(model.MOTOR_SAMPLE_OUTPUT.shape, dtype=model.MOTOR_SAMPLE_OUTPUT.dtype, buffer=self.motor_out_shm.buf)

        motor_input_name = 'motor_input_sharedmem'
        while not os.path.exists('/dev/shm/' + motor_input_name):
            self.logger.warn(f"Waiting for shared memory {motor_input_name}")
            time.sleep(0.5)
        self.motor_in_shm = shared_memory.SharedMemory(name=motor_input_name)
        self.logger.info(f"Connected to existing shared memory {motor_input_name}")
        self.motor_input = np.ndarray(model.MOTOR_SAMPLE_INPUT.shape, dtype=model.MOTOR_SAMPLE_INPUT.dtype, buffer=self.motor_in_shm.buf)

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
            try:
                if path.reset_steering:
                    self.reset_steering_timer = time.time()
            except:
                self.logger.error("Path does not have reset_steering attribute.")
        else:
            # Legacy paths
            nav_spline = spline_lib.GpsSpline(path,
                                              smooth_factor=0,
                                              num_points=200)
            self.nav_path = PathSection(points=nav_spline.points,
                                        control_values=self.default_path_control_vals,
                                        navigation_parameters=self.default_navigation_parameters,
                                        max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                                        max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                                        end_dist=_DEFAULT_PATH_END_DISTANCE_METERS,
                                        end_angle=_DEFAULT_PATH_END_ANGLE_DEGREES)
            path_ends_distance = gps_tools.get_distance(self.nav_path.points[-1], self.nav_path.points[0])
            if path_ends_distance < _CLOSED_LOOP_ENDS_DISTANCE_METERS:
                self.nav_path.closed_loop = True
            self.nav_path.spline = nav_spline
        self.loaded_path_name = self.robot_object.loaded_path_name
        self.last_closest_path_u = 0
        if self.simulated_hardware and simulation_teleport:
            # Place simulated robot on the path.
            if len(self.nav_path.points) == 2:
                start_index = 0
            else:
                start_index = int(len(self.nav_path.spline.points) / 2) - 5
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
                self.gps.last_sample(), self.nav_path.points[-1])
            # This may be overridden farther down.
            if dist_end < dist_start:
                self.nav_direction = -1
            else:
                self.nav_direction = 1
        elif direction == Direction.FORWARD:
            self.nav_direction = 1
        elif direction == Direction.BACKWARD:
            self.nav_direction = -1

    def run_loop(self):
        self.reloaded_path = True

        try:
            while not self.stop_signal.is_set():
                self.loop_count += 1
                self.run_single_loop()
        except KeyboardInterrupt:
            pass

    """
    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
     ____  _             _     __  __       _         _
    / ___|| |_ __ _ _ __| |_  |  \/  | __ _(_)_ __   | |    ___   ___  _ __
    \___ \| __/ _` | '__| __| | |\/| |/ _` | | '_ \  | |   / _ \ / _ \| '_ \
     ___) | || (_| | |  | |_  | |  | | (_| | | | | | | |__| (_) | (_) | |_) |
    |____/ \__\__,_|_|   \__| |_|  |_|\__,_|_|_| |_| |_____\___/ \___/| .__/
                                                                      |_|
    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    """

    def run_single_loop(self):
        """
        This is the main functional loop for this file. It handles all the
        logistics of navigation and remote control. It receives high level
        commands from the main process, reads navigation parameters, joystick,
        and gps data, computes desired motor control values, and sends those
        values to the motor control process.
        """
        self.period = time.time() - self.tick_time
        self.tick_time = time.time()


        # Get real or simulated GPS data.
        if self.simulated_hardware or SYSTEM_HAS_GPS == False:
            if self.loop_count % GPS_PRINT_INTERVAL == 0:
                self.gps.dump()
            s = self.gps.last_sample()
            self.gps.update_simulated_sample(s.lat, s.lon, s.azimuth_degrees)
            time.sleep(1.0/_SIMULATED_LOOP_RATE_HZ)
        else:
            # if self.gps.last_sample() and self.gps.last_sample().status:
            #     self.fusion_data_collector.append((self.motor_encoders.copy(), self.gps.last_sample(), self.gps.raw_sample))
            #     if len(self.fusion_data_collector) > _FUSION_LOG_CUTOFF:
            #         timestr = time.strftime("%Y%m%d-%H%M%S")
            #         path  = os.path.join(_ENCODER_LOG_PATH, f"encoder_log_dual{timestr}.pickle")
            #         self.logger.info(f"Saving encoder data to {path}")
            #         with open(path, 'wb') as handle:
            #             pickle.dump(self.fusion_data_collector, handle, protocol=pickle.HIGHEST_PROTOCOL)
            #         self.fusion_data_collector = []

            self.gps.new_gps_sample(print_gps=self.loop_count % GPS_PRINT_INTERVAL == 0)
        if self.loop_count == 200 and _RUN_PROFILER:
            import cProfile
            prof = cProfile.Profile()
            prof.enable()
        # print(self.gps.last_sample())



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

        if not isinstance(recieved_robot_object, model.RobotSubset):
            self.logger.info("Got unexpected type {}".format(type(recieved_robot_object)))
            self.logger.info("Waiting for valid robot object before running remote control code.")
            time.sleep(_SLOW_POLLING_SLEEP_S)
            return

        self.robot_object = recieved_robot_object
        self.logger.debug("Remote received new robot object.")
        time3 = time.time() - debug_time
        if not self.nav_path.points or self.loaded_path_name != self.robot_object.loaded_path_name:
            if self.robot_object.loaded_path and self.gps.last_sample() is not None:
                self.reload_path()
                self.gps.flush_serial()
        self.activate_autonomy = self.robot_object.activate_autonomy
        # Autonomy is disabled until robot is ready or if joystick is used.
        if self.autonomy_hold:
            self.activate_autonomy = False
        # Reset disabled autonomy if autonomy is turned off in the command.
        if self.robot_object.clear_autonomy_hold:
            self.gps.flush_serial()
            self.logger.debug("clearing autonomy hold.")
            self.autonomy_hold = False
            # Reset disengagement timer.
            self.disengagement_time = time.time() - _DISENGAGEMENT_RETRY_DELAY_SEC

        time4 = time.time() - debug_time

        debug_points = (None, None, None, None)
        gps_path_angle_error = 0
        absolute_path_distance = math.inf
        time5 = 0
        gps_sample_valid = False
        sample = self.gps.last_sample()
        if sample is not None and SYSTEM_HAS_GPS == True:
            gps_sample_valid = True
            vehicle_front = gps_tools.project_point(sample, sample.azimuth_degrees, 2.0)
            vehicle_rear = gps_tools.project_point(sample, sample.azimuth_degrees, -2.0)

            time5 = time.time() - debug_time
            (projected_path_tangent_point, closest_path_point,
             absolute_path_distance, drive_reverse, calculated_rotation, calculated_strafe) = steering_calc(self)


            # front_raw = gps_tools.project_point(self.gps.raw_samples[0], sample.azimuth_degrees, 0)
            # rear_raw = gps_tools.project_point(self.gps.raw_samples[1], sample.azimuth_degrees, 0)



            # These extra points can be displayed in the web UI.
            debug_points = (vehicle_front, vehicle_rear, projected_path_tangent_point, closest_path_point)
            # debug_points = (vehicle_front, vehicle_rear, front_raw, rear_raw)


        time6 = time.time() - debug_time

        if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0:
            self.gps.print_sample_history_stats()


        self.joy.update_value()
        if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0:
            if not self.joy.initial_value_has_changed:
                self.logger.info("Joystick Not Yet Detected")
            elif not self.joy.safety_check_passed_switches_up:
                self.logger.info("Awaiting Joystick Safety Checks")
        #self.logger.info(f"Initial change: {self.joy.initial_value_has_changed},  Check 1: {self.joy.safety_check_passed_switches_down}, Check 2: {self.joy.safety_check_passed_switches_up}")
        if self.joy.activated():
            self.logger.info("DISABLED AUTONOMY becase joystick is activated: {}".format(self.joy))
            self.autonomy_hold = True
            self.activate_autonomy = False
            self.control_state = model.CONTROL_OVERRIDE
        if self.joy.autonomy_allowed == False:
            self.joy.autonomy_requested = False
            self.autonomy_hold = True
            self.activate_autonomy = False
            self.control_state = model.CONTROL_OVERRIDE
        elif self.joy.autonomy_requested == True and self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0:
            self.gps.flush_serial()
            self.autonomy_hold = False
            self.activate_autonomy = True
            self.joy.autonomy_requested = False
            self.disengagement_time = time.time() - _DISENGAGEMENT_RETRY_DELAY_SEC
            self.logger.info("Joystick requested autonomy.")

        # self.joy.autonomy_requested = False
        # self.autonomy_hold = True
        # self.activate_autonomy = False

        if gps_sample_valid:
            if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0 and _DEBUG_STEERING:
                self.logger.warn("{} {} {}".format(calculated_rotation, calculated_strafe, drive_reverse))

            (user_web_page_plot_steer_cmd, user_web_page_plot_strafe_cmd,
             strafe_d, steer_d, strafe_p, steer_p,
             autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd
             ) = self.calc_commands_for_autonomy(calculated_rotation, calculated_strafe, drive_reverse)
        else:
            autonomy_vel_cmd = 0.0
            autonomy_steer_cmd = 0.0
            autonomy_strafe_cmd = 0.0
            strafe_d = 0.0
            steer_d = 0.0
            strafe_p = 0.0
            steer_p = 0.0
            user_web_page_plot_steer_cmd = 0.0
            user_web_page_plot_strafe_cmd = 0.0

        time7 = time.time() - debug_time

        error_messages, fatal_error, zero_output = self.safety_checks(absolute_path_distance, gps_path_angle_error)

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
            self.control_state = model.CONTROL_ONLINE

        time8 = time.time() - debug_time

        # Activate a brief pause at the end of a track.
        if time.time() - self.reset_steering_timer < 0.5:
            zero_output = True

        # Activate a brief pause at the end of a track.
        if time.time() - self.load_path_time < _PATH_END_PAUSE_SEC and not zero_output:
            zero_output = True
            self.control_state = model.CONTROL_AUTONOMY_PAUSE
            # Don't use the motion timer here as it reactivates alarm.
            if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0:
                self.logger.info("Taking a short delay at the end of the path so zeroing out autonomy commands.")

        # If a non-fatal error disengaged autonomy, wait
        # _DISENGAGEMENT_RETRY_DELAY_SEC and then try to re-enable autonomy.
        if time.time() - self.disengagement_time < _DISENGAGEMENT_RETRY_DELAY_SEC:
            zero_output = True
            self.resume_motion_timer = time.time()
            if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0:
                self.logger.info(
                    "Disengaged for {:.1f} more seconds so zeroing out autonomy commands."
                    .format(_DISENGAGEMENT_RETRY_DELAY_SEC - (time.time() - self.disengagement_time)))

        # When autonomy is enabled, hold motion for a few seconds while sounding
        # the motion alarm as a warning to bystanders.
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

        """
         _____ ___  ____   ___
        |_   _/ _ \|  _ \ / _ \
          | || | | | | | | | | |
          | || |_| | |_| | |_| |
          |_| \___/|____/ \___/
        """
        # TODO: Flush GPS serial once when autonomy has been activated!

        if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0 and _DEBUG_STEERING and gps_sample_valid:
            self.logger.warn("autonomy_vel_cmd = {}, autonomy_steer_cmd = {}, autonomy_strafe_cmd = {}".format(
                    autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd))

        # Determine final drive commands for autonomy or joystick.
        if self.motor_state == model.MOTOR_ENABLED:
            vel_cmd, steer_cmd, strafe_cmd = self.calc_drive_commands(
                autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output)
        else:
            vel_cmd = 0
            steer_cmd = 0
            strafe_cmd = 0

        # Slow Vel down by 50% when steering is at max.
        # TODO: Looks like I altered this to 100%. Make reduction a variable.
        self.vel_cmd = vel_cmd * 1.0 / (1.0 + abs(steer_cmd))

        # Fixed factor reduction TODO: eliminate by reducing vel where assigned
        self.vel_cmd = self.vel_cmd * 0.6

        # Perform acceleration on self.vel_cmd value.
        profiled_vel = get_profiled_velocity(self.last_vel_cmd, self.vel_cmd,
                                             self.period)
        self.logger.debug("self.last_vel_cmd {}, self.vel_cmd {}, profiled_vel {}".format(
            self.last_vel_cmd, self.vel_cmd, profiled_vel))
        self.vel_cmd = profiled_vel
        self.last_vel_cmd = self.vel_cmd

        self.logger.debug("Final values: Steer {}, Vel {}, Strafe {}".format(
            steer_cmd, self.vel_cmd, strafe_cmd))

        calc = calculate_steering(steer_cmd, self.vel_cmd, strafe_cmd)

        if self.vel_cmd < 0:
            # find steering angles that are closest to present steering angles
            calc = recalculate_steering_values(calc, self.last_calculated_steering)

        # steering_is_zero = True
        # for key in calc.keys():
        #     if abs(calc[key][0]) > 0.01:
        #         steering_is_zero = False

        # if abs(steer_cmd) > 0:
        #     f = open(self.debug_file_name, "a")
        #     f.write(f"{calc}\n")
        #     f.close()

        steering_debug = (calc, steer_cmd, self.vel_cmd, strafe_cmd)

        # Update main process on latest calculations.
        time8b = time.time() - debug_time

        # TODO: Note this scheme reduces pickling time but is not guranteed
        # to be read by main within 1 sec, so consider something better.
        if time.time() - self.load_path_time < 1.0:
            points_to_send = self.nav_path.points
        else:
            points_to_send = []


        send_data = (self.gps.last_sample(), points_to_send,
                     self.next_point_heading, debug_points,
                     self.control_state,
                     model.MOTOR_STATE_STRINGS[self.motor_state],
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
                     self.temperatures,
                     steering_debug)
        # time8c = time.time()
        with self.remote_to_main_lock:
            self.remote_to_main_string["value"] = pickle.dumps(send_data)
        # self.logger.warn("dumptime {}".format(time.time()-time8c))

        self.last_energy_segment = None
        time9 = time.time() - debug_time

        if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0 and _DEBUG_STEERING:
            self.logger.warn(
                    "vel_cmd = {}, steer_cmd = {}, strafe_cmd = {}".format(
                    self.vel_cmd, steer_cmd, strafe_cmd))
            self.logger.warn("{}".format(calc))

        if self.joy.activated() and self.autonomy_hold:
            steering_okay, steering_error_string = True, ""
        else:
            steering_okay, steering_error_string = compare_steering_values(
                self.last_calculated_steering, calc)
        if not steering_okay:
            self.logger.error("{}".format(steering_error_string))
            self.logger.error("Final values: Steer {}, Vel {}, Strafe {}".format(
                steer_cmd, self.vel_cmd, strafe_cmd))
            self.logger.error("old 4ws values: Steer {}".format(
                self.last_calculated_steering))
            self.logger.error("new 4ws values: Steer {}".format(calc))

        self.last_calculated_steering = calc

        # TODO: This hack temporarily disables autonomy via remote for debugging.
        # if not self.joy.activated() and not self.joy.strafe_allowed and not self.simulated_hardware:
        #     calc = calculate_steering(0, 0, 0)
        #     self.last_calculated_steering = calc

        # If the robot is simulated, estimate movement odometry.
        if self.simulated_hardware:
            if abs(steer_cmd) > 0.001 or abs(self.vel_cmd) > 0.001 or abs(strafe_cmd) > 0.001:
                self.simulate_next_gps_sample(steer_cmd, strafe_cmd)

        self.logger.debug("Communicate to motors")
        self.logger.debug(calc)
        time10 = self.communicate_to_motors_sharedmem(calc, debug_time)

        # self.logger.warn("time: {} {}".format(self.period, time.time() - debug_time))

        if self.loop_count == 200 and _RUN_PROFILER:
            prof.disable()
            prof.print_stats()
            prof.dump_stats("remote_control_loop.prof")
            import sys
            self.stop_signal.set()
            sys.exit()

        self.logger.debug(
            "Took {} sec to get here. 1:{} 2:{} 3:{} 4:{} 5:{} 6:{} 7:{} 8:{} 8b:{} 9:{} 10:{} {}"
            .format(time.time() - debug_time, time1, time2, time3,
                    time4, time5, time6, time7, time8, time8b, time9,
                    time10, self.robot_object.wifi_ap_name))

    """
    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
     _____           _   __  __       _         _
    | ____|_ __   __| | |  \/  | __ _(_)_ __   | |    ___   ___  _ __
    |  _| | '_ \ / _` | | |\/| |/ _` | | '_ \  | |   / _ \ / _ \| '_ \
    | |___| | | | (_| | | |  | | (_| | | | | | | |__| (_) | (_) | |_) |
    |_____|_| |_|\__,_| |_|  |_|\__,_|_|_| |_| |_____\___/ \___/| .__/
                                                                |_|
    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    """

    # TODO: in refactoring this method was created, but name is too similar
    # to load_path. Figure out what this is doing and whether it is needed,
    # and if needed give it a more descriptive name. It may be that load_path
    # has been expanded enough to make some of this redundant?
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


    def calc_closest_path_point_and_heading(self):
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
            closest_u = self.nav_path.spline.closestUOnSplinePointsNearU(self.gps.last_sample(), self.last_closest_path_u)
            self.last_closest_path_u = closest_u
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

            steer_p = calculated_rotation * self.nav_path.control_values.angular_p
            strafe_p = calculated_strafe * self.nav_path.control_values.lateral_p
            steer_d = self.gps_path_angular_error_rate() * self.nav_path.control_values.angular_d
            strafe_d = self.gps_path_lateral_error_rate() * self.nav_path.control_values.lateral_d * -1

            self.logger.debug("strafe_p {}, strafe_d {}, steer_p {} steer_d {}".format(
                strafe_p, strafe_d, strafe_d, steer_d))

            steer_command_value = steer_p + steer_d
            strafe_command_value = strafe_p + strafe_d

            _STEER_LIMIT = 45
            self.logger.debug("steer_command_value {}".format(steer_command_value))
            unfiltered_steer_cmd = clamp(steer_command_value, -_STEER_LIMIT, _STEER_LIMIT)
            self.logger.debug("unfiltered_steer_cmd {}".format(unfiltered_steer_cmd))
            unfiltered_steer_cmd /= 45.0
            _STRAFE_LIMIT = 0.25
            unfiltered_strafe_cmd = clamp(strafe_command_value, -_STRAFE_LIMIT, _STRAFE_LIMIT)
            unfiltered_strafe_cmd *= strafe_multiplier

            # Rate of change clamping
            steer_rate = 1.0
            strafe_rate = 4.0
            self.logger.debug("unfiltered_steer_cmd {}".format(unfiltered_steer_cmd))
            self.logger.debug("last_autonomy_steer_cmd {}".format(self.last_autonomy_steer_cmd))
            unfiltered_steer_cmd *= self.driving_direction
            autonomy_steer_cmd = clamp(unfiltered_steer_cmd,
                                       self.last_autonomy_steer_cmd - steer_rate,
                                       self.last_autonomy_steer_cmd + steer_rate)

            self.logger.debug("autonomy_steer_cmd {}".format(autonomy_steer_cmd))
            autonomy_strafe_cmd = clamp(unfiltered_strafe_cmd,
                                        self.last_autonomy_strafe_cmd - strafe_rate,
                                        self.last_autonomy_strafe_cmd + strafe_rate)


            if abs(unfiltered_steer_cmd) > 0.5:
                # Maxed out steer and strafe can result in strafe only
                # due to steering limits, so reduce strafe with maxed
                # steering. using unfiltered_steer_cmd which will peg
                # immediately
                strafe_scalar = 0.5 - (abs(unfiltered_steer_cmd)-0.5)
                strafe_scalar = clamp(strafe_scalar, 0.2, 1.0)
                autonomy_strafe_cmd *= strafe_scalar

            self.last_autonomy_steer_cmd = autonomy_steer_cmd
            self.last_autonomy_strafe_cmd = autonomy_strafe_cmd
            user_web_page_plot_steer_cmd = autonomy_steer_cmd * self.driving_direction
            user_web_page_plot_strafe_cmd = autonomy_strafe_cmd * self.driving_direction

            if 0.0 <= self.nav_path.navigation_parameters.travel_speed <= self.maximum_velocity:
                AUTONOMY_FACTOR = 3.0
                autonomy_vel_cmd = (AUTONOMY_FACTOR * self.nav_path.navigation_parameters.travel_speed
                                    * self.driving_direction * drive_reverse)
                self.logger.debug("Travel speed: {}".format(AUTONOMY_FACTOR * self.nav_path.navigation_parameters.travel_speed))
            else:
                self.logger.error("Invalid travel speed specified! Got {}. Maximum allowed is {}"
                                  .format(self.nav_path.navigation_parameters.travel_speed, self.maximum_velocity))
                autonomy_vel_cmd = 0.0
            self.logger.debug(
                "self.autonomy_velocity {}, self.driving_direction {}, self.nav_direction {}, drive_reverse {} , autonomy_vel_cmd {}"
                .format(self.autonomy_velocity, self.driving_direction, self.nav_direction,
                        drive_reverse, autonomy_vel_cmd))

            self.joy.clear_steer()  # ensures that vel goes to zero when autonomy disabled
            logger_string = (f"steer_cmd: {autonomy_steer_cmd:.2f}, "
                             f"strafe_cmd: {autonomy_strafe_cmd:.2f}, "
                             f"vel_cmd: {self.vel_cmd:.2f}, "
                             f"calculated_rotation: {calculated_rotation:.2f}, "
                             f"calculated_strafe: {calculated_strafe:.2f}")
            if self.loop_count % _DEFAULT_LOGGING_SKIP_RATE == 0:
                self.logger.info(logger_string)
            else:
                self.logger.debug(logger_string)
        return (user_web_page_plot_steer_cmd, user_web_page_plot_strafe_cmd,
                strafe_d, steer_d, strafe_p, steer_p,
                autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd)

    def safety_checks(self, absolute_path_distance, gps_path_angle_error):
        # Begin Safety Checks
        error_messages = []
        fatal_error = False
        zero_output = False
        try:
            if self.gps.last_sample() is not None:
                if isinstance(self.gps.last_sample().time_stamp, datetime.datetime):
                    local_timezone = datetime.datetime.now().astimezone().tzinfo
                    solution_age = datetime.datetime.now(local_timezone) - self.gps.last_sample().time_stamp
                    solution_age = solution_age.total_seconds()
                else:
                    solution_age = time.time() - self.gps.last_sample().time_stamp
            else:
                solution_age = _ALLOWED_SOLUTION_AGE_SEC + 9999999
            # self.logger.warning(solution_age)

        except Exception as e:
            raise e
            solution_age = _ALLOWED_SOLUTION_AGE_SEC + 9999999

        # TODO: We should not let old data build up. Examine buffer lengths to
        # determine when we are behind by one sample and automatically catch up.
        # Do this in the GPS module.
        if self.gps.last_sample()!= None and self.gps.last_sample().rtk_age > _ALLOWED_RTK_AGE_SEC/2.0:
            self.gps.flush_serial()
            self.logger.warning("RTK age approaching limit. Flushed serial to mitigate.")


        if self.motor_state != model.MOTOR_ENABLED:
            error_messages.append("Motors not enabled so zeroing out autonomy commands.")
            zero_output = True
            self.control_state = model.CONTROL_MOTOR_ERROR
            self.resume_motion_timer = time.time()

        if self.voltage_average < _VOLTAGE_CUTOFF:
            fatal_error = True
            error_messages.append("Voltage low so zeroing out autonomy commands.")
            zero_output = True
            self.control_state = model.CONTROL_LOW_VOLTAGE
            self.resume_motion_timer = time.time()

        if not self.gps.is_dual_fix():
            error_messages.append("No GPS fix so zeroing out autonomy commands.")
            # try:
            #     print(self.gps.last_sample().status)
            # except:
            #     pass
            zero_output = True
            self.control_state = model.CONTROL_GPS_STARTUP
            self.resume_motion_timer = time.time()
        elif abs(absolute_path_distance) > self.nav_path.maximum_allowed_distance_meters:
            # Distance from path exceeds allowed limit.
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = model.CONTROL_AUTONOMY_ERROR_DISTANCE
            error_messages.append(
                "GPS distance {:.1f} meters too far from path so zeroing out autonomy commands."
                .format(abs(absolute_path_distance)))
        elif abs(gps_path_angle_error) > self.nav_path.maximum_allowed_angle_error_degrees:
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = model.CONTROL_AUTONOMY_ERROR_ANGLE
            error_messages.append(
                "GPS path angle {} exceeds allowed limit {} so zeroing out autonomy commands."
                .format(
                    abs(gps_path_angle_error),
                    self.nav_path.maximum_allowed_angle_error_degrees))
        elif self.gps.last_sample().rtk_age > _ALLOWED_RTK_AGE_SEC:
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = model.CONTROL_AUTONOMY_ERROR_RTK_AGE
            error_messages.append("RTK base station data too old so zeroing out autonomy commands.")
        elif solution_age > _ALLOWED_SOLUTION_AGE_SEC:
            zero_output = True
            self.resume_motion_timer = time.time()
            self.control_state = model.CONTROL_AUTONOMY_ERROR_SOLUTION_AGE
            self.logger.error(solution_age)
            self.logger.error(self.gps.last_sample().time_stamp)
            self.logger.error(self.gps.last_sample())
            self.gps.flush_serial()
            error_messages.append("RTK solution too old so zeroing out autonomy commands.")
        # self.logger.error(f"solution_age: {solution_age}")

        if time.time() - self.robot_object.last_server_communication_stamp > SERVER_COMMUNICATION_DELAY_LIMIT_SEC and IGNORE_SERVER_COMMUNICATIONS_ERRORS == False:
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
                if len(self.nav_path.points) > 2:
                    # Limit strafe at the start of normal paths to quell a strafe impulse.
                    autonomy_strafe_cmd *= autonomy_time_elapsed / _BEGIN_AUTONOMY_SPEED_RAMP_SEC
            if not zero_output:
                self.control_state = model.CONTROL_AUTONOMY
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
        steering_multiplier = 0.2
        vel_multiplier = 0.2
        strafe_multiplier = 0.2
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
        rand_dist = random.uniform(-0.02, 0.02)
        rand_angle = random.uniform(0, 360.0)
        next_point = gps_tools.project_point(next_point, rand_angle, rand_dist)
        new_heading_degrees += random.uniform(-3.0, 3.0)
        self.gps.update_simulated_sample(next_point.lat, next_point.lon, new_heading_degrees)


    def communicate_to_motors_sharedmem(self, calc, debug_time):
        if calc is not None:
            new_calc = []
            for key in model.CORNER_NAMES.keys():
                new_calc.append(calc[key])
            new_calc.append([model.FRESH_MESSAGE,0])
            new_calc = np.array(new_calc)
            tick_count = 0
            while self.motor_output[0][1] != model.CLEAR_TO_WRITE:
                time.sleep(model.MOTOR_READ_DELAY_SECONDS)
                tick_count += 1
                if tick_count > 2000:
                    self.logger.error("Excessive wait time on communicate_to_motors_sharedmem wait ready loop.")
                    return
            self.motor_input[:] = new_calc[:]
            motor_message = self.motor_output[:]
            # print("#############")
            # print("#############")
            # print("#############")
            # print("#############")
            # print(motor_message)
            checksum = 0
            for section in motor_message:
                for value in section:
                    checksum+=value
            # print(checksum)
            if checksum == self.motor_comms_last_checksum:
                if time.time() - self.last_motor_comms_checksum_change > 60:
                    self.setup_motor_shared_memory()
                    self.last_motor_comms_checksum_change = time.time()
                    print("#############")
                    print("#############")
                    print("#############")
                    print("CHECKSUM UNCHANGING!")
                    print("#############")
                    print("#############")
                    print("#############")
            else:
                self.last_motor_comms_checksum_change = time.time()
            self.motor_comms_last_checksum = checksum
            # print("#############")
            # print(checksum)
            # print("#############")
            # print("#############")
            # print("#############")
            # print("#############")
            # print("#############")
            self.motor_state = int(motor_message[0][0])
            self.motor_encoders = motor_message[-4:]
            # print(self.motor_encoders[0][-2:])
            if motor_message[1][0] > 0:
                self.voltages = motor_message[1]
                self.bus_currents = motor_message[2]
                self.total_watts = 0
                if len(self.voltages) > 0:
                    self.voltage_average = sum(self.voltages) / len(self.voltages)
                    self.display.display_voltage(self.voltage_average)
                self.wattages = []
                for volt, current in zip(self.voltages,
                                         self.bus_currents):
                    self.total_watts += volt * current
                    self.wattages.append(volt * current)
                # self.logger.info(self.wattages)
                self.logger.debug("Drawing {} Watts.".format(int(self.total_watts)))

            if motor_message[3][0] > 0:
                self.temperatures  = motor_message[3]

        time10 = time.time() - debug_time
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
                    # print(type(sample_duration))
                    if isinstance(sample_duration, datetime.timedelta):
                        sample_duration = sample_duration.total_seconds()
                    # print(type(sample_duration))
                    # print("%%%%%%%%%%%%")
                    # print("%%%%%%%%%%%%")
                    # print("%%%%%%%%%%%%")
                    # import sys
                    # sys.exit()
                    sample_avg_watts = (sample1[1] + sample2[1]) / 2.0
                    if sample_num > sample_collector_index:
                        list_subsamples.append(sample1[0])
                        if len(self.power_consumption_list) > _NUM_GPS_SUBSAMPLES:
                            sample_collector_index += int(len(self.power_consumption_list) / _NUM_GPS_SUBSAMPLES)
                        else:
                            sample_collector_index += 1
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


def run_control(stop_signal, remote_to_main_lock,
                main_to_remote_lock,
                remote_to_main_string,
                main_to_remote_string,
                logging,
                logging_details,
                simulated_hardware=False):
    remote_control = RemoteControl(stop_signal, remote_to_main_lock, main_to_remote_lock,
                                   remote_to_main_string,
                                   main_to_remote_string, logging,
                                   logging_details, simulated_hardware)
    remote_control.run_setup()
    remote_control.run_loop()


if __name__ == "__main__":
    run_control()
