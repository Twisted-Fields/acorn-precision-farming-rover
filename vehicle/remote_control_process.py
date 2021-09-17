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
import serial
import time
import sys
import math
from odrive.utils import dump_errors
from evdev import InputDevice, list_devices, categorize, ecodes, KeyEvent
from steering import calculate_steering
import zmq
import pickle
import gps_tools
from collections import namedtuple
import numpy as np
import spline_lib
import os
import datetime
from motors import _STATE_ENABLED, STATE_DISCONNECTED
import rtk_process
import coloredlogs
import subprocess
from enum import Enum

# This file gets imported by server but we should only import GPIO on raspi.
if "arm" in os.uname().machine:
    import board
    import busio
    import digitalio
    from adafruit_mcp230xx.mcp23017 import MCP23017

COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

ACCELERATION_COUNTS_SEC = 0.5

_RESUME_MOTION_WARNING_TIME_SEC = 4
#_RESUME_MOTION_WARNING_TIME_SEC = -1

_SEC_IN_ONE_MINUTE = 60

_MAXIMUM_ALLOWED_DISTANCE_METERS = 3.5
_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES = 120 #20
_VOLTAGE_CUTOFF = 30
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

_ERROR_RATE_AVERAGING_COUNT = 4

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

_JOYSTICK_MIN = 0.02


def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel-last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel-last_vel
    else:
        increment = math.copysign(ACCELERATION_COUNTS_SEC, unfiltered_vel-last_vel) * period_s
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
    def __init__(self, travel_speed, path_following_direction, vehicle_travel_direction, loop_path):
        self.travel_speed = travel_speed
        self.path_following_direction = path_following_direction
        self.vehicle_travel_direction = vehicle_travel_direction
        self.loop_path = loop_path

class PathSection():
    def __init__(self, points, control_values, navigation_parameters, max_dist=0,
                 max_angle=0, end_dist=0, end_angle=0):
        self.points = points
        self.maximum_allowed_distance_meters = max_dist
        self.maximum_allowed_angle_error_degrees = max_angle
        self.control_values = control_values
        self.end_distance_m = end_dist
        self.end_angle_degrees = end_angle
        self.navigation_parameters = navigation_parameters

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
        self.watt_seconds_per_meter = total_watt_seconds/distance_sum
        self.height_change = end_gps.height_m - start_gps.height_m
        self.avg_watts = avg_watts
        self.per_motor_total_watt_seconds = per_motor_total_watt_seconds
        self.per_motor_watt_average = per_motor_watt_average
        self.subsampled_points = subsampled_points
        self.autonomy_operating = autonomy_operating
        self.wifi_ap_name = wifi_ap_name
        self.wifi_signal_strength = wifi_signal_strength


class RemoteControl():

    def __init__(self, remote_to_main_lock, main_to_remote_lock, remote_to_main_string, main_to_remote_string, logging, logging_details, simulated_hardware=False):
        self.joy = None
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
            mcp = MCP23017(i2c)#, address=0x20)  # MCP23017
            self.alarm1 = mcp.get_pin(0)
            self.alarm2 = mcp.get_pin(1)
            self.alarm3 = mcp.get_pin(2)

            self.alarm1.switch_to_output(value=False)
            self.alarm2.switch_to_output(value=False)
            self.alarm3.switch_to_output(value=False)
        self.connect_to_motors()
        self.connect_joystick()

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

    def get_joystick_values(self, st_old, th_old, stf_old):
        steer = None
        throttle = None
        strafe = None
        count = 0
        while True:
            event = None
            try:
                event = self.joy.read_one()
            except Exception as e:
                self.logger.error("Joystick read exception: {}".format(e))
            if event == None:
                break
            if event and event.type == ecodes.EV_ABS:
                absevent = categorize(event)
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RX':
                    steer = absevent.event.value / 32768.0
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Y':
                    throttle = -absevent.event.value / 32768.0

                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_X':
                    strafe = absevent.event.value / 32768.0
            count += 1

        if not steer:
            steer = st_old
        if not throttle:
            throttle = th_old
        if not strafe:
            strafe = stf_old

        return steer, throttle, strafe

    def connect_joystick(self):
        devices = [InputDevice(fn) for fn in list_devices()]
        for dev in devices:
            if "Microsoft" in dev.name:
                self.joy = dev
                return
            if "Logitech" in dev.name:
                self.joy = dev
                return

    def run_loop(self):
        joy_steer = 0
        joy_throttle = 0
        joy_strafe = 0
        vel_cmd = 0
        last_vel_cmd = 0
        tick_time = time.time()

        #self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
        #self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)
        self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.EITHER, vehicle_travel_direction=Direction.EITHER, loop_path=True)

        #self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
        #self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)

        self.default_path_control_vals = PathControlValues(angular_p=0.9, lateral_p=-0.25, angular_d=0.3, lateral_d=-0.05)
        self.nav_path = PathSection(points=[],
                                    control_values=self.default_path_control_vals,
                                    navigation_parameters=self.default_navigation_parameters,
                                    max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                                    max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                                    end_dist=0,
                                    end_angle=0)
        self.gps_path = []
        load_path_time = time.time()
        auto_throttle = 0
        self.loaded_path_name = ""
        self.autonomy_hold = True
        self.nav_spline = None
        self.control_state = CONTROL_STARTUP
        self.motor_state = STATE_DISCONNECTED
        self.gps_path_lateral_error = 0
        self.gps_path_lateral_error_rate = 0
        self.gps_path_angular_error = 0
        self.gps_path_angular_error_rate = 0
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
        self.simulated_sample = gps_tools.GpsSample(37.353039233, -122.333725682, 100, ("fix","fix"), 20, 0, time.time(), 0.5)

        autonomy_vel_cmd = 0
        last_wifi_restart_time = 0
        if self.simulated_hardware:
            rtk_socket1 = None
            rtk_socket2 = None
        else:
            rtk_process.launch_rtk_sub_procs(self.logger)
            rtk_socket1, rtk_socket2 = rtk_process.connect_rtk_procs(self.logger)

        self.latest_gps_sample = None
        self.last_good_gps_sample = None
        self.gps_buffers = ["",""]
        debug_time = time.time()
        try:
            loop_count = -1
            while True:
                loop_count += 1

                # Get real or simulated GPS data.
                if self.simulated_hardware:
                    if loop_count % GPS_PRINT_INTERVAL == 0:
                        self.logger.info("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fixes: ({}, {}), Period: {:.2f}".format(self.simulated_sample.lat, self.simulated_sample.lon, self.simulated_sample.azimuth_degrees, 2.8, True, True, 0.100))
                    self.latest_gps_sample = gps_tools.GpsSample(self.simulated_sample.lat, self.simulated_sample.lon, self.simulated_sample.height_m, ("fix","fix"), 20, self.simulated_sample.azimuth_degrees, time.time(), 0.5)
                else:
                    self.gps_buffers, self.latest_gps_sample = (
                        rtk_process.rtk_loop_once(rtk_socket1, rtk_socket2,
                        buffers=self.gps_buffers,
                        print_gps=loop_count % GPS_PRINT_INTERVAL == 0,
                        last_sample=self.latest_gps_sample,
                        retries=_GPS_ERROR_RETRIES,
                        logger=self.logger))
                debug_time = time.time()
                if self.latest_gps_sample is not None:
                    self.last_good_gps_sample = self.latest_gps_sample

                try:
                    # Read robot object from shared memory.
                    # Object is sent by main process.
                    recieved_robot_object = None
                    time1 = time.time() - debug_time
                    with self.main_to_remote_lock:
                        recieved_robot_object = pickle.loads(self.main_to_remote_string["value"])
                    time2 = time.time() - debug_time
                except Exception as e:
                    self.logger.error("Exception reading remote string.")
                    raise(e)
                if recieved_robot_object:
                    if str(type(recieved_robot_object))=="<class '__main__.Robot'>":
                        self.robot_object = recieved_robot_object
                        self.logger.debug("Remote received new robot object.")
                        time3 = time.time() - debug_time
                        if len(self.nav_path.points) == 0 or self.loaded_path_name != self.robot_object.loaded_path_name:
                            if len(self.robot_object.loaded_path) > 0  and self.latest_gps_sample is not None:
                                self.logger.debug(self.robot_object.loaded_path)
                                self.nav_spline = spline_lib.GpsSpline(self.robot_object.loaded_path, smooth_factor=10, num_points=1000)

                                reversed_path = self.robot_object.loaded_path
                                reversed_path.reverse()
                                self.nav_spline_reverse = spline_lib.GpsSpline(reversed_path, smooth_factor=10, num_points=1000)

                                self.nav_path = PathSection(points=self.nav_spline.points, control_values=self.default_path_control_vals, navigation_parameters=self.default_navigation_parameters, max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS, max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES, end_dist=1.0, end_angle=45)
                                load_path_time = time.time()
                                self.loaded_path_name = self.robot_object.loaded_path_name
                                if self.simulated_hardware:
                                    # Place simulated robot at start of path.
                                    start_index = int(len(self.nav_spline.points)/2) - 5
                                    initial_heading = gps_tools.get_heading(self.nav_spline.points[start_index], self.nav_spline.points[start_index+1])
                                    self.simulated_sample = gps_tools.GpsSample(self.nav_spline.points[start_index].lat, self.nav_spline.points[start_index].lon, self.simulated_sample.height_m, ("fix","fix"), 20, initial_heading + 30, time.time(), 0.5)

                                # Set initial nav_direction.
                                if self.nav_path.navigation_parameters.path_following_direction == Direction.EITHER:
                                    dist_start = gps_tools.get_distance(self.latest_gps_sample, self.nav_path.points[0])
                                    dist_end = gps_tools.get_distance(self.latest_gps_sample, self.nav_path.points[len(self.nav_path.points)-1])
                                    # This may be overridden farther down.
                                    if dist_start < dist_end:
                                        self.nav_direction = 1
                                    else:
                                        self.nav_direction = -1
                                elif self.nav_path.navigation_parameters.path_following_direction == Direction.FORWARD:
                                    self.nav_direction = 1
                                elif self.nav_path.navigation_parameters.path_following_direction == Direction.BACKWARD:
                                    self.nav_direction = -1

                        self.activate_autonomy = self.robot_object.activate_autonomy
                        self.autonomy_velocity = self.robot_object.autonomy_velocity
                        # Autonomy is disabled until robot is ready or if joystick is used.
                        if self.autonomy_hold:
                            self.activate_autonomy = False
                        # Reset disabled autonomy if autonomy is turned off in the command.
                        if self.robot_object.clear_autonomy_hold:
                            self.autonomy_hold = False
                            # Reset disengagement timer.
                            self.disengagement_time  = time.time() - _DISENGAGEMENT_RETRY_DELAY_SEC

                if self.robot_object==None:
                    self.logger.info("Waiting for valid robot object before running remote control code.")
                    time.sleep(_SLOW_POLLING_SLEEP_S)
                    continue


                time4 = time.time() - debug_time

            #    print("begin robot calc")

                debug_points = (None, None, None, None)
                calculated_rotation = None
                calculated_strafe = None
                gps_lateral_distance_error = 0
                gps_path_angle_error = 0
                absolute_path_distance = math.inf
                time5 = 0
                if self.robot_object and self.latest_gps_sample is not None:

                    vehicle_front = gps_tools.project_point(self.latest_gps_sample, self.latest_gps_sample.azimuth_degrees, 1.0)
                    vehicle_rear = gps_tools.project_point(self.latest_gps_sample, self.latest_gps_sample.azimuth_degrees, -1.0)

                    # if time.time() - self.latest_gps_sample.time_stamp > _ALLOWED_SOLUTION_AGE_SEC:
                    #     self.logger.error("SOLUTION AGE {} NOT OKAY AND LATEST GPS SAMPLE IS: {}".format(time.time() - self.latest_gps_sample.time_stamp, self.latest_gps_sample))
                    #     self.logger.error("Took {} sec to get here. {} {} {} {}".format(time.time()-debug_time, time1, time2, time3, time4))

                    time5 = time.time() - debug_time


                    """
                    Begin steering calculation.
                    TODO: Break this out to a separate module.
                    """

                    projected_path_tangent_point = gps_tools.GpsPoint(0, 0)
                    closest_path_point = gps_tools.GpsPoint(0, 0)
                    if(len(self.nav_path.points)>0):

                        """
                        REFACTOR END CONDITIONS
                        """
                        # Check if we meet the end condition.
                        if gps_tools.get_distance(self.latest_gps_sample, self.nav_path.points[0]) < self.nav_path.end_distance_m:
                            if self.nav_direction == -1:
                                self.nav_path.points = []
                        elif gps_tools.get_distance(self.latest_gps_sample, self.nav_path.points[-1]) < self.nav_path.end_distance_m:
                            if self.nav_direction == 1:
                                self.nav_path.points = []


                        closest_u = self.nav_spline.closestUOnSpline(self.latest_gps_sample)
                        closest_path_point = self.nav_spline.coordAtU(closest_u)
                        absolute_path_distance = gps_tools.get_distance(self.latest_gps_sample, closest_path_point)
                        # Heading specified at this point on the path.
                        path_point_heading = math.degrees(self.nav_spline.slopeRadiansAtU(closest_u))

                        # closest_u_r = self.nav_spline_reverse.closestUOnSpline(self.latest_gps_sample)
                        # closest_path_point_r = self.nav_spline_reverse.coordAtU(closest_u_r)
                        # # Heading specified at this point on the path.
                        # path_point_heading_r = math.degrees(self.nav_spline_reverse.slopeRadiansAtU(closest_u_r))
                        #
                        # self.logger.info("%%%%%%%%%%%%%%%%%%")
                        #
                        # self.logger.info("{}, {}".format(path_point_heading_r, path_point_heading))
                        # self.logger.info("%%%%%%%%%%%%%%%%%%")





                        projected_path_tangent_point = gps_tools.project_point(closest_path_point, path_point_heading, 3.0)
                        gps_lateral_distance_error = gps_tools.get_approx_distance_point_from_line(self.latest_gps_sample, closest_path_point, projected_path_tangent_point)
                        self.logger.debug("DISTANCE: {}".format(abs(gps_lateral_distance_error)))

                        self.logger.debug("robot heading {}, path heading {}".format(self.latest_gps_sample.azimuth_degrees, path_point_heading))
                        calculated_rotation = path_point_heading - self.latest_gps_sample.azimuth_degrees
                        calculated_strafe = gps_lateral_distance_error

                        self.logger.info("calculated_rotation: {}".format(calculated_rotation))

                        # Truncate values to between 0 and 360
                        calculated_rotation %= 360

                        # Set value to +/- 180
                        if calculated_rotation > 180:
                            calculated_rotation -= 360

                        self.logger.info("calculated_rotation: {}".format(calculated_rotation))



                        drive_solution_okay = True
                        if self.nav_path.navigation_parameters.vehicle_travel_direction == Direction.EITHER:
                            self.driving_direction = 1
                            if abs(calculated_rotation) > 90:
                                calculated_rotation -= math.copysign(180, calculated_rotation)
                                self.driving_direction = -1
                            if self.nav_direction == -1:
                                self.driving_direction *= -1
                                calculated_strafe *= -1
                        elif self.nav_path.navigation_parameters.vehicle_travel_direction == Direction.FORWARD:
                            self.driving_direction = 1
                            if abs(calculated_rotation) > 90:
                                if self.nav_path.navigation_parameters.path_following_direction in (Direction.EITHER, Direction.BACKWARD):
                                    calculated_rotation -= math.copysign(180, calculated_rotation)
                                    calculated_strafe *= -1
                                    #calculated_rotation *= -1
                                    self.nav_direction = -1
                                if self.nav_path.navigation_parameters.path_following_direction == Direction.FORWARD:
                                    drive_solution_okay = False
                            elif self.nav_path.navigation_parameters.path_following_direction == Direction.BACKWARD:
                                drive_solution_okay = False
                        elif self.nav_path.navigation_parameters.vehicle_travel_direction == Direction.BACKWARD:
                            self.driving_direction = -1
                            if abs(calculated_rotation) > 90:
                                if self.nav_path.navigation_parameters.path_following_direction in (Direction.EITHER, Direction.FORWARD):
                                    calculated_rotation -= math.copysign(180, calculated_rotation)
                                    #calculated_strafe *= -1
                                    self.nav_direction = 1
                                elif self.nav_path.navigation_parameters.path_following_direction == Direction.BACKWARD:
                                    drive_solution_okay = False
                            elif self.nav_path.navigation_parameters.path_following_direction == Direction.FORWARD:
                                drive_solution_okay = False
                            elif self.nav_path.navigation_parameters.path_following_direction == Direction.BACKWARD:
                                calculated_strafe *= -1

                        if not drive_solution_okay:
                            self.autonomy_hold = True
                            self.activate_autonomy = False
                            self.control_state = CONTROL_NO_STEERING_SOLUTION
                            self.logger.error("Could not find drive solution. Disabling autonomy.")
                            self.logger.error("calculated_rotation: {}, vehicle_travel_direction {}, path_following_direction {}".format(calculated_rotation,self.nav_path.navigation_parameters.vehicle_travel_direction, self.nav_path.navigation_parameters.path_following_direction))

                        self.logger.info("calculated_rotation: {}, vehicle_travel_direction {}, path_following_direction {}, self.nav_direction {}, self.driving_direction {}".format(calculated_rotation,self.nav_path.navigation_parameters.vehicle_travel_direction, self.nav_path.navigation_parameters.path_following_direction, self.nav_direction, self.driving_direction))


                        gps_path_angle_error = calculated_rotation

                        # Accumulate a list of error values for angular and
                        # lateral error. This allows averaging of errors
                        # and also determination of their rate of change.
                        time_delta = time.time() - self.gps_error_update_time
                        self.gps_error_update_time = time.time()
                        self.gps_lateral_error_rate_averaging_list.append((gps_lateral_distance_error - self.gps_path_lateral_error) / time_delta)
                        self.gps_angle_error_rate_averaging_list.append((gps_path_angle_error - self.gps_path_angular_error) / time_delta)
                        self.gps_path_lateral_error = gps_lateral_distance_error
                        self.gps_path_angular_error = gps_path_angle_error

                        while len(self.gps_lateral_error_rate_averaging_list) > _ERROR_RATE_AVERAGING_COUNT:
                            self.gps_lateral_error_rate_averaging_list.pop(0)

                        while len(self.gps_angle_error_rate_averaging_list) > _ERROR_RATE_AVERAGING_COUNT:
                            self.gps_angle_error_rate_averaging_list.pop(0)

                        self.gps_path_lateral_error_rate = sum(self.gps_lateral_error_rate_averaging_list) / len(self.gps_lateral_error_rate_averaging_list)
                        self.gps_path_angular_error_rate = sum(self.gps_angle_error_rate_averaging_list) / len(self.gps_angle_error_rate_averaging_list)

                        self.next_point_heading = calculated_rotation

                    # These extra points can be displayed in the web UI.
                    # TODO: That's commented out in the server code. Resolve?
                    debug_points = (vehicle_front, vehicle_rear, projected_path_tangent_point, closest_path_point)

                time6 = time.time() - debug_time

                # Get joystick value
                if self.simulated_hardware:
                    joy_steer, joy_throttle, joy_strafe = 0.0, 0.0, 0.0
                else:
                    joy_steer, joy_throttle, joy_strafe = self.get_joystick_values(joy_steer, joy_throttle, joy_strafe)
                    if abs(joy_throttle) < _JOYSTICK_MIN:
                        joy_throttle = 0.0
                    if abs(joy_steer) < _JOYSTICK_MIN:
                        joy_steer = 0.0

                # Disable autonomy if manual control is activated.
                if abs(joy_steer) > 0.1 or abs(joy_throttle) > 0.1 or abs(joy_strafe) > 0.1:
                    self.logger.info("DISABLED AUTONOMY Steer: {}, Joy {}".format(joy_steer, joy_throttle))
                    self.autonomy_hold = True
                    self.activate_autonomy = False
                    self.control_state = CONTROL_OVERRIDE

                strafe_d = 0
                steer_d = 0
                strafe_p = 0
                steer_p = 0
                user_web_page_plot_steer_cmd = 0
                user_web_page_plot_strafe_cmd = 0

                # Calculate driving commands for autonomy.
                if self.next_point_heading != -180 and self.activate_autonomy and calculated_rotation!= None and calculated_strafe!= None:

                    # calculated_rotation *= 2.0
                    # calculated_strafe *= -0.35
                    # steer_d = self.gps_path_angular_error_rate * 0.8
                    # strafe_d = self.gps_path_lateral_error_rate * -0.2

                    steer_p = calculated_rotation * self.nav_path.control_values.angular_p
                    strafe_p = calculated_strafe * self.nav_path.control_values.lateral_p
                    steer_d = self.gps_path_angular_error_rate * self.nav_path.control_values.angular_d
                    strafe_d = self.gps_path_lateral_error_rate * self.nav_path.control_values.lateral_d

                    # self.logger.debug(self.nav_path.control_values)

                    self.logger.debug("strafe_p {}, strafe_d {}, steer_p {} steer_d {}".format(strafe_p, strafe_d, strafe_d, steer_d))

                    steer_command_value = steer_p + steer_d
                    strafe_command_value = strafe_p + strafe_d

                    _STRAFE_LIMIT = 0.25
                    _STEER_LIMIT = 45

                    # Value clamping
                    steering_angle = steer_command_value
                    if steering_angle > _STEER_LIMIT:
                        steering_angle = _STEER_LIMIT
                    if steering_angle < -_STEER_LIMIT:
                        steering_angle = -_STEER_LIMIT
                    if strafe_command_value > _STRAFE_LIMIT:
                        unfiltered_strafe_cmd = _STRAFE_LIMIT
                    if strafe_command_value < -_STRAFE_LIMIT:
                        unfiltered_strafe_cmd = -_STRAFE_LIMIT
                    if math.fabs(strafe_command_value) < _STRAFE_LIMIT:
                        unfiltered_strafe_cmd = strafe_command_value



                        #vehicle_travel_direction Direction.BACKWARD, path_following_direction Direction.FORWARD, self.nav_direction 1, self.driving_direction -1



                    unfiltered_steer_cmd = steering_angle/45.0
                    unfiltered_strafe_cmd *= self.driving_direction

                    autonomy_steer_diff = unfiltered_steer_cmd - self.last_autonomy_steer_cmd
                    autonomy_strafe_diff = unfiltered_strafe_cmd - self.last_autonomy_strafe_cmd

                    self.logger.debug("diffs: {}, {}".format(autonomy_steer_diff * _LOOP_RATE, autonomy_strafe_diff * _LOOP_RATE))

                    # Rate of change clamping
                    steer_rate = 4.0/_LOOP_RATE
                    strafe_rate = 5.0/_LOOP_RATE
                    if autonomy_steer_diff > steer_rate:
                        autonomy_steer_cmd = self.last_autonomy_steer_cmd + steer_rate
                    elif autonomy_steer_diff < -steer_rate:
                        autonomy_steer_cmd = self.last_autonomy_steer_cmd - steer_rate
                    else:
                        autonomy_steer_cmd = unfiltered_steer_cmd

                    if autonomy_strafe_diff > strafe_rate:
                        autonomy_strafe_cmd = self.last_autonomy_strafe_cmd + strafe_rate
                    elif autonomy_strafe_diff < -strafe_rate:
                        autonomy_strafe_cmd = self.last_autonomy_strafe_cmd - strafe_rate
                    else:
                        autonomy_strafe_cmd = unfiltered_strafe_cmd

                    self.last_autonomy_steer_cmd = autonomy_steer_cmd
                    self.last_autonomy_strafe_cmd = autonomy_strafe_cmd
                    user_web_page_plot_steer_cmd = autonomy_steer_cmd * self.driving_direction
                    user_web_page_plot_strafe_cmd = autonomy_strafe_cmd * self.driving_direction


                    autonomy_vel_cmd = self.autonomy_velocity  * self.driving_direction
                    joy_steer = 0.0 # ensures that vel goes to zero when autonomy disabled
                    if loop_count % 10 == 0:
                        #print(loop_count)
                        self.logger.info("steer_cmd: {:.2f}, strafe_cmd: {:.2f}, vel_cmd: {:.2f}, calculated_rotation: {:.2f}, calculated_strafe: {:.2f}".format(autonomy_steer_cmd, autonomy_strafe_cmd, vel_cmd, calculated_rotation, calculated_strafe))
                    #print("Steer: {}, Throttle: {}".format(steer_cmd, vel_cmd))
                zero_output = False


                # In the following list the order for when we set control state
                # matters
                # TODO: clarify? What does this note mean?

                time7 = time.time() - debug_time

                # Begin Safety Checks
                error_messages = []

                fatal_error = False

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

                if gps_tools.is_dual_fix(self.latest_gps_sample) == False:
                    error_messages.append("No GPS fix so zeroing out autonomy commands.")
                    zero_output = True
                    self.control_state = CONTROL_GPS_STARTUP
                    self.resume_motion_timer = time.time()
                elif abs(absolute_path_distance) > self.nav_path.maximum_allowed_distance_meters:
                    # Distance from path exceeds allowed limit.
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_AUTONOMY_ERROR_DISTANCE
                    error_messages.append("GPS distance {} meters too far from path so zeroing out autonomy commands.".format(abs(absolute_path_distance)))
                elif abs(gps_path_angle_error) > self.nav_path.maximum_allowed_angle_error_degrees:
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_AUTONOMY_ERROR_ANGLE
                    error_messages.append("GPS path angle {} exceeds allowed limit {} so zeroing out autonomy commands.".format(abs(gps_path_angle_error),self.nav_path.maximum_allowed_angle_error_degrees))
                elif self.latest_gps_sample.rtk_age > _ALLOWED_RTK_AGE_SEC:
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_AUTONOMY_ERROR_RTK_AGE
                    error_messages.append("RTK base station data too old so zeroing out autonomy commands.")
                elif time.time() - self.latest_gps_sample.time_stamp > _ALLOWED_SOLUTION_AGE_SEC:
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_AUTONOMY_ERROR_SOLUTION_AGE
                    error_messages.append("RTK solution too old so zeroing out autonomy commands.")


                if time.time() - self.robot_object.last_server_communication_stamp > SERVER_COMMUNICATION_DELAY_LIMIT_SEC:
                    server_communication_okay = False
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    error_messages.append("Server communication error so zeroing out autonomy commands. Last stamp age {} exceeds allowed age of {} seconds. AP name: {}, Signal Strength {} dbm\r\n".format(time.time() - self.robot_object.last_server_communication_stamp, SERVER_COMMUNICATION_DELAY_LIMIT_SEC, self.robot_object.wifi_ap_name, self.robot_object.wifi_strength))

                if loop_count % _ERROR_SKIP_RATE == 0:
                    for error in error_messages:
                        self.logger.error(error)

                if time.time() > last_wifi_restart_time + 500 and time.time() - self.robot_object.last_server_communication_stamp > _SERVER_DELAY_RECONNECT_WIFI_SECONDS and self.robot_object.last_server_communication_stamp > 0:
                    self.logger.error("Last Wifi signal strength: {} dbm\r\n".format(self.robot_object.wifi_strength))
                    self.logger.error("Last Wifi AP associated: {}\r\n".format(self.robot_object.wifi_ap_name))
                    self.logger.error("Restarting wlan1...")
                    try:
                        subprocess.check_call("ifconfig wlan1 down", shell=True)
                        subprocess.check_call("ifconfig wlan1 up", shell=True)
                    except:
                        pass
                    last_wifi_restart_time = time.time()
                    self.logger.error("Restarted wlan1.")


                if zero_output == True:
                    if self.activate_autonomy and time.time() - self.disengagement_time > _DISENGAGEMENT_RETRY_DELAY_SEC:
                        self.disengagement_time = time.time()
                        self.logger.error("Disengaging Autonomy.")
                        # Ensure we always print errors if we are deactivating autonomy.
                        if loop_count % _ERROR_SKIP_RATE != 0:
                            for error in error_messages:
                                self.logger.error(error)
                        self.logger.error("Last Wifi signal strength: {} dbm\r\n".format(self.robot_object.wifi_strength))
                        self.logger.error("Last Wifi AP associated: {}\r\n".format(self.robot_object.wifi_ap_name))
                        self.logger.error("Last CPU Temp: {}\r\n".format(self.robot_object.cpu_temperature_c))
                        with open("error_log.txt", 'a+') as file1:
                            file1.write("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n")
                            file1.write("Disegagement Log\r\n")
                            file1.write(datetime.datetime.now().strftime("%a %b %d, %I:%M:%S %p\r\n"))
                            file1.write("Last Wifi signal strength: {} dbm\r\n".format(self.robot_object.wifi_strength))
                            file1.write("Last Wifi AP associated: {}\r\n".format(self.robot_object.wifi_ap_name))
                            file1.write("Last CPU Temp: {}\r\n".format(self.robot_object.cpu_temperature_c))
                            if self.last_good_gps_sample is not None:
                                file1.write("Last known GPS location: {}, {}\r\n".format(self.last_good_gps_sample.lat, self.last_good_gps_sample.lon))
                            else:
                                file1.write("No valid GPS location recorded.")
                            error_count = 1
                            for error in error_messages:
                                file1.write("Error {}: {}\r\n".format(error_count, error))
                                error_count += 1
                            file1.write("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n")
                        if fatal_error:
                            self.autonomy_hold = True
                            self.activate_autonomy = False
                elif not self.activate_autonomy:
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_ONLINE

                time8 = time.time() - debug_time

                # Activate a brief pause at the end of a track.
                if time.time() - load_path_time < _PATH_END_PAUSE_SEC and not zero_output:
                    zero_output = True
                    # Don't use the motion timer here as it reactivates alarm.
                    if loop_count % 10 == 0:
                        self.logger.info("Taking a short delay at the end of the path so zeroing out autonomy commands.")

                if time.time() - self.disengagement_time < _DISENGAGEMENT_RETRY_DELAY_SEC:
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    if loop_count % 10 == 0:
                        self.logger.info("Disengaged for {:.1f} more seconds so zeroing out autonomy commands.".format(_DISENGAGEMENT_RETRY_DELAY_SEC - (time.time() - self.disengagement_time)))

                if self.activate_autonomy == True and zero_output == False and time.time() - self.resume_motion_timer < _RESUME_MOTION_WARNING_TIME_SEC:
                    zero_output = True
                    self.alarm1.value = True
                    self.alarm2.value = False
                    self.alarm3.value = True
                else:
                    self.alarm1.value = False
                    self.alarm2.value = False
                    self.alarm3.value = False

                # Determine final drive commands.
                if self.activate_autonomy:
                    autonomy_time_elapsed = time.time() - load_path_time - _PATH_END_PAUSE_SEC
                    if autonomy_time_elapsed < _BEGIN_AUTONOMY_SPEED_RAMP_SEC:
                        autonomy_vel_cmd*= autonomy_time_elapsed/_BEGIN_AUTONOMY_SPEED_RAMP_SEC
                    self.control_state = CONTROL_AUTONOMY
                    if zero_output:
                        self.control_state = CONTROL_AUTONOMY_PAUSE
                        vel_cmd = 0.0
                        steer_cmd = 0.0
                        strafe = 0
                    else:
                        vel_cmd = autonomy_vel_cmd
                        steer_cmd = autonomy_steer_cmd
                        strafe = autonomy_strafe_cmd
                else:
                    if self.voltage_average < _VOLTAGE_CUTOFF:
                        vel_cmd = 0.0
                        steer_cmd = 0.0
                        strafe = 0
                        if loop_count % _ERROR_SKIP_RATE == 0:
                            self.logger.error("LOW VOLTAGE PAUSE. Voltage average: {:.2f}".format(self.voltage_average))
                    else:
                        vel_cmd = joy_throttle
                        steer_cmd = joy_steer
                        if math.fabs(joy_strafe) < 0.1:
                            strafe = 0
                        else:
                            strafe = math.copysign(math.fabs(joy_strafe) - 0.1, joy_strafe)

                vel_cmd = vel_cmd * 1.0/(1.0 + abs(steer_cmd)) # Slow Vel down by 50% when steering is at max.

                time8b = time.time() - debug_time
                # Update master on latest calculations.
                send_data = (self.latest_gps_sample,self.nav_path.points,self.next_point_heading, debug_points, self.control_state, self.motor_state, self.autonomy_hold, self.gps_path_lateral_error, self.gps_path_angular_error, self.gps_path_lateral_error_rate, self.gps_path_angular_error_rate, strafe_p, steer_p, strafe_d, steer_d, user_web_page_plot_steer_cmd, user_web_page_plot_strafe_cmd, gps_tools.is_dual_fix(self.latest_gps_sample), self.voltage_average, self.last_energy_segment, self.temperatures)

                with self.remote_to_main_lock:
                    self.remote_to_main_string["value"] = pickle.dumps(send_data)

                period = time.time() - tick_time
                self.last_energy_segment = None

                time9 = time.time() - debug_time
                vel_cmd = vel_cmd * 0.6 # Fixed factor reduction

                # Perform acceleration on vel_cmd value.
                vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
                last_vel_cmd = vel_cmd
                tick_time = time.time()

                self.logger.debug("Final values: Steer {}, Vel {}".format(steer_cmd, vel_cmd))
                calc = calculate_steering(steer_cmd, vel_cmd, strafe)

                # If the robot is simulated, estimate movement odometry.
                if self.simulated_hardware and self.activate_autonomy:
                    new_heading_degrees = self.latest_gps_sample.azimuth_degrees + steer_cmd * 45.0 * 0.4
                    new_heading_degrees %= 360
                    next_point = gps_tools.project_point(self.latest_gps_sample, new_heading_degrees, vel_cmd * 0.5)
                    # Calculate translation for strafe, which is movement 90 degrees from heading.
                    next_point = gps_tools.project_point(next_point, new_heading_degrees + 90, strafe * 0.2)
                    self.simulated_sample = gps_tools.GpsSample(next_point.lat, next_point.lon, self.simulated_sample.height_m, ("fix","fix"), 20, new_heading_degrees, time.time(), 0.5)


                if not self.motor_socket:
                    self.logger.error("Connect to motor control socket")
                    self.connect_to_motors()

                time10 = 0

                # Try to send final drive commands to motor process.
                try:
                    if self.motor_send_okay == True:
                        #print("send_calc")
                        self.motor_socket.send_pyobj(pickle.dumps(calc), flags=zmq.NOBLOCK)
                        self.motor_send_okay = False
                        self.motor_last_send_time = time.time()

                    time10 = time.time() - debug_time
                    # else:
                    #     print("NOT OKAY TO SEND")
                    while self.motor_socket.poll(timeout=_VERY_FAST_POLL_MILLISECONDS):
                        #print("poll motor message")
                        motor_message = pickle.loads(self.motor_socket.recv_pyobj())
                        self.motor_send_okay = True
                        #print("motor_message: {}".format(motor_message))
                        try:
                            self.motor_state = motor_message[0]
                            self.voltages = motor_message[1]
                            self.bus_currents = motor_message[2]
                            self.temperatures = motor_message[3]
                            self.total_watts = 0
                            if len(self.voltages) > 0:
                                self.voltage_average = sum(self.voltages)/len(self.voltages)
                            for volt, current in zip(self.voltages, self.bus_currents):
                                self.total_watts += volt * current
                            #print("Drawing {} Watts.".format(int(self.total_watts)))
                        except Exception as e:
                            self.logger.error("Error reading motor state message.")
                            self.logger.error(e)
                            self.motor_state = STATE_DISCONNECTED
                except zmq.error.Again as e:
                    self.logger.error("Remote server unreachable.")
                except zmq.error.ZMQError as e:
                    self.logger.error("ZMQ error with motor command socket. Resetting.")
                    self.motor_state = STATE_DISCONNECTED
                    self.close_motor_socket()


                # If we have a GPS fix, update power consumption metrics.
                if gps_tools.is_dual_fix(self.latest_gps_sample):
                    # Calculate power consumption metrics in 1 meter segments
                    self.power_consumption_list.append((self.latest_gps_sample, self.total_watts, self.voltages, self.bus_currents))
                    oldest_power_sample_gps = self.power_consumption_list[0][0]
                    distance = gps_tools.get_distance(oldest_power_sample_gps, self.latest_gps_sample)
                    if distance > 1.0:
                        total_watt_seconds = 0
                        watt_average = self.power_consumption_list[0][1]
                        distance_sum = 0
                        duration = 0
                        height_change = 0
                        last_sample_gps = None
                        motor_total_watt_seconds = [0, 0, 0, 0]
                        motor_watt_average = [0, 0, 0, 0]
                        list_subsamples = []
                        sample_collector_index = 0
                        for sample_num in range(1, len(self.power_consumption_list)):
                            sample1 = self.power_consumption_list[sample_num - 1]
                            sample2 = self.power_consumption_list[sample_num]
                            if sample1 == None or sample2 == None:
                                continue
                            sample_distance = gps_tools.get_distance(sample1[0], sample2[0])
                            sample_duration = sample2[0].time_stamp - sample1[0].time_stamp
                            sample_avg_watts = (sample1[1] + sample2[1])/2.0
                            if sample_num > sample_collector_index:
                                list_subsamples.append(sample1[0])
                                if len(self.power_consumption_list) > _NUM_GPS_SUBSAMPLES:
                                    sample_collector_index+=int(len(self.power_consumption_list)/_NUM_GPS_SUBSAMPLES)
                                else:
                                    sample_collector_index+=1
                            #print(sample1[2])
                            try:
                                for idx in range(len(sample1[2])):
                                    motor_watt_average[idx] = (sample1[2][idx] * sample1[3][idx] + sample2[2][idx] * sample2[3][idx]) * 0.5
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


                        height_change = last_sample_gps.height_m - oldest_power_sample_gps.height_m
                        avg_watts = (watt_average)/len(self.power_consumption_list)
                        list_subsamples.append(last_sample_gps)


                        self.last_energy_segment = EnergySegment(loop_count, oldest_power_sample_gps, last_sample_gps, distance_sum, total_watt_seconds, avg_watts, motor_total_watt_seconds, motor_watt_average, list_subsamples, self.activate_autonomy, self.robot_object.wifi_ap_name, self.robot_object.wifi_strength)

                        # Reset power consumption list.
                        self.power_consumption_list = []
                        self.logger.info("                                                                                                                                      Avg watts {:.1f}, watt seconds per meter: {:.1f}, meters per second: {:.2f}, height change {:.2f}".format(self.last_energy_segment.avg_watts, self.last_energy_segment.watt_seconds_per_meter, self.last_energy_segment.meters_per_second, self.last_energy_segment.height_change))

                if self.simulated_hardware:
                    time.sleep(_POLL_MILLISECONDS/_MILLISECONDS_PER_SECOND)
                self.logger.debug("Took {} sec to get here. {} {} {} {} {} {} {} {} {} {} {} {}".format(time.time()-debug_time, time1, time2, time3, time4, time5, time6, time7, time8, time8b, time9, time10, self.robot_object.wifi_ap_name))
        except KeyboardInterrupt:
            pass



def run_control(remote_to_main_lock, main_to_remote_lock, remote_to_main_string, main_to_remote_string, logging, logging_details, simulated_hardware=False):
    remote_control = RemoteControl(remote_to_main_lock, main_to_remote_lock, remote_to_main_string, main_to_remote_string, logging, logging_details, simulated_hardware)
    remote_control.run_setup()
    remote_control.run_loop()


if __name__=="__main__":
    run_control()
