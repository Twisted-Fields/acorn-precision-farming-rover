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

        # port = "5996"
        # context = zmq.Context()
        # self.master_conn = context.socket(zmq.PAIR)
        # self.master_conn.connect("tcp://localhost:%s" % port)

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
        #print("Enter_joy")
        while True:
            event = None
            try:
                event = self.joy.read_one()
            except Exception as e:
                self.logger.error("Joystick read exception: {}".format(e))
            if event == None:
                break
            #print(event)
            if event and event.type == ecodes.EV_ABS:
                absevent = categorize(event)
                #print(ecodes.bytype[absevent.event.type][absevent.event.code])
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RX':
                    steer = absevent.event.value / 32768.0
                    # print("ABS_X: {}".format(absevent.event.value))
                #
                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Y':
                    throttle = -absevent.event.value / 32768.0

                if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_X':
                    strafe = absevent.event.value / 32768.0
                    #print(strafe)
            count += 1
            # if steer and throttle and strafe:
            #     break
            # if count > 12:
            #     break
        #print("Took {} reads to read controller.".format(count))
        if not steer:
            steer = st_old
        if not throttle:
            throttle = th_old
        if not strafe:
            strafe = stf_old
        # Consume all existing read events.
        # while self.joy.read_one() != None:
        #     pass
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
        self.nav_path = []
        self.gps_path = []
        load_path_time = time.time()
        auto_throttle = 0
        self.loaded_path_name = ""
        self.autonomy_hold = True
        self.nav_path_next_point_index = 0
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
        autonomy_vel_cmd = 0
        last_wifi_restart_time = 0
        if self.simulated_hardware:
            rtk_socket1 = None
            rtk_socket2 = None
        else:
            rtk_process.launch_rtk_sub_procs(self.logger)
            rtk_socket1, rtk_socket2 = rtk_process.connect_rtk_procs(self.logger)
        allowed_rtk_errors = 60
        rtk_error_count = 0
        print_gps_counter = 0
        self.latest_gps_sample = None
        self.last_good_gps_sample = None
        self.gps_buffers = ["",""]
        debug_time = time.time()
        last_closest_u = -1
        try:
            loop_count = -1
            while True:
                loop_count += 1
                # if loop_count >= 80:
                #     time.sleep(0.5) #DEBUGGING
                #print("rtk_loop_once_start")
                self.gps_buffers, self.latest_gps_sample = (
                    rtk_process.rtk_loop_once(rtk_socket1, rtk_socket2,
                    buffers=self.gps_buffers,
                    print_gps=loop_count % GPS_PRINT_INTERVAL == 0,
                    last_sample=self.latest_gps_sample,
                    retries=_GPS_ERROR_RETRIES,
                    return_simulated_data=self.simulated_hardware,
                    logger=self.logger))
                debug_time = time.time()
                # print(type(self.latest_gps_sample))
                if self.latest_gps_sample is not None:
                    self.last_good_gps_sample = self.latest_gps_sample
                    if rtk_error_count > 0:
                        rtk_error_count-=1
                else:
                    rtk_error_count += 1
                    if rtk_error_count > allowed_rtk_errors:
                        pass
                        #rtk_socket1, rtk_socket2 = rtk_process.reset_and_reconnect_rtk(rtk_socket1, rtk_socket2)
                #print("rtk_loop_once_completed")

                # print(self.main_to_remote_string["value"])
                # import sys
                # sys.exit()


                try:
                    recieved_robot_object = None
                    time1 = time.time() - debug_time
                    # while self.master_conn.poll(_FAST_POLL_MILLISECONDS):
                    #     recieved_robot_object = pickle.loads(self.master_conn.recv_pyobj())
                    with self.main_to_remote_lock:
                        #print(self.main_to_remote_string["value"])
                        recieved_robot_object = pickle.loads(self.main_to_remote_string["value"])
                        #print(recieved_robot_object)
                    time2 = time.time() - debug_time
                except Exception as e:
                    self.logger.error("Exception reading remote string.")
                    raise(e)
                if recieved_robot_object:
                    if str(type(recieved_robot_object))=="<class '__main__.Robot'>":
                        self.robot_object = recieved_robot_object
                        # print("Remote received new robot object.")
                        time3 = time.time() - debug_time
                        if len(self.nav_path) == 0 or self.loaded_path_name != self.robot_object.loaded_path_name:
                            if len(self.robot_object.loaded_path) > 0  and self.latest_gps_sample is not None:
                                #print(self.robot_object.loaded_path)
                                last_closest_u = -1
                                self.nav_spline = spline_lib.GpsSpline(self.robot_object.loaded_path, smooth_factor=10, num_points=1000)
                                self.nav_path = self.nav_spline.points
                                dist_start = gps_tools.get_distance(self.latest_gps_sample, self.nav_path[0])
                                dist_end = gps_tools.get_distance(self.latest_gps_sample, self.nav_path[len(self.nav_path)-1])
                                if dist_start < dist_end:
                                    self.nav_direction = 1
                                else:
                                    self.nav_direction = -1

                                load_path_time = time.time()
                                self.loaded_path_name = self.robot_object.loaded_path_name
                                self.nav_path_next_point_index = 0
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

                if str(type(self.robot_object))!="<class '__main__.Robot'>":
                    self.logger.info("Waiting for valid robot object before running remote control code.")
                    time.sleep(_SLOW_POLLING_SLEEP_S)
                    continue


                time4 = time.time() - debug_time

            #    print("begin robot calc")

                debug_points = (None, None, None, None)
                calculated_rotation = None
                calculated_strafe = None
                gps_fix = False
                distance_from_path_okay = False
                angle_from_path_okay = False
                rtk_age_okay = False
                solution_age_okay = False
                _gps_lateral_distance_error = 0
                _gps_path_angle_error = 0
                time5 = 0
                if self.robot_object and self.latest_gps_sample is not None:
                    front = gps_tools.project_point(self.latest_gps_sample, self.latest_gps_sample.azimuth_degrees, 1.0)
                    rear = gps_tools.project_point(self.latest_gps_sample, self.latest_gps_sample.azimuth_degrees, -1.0)
                    #print("GPS DEBUG: FIX reads... : {}".format(self.latest_gps_sample.status))
                    if len(self.latest_gps_sample.status) == 2:
                        if self.latest_gps_sample.status[0] == 'fix' and self.latest_gps_sample.status[1] == 'fix':
                            #print("WE HAVE A FIX")
                            gps_fix = True
                    if self.latest_gps_sample.rtk_age < _ALLOWED_RTK_AGE_SEC:
                        rtk_age_okay = True

                    if time.time() - self.latest_gps_sample.time_stamp < _ALLOWED_SOLUTION_AGE_SEC:
                        solution_age = time.time() - self.latest_gps_sample.time_stamp
                        # self.solution_age_averaging_list.append(solution_age)
                        # while len(self.solution_age_averaging_list) > 30:
                        #     self.solution_age_averaging_list.pop(0)
                        # age_avg = sum(self.solution_age_averaging_list)/len(self.solution_age_averaging_list)
                        # print("solution age average: {}".format(age_avg))
                        solution_age_okay = True
                    if not solution_age_okay:
                        self.logger.error("SOLUTION AGE {} NOT OKAY AND LATEST GPS SAMPLE IS: {}".format(time.time() - self.latest_gps_sample.time_stamp, self.latest_gps_sample))
                        self.logger.error("Took {} sec to get here. {} {} {} {}".format(time.time()-debug_time, time1, time2, time3, time4))

                    time5 = time.time() - debug_time

                    closest_front = gps_tools.GpsPoint(0, 0)
                    closest_rear = gps_tools.GpsPoint(0, 0)
                    if(len(self.nav_path)>0):

                        # Check if we meet the end condition.
                        dist_start = gps_tools.get_distance(self.latest_gps_sample, self.nav_path[0])
                        dist_end = gps_tools.get_distance(self.latest_gps_sample, self.nav_path[len(self.nav_path)-1])
                        if dist_start < 1.0:
                            if self.nav_direction == -1:
                                self.nav_path = []
                        if dist_end < 1.0:
                            if self.nav_direction == 1:
                                self.nav_path = []


                        closest_u = self.nav_spline.closestUOnSpline(self.latest_gps_sample)#, last_closest_u, search_range=1.0)
                        closest_point = self.nav_spline.coordAtU(closest_u)
                        #print("closest_point {}".format(closest_point))
                        spline_angle_rad = self.nav_spline.slopeRadiansAtU(closest_u)
                        last_closest_u = closest_u

                        path_point_heading = math.degrees(spline_angle_rad)


                        projected_point = gps_tools.project_point(closest_point, path_point_heading, 3.0)
                        _gps_lateral_distance_error = gps_tools.get_approx_distance_point_from_line(self.latest_gps_sample, closest_point, projected_point)

                        if abs(_gps_lateral_distance_error) < _MAXIMUM_ALLOWED_DISTANCE_METERS:
                            distance_from_path_okay = True
#                        print("DISTANCE: {}".format(abs(_gps_lateral_distance_error)))


                        closest_front = projected_point
                        closest_rear = closest_point

                        #print("robot heading {}, path heading {}".format(self.latest_gps_sample.azimuth_degrees, path_point_heading))
                        calculated_rotation = path_point_heading - self.latest_gps_sample.azimuth_degrees
                        calculated_strafe = _gps_lateral_distance_error

                        while calculated_rotation > 180:
                            calculated_rotation-= 360

                        while calculated_rotation < -180:
                            calculated_rotation+= 360

                        direction = 1
                        if calculated_rotation > 90:
                            calculated_rotation-= 180
                            direction = -1

                        if calculated_rotation < -90:
                            calculated_rotation+= 180
                            direction = -1

                        _gps_path_angle_error = calculated_rotation

                        if abs(_gps_path_angle_error) < _MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES:
                            angle_from_path_okay = True


                        time_delta = time.time() - self.gps_error_update_time
                        self.gps_error_update_time = time.time()
                        #print(time_delta)


                        self.gps_lateral_error_rate_averaging_list.append((_gps_lateral_distance_error - self.gps_path_lateral_error) / time_delta)
                        self.gps_angle_error_rate_averaging_list.append((_gps_path_angle_error - self.gps_path_angular_error) / time_delta)
                        self.gps_path_lateral_error = _gps_lateral_distance_error
                        self.gps_path_angular_error = _gps_path_angle_error

                        while len(self.gps_lateral_error_rate_averaging_list) > _ERROR_RATE_AVERAGING_COUNT:
                            self.gps_lateral_error_rate_averaging_list.pop(0)

                        while len(self.gps_angle_error_rate_averaging_list) > _ERROR_RATE_AVERAGING_COUNT:
                            self.gps_angle_error_rate_averaging_list.pop(0)

                        self.gps_path_lateral_error_rate = sum(self.gps_lateral_error_rate_averaging_list) / len(self.gps_lateral_error_rate_averaging_list)
                        self.gps_path_angular_error_rate = sum(self.gps_angle_error_rate_averaging_list) / len(self.gps_angle_error_rate_averaging_list)

                        self.next_point_heading = calculated_rotation


                    debug_points = (front, rear, closest_front, closest_rear)

            #    print("mid robot calc")
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
                #print(joy_throttle)
                if abs(joy_steer) > 0.1 or abs(joy_throttle) > 0.1 or abs(joy_strafe) > 0.1:
                    self.logger.info("DISABLED AUTONOMY Steer: {}, Joy {}".format(joy_steer, joy_throttle))
                    self.autonomy_hold = True
                    self.activate_autonomy = False
                    self.control_state = CONTROL_OVERRIDE

                strafeD = 0
                steerD = 0
                strafeP = 0
                steerP = 0
                user_web_page_plot_steer_cmd = 0
                user_web_page_plot_strafe_cmd = 0

                # Calculate driving commands for autonomy.
                if self.next_point_heading != -180 and self.activate_autonomy and calculated_rotation and calculated_strafe:

                    direction = direction * self.nav_direction

                    # calculated_rotation *= 2.0
                    # calculated_strafe *= -0.35
                    # steerD = self.gps_path_angular_error_rate * 0.8
                    # strafeD = self.gps_path_lateral_error_rate * -0.2

                    calculated_rotation *= 0.9
                    calculated_strafe *= -0.25
                    steerD = self.gps_path_angular_error_rate * 0.3
                    strafeD = self.gps_path_lateral_error_rate * -0.05

                    #print("StrafeP {}, StrafeD {}, SteerP {} SteerD {}".format(calculated_strafe, strafeD, calculated_rotation, steerD))

                    steerP = calculated_rotation
                    strafeP = calculated_strafe

                    calculated_rotation += steerD
                    calculated_strafe += strafeD

                    _STRAFE_LIMIT = 0.25
                    _STEER_LIMIT = 45

                    # Value clamping
                    steering_angle = calculated_rotation
                    if steering_angle > _STEER_LIMIT:
                        steering_angle = _STEER_LIMIT
                    if steering_angle < -_STEER_LIMIT:
                        steering_angle = -_STEER_LIMIT
                    if calculated_strafe > _STRAFE_LIMIT:
                        unfiltered_strafe_cmd = _STRAFE_LIMIT
                    if calculated_strafe < -_STRAFE_LIMIT:
                        unfiltered_strafe_cmd = -_STRAFE_LIMIT
                    if math.fabs(calculated_strafe) < _STRAFE_LIMIT:
                        unfiltered_strafe_cmd = calculated_strafe


                    unfiltered_steer_cmd = steering_angle/45.0 * direction
                    unfiltered_strafe_cmd *= direction * self.nav_direction

                    autonomy_steer_diff = unfiltered_steer_cmd - self.last_autonomy_steer_cmd
                    autonomy_strafe_diff = unfiltered_strafe_cmd - self.last_autonomy_strafe_cmd



                    #print("diffs: {}, {}".format(autonomy_steer_diff * _LOOP_RATE, autonomy_strafe_diff * _LOOP_RATE))


                    # Rate of change clamping
                    # steer_rate = 0.4/_LOOP_RATE
                    # strafe_rate = 2.5/_LOOP_RATE
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
                    user_web_page_plot_steer_cmd = autonomy_steer_cmd * direction
                    user_web_page_plot_strafe_cmd = autonomy_strafe_cmd * direction * self.nav_direction


                    autonomy_vel_cmd = self.autonomy_velocity * direction # * self.nav_direction
                    joy_steer = 0.0 # ensures that vel goes to zero when autonomy disabled
                    if loop_count % 10 == 0:
                        #print(loop_count)
                        self.logger.info("calc rotation: {:.2f}, calc strafe: {:.2f}, steer: {:.2f}, strafe: {:.2f}, vel_cmd: {:.2f}".format(steerP, strafeP, steer_cmd, strafeP, vel_cmd))
                    #print("Steer: {}, Throttle: {}".format(steer_cmd, vel_cmd))
                zero_output = False


                # In the following list the order for when we set control state matters

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

                if gps_fix == False:
                    error_messages.append("No GPS fix so zeroing out autonomy commands.")
                    zero_output = True
                    self.control_state = CONTROL_GPS_STARTUP
                    self.resume_motion_timer = time.time()
                elif distance_from_path_okay == False:
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_AUTONOMY_ERROR_DISTANCE
                    error_messages.append("GPS distance {} meters too far from path so zeroing out autonomy commands.".format(abs(_gps_lateral_distance_error)))
                elif angle_from_path_okay == False:
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_AUTONOMY_ERROR_ANGLE
                    error_messages.append("GPS path angle exceeds allowed limit so zeroing out autonomy commands.")
                elif rtk_age_okay == False:
                    zero_output = True
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_AUTONOMY_ERROR_RTK_AGE
                    error_messages.append("RTK base station data too old so zeroing out autonomy commands.")
                elif solution_age_okay == False:
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

                if self.robot_object.wifi_ap_name == 'Farmhouse Exterior HP':
                    if self.robot_object.loaded_path_name == 'new_big_field_row':
                        if self.activate_autonomy:
                            self.logger.error("WARNING BAD ACCESS POINT DETECTED!")
                            self.logger.error("SIGNAL STRENGTH --  {}".format(self.robot_object.wifi_strength))



                #print("late robot calc")
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
                send_data = (self.latest_gps_sample,self.nav_path,self.next_point_heading, debug_points, self.control_state, self.motor_state, self.autonomy_hold, self.gps_path_lateral_error, self.gps_path_angular_error, self.gps_path_lateral_error_rate, self.gps_path_angular_error_rate, strafeP, steerP, strafeD, steerD, user_web_page_plot_steer_cmd, user_web_page_plot_strafe_cmd, gps_fix, self.voltage_average, self.last_energy_segment, self.temperatures)

                with self.remote_to_main_lock:
                    self.remote_to_main_string["value"] = pickle.dumps(send_data)


                #self.master_conn.send_pyobj(pickle.dumps(send_data))
                period = time.time() - tick_time
                self.last_energy_segment = None

                time9 = time.time() - debug_time
                vel_cmd = vel_cmd * 0.6 # Fixed factor reduction

                # Perform acceleration on vel_cmd value.
                vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
                last_vel_cmd = vel_cmd
                tick_time = time.time()

                #print("Final values: Steer {}, Vel {}".format(steer_cmd, vel_cmd))
                calc = calculate_steering(steer_cmd, vel_cmd, strafe)

                # print("TICK")

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


                if gps_fix:
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
                #print("Took {} sec to get here. {} {} {} {} {} {} {} {} {} {} {} {}".format(time.time()-debug_time, time1, time2, time3, time4, time5, time6, time7, time8, time8b, time9, time10, self.robot_object.wifi_ap_name))
        except KeyboardInterrupt:
            pass



def run_control(remote_to_main_lock, main_to_remote_lock, remote_to_main_string, main_to_remote_string, logging, logging_details, simulated_hardware=False):
    remote_control = RemoteControl(remote_to_main_lock, main_to_remote_lock, remote_to_main_string, main_to_remote_string, logging, logging_details, simulated_hardware)
    remote_control.run_setup()
    remote_control.run_loop()


if __name__=="__main__":
    run_control()
