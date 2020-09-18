
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
from motors import _STATE_ENABLED

# This file gets imported by server but we should only import GPIO on raspi.
if "arm" in os.uname().machine:
    import board
    import busio
    import digitalio
    from adafruit_mcp230xx.mcp23017 import MCP23017

COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

ACCELERATION_COUNTS_SEC = 0.5

__gps_lateral_distance_SCALAR = 100000

_RESUME_MOTION_WARNING_TIME_SEC = 20

_MAXIMUM_ALLOWED_DISTANCE_METERS = 2
_VOLTAGE_CUTOFF = 30

CONTROL_STARTUP = "Initializing..."
CONTROL_GPS_STARTUP = "Waiting for GPS fix"
CONTROL_ONLINE = "Online and awaiting commands"
CONTROL_AUTONOMY = "Autonomy Operating"
CONTROL_LOW_VOLTAGE = "Low Voltage Pause"
CONTROL_AUTONOMY_ERROR_DISTANCE = "Autonomy failed - too far from path."
CONTROL_AUTONOMY_ERROR_RTK_AGE = "Autonomy failed - rtk base data too old."
CONTROL_AUTONOMY_ERROR_SOLUTION_AGE = "Autonomy failed - gps solution too old."
CONTROL_OVERRIDE = "Remote control override"
CONTROL_SERVER_ERROR = "Server communication error."
CONTROL_MOTOR_ERROR = "Motor error detected."

_ALLOWED_RTK_AGE_SEC = 20.0
_ALLOWED_SOLUTION_AGE_SEC = 2.0

SERVER_COMMUNICATION_DELAY_LIMIT_SEC = 5

_SLOW_POLLING_SLEEP_S = 0.5

_POLL_MILLISECONDS = 50


def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel-last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel-last_vel
    else:
        increment = math.copysign(ACCELERATION_COUNTS_SEC, unfiltered_vel-last_vel) * period_s
    return last_vel + increment


class RemoteControl():

    def __init__(self, master_conn):
        self.joy = None
        self.motor_socket = None
        self.master_conn = master_conn
        self.robot_object = None
        self.next_point_heading = -180
        self.activate_autonomy = False
        self.autonomy_velocity = 0
        self.resume_motion_timer = 0

    def run_setup(self):
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

    def connect_to_motors(self):
        context = zmq.Context()
        #  Socket to talk to motor control process
        self.motor_socket = context.socket(zmq.PAIR)
        self.motor_socket.connect("tcp://localhost:5590")

    def get_joystick_values(self, st_old, th_old, stf_old):
        steer = None
        throttle = None
        strafe = None
        count = 0
        #print("Enter_joy")
        while True:
            event = self.joy.read_one()
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
        self.motor_state = "Motor state undefined."
        self.gps_path_lateral_error = 0
        self.gps_path_lateral_error_rate = 0
        self.gps_path_angular_error = 0
        self.gps_path_angular_error_rate = 0
        self.gps_error_update_time = 0
        try:
            loop_count = 0
            while True:
                loop_count += 1

                # Consume the incoming messages.
                if self.master_conn.poll():
                    recieved_robot_object = self.master_conn.recv()
                    # print(type(self.robot_object))
                    if str(type(recieved_robot_object))=="<class '__main__.Robot'>":
                        self.robot_object = recieved_robot_object
                        if len(self.nav_path) == 0 or self.loaded_path_name != self.robot_object.loaded_path_name:
                            if len(self.robot_object.loaded_path) > 0:
                                self.nav_spline = spline_lib.GpsSpline(self.robot_object.loaded_path, smooth_factor=10, num_points=200)
                                self.nav_path = self.nav_spline.points
                                dist_start = gps_tools.get_distance(self.robot_object.location, self.nav_path[0])
                                dist_end = gps_tools.get_distance(self.robot_object.location, self.nav_path[len(self.nav_path)-1])
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

                if str(type(self.robot_object))!="<class '__main__.Robot'>":
                    print("Waiting for valid robot object before running remote control code.")
                    time.sleep(_SLOW_POLLING_SLEEP_S)
                    continue


                debug_points = (None, None, None, None)
                calculated_rotation = None
                calculated_strafe = None
                gps_fix = False
                distance_from_path_okay = False
                rtk_age_okay = False
                solution_age_okay = False
                _gps_lateral_distance = 0
                _gps_path_angle = 0
                if self.robot_object:
                    front = gps_tools.project_point(self.robot_object.location, self.robot_object.location.azimuth_degrees, 1.0)
                    rear = gps_tools.project_point(self.robot_object.location, self.robot_object.location.azimuth_degrees, -1.0)
                    #print("GPS DEBUG: FIX reads... : {}".format(self.robot_object.location.status))
                    if len(self.robot_object.location.status) == 2:
                        if self.robot_object.location.status[0] == 'fix' and self.robot_object.location.status[1] == 'fix':
                            #print("WE HAVE A FIX")
                            gps_fix = True
                    if self.robot_object.location.rtk_age < _ALLOWED_RTK_AGE_SEC:
                        rtk_age_okay = True

                    if time.time() - self.robot_object.location.time_stamp < _ALLOWED_SOLUTION_AGE_SEC:
                        solution_age_okay = True


                    closest_front = gps_tools.GpsPoint(0, 0)
                    closest_rear = gps_tools.GpsPoint(0, 0)
                    if(len(self.nav_path)>0):

                        # Check if we meet the end condition.
                        dist_start = gps_tools.get_distance(self.robot_object.location, self.nav_path[0])
                        dist_end = gps_tools.get_distance(self.robot_object.location, self.nav_path[len(self.nav_path)-1])
                        if dist_start < 3.0:
                            if self.nav_direction == -1:
                                self.nav_path = []
                        if dist_end < 3.0:
                            if self.nav_direction == 1:
                                self.nav_path = []


                        closest_u = self.nav_spline.closestUOnSpline(self.robot_object.location)
                        closest_point = self.nav_spline.coordAtU(closest_u)
                        #print("closest_point {}".format(closest_point))
                        spline_angle_rad = self.nav_spline.slopeRadiansAtU(closest_u)

                        path_point_heading = math.degrees(spline_angle_rad)


                        projected_point = gps_tools.project_point(closest_point, path_point_heading, 3.0)
                        _gps_lateral_distance = gps_tools.get_approx_distance_point_from_line(self.robot_object.location, closest_point, projected_point)

                        if abs(_gps_lateral_distance) < _MAXIMUM_ALLOWED_DISTANCE_METERS:
                            distance_from_path_okay = True
                        #print("DISTANCE: {}".format(abs(distance_from_line)))


                        closest_front = projected_point
                        closest_rear = closest_point

                        #print("robot heading {}, path heading {}".format(self.robot_object.location.azimuth_degrees, path_point_heading))
                        calculated_rotation = path_point_heading - self.robot_object.location.azimuth_degrees
                        calculated_strafe = _gps_lateral_distance

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

                        _gps_path_angle = calculated_rotation


                        time_delta = time.time() - self.gps_error_update_time
                        self.gps_error_update_time = time.time()

                        self.gps_path_lateral_error_rate = (_gps_lateral_distance - self.gps_path_lateral_error) / time_delta
                        self.gps_path_angular_error_rate = (_gps_lateral_distance - self.gps_path_lateral_error) / time_delta
                        self.gps_path_lateral_error = _gps_lateral_distance
                        self.gps_path_angular_error = _gps_path_angle

                        self.next_point_heading = calculated_rotation


                    debug_points = (front, rear, closest_front, closest_rear)


                # Check for updated motor status message.
                if self.motor_socket.poll(_POLL_MILLISECONDS):
                    motor_message = self.motor_socket.recv_multipart()
                    try:
                        self.motor_state = motor_message[0].decode("utf-8")
                        #print("GOT_MOTOR_MESSAGE: {}".format(self.motor_state))
                        #print(self.motor_state)
                    except:
                        print("Error reading motor state message.")


                # Get joystick value
                joy_steer, joy_throttle, joy_strafe = self.get_joystick_values(joy_steer, joy_throttle, joy_strafe)
                #print(joy_throttle)
                if abs(joy_steer) > 0.1 or abs(joy_throttle) > 0.1 or abs(joy_strafe) > 0.1:
                    print("DISABLED AUTONOMY Steer: {}, Joy {}".format(joy_steer, joy_throttle))
                    self.autonomy_hold = True
                    self.activate_autonomy = False
                    self.control_state = CONTROL_OVERRIDE


                # Calculate driving commands for autonomy.
                if self.next_point_heading != -180 and self.activate_autonomy:

                    direction = direction * self.nav_direction

                    if calculated_rotation and calculated_strafe:
                        steering_angle = calculated_rotation * 1.2
                        if steering_angle > 45:
                            steering_angle = 45
                        if steering_angle < -45:
                            steering_angle = -45
                        autonomy_steer_cmd = steering_angle/45.0 * direction
                        if calculated_strafe > 1:
                            autonomy_strafe = 0.5
                        if calculated_strafe < -1:
                            autonomy_strafe = -0.5
                        if math.fabs(calculated_strafe) < 1.0:
                            autonomy_strafe = calculated_strafe/2.0
                        autonomy_strafe *= -0.5 * direction * self.nav_direction

                    autonomy_vel_cmd = self.autonomy_velocity * direction # * self.nav_direction
                    joy_steer = 0.0 # ensures that vel goes to zero when autonomy disabled
                    if loop_count % 10 == 0:
                        #print(loop_count)
                        print("calc rotation: {}, calc strafe: {}, steer: {}, strafe: {}, vel_cmd: {}".format(calculated_rotation, calculated_strafe, steer_cmd, strafe, vel_cmd))
                    #print("Steer: {}, Throttle: {}".format(steer_cmd, vel_cmd))
                zero_output = False


                # In the following list the order for when we set control state matters

                error_messages = []

                if self.motor_state != _STATE_ENABLED:
                    error_messages.append("Motor error so zeroing out autonomy commands.")
                    zero_output = True
                    self.control_state = CONTROL_MOTOR_ERROR
                    self.resume_motion_timer = time.time()

                if self.robot_object.voltage < _VOLTAGE_CUTOFF:
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
                    error_messages.append("GPS distance too far from path so zeroing out autonomy commands.")
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
                    error_messages.append("Server communication error so zeroing out autonomy commands.")

                if loop_count % 10 == 0:
                    for error in error_messages:
                        print(error)

                if zero_output == True:
                    if self.activate_autonomy:
                        print("Deactivating Autonomy.")
                        # Ensure we always print errors if we are deactivating autonomy.
                        if loop_count % 10 != 0:
                            for error in error_messages:
                                print(error)
                        with open("error_log.txt", 'a+') as file1:
                            file1.write("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n")
                            file1.write("Deactivation Log\r\n")
                            file1.write(datetime.datetime.now().strftime("%a %b %d, %I:%M:%S %p\r\n"))
                            file1.write("Last Wifi signal strength: {} dbm\r\n".format(self.robot_object.wifi_strength))
                            file1.write("Last Wifi AP associated: {}\r\n".format(self.robot_object.wifi_ap_name))
                            file1.write("Error location: {}, {}\r\n".format(self.robot_object.location.lat, self.robot_object.location.lon))
                            error_count = 1
                            for error in error_messages:
                                file1.write("Error {}: {}\r\n".format(error_count, error))
                                error_count += 1
                            file1.write("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n")
                    self.autonomy_hold = True
                    self.activate_autonomy = False
                elif not self.activate_autonomy:
                    self.resume_motion_timer = time.time()
                    self.control_state = CONTROL_ONLINE

                if time.time() - load_path_time < 5 and not zero_output:
                    zero_output = True
                    # Don't use the motion timer here as it reactivates alarm.
                    if loop_count % 10 == 0:
                        print("Taking a short delay at the end of the path so zeroing out autonomy commands.")

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
                    self.control_mode = CONTROL_AUTONOMY
                    if zero_output:
                        vel_cmd = 0.0
                        steer_cmd = 0.0
                        strafe = 0
                    else:
                        vel_cmd = autonomy_vel_cmd
                        steer_cmd = autonomy_steer_cmd
                        strafe = autonomy_strafe
                else:
                    vel_cmd = joy_throttle
                    steer_cmd = joy_steer
                    if math.fabs(joy_strafe) < 0.1:
                        strafe = 0
                    else:
                        strafe = math.copysign(math.fabs(joy_strafe) - 0.1, joy_strafe)


                self.master_conn.send((self.nav_path,self.next_point_heading, debug_points, self.control_state, self.motor_state, self.autonomy_hold, self.gps_path_lateral_error, self.gps_path_angular_error, self.gps_path_lateral_error_rate, self.gps_path_angular_error_rate))

                #print(self.activate_autonomy)
                period = time.time() - tick_time


                # Perform acceleration on vel_cmd value
                vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
                last_vel_cmd = vel_cmd
                tick_time = time.time()

                #print("Final values: Steer {}, Vel {}".format(steer_cmd, vel_cmd))
                calc = calculate_steering(steer_cmd, vel_cmd, strafe)
                try:
                    self.motor_socket.send_pyobj(pickle.dumps(calc), flags=zmq.NOBLOCK)
                except zmq.ZMQError:
                    pass
                    #print("Warning: Motor Control pipe full, or other ZMQ error raised.")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass



def run_control(parent_process_connection):
    remote_control = RemoteControl(parent_process_connection)
    remote_control.run_setup()
    remote_control.run_loop()


if __name__=="__main__":
    run_control(None)
