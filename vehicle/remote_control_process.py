
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


COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

ACCELERATION_COUNTS_SEC = 0.5


def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel-last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel-last_vel
    else:
        increment = math.copysign(ACCELERATION_COUNTS_SEC, unfiltered_vel-last_vel) * period_s
    return last_vel + increment

def get_joystick(joy, st_old, th_old):
    steer = None
    throttle = None
    count = 0
    #print("Enter_joy")
    while True:
        event = joy.read_one()
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
        count += 1
        if steer and throttle:
            break
        if count > 12:
            break
    if not steer:
        steer = st_old
    if not throttle:
        throttle = th_old
    #print("Exit_joy")
    return steer, throttle


class RemoteControl():

    def __init__(self, master_conn):
        self.joy = None
        self.motor_socket = None
        self.master_conn = master_conn
        self.robot_object = None
        self.turn_intent_degrees = -180
        self.activate_autonomy = False
        self.autonomy_velocity = 0

    def run_setup(self):
        self.connect_to_motors()
        self.get_joystick()

    def connect_to_motors(self):
        context = zmq.Context()
        #  Socket to talk to motor control process
        self.motor_socket = context.socket(zmq.PUSH)
        self.motor_socket.connect("tcp://localhost:5590")


    def get_joystick(self):
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
        vel_cmd = 0
        last_vel_cmd = 0
        tick_time = time.time()
        self.nav_path = []
        self.gps_path = []
        load_path_time = time.time()
        auto_throttle = 0
        self.loaded_path_name = ""
        self.autonomy_disabled = False
        try:
            while True:
                # Consume the incoming messages.
                while self.master_conn.poll():
                    self.robot_object = self.master_conn.recv()
                    # print(type(self.robot_object))
                if str(type(self.robot_object))=="<class '__main__.Robot'>":
                    if len(self.nav_path) == 0 or self.loaded_path_name != self.robot_object.loaded_path_name:
                        self.nav_path=self.robot_object.loaded_path
                        load_path_time = time.time()
                        self.loaded_path_name = self.robot_object.loaded_path_name
                    if self.robot_object.activate_autonomy and not self.autonomy_disabled:
                        self.activate_autonomy = self.robot_object.activate_autonomy
                        self.autonomy_velocity = self.robot_object.autonomy_velocity
                    if not self.robot_object.activate_autonomy:
                        self.autonomy_disabled = False

                #if self.robot_object:
                #    print("REMOTE CONTROL READS AZIMUTH AS {} degrees".format(self.robot_object.location.azimuth_degrees))
                if(len(self.nav_path)>0):
                    #try:
                    if True:
                        goal, index = gps_tools.get_closest_point(self.robot_object.location, self.nav_path)
                        #print(goal)
                        goal_dist = gps_tools.get_distance(self.robot_object.location, goal)
                        if goal_dist < 3.0:
                            self.nav_path.remove(goal)
                        else:
                            self.next_nav_point = goal
                            #self.turn_intent = 0
                            self.turn_intent_degrees = gps_tools.get_heading(self.robot_object.location, self.next_nav_point)
                            self.turn_intent_degrees -= self.robot_object.location.azimuth_degrees
                            if self.turn_intent_degrees > 180:
                                self.turn_intent_degrees -= 360
                            if self.turn_intent_degrees < -180:
                                self.turn_intent_degrees += 360
                            #print("CALCULATED TURN INTENT: {}".format(self.turn_intent_degrees))
                    # except Exception as e:
                    #     print(e)
                    #     print(self.robot_object.location)
                else:
                    self.turn_intent_degrees = -180

                self.master_conn.send((self.nav_path,self.turn_intent_degrees))

                    #print(type(Robot()))
                # Get joystick value
                joy_steer, joy_throttle = get_joystick(self.joy, joy_steer, joy_throttle)
                #print(joy_throttle)
                if abs(joy_steer) > 0.1 or abs(joy_throttle) > 0.1:
                    print("DISABLED AUTONOMY Steer: {}, Joy {}".format(joy_steer, joy_throttle))
                    self.autonomy_disabled = True
                    self.activate_autonomy = False



                if self.turn_intent_degrees != -180 and self.activate_autonomy:
                    if -90 < self.turn_intent_degrees <= 0:
                        steering_angle = max(self.turn_intent_degrees * 2, -45)
                        direction = 1
                    if -180 < self.turn_intent_degrees <= -90:
                        steering_angle = max((180+self.turn_intent_degrees) * -2, -45)
                        direction = -1
                    if  0 < self.turn_intent_degrees <= 90:
                        steering_angle = min(self.turn_intent_degrees * 2, 45)
                        direction = 1
                    if  90 < self.turn_intent_degrees <= 180:
                        steering_angle = min((self.turn_intent_degrees-180) * -2, 45)
                        direction = -1
                    steer_cmd = steering_angle/45.0
                    vel_cmd = self.autonomy_velocity*direction
                    joy_steer = 0.0 # ensures that vel goes to zero when autonomy disabled
                    #print("Steer: {}, Throttle: {}".format(steer_cmd, vel_cmd))
                    if time.time() - load_path_time < 5:
                        vel_cmd = 0
                        print("ARRRRRRR")
                else:
                    vel_cmd = joy_throttle
                    steer_cmd = joy_steer
                #print(self.activate_autonomy)
                period = time.time() - tick_time
                # Perform acceleration on vel_cmd value
                vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
                last_vel_cmd = vel_cmd
                tick_time = time.time()
                #print("Final values: Steer {}, Vel {}".format(steer_cmd, vel_cmd))
                calc = calculate_steering(steer_cmd, vel_cmd)
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
