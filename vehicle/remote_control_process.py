
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


COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

ACCELERATION_COUNTS_SEC = 0.5

_GPS_DISTANCE_SCALAR = 100000


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

    def run_setup(self):
        self.connect_to_motors()
        self.connect_joystick()

    def connect_to_motors(self):
        context = zmq.Context()
        #  Socket to talk to motor control process
        self.motor_socket = context.socket(zmq.PUSH)
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
        self.autonomy_disabled = False
        self.nav_path_next_point_index = 0
        self.nav_spline = None
        try:
            while True:
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
                        # Autonomy is disabled if the joystick is used.
                        if self.autonomy_disabled:
                            self.activate_autonomy = False
                        # Reset disabled autonomy if autonomy is turned off in the command.
                        if not self.robot_object.activate_autonomy:
                            self.autonomy_disabled = False


                #if self.robot_object:
                #    print("REMOTE CONTROL READS AZIMUTH AS {} degrees".format(self.robot_object.location.azimuth_degrees))
                # if(len(self.nav_path)>0):
                #     #try:
                #     if True:
                #         #goal, index = gps_tools.get_closest_point(self.robot_object.location, self.nav_path)
                #         goal = self.nav_path[self.nav_path_next_point_index]
                #         goal_dist = gps_tools.get_distance(self.robot_object.location, goal)
                #
                #         if goal_dist < 3.0:
                #             #self.nav_path.remove(goal)
                #             self.nav_path_next_point_index += 1
                #             if self.nav_path_next_point_index >= len(self.nav_path):
                #                 self.nav_path_next_point_index = 0
                #         else:
                #             self.next_nav_point = goal
                            # self.next_point_heading = gps_tools.get_heading(self.robot_object.location, self.next_nav_point)
                            # self.next_point_heading -= self.robot_object.location.azimuth_degrees
                            # if self.next_point_heading > 180:
                            #     self.next_point_heading -= 360
                            # if self.next_point_heading < -180:
                            #     self.next_point_heading += 360
                # else:
                #     self.next_point_heading = -180






                debug_points = (None, None, None, None)
                calculated_rotation = None
                calculated_strafe = None
                if self.robot_object:
                    front = gps_tools.project_point(self.robot_object.location, self.robot_object.location.azimuth_degrees, 1.0)
                    rear = gps_tools.project_point(self.robot_object.location, self.robot_object.location.azimuth_degrees, -1.0)
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


                        # closest_front, _ = gps_tools.get_closest_point(front, self.nav_path)
                        # closest_rear, _ = gps_tools.get_closest_point(rear, self.nav_path)

                        #closest_point, closest_point_index = gps_tools.get_closest_point(self.robot_object.location, self.nav_path)

                        closest_u = self.nav_spline.closestUOnSpline(self.robot_object.location)
                        closest_point = self.nav_spline.coordAtU(closest_u)
                        #print("closest_point {}".format(closest_point))
                        spline_angle_rad = self.nav_spline.slopeRadiansAtU(closest_u)

                        path_point_heading = math.degrees(spline_angle_rad)

                        # Find the appropriate heading that is roughly tangent to the closest point.
                        # p1, p2 = gps_tools.get_closest_points_at_distance(closest_point_index, 1.0, self.nav_path)
                        # path_point_heading = gps_tools.get_heading(p1, p2)
                        #
                        # closest_front = p1
                        # closest_rear = p2


                        projected_point = gps_tools.project_point(closest_point, path_point_heading, 3.0)
                        distance_from_line = gps_tools.get_approx_distance_point_from_line(self.robot_object.location, closest_point, projected_point)

                        closest_front = projected_point
                        closest_rear = closest_point

                        # p1 = np.asarray((front.lat, front.lon))
                        # p2 = np.asarray((rear.lat, rear.lon))
                        # p3 = np.asarray((closest_front.lat, closest_front.lon))
                        # p4 = np.asarray((closest_rear.lat, closest_rear.lon))
                        #
                        #
                        # # https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points#
                        # d1 = np.cross(p2-p1, p1-p3) / np.linalg.norm(p2-p1) * -1
                        # d2 = np.cross(p2-p1, p1-p4) / np.linalg.norm(p2-p1) * -1
                        # d1 *= _GPS_DISTANCE_SCALAR
                        # d2 *= _GPS_DISTANCE_SCALAR
                        #print("robot heading {}, path heading {}".format(self.robot_object.location.azimuth_degrees, path_point_heading))
                        calculated_rotation = path_point_heading - self.robot_object.location.azimuth_degrees
                        calculated_strafe = distance_from_line

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


                        self.next_point_heading = calculated_rotation



                        #print("path heading: {}, Rotate: {}, Strafe: {}".format(path_point_heading, calculated_rotation, calculated_strafe))


                    #print("Front: {}, Rear: {}, {}, {}".format(front, rear, closest_front, closest_rear))
                    debug_points = (front, rear, closest_front, closest_rear)
                # except Exception as e:
                #     print(e)

                self.master_conn.send((self.nav_path,self.next_point_heading, debug_points))

                    #print(type(Robot()))
                # Get joystick value
                joy_steer, joy_throttle, joy_strafe = self.get_joystick_values(joy_steer, joy_throttle, joy_strafe)
                #print(joy_throttle)
                if abs(joy_steer) > 0.1 or abs(joy_throttle) > 0.1 or abs(joy_strafe) > 0.1:
                    print("DISABLED AUTONOMY Steer: {}, Joy {}".format(joy_steer, joy_throttle))
                    self.autonomy_disabled = True
                    self.activate_autonomy = False



                if self.next_point_heading != -180 and self.activate_autonomy:

                    direction = direction * self.nav_direction

                    if calculated_rotation and calculated_strafe:
                        steering_angle = calculated_rotation * 1.5
                        if steering_angle > 45:
                            steering_angle = 45
                        if steering_angle < -45:
                            steering_angle = -45
                        steer_cmd = steering_angle/45.0 * direction
                        if calculated_strafe > 1:
                            strafe = 0.5
                        if calculated_strafe < -1:
                            strafe = -0.5
                        if math.fabs(calculated_strafe) < 1.0:
                            strafe = calculated_strafe/2.0
                        strafe *= -0.5 * direction * self.nav_direction


                    vel_cmd = self.autonomy_velocity * direction# * self.nav_direction
                    joy_steer = 0.0 # ensures that vel goes to zero when autonomy disabled
                    print("calc rotation: {}, calc strafe: {}, steer: {}, strafe: {}, vel_cmd: {}".format(calculated_rotation, calculated_strafe, steer_cmd, strafe, vel_cmd))
                    #print("Steer: {}, Throttle: {}".format(steer_cmd, vel_cmd))
                    if time.time() - load_path_time < 5:
                        vel_cmd = 0
                        steer_cmd = 0
                        strafe = 0
                else:
                    vel_cmd = joy_throttle
                    steer_cmd = joy_steer
                    if math.fabs(joy_strafe) < 0.1:
                        strafe = 0
                    else:
                        strafe = math.copysign(math.fabs(joy_strafe) - 0.1, joy_strafe)
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
