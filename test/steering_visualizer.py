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


import sys
sys.path.append('../vehicle')
import numpy as np
from collections import namedtuple
import pygame as py
import serial
import time
import math
from evdev import InputDevice, list_devices, categorize, ecodes, KeyEvent
from steering import calculate_steering, recalculate_steering_values
import redis
import pickle
import gps_tools

COUNTS_PER_REVOLUTION = 9797.0

ACCELERATION_COUNTS_SEC = 0.5


_JOYSTICK = 0
_DATABASE = 1

_DATA_SOURCE = _DATABASE



if _DATA_SOURCE == _DATABASE:
    r = redis.Redis(
        host='localhost',
        port=6379)

    robot_key = ""

    for key in r.scan_iter():
        if 'twistedfields:robot:acorn1:key' in str(key):
            robot_key = key
    robot = pickle.loads(r.get(robot_key))
    print(robot.steering_debug[0])


def rotate_origin_clockwise(xy, radians):
    """Only rotate a point around the origin (0, 0)."""
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return xx, yy

def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel-last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel-last_vel
    else:
        increment = math.copysign(
            ACCELERATION_COUNTS_SEC, unfiltered_vel-last_vel) * period_s
    return last_vel + increment


def get_joystick(joy, st_old, th_old, stf_old):
    steer = None
    throttle = None
    strafe = None
    count = 0
    # print("Enter_joy")
    while True:
        event = joy.read_one()
        # print(event)
        if event and event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            # print(ecodes.bytype[absevent.event.type][absevent.event.code])
            if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_RX':
                steer = absevent.event.value / 32768.0
                # print("ABS_X: {}".format(absevent.event.value))
            #
            if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_Y':
                throttle = -absevent.event.value / 32768.0

            if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_X':
                strafe = -absevent.event.value / 32768.0
                #print(strafe)
        count += 1
        if steer and throttle and strafe:
            break
        if count > 12:
            break
    if not steer:
        steer = st_old
    if not throttle:
        throttle = th_old
    if not strafe:
        strafe = stf_old
    # print("Exit_joy")
    return steer, throttle, strafe


class RemoteControl():

    def __init__(self):
        self.joy = None

    def run_setup(self):
        self.get_joystick()

    def get_joystick(self):
        devices = [InputDevice(fn) for fn in list_devices()]
        for dev in devices:
            if "Microsoft" in dev.name:
                self.joy = dev
                return
            if "Logitech" in dev.name:
                self.joy = dev
                return


if _DATA_SOURCE == _JOYSTICK:
    remote_control = RemoteControl()
    remote_control.run_setup()

WINDOW_SCALING = 1.0

# define constants
WIDTH = int(1000 * WINDOW_SCALING)
HEIGHT = int(1000 * WINDOW_SCALING)
FPS = 30

# define colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

_RW = int(150 * WINDOW_SCALING)
_RH = int(300 * WINDOW_SCALING)

# initialize pygame and create screen
py.init()
screen = py.display.set_mode((WIDTH, HEIGHT))
# for setting FPS
clock = py.time.Clock()

rot = 0
rot_speed = 1

# define a surface (RECTANGLE)
image_orig = py.Surface((50, 200))
# for making transparent background while rotating an image
image_orig.set_colorkey(BLACK)
# fill the rectangle / surface with green color
image_orig.fill(GREEN)

py.draw.circle(image_orig, RED, (25, 20), 15, 0)
# creating a copy of orignal image for smooth rotation


image_forward_orig = py.Surface((50, 200))
# for making transparent background while rotating an image
image_forward_orig.set_colorkey(BLACK)
# fill the rectangle / surface with green color
image_forward_orig.fill(RED)

py.draw.circle(image_forward_orig, GREEN, (25, 20), 15, 0)

image_reverse_orig = image_forward_orig.copy()

py.draw.circle(image_forward_orig, BLUE, (25, 60), 10, 0)

py.draw.circle(image_reverse_orig, BLUE, (25, 200 - 60), 10, 0)

keys = ('front_left', 'front_right', 'rear_left', 'rear_right')

pos = ((-1, -1), (+1, -1), (-1, +1), (+1, +1))


centers = []

for i in range(4):
    center = (WIDTH // 2 + pos[i][0] * _RW, HEIGHT // 2 + pos[i][1] * _RH)
    centers.append(center)


joy_steer = 0
joy_throttle = 0
joy_strafe = 0
vel_cmd = 0
steer_cmd = 0
last_vel_cmd = 0
tick_time = time.time()
last_calc = calculate_steering(0, 0, 0, 120)

strafe = 0

myFont = py.font.SysFont("Times New Roman", 40)

running = True
while running:
    # set FPS
    clock.tick(FPS)
    # clear the screen every time before drawing new objects
    screen.fill(BLACK)
    # check for the exit
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False

    period = time.time() - tick_time

    if _DATA_SOURCE == _JOYSTICK:
        # Get joystick value
        joy_steer, joy_throttle, joy_strafe = get_joystick(
            remote_control.joy, joy_steer, joy_throttle, joy_strafe)

        vel_cmd = joy_throttle
        steer_cmd = joy_steer

        # Perform acceleration on vel_cmd value
        #vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
        last_vel_cmd = vel_cmd
        tick_time = time.time()

        if math.fabs(joy_strafe) < 0.1:
            strafe_cmd = 0
        else:
            strafe_cmd = math.copysign(math.fabs(joy_strafe) - 0.1, joy_strafe)

        strafe_cmd *= -1


    if _DATA_SOURCE == _DATABASE:

        try:
            robot = pickle.loads(r.get(robot_key))
        except:
            try:
                r = redis.Redis(
                    host='localhost',
                    port=6379)
            except:
                pass
        try:
            calc, steer_cmd, vel_cmd, strafe_cmd = robot.steering_debug

            vehicle_front, vehicle_rear, projected_path_tangent_point, closest_path_point = robot.debug_points

            heading = gps_tools.get_heading(vehicle_rear, vehicle_front)
            center = (vehicle_front.lat + vehicle_rear.lat)/2, (vehicle_front.lon + vehicle_rear.lon)/2
            projected_path_tangent_point = (center[0] - projected_path_tangent_point.lat) * 100000, (center[1] - projected_path_tangent_point.lon) * 100000
            closest_path_point = (center[0] - closest_path_point.lat) * 100000, (center[1] - closest_path_point.lon) * 100000
            projected_path_tangent_point = rotate_origin_clockwise(projected_path_tangent_point, math.radians(90  + heading))
            closest_path_point = rotate_origin_clockwise(closest_path_point, math.radians(90 + heading))

            factor = 250 * WINDOW_SCALING

            projected_path_tangent_point = int(WIDTH // 2 - projected_path_tangent_point[0] * factor), int(HEIGHT // 2 - projected_path_tangent_point[1] * factor)
            closest_path_point = int(WIDTH // 2 - closest_path_point[0] * factor), int(HEIGHT // 2 - closest_path_point[1] * factor)
            py.draw.circle(screen, GREEN, projected_path_tangent_point, 20, 0)
            py.draw.circle(screen, GREEN, closest_path_point, 20, 0)
            py.draw.circle(screen, GREEN, (WIDTH // 2, HEIGHT // 2), 5, 0)
        except Exception as e:
            print(e)
            continue




    rect_dim = 150 * WINDOW_SCALING

    if steer_cmd < 0:
        start_angle = -math.pi/2.0
        stop_angle = -1 * steer_cmd * math.pi - math.pi/2.0
    else:
        stop_angle = -math.pi/2.0
        start_angle = -1 * steer_cmd * math.pi - math.pi/2.0

    left = WIDTH // 2 - rect_dim/2
    top = HEIGHT // 2 - rect_dim/2

    rect = py.Rect(left, top, rect_dim, rect_dim)
    steer_arc = py.draw.arc(screen, RED, rect, start_angle, stop_angle, 4)

    # print(steer_cmd)
    # strafe_cmd = strafe

    py.draw.line(screen, RED, (WIDTH // 2, HEIGHT // 2), (WIDTH // 2 + 1500 * strafe_cmd, HEIGHT // 2 - 1500 * vel_cmd), 5)

    #print("steer_cmd {}, vel_cmd {}, strafe_cmd {}".format(steer_cmd, vel_cmd, strafe_cmd))

    if _DATA_SOURCE == _JOYSTICK:
        calc = calculate_steering(steer_cmd, vel_cmd, strafe_cmd)
        calc = recalculate_steering_values(calc, last_calc)

    idx = 0
    last_calc = calc
    for center in centers:
        # making a copy of the old center of the rectangle

        throttle = math.degrees(calc[keys[idx]][1])
        diceDisplay = myFont.render("{:0.0f}".format(throttle), 1, RED)

        width = WIDTH // 2 + pos[idx][0] * _RW*1.5
        height = HEIGHT // 2 + pos[idx][1] * _RH

        screen.blit(diceDisplay, (width, height-30))

        rot = math.degrees(calc[keys[idx]][0]) * -1
        idx += 1
        # rotating the original image
        if throttle > 0:
            new_image = py.transform.rotate(image_forward_orig, rot)
        else:
            new_image = py.transform.rotate(image_reverse_orig, rot)
        rect = new_image.get_rect()
        # set the rotated rectangle to the old center
        rect.center = center
        # drawing the rotated rectangle to the screen
        screen.blit(new_image, rect)
    # flipping the display after drawing everything
    py.display.flip()

py.quit()
