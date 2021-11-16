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

import serial
import time
import sys
import math
from odrive.utils import dump_errors
from evdev import InputDevice, list_devices, categorize, ecodes, KeyEvent
import pygame as py
from collections import namedtuple
import numpy as np
from multiprocessing import shared_memory, resource_tracker

COUNTS_PER_REVOLUTION = 9797.0

ACCELERATION_COUNTS_SEC = 0.5


WINDOW_SCALING = 0.75

# define constants
WIDTH = int(1000 * WINDOW_SCALING)
HEIGHT = int(1000 * WINDOW_SCALING)
FPS = 30

# define colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
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

# # define a surface (RECTANGLE)
# image_orig = py.Surface((50 , 200))
# # for making transparent background while rotating an image
# image_orig.set_colorkey(BLACK)
# # fill the rectangle / surface with green color
# image_orig.fill(GREEN)
#
# py.draw.circle(image_orig, RED, (25,20), 15, 0)
# creating a copy of orignal image for smooth rotation


image_red_orig = py.Surface((30 * WINDOW_SCALING, 100 * WINDOW_SCALING))
# for making transparent background while rotating an image
image_red_orig.set_colorkey(BLACK)
# fill the rectangle / surface with green color
image_red_orig.fill(RED)

py.draw.circle(image_red_orig, GREEN, (15 * WINDOW_SCALING,
                                       20 * WINDOW_SCALING), 10 * WINDOW_SCALING, 0)


keys = ('front_left', 'front_right', 'rear_left', 'rear_right')

pos = ((-1, -1), (+1, -1), (-1, +1), (+1, +1))

# rects = []
#
# for i in range(4):
#     image = image_orig.copy()
#     image.set_colorkey(BLACK)
#     # define rect for placing the rectangle at the desired position
#     rect = image.get_rect()
#     rect.center = (WIDTH // 2 + pos[i][0] * _RW, HEIGHT // 2 + pos[i][1] * _RH)
#     rects.append(rect)


red_rects = []

for i in range(4):
    image = image_red_orig.copy()
    image.set_colorkey(BLACK)
    # define rect for placing the rectangle at the desired position
    rect = image.get_rect()
    rect.center = (WIDTH // 2 + pos[i][0] * _RW, HEIGHT // 2 + pos[i][1] * _RH)
    red_rects.append(rect)
# keep rotating the rectangle until running is set to False

joy_steer = 0
joy_throttle = 0
joy_strafe = 0
vel_cmd = 0
steer_cmd = 0
last_vel_cmd = 0
tick_time = time.time()

strafe = 0

myFont = py.font.SysFont("Times New Roman", int(40 * WINDOW_SCALING))

existing_shm = shared_memory.SharedMemory(name='acorn_steering_debug')
# Untrack the resource so it does not get destroyed. This allows the
# steering debug window to stay open.
resource_tracker.unregister(existing_shm._name, 'shared_memory')
calc = np.ndarray((8,), dtype=np.float64, buffer=existing_shm.buf)

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

    # while True:
    #     print(calc)
    #     time.sleep(0.1)
    # # calc = calculate_steering(steer_cmd, vel_cmd)

    horiz = WIDTH//2
    coord1 = (horiz + 200 * WINDOW_SCALING, 50 * WINDOW_SCALING)
    coord2 = (horiz - 300 * WINDOW_SCALING, HEIGHT-50 * WINDOW_SCALING)
    py.draw.circle(screen, RED, coord1, 15 * WINDOW_SCALING, 0)
    py.draw.circle(screen, RED, coord2, 15 * WINDOW_SCALING, 0)

    py.draw.line(screen, RED, coord1, coord2, 1)

    coord3 = (horiz, 350 * WINDOW_SCALING)
    coord4 = (horiz, HEIGHT-350 * WINDOW_SCALING)
    py.draw.circle(screen, GREEN, coord3, 15 * WINDOW_SCALING, 0)
    py.draw.circle(screen, GREEN, coord4, 15 * WINDOW_SCALING, 0)

    p1 = np.asarray(coord1)
    p2 = np.asarray(coord2)
    p3 = np.asarray(coord3)
    p4 = np.asarray(coord4)

    # https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points#
    d1 = np.cross(p2-p1, p1-p3) / np.linalg.norm(p2-p1) * -1
    d2 = np.cross(p2-p1, p1-p4) / np.linalg.norm(p2-p1) * -1
    #d1 = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
    #d2 = np.linalg.norm(np.cross(p2-p1, p1-p4))/np.linalg.norm(p2-p1)

    coord3_p2 = (coord3[0] + d1, coord3[1])
    py.draw.line(screen, RED, coord3, coord3_p2, 5)

    coord4_p2 = (coord4[0] + d2, coord4[1])
    py.draw.line(screen, RED, coord4, coord4_p2, 5)

    idx = 0
    for rect in red_rects:
        # making a copy of the old center of the rectangle

        throttle = math.degrees(calc[idx*2+1])
        throttle_text = myFont.render("{:0.0f}".format(throttle), 1, RED)

        width = WIDTH // 2 + pos[idx][0] * _RW*2.0
        height = HEIGHT // 2 + pos[idx][1] * _RH

        screen.blit(throttle_text, (width, height-30))

        old_center = rect.center

        rot = math.degrees(calc[idx*2]) * -1
        idx += 1
        # rotating the orignal image
        new_image = py.transform.rotate(image_red_orig, rot)
        rect = new_image.get_rect()
        # set the rotated rectangle to the old center
        rect.center = old_center
        # drawing the rotated rectangle to the screen
        screen.blit(new_image, rect)
    # flipping the display after drawing everything
    py.display.flip()

# existing_shm.close()
py.quit()
