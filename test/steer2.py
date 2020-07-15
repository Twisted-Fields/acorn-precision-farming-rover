
import serial
import time
import sys
import math
from odrive.utils import dump_errors
from evdev import InputDevice, list_devices, categorize, ecodes, KeyEvent
from steering import calculate_steering, calculate_steering2
import pygame as py
from collections import namedtuple
import numpy as np

COUNTS_PER_REVOLUTION = 9797.0

ACCELERATION_COUNTS_SEC = 0.5

def get_profiled_velocity(last_vel, unfiltered_vel, period_s):
    if math.fabs(unfiltered_vel-last_vel) < ACCELERATION_COUNTS_SEC * period_s:
        increment = unfiltered_vel-last_vel
    else:
        increment = math.copysign(ACCELERATION_COUNTS_SEC, unfiltered_vel-last_vel) * period_s
    return last_vel + increment

def get_joystick(joy, st_old, th_old, stf_old):
    steer = None
    throttle = None
    strafe = None
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

            if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_X':
                strafe = -absevent.event.value / 32768.0
                print(strafe)
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
    #print("Exit_joy")
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


#if __name__=="__main__":
remote_control = RemoteControl()
remote_control.run_setup()

# define constants
WIDTH = 2000
HEIGHT = 2000
FPS = 30

# define colors
BLACK = (0 , 0 , 0)
GREEN = (0 , 255 , 0)
RED = (255 , 0 , 0)

_RW = 300
_RH = 500

# initialize pygame and create screen
py.init()
screen = py.display.set_mode((WIDTH , HEIGHT))
# for setting FPS
clock = py.time.Clock()

rot = 0
rot_speed = 1

# define a surface (RECTANGLE)
image_orig = py.Surface((50 , 200))
# for making transparent background while rotating an image
image_orig.set_colorkey(BLACK)
# fill the rectangle / surface with green color
image_orig.fill(GREEN)

py.draw.circle(image_orig, RED, (25,20), 15, 0)
# creating a copy of orignal image for smooth rotation



image_red_orig = py.Surface((50 , 200))
# for making transparent background while rotating an image
image_red_orig.set_colorkey(BLACK)
# fill the rectangle / surface with green color
image_red_orig.fill(RED)

py.draw.circle(image_red_orig, GREEN, (25,20), 15, 0)


keys = ('front_left', 'front_right', 'rear_left', 'rear_right')


pos = ((-1, +1), (+1, +1), (-1, -1), (+1, -1))

rects = []

for i in range(4):
    image = image_orig.copy()
    image.set_colorkey(BLACK)
    # define rect for placing the rectangle at the desired position
    rect = image.get_rect()
    rect.center = (WIDTH // 2 + pos[i][0] * _RW, HEIGHT // 2 + pos[i][1] * _RH)
    rects.append(rect)


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


    # Get joystick value
    joy_steer, joy_throttle, joy_strafe = get_joystick(remote_control.joy, joy_steer, joy_throttle, joy_strafe)


    vel_cmd = joy_throttle
    steer_cmd = joy_steer
    #print(self.activate_autonomy)
    period = time.time() - tick_time
    # Perform acceleration on vel_cmd value
    #vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
    last_vel_cmd = vel_cmd
    tick_time = time.time()
    #print("Final values: Steer {}, Vel {}".format(steer_cmd, vel_cmd))
    calc = calculate_steering(steer_cmd, vel_cmd)
    #print(calc)

        #print("Warning: Motor Control pipe full, or other ZMQ error raised.")

    idx = 0
    for rect in rects:
        # making a copy of the old center of the rectangle
        old_center = rect.center

        throttle = math.degrees(calc[keys[idx]][1])
        diceDisplay = myFont.render("{:0.0f}".format(throttle), 1, GREEN)


        width = WIDTH // 2 + pos[idx][0] * _RW*1.5
        height = HEIGHT // 2 + pos[idx][1] * _RH

        screen.blit(diceDisplay, (width, height))

        rot = math.degrees(calc[keys[idx]][0])
        idx += 1
        # rotating the orignal image
        new_image = py.transform.rotate(image_orig , rot)
        rect = new_image.get_rect()
        # set the rotated rectangle to the old center
        rect.center = old_center
        # drawing the rotated rectangle to the screen
        screen.blit(new_image , rect)
    idx = 0


    if math.fabs(joy_strafe) < 0.1:
        strafe = 0
    else:
        strafe = math.copysign(math.fabs(joy_strafe) - 0.1, joy_strafe)

    horiz = WIDTH//2
    coord1 = (horiz + 200, 50)
    coord2 = (horiz - 300, HEIGHT-50)
    py.draw.circle(screen, RED, coord1, 15, 0)
    py.draw.circle(screen, RED, coord2, 15, 0)

    py.draw.line(screen, RED, coord1, coord2, 1)

    coord3 = (horiz, 350)
    coord4 = (horiz, HEIGHT-350)
    py.draw.circle(screen, GREEN, coord3, 15, 0)
    py.draw.circle(screen, GREEN, coord4, 15, 0)


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


    strafe *= vel_cmd
    calc2 = calculate_steering2(steer_cmd, vel_cmd, strafe)
    for rect in red_rects:
        # making a copy of the old center of the rectangle


        throttle = math.degrees(calc2[keys[idx]][1])
        diceDisplay = myFont.render("{:0.0f}".format(throttle), 1, RED)


        width = WIDTH // 2 + pos[idx][0] * _RW*1.5
        height = HEIGHT // 2 + pos[idx][1] * _RH

        screen.blit(diceDisplay, (width, height-30))


        old_center = rect.center

        rot = math.degrees(calc2[keys[idx]][0])
        idx += 1
        # rotating the orignal image
        new_image = py.transform.rotate(image_red_orig , rot)
        rect = new_image.get_rect()
        # set the rotated rectangle to the old center
        rect.center = old_center
        # drawing the rotated rectangle to the screen
        screen.blit(new_image , rect)
    # flipping the display after drawing everything
    py.display.flip()

py.quit()
