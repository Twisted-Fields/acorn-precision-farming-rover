
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

COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

ACCELERATION_COUNTS_SEC = 0.5

context = zmq.Context()

#  Socket to talk to server
socket = context.socket(zmq.PUSH)
socket.connect("tcp://localhost:5555")

joy = None
devices = [InputDevice(fn) for fn in list_devices()]
for dev in devices:
    if "Microsoft" in dev.name:
        joy = dev
        break
    if "Logitech" in dev.name:
        joy = dev
        break
if not joy:
    print("CONTROLLER NOT FOUND")
    sys.exit()


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

joy_steer = 0
joy_throttle = 0
vel_cmd = 0
last_vel_cmd = 0

# A sine wave to test
tick_time = time.time()
try:
    while True:
        # Get joystick value
        joy_steer, joy_throttle = get_joystick(joy, joy_steer, joy_throttle)
        print("{} {}".format(joy_throttle, joy_steer))
        vel_cmd = joy_throttle
        period = time.time() - tick_time
        # Perform acceleration on joystick value
        vel_cmd = get_profiled_velocity(last_vel_cmd, vel_cmd, period)
        last_vel_cmd = vel_cmd
        #print(vel_cmd)
        tick_time = time.time()
        calc = calculate_steering(joy_steer, vel_cmd)
        socket.send_pyobj(pickle.dumps(calc))

        time.sleep(0.1)
except KeyboardInterrupt:
    pass
