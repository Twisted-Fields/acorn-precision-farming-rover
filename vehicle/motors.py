
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
import click


UP_KEYCODE = '\x1b[A'
LEFT_KEYCODE = '\x1b[D'
RIGHT_KEYCODE = '\x1b[C'
DOWN_KEYCODE = '\x1b[B'


COUNTS_PER_REVOLUTION = corner_actuator.COUNTS_PER_REVOLUTION

SHUT_DOWN_MOTORS_COMMS_DELAY_S = 1.0

ACCELERATION_COUNTS_SEC = 0.5

_USE_JOYSTICK = True

odrive_devices = {
"front_right":"335E31483536",
"front_left":"335B314C3536",
"rear_right":"3352316E3536",
"rear_left":"205332784D4B"
}

context = zmq.Context()
command_socket = context.socket(zmq.PULL)
command_socket.bind("tcp://*:5555")

print(odrive_devices)

front_right = corner_actuator.CornerActuator(serial_number=odrive_devices['front_right'], name='front_right')
front_left = corner_actuator.CornerActuator(serial_number=odrive_devices['front_left'], name='front_left')
rear_right = corner_actuator.CornerActuator(serial_number=odrive_devices['rear_right'], name='rear_right')
rear_left = corner_actuator.CornerActuator(serial_number=odrive_devices['rear_left'], name='rear_left')

odrives = [front_left, front_right, rear_right, rear_left]
#odrives = [rear_left]

skip_homing = True

for drive in odrives:
    drive.print_errors(clear_errors=True)
for drive in odrives:
    steering = 0.0
    if "rear" in drive.name:
        drive.initialize_steering(steering_flipped=True, skip_homing=skip_homing)
    else:
        drive.initialize_steering(steering_flipped=False, skip_homing=skip_homing)
    drive.update_actuator(steering, 0.0)
for drive in odrives:
    drive.initialize_traction()



def adjust_steering():
    if input('Adjust Steering? y/n:') == 'y':
        while True:
            print(nput("press key"))



def adjust_steering(odrives):
    click.echo('Adjust Steering? [y/n] ', nl=False)
    c = click.getchar()
    click.echo()
    if c != 'y':
        return
    index = 0
    while True:
        drive = odrives[index]
        print("Adjusting Odrive: {} with home position: {}".format(drive.name, drive.home_position))
        c = click.getchar()
        click.echo()
        if c == 'y':
            click.echo('We will go on')
        elif c == 'd':
            click.echo('Done adjusting steering.')
            return
        elif c == UP_KEYCODE:
            print("UP")
            index += 1
            if index >= len(odrives):
                index = 0
        elif c == LEFT_KEYCODE:
            drive.home_position -= 10
            print("LEFT")
        elif c == RIGHT_KEYCODE:
            drive.home_position += 10
            print("RIGHT")
        elif c == DOWN_KEYCODE:
            print("DOWN")
            index -= 1
            if index < 0:
               index = len(odrives) - 1
        else:
            print(repr(c))

        drive.update_actuator(0.0, 0.0)

adjust_steering(odrives)

print("Initialized Steering")

tick_time = time.time()
try:
    while True:

        calc = None
        while command_socket.poll(timeout=0):
            calc = pickle.loads(command_socket.recv_pyobj())
            tick_time = time.time()

        if calc:
            for drive in odrives:
                this_pos, this_vel_cmd = calc[drive.name]
                this_pos = math.degrees(this_pos)
                drive.update_actuator(this_pos, this_vel_cmd * 1000)
                time.sleep(0.02)

        error_found = False
        for drive in odrives:
            if drive.odrv0.axis0.error or drive.odrv0.axis1.error:
                error_found = True
        if error_found:
            print("ERROR DETECTED IN ODRIVES.")
            for drive in odrives:
                print("Drive: {}:".format(drive.name))
                drive.print_errors(clear_errors=True)
            print("ERROR DETECTED IN ODRIVES.")

        if time.time() - tick_time > SHUT_DOWN_MOTORS_COMMS_DELAY_S:
            print("COMMS LOST SLOWING ACTUATOR.")
            for drive in odrives:
                drive.slow_actuator(0.95)
            time.sleep(0.05)

except KeyboardInterrupt:
    for drive in odrives:
        drive.stop_actuator()
    pass
