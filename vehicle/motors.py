
import corner_actuator
from corner_actuator import COUNTS_PER_REVOLUTION
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
import argparse


_UP_KEYCODE = '\x1b[A'
_LEFT_KEYCODE = '\x1b[D'
_RIGHT_KEYCODE = '\x1b[C'
_DOWN_KEYCODE = '\x1b[B'

_STATE_DISCONNECTED = "Not connected."
_STATE_DISABLED = "Running, motors disabled."
_STATE_ENABLED = "Motors enabled"


_SHUT_DOWN_MOTORS_COMMS_DELAY_S = 1.0
_ERROR_RECOVERY_DELAY_S = 5
_ACCELERATION_COUNTS_SEC = 0.5

_USE_JOYSTICK = True



class AcornMotorInterface():

    def __init__(self, skip_homing):
        self.odrive_devices = {
        "front_right":"335E31483536",
        "front_left":"335B314C3536",
        "rear_right":"3352316E3536",
        "rear_left":"205332784D4B"
        }

        context = zmq.Context()
        self.command_socket = context.socket(zmq.PULL)
        self.command_socket.bind("tcp://*:5590")
        self.odrives_connected = False
        self.motors_initialized = False
        self.steering_adjusted = False
        self.skip_homing=skip_homing



    def ask_if_adjust_steering(self):
        click.echo('Adjust Steering? [y/n] ', nl=False)
        c = click.getchar()
        click.echo()
        if c != 'y':
            return
        index = 0
        while True:
            drive = self.odrives[index]
            print("(Use arrow keys or d if done) Adjusting Odrive: {} with home position: {}".format(drive.name, drive.home_position))
            c = click.getchar()
            click.echo()
            if c == 'd':
                click.echo('Done adjusting steering.')
                self.steering_adjusted = True
                return
            elif c == _UP_KEYCODE:
                print("UP")
                index += 1
                if index >= len(self.odrives):
                    index = 0
            elif c == _LEFT_KEYCODE:
                drive.home_position -= 10
                print("LEFT")
            elif c == _RIGHT_KEYCODE:
                drive.home_position += 10
                print("RIGHT")
            elif c == _DOWN_KEYCODE:
                print("DOWN")
                index -= 1
                if index < 0:
                   index = len(self.odrives) - 1
            else:
                print(repr(c))

            drive.update_actuator(0.0, 0.0)

    def connect_to_motors(self):
        front_right = corner_actuator.CornerActuator(serial_number=self.odrive_devices['front_right'], name='front_right', path="serial:/dev/motor4")
        front_left = corner_actuator.CornerActuator(serial_number=self.odrive_devices['front_left'], name='front_left', path="serial:/dev/motor2")
        rear_right = corner_actuator.CornerActuator(serial_number=self.odrive_devices['rear_right'], name='rear_right', path="serial:/dev/motor3")
        rear_left = corner_actuator.CornerActuator(serial_number=self.odrive_devices['rear_left'], name='rear_left', path="serial:/dev/motor1")

        self.odrives = [front_left, front_right, rear_right, rear_left]
        self.odrives_connected = True


    def initialize_motors(self):
        for drive in self.odrives:
            drive.print_errors(clear_errors=True)
        for drive in self.odrives:
            steering = 0.0
            if "rear" in drive.name:
                drive.initialize_steering(steering_flipped=True, skip_homing=self.skip_homing)
            else:
                drive.initialize_steering(steering_flipped=False, skip_homing=self.skip_homing)
            drive.update_actuator(steering, 0.0)
        for drive in self.odrives:
            drive.initialize_traction()
        self.motors_initialized = True

    def check_errors(self):
        for drive in self.odrives:
            if drive.odrv0.axis0.error or drive.odrv0.axis1.error:
                return True
        return False

    def try_to_send_state(self, state):
            try:
                self.command_socket.send_string(state)
            except zmq.error.ZMQError as e:
                pass

    def run_main(self):
        motor_error = False
        tick_time = time.time()
        try:
            while True:
                if not self.odrives_connected:
                    self.try_to_send_state(_STATE_DISCONNECTED)
                    self.connect_to_motors()
                elif not self.motors_initialized:
                    try:
                        self.initialize_motors()
                    except ValueError as e:
                        print("Unrecoverable error initializing steering.")
                        raise e
                    except RuntimeError as e:
                        print("Motor problem while initializing steering.")
                elif not self.steering_adjusted:
                    try:
                        self.steering_adjusted = True
                        self.ask_if_adjust_steering()
                        print("Initialized Steering")
                    except RuntimeError as e:
                        print("Motor problem while adjusting steering.")

                if self.check_errors():
                    motor_error = True
                    print("ERROR DETECTED IN ODRIVES.")
                    for drive in self.odrives:
                        print("Drive: {}:".format(drive.name))
                        drive.print_errors(clear_errors=True)
                    print("ERROR DETECTED IN ODRIVES.")
                    time.sleep(_ERROR_RECOVERY_DELAY_S)
                    self.try_to_send_state(_STATE_DISABLED)
                    # Consume command messages.
                    while self.command_socket.poll(timeout=0):
                        self.command_socket.recv_pyobj()
                else:
                    if motor_error:
                        for odrive in self.odrives:
                            odrive.recover_from_estop()
                        if not self.check_errors():
                            print("Recovered from e-stop.")
                            motor_error = False
                    calc = None
                    # Loop here to consume any buffered messages.
                    while self.command_socket.poll(timeout=0):
                        calc = pickle.loads(self.command_socket.recv_pyobj())
                        tick_time = time.time()
                    if calc:
                        self.try_to_send_state(_STATE_ENABLED)
                        for drive in self.odrives:
                            this_pos, this_vel_cmd = calc[drive.name]
                            this_pos = math.degrees(this_pos)
                            try:
                                drive.update_actuator(this_pos, this_vel_cmd * 1000)
                            except RuntimeError as e:
                                print("Error updating actuator.")
                                print(e)
                            except AttributeError as e:
                                print("Attribute Error while updating actuator.")
                                print(e)
                                print("self.motors_initialized {}".format(self.motors_initialized))
                                print("self {}".format(self))
                            time.sleep(0.02)

                    if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                        print("COMMS LOST SLOWING ACTUATOR.")
                        for drive in self.odrives:
                            try:
                                drive.slow_actuator(0.95)
                            except RuntimeError as e:
                                print("Error updating actuator.")
                                print(e)
                        time.sleep(0.05)

        except KeyboardInterrupt:
            for drive in self.odrives:
                drive.stop_actuator()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run the Acorn motors process.')
    parser.add_argument('--home', dest='skip_homing', default=True, action='store_false')
    args = parser.parse_args()
    control = AcornMotorInterface(args.skip_homing)
    control.run_main()
