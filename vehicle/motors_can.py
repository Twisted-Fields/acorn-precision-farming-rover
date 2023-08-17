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

import time
import math
import pickle
import click
import argparse
from multiprocessing import Process, shared_memory
import subprocess
import os
import numpy as np
import traceback

import corner_actuator_can as corner_actuator

import model
from model import CORNER_NAMES

import isotp
import motor_controller as mot



"""
TODO: lots of conversion factors in this codebase for steering and drive.
roll them all up in to one location.
"""


BYPASS_STEERING_LIMITS = True


# This file gets imported by server but we should only import GPIO on raspi.
if os.uname().machine in ['armv7l','aarch64']:
    import RPi.GPIO as GPIO


BOARD_VERSION = 2

if BOARD_VERSION==1:
    VOLT_OUT_PIN = 5
elif BOARD_VERSION==2:
    VOLT_OUT_PIN = 23


_UP_KEYCODE = '\x1b[A'
_LEFT_KEYCODE = '\x1b[D'
_RIGHT_KEYCODE = '\x1b[C'
_DOWN_KEYCODE = '\x1b[B'

_SHUT_DOWN_MOTORS_COMMS_DELAY_S = 1.0
_ERROR_RECOVERY_DELAY_S = 5
_ACCELERATION_COUNTS_SEC = 0.5


class simulated_GPIO():
    def __init__(self):
        self.estop_state = True

    def output(self, *args):
        pass


class AcornMotorInterface():

    def __init__(self, manual_control=False, simulated_hardware=False):

        self.motor_connections = [
            corner_actuator.MotorConnection(name=CORNER_NAMES['front_left'], id=0x8,
                             port="can1", enable_steering=True, enable_traction=True),
            corner_actuator.MotorConnection(name=CORNER_NAMES['front_right'], id=0x7,
                             port="can1", enable_steering=True, enable_traction=True),
            corner_actuator.MotorConnection(name=CORNER_NAMES['rear_left'], id=0x6,
                             port="can1", enable_steering=True, enable_traction=True),
            corner_actuator.MotorConnection(name=CORNER_NAMES['rear_right'], id=0x9,
                             port="can1", enable_steering=True, enable_traction=True)
        ]

        self.setup_shared_memory()

        self.motors_connected = False
        self.motors_initialized = False
        self.steering_adjusted = False

        self.manual_control = manual_control
        self.simulated_hardware = simulated_hardware
        self.motors = []
        self.voltages = [-1,-1,-1,-1]
        # self.voltages = [50.2,50.2,50.2,50.2]
        self.bus_currents = [-1,-1,-1,-1]
        self.temperatures = [-1,-1,-1,-1]

        if self.simulated_hardware:
            self.GPIO = simulated_GPIO()
        else:
            self.GPIO = GPIO
            self.setup_GPIO(self.GPIO)

    def setup_shared_memory(self):
        self.output_shm_name = 'motor_output_sharedmem'
        try:
            self.out_shm = shared_memory.SharedMemory(name=self.output_shm_name)
            print(f"Connected to existing shared memory {self.output_shm_name}")
        except FileNotFoundError:
            self.out_shm = shared_memory.SharedMemory(
                name=self.output_shm_name, create=True,
                size=model.MOTOR_SAMPLE_OUTPUT.nbytes)
            print(f"Created shared memory {self.output_shm_name}")
        self.motor_output_values = np.ndarray(model.MOTOR_SAMPLE_OUTPUT.shape,
                                         dtype=model.MOTOR_SAMPLE_OUTPUT.dtype,
                                         buffer=self.out_shm.buf)
        self.motor_output_values[:] = model.MOTOR_SAMPLE_OUTPUT[:]

        self.input_shm_name = 'motor_input_sharedmem'
        try:
            self.in_shm = shared_memory.SharedMemory(name=self.input_shm_name)
            print(f"Connected to existing shared memory {self.input_shm_name}")
        except FileNotFoundError:
            self.in_shm = shared_memory.SharedMemory(
                name=self.input_shm_name, create=True,
                size=model.MOTOR_SAMPLE_INPUT.nbytes)
            print(f"Created shared memory {self.input_shm_name}")
        self.motor_input_values = np.ndarray(model.MOTOR_SAMPLE_INPUT.shape,
                                         dtype=model.MOTOR_SAMPLE_INPUT.dtype,
                                         buffer=self.in_shm.buf)
        self.motor_input_values[:] = model.MOTOR_SAMPLE_INPUT[:]

    def reset_can_port(self, interface):
        if not self.simulated_hardware:
            subprocess.run(["ifconfig", interface, "down"])
            time.sleep(0.2)
            subprocess.run(["ifconfig", interface, "up"])
            time.sleep(0.2)

    def ask_if_adjust_steering(self):
        self.square_wave = Process(target=e_stop_square_wave, args=(GPIO,))
        self.square_wave.start()
        click.echo('Adjust Steering? [y/n] ', nl=False)
        c = click.getchar()
        click.echo()
        if c != 'y':
            self.square_wave.terminate()
            return
        index = 0
        speed = 0
        while True:
            drive = self.motors[index]
            print("(Use arrow keys or d if done. w/s drives wheel) Adjusting Motor: {} with home position: {}, pot: {}".format(
                list(CORNER_NAMES)[drive.name], drive.home_position, drive.rotation_sensor_val))
            c = click.getchar()
            click.echo()
            if c == 'd':
                click.echo('Done adjusting steering.')
                self.steering_adjusted = True
                return
            elif c == 'w':
                speed += 10
                print("SPEED UP: {}".format(speed))
            elif c == 's':
                speed -= 10
                print("SPEED DOWN: {}".format(speed))
            elif c == _UP_KEYCODE:
                print("UP")
                speed = 0.0
                index += 1
                if index >= len(self.motors):
                    index = 0
            elif c == _LEFT_KEYCODE:
                drive.home_position -= 0.02
                print("LEFT")
            elif c == _RIGHT_KEYCODE:
                drive.home_position += 0.02
                print("RIGHT")
            elif c == _DOWN_KEYCODE:
                print("DOWN")
                speed = 0.0
                index -= 1
                if index < 0:
                    index = len(self.motors) - 1
            else:
                print(repr(c))
                speed = 0.0

            drive.update_actuator(0.0, speed)
            drive.sample_sensors()

    def connect_to_motors(self, enable_steering=True, enable_traction=True):

        # print("%%%%%%%%%%%%")
        # print(self.simulated_hardware)
        # import sys
        # sys.exit()
        disable_steering_limits = False
        if BYPASS_STEERING_LIMITS or self.manual_control:
            disable_steering_limits = True
        for motor in self.motor_connections:
            corner = corner_actuator.CornerActuator(GPIO=self.GPIO,
                                                    name=motor.name,
                                                    connection_definition=motor,
                                                    enable_steering=motor.enable_steering,
                                                    enable_traction=motor.enable_traction,
                                                    simulated_hardware=self.simulated_hardware,
                                                    disable_steering_limits=disable_steering_limits)

            self.motors.append(corner)
        self.motors_connected = True

    def initialize_motors(self):
        # for drive in self.motors:
        #     drive.print_errors(clear_errors=True)
        print(self.motors)
        for drive in self.motors:
            if drive.enable_steering:
                steering = 0.0
                print(f"Initializing {list(CORNER_NAMES)[drive.name]}")
                if "rear" in list(CORNER_NAMES)[drive.name]:
                    drive.initialize_steering(
                        steering_flipped=True, skip_homing=self.manual_control)
                else:
                    drive.initialize_steering(
                        steering_flipped=False, skip_homing=self.manual_control)
                print(f"Updating {list(CORNER_NAMES)[drive.name]}")
                drive.update_actuator(steering, 0.0)
        corner_actuator.gpio_toggle(self.GPIO)
        for drive in self.motors:
            if drive.enable_traction:
                drive.initialize_traction()
        self.motors_initialized = True


    def communicate_message(self, state):
        try:
            if len(self.motors) != 4:
                print(len(self.motors))
                return
            # Set some values (MOTOR_READING, CLEAR_TO_WRITE) to notify the
            # remote control process when it is safe to read and write these
            # values.
            self.motor_output_values[0][1] = model.MOTOR_READING
            # This very short (0.5 millisecond) sleep ensures no race condition
            # between access on these values during important reads.
            time.sleep(model.MOTOR_READ_DELAY_SECONDS)
            received_values = np.copy(self.motor_input_values[:])
            # Now set this flag to STALE so we know we have read it.
            self.motor_input_values[-1] = [model.STALE_MESSAGE, 0]
            send_vals = np.array([[state,model.CLEAR_TO_WRITE,0,0],
                                    self.voltages,self.bus_currents,self.temperatures,
                                    self.motors[0].encoder_estimates,
                                    self.motors[1].encoder_estimates,
                                    self.motors[2].encoder_estimates,
                                    self.motors[3].encoder_estimates])
            self.motor_output_values[:] = send_vals[:]
            return received_values
        except Exception as e:
            raise e

    def run_debug_control(self, enable_steering=True, enable_traction=True):
        print("RUN_MAIN_MOTORS_DEBUG")
        while True:
            try:
                if not self.motors_connected:
                    self.connect_to_motors(
                        enable_steering=enable_steering, enable_traction=enable_traction)
                    print("CONNECTED TO MOTORS")
                elif not self.motors_initialized:
                    try:
                        print("BEGIN INTIALIZE MOTORS")
                        self.initialize_motors()
                        print("SUCCESS INTIALIZE MOTORS")
                    except ValueError as e:
                        print("Unrecoverable error initializing steering.")
                        raise e
                    except RuntimeError:
                        print("Motor problem while initializing steering.")
                elif not self.steering_adjusted:
                    try:
                        self.steering_adjusted = True
                        self.ask_if_adjust_steering()
                        print("Initialized Steering")
                    except RuntimeError:
                        print("Motor problem while adjusting steering.")
            except Exception as e:
                print("Exception:")
                print(e)
                time.sleep(1)

    def run_main(self):
        existing_motor_error = False
        tick_time = time.time()
        display_tick_time = time.time()
        print("RUN_MAIN_MOTORS")
        debug_time = time.time()
        debug_tick = 0
        loop_tick = 0
        self.reset_can_port("can1")
        error_count = 0
        start_time = time.time()
        try:
            while True:
                # print("loop main")
                # Due to a bug in python, closing another process with access
                # to this shared memory will close it. So if it gets closed,
                # just make it again.
                # https://github.com/python/cpython/issues/82300
                if not os.path.exists(f'/dev/shm/{self.input_shm_name}') or \
                    not os.path.exists(f'/dev/shm/{self.output_shm_name}'):
                    self.setup_shared_memory()
                if not self.motors_connected:
                    self.communicate_message(model.MOTOR_DISCONNECTED)
                    try:
                        print("Try to connect to motors.")
                        self.connect_to_motors()
                        print("Motors connected.")
                    except Exception as e:
                        print(e)
                        pass
                        raise e
                elif not self.motors_initialized:
                    try:
                        print("Try to initialize motors.")
                        self.initialize_motors()
                        print("Motors initialized.")
                    except ValueError as e:
                        print("Unrecoverable error initializing steering.")
                        raise e
                    except RuntimeError:
                        print("Motor problem while initializing steering.")
                    except OSError as e:
                        traceback.print_exc()
                        print("OSError while initializing steering.")
                elif not self.steering_adjusted:
                    try:
                        self.steering_adjusted = True
                        if self.manual_control:
                            self.ask_if_adjust_steering()
                        print("Initialized Steering")
                    except RuntimeError:
                        print("Motor problem while adjusting steering.")

                state_to_send = model.MOTOR_ENABLED
                for drive in self.motors:
                    if not drive.controller.motion_allowed:
                        state_to_send = model.MOTOR_DISABLED
                calc = self.communicate_message(state_to_send)

                corner_actuator.gpio_toggle(self.GPIO)

                if calc is not None and len(calc) > 0 and calc[-1][0] == model.FRESH_MESSAGE:
                    # print(calc)
                    if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                        print("Regained Motor Comms")
                    tick_time = time.time()

                    # print(f"{loop_tick} UPDATE ERRORS {update_errors}")
                    # print("UPDATE ERRORS")

                    reset_can = False

                    for drive in self.motors:
                        this_pos, this_vel_cmd = calc[drive.name]
                        # this_pos = math.degrees(this_pos)
                        try:
                            this_vel_cmd *= 150
                            drive.update_actuator(steering_pos_deg=this_pos, drive_velocity=this_vel_cmd)
                            drive.controller.read_error = False

                        except RuntimeError as e:
                            print("Error updating actuator.")
                            print(e)
                            drive.controller.read_error = True
                            error_count+=1
                        except AttributeError as e:
                            print("Attribute Error while updating actuator.")
                            print(e)
                            print("self.motors_initialized {}".format(
                                self.motors_initialized))
                            print("self {}".format(self))
                            drive.controller.read_error = True
                            error_count+=1
                            raise e
                        except OSError as e:
                            print("OSError while updating actuator.")
                            print("Will reset CAN port.")
                            drive.controller.read_error = True
                            error_count+=1
                            reset_can = True

                    if time.time() - display_tick_time > 0.5:
                        display_tick_time = time.time()
                        motor_state = "ENABLED"
                        for drive in self.motors:
                            if not drive.controller.motion_allowed:
                                motor_state = "STOPPED"
                            if drive.controller.read_error:
                                print(f"|  ID:{drive.controller.id} --------------- |", end='')
                            else:
                                print(f"| ID:{drive.controller.id}, {drive.controller.voltage:.2f}, {drive.controller.therm_bridge1}, {drive.controller.therm_bridge2} |", end='')
                        runtime = time.time() - start_time
                        print(f" | ERRORS: {error_count} | time: {int(runtime/60)}:{int(runtime%60):02d} | {motor_state}")
                        for drive in self.motors:
                            if len(drive.controller.log_messages) > 0:
                                print(drive.controller.log_messages)
                            while len(drive.controller.log_messages) > 10:
                                drive.controller.log_messages.pop(0)
                    if reset_can:
                        print("Setting velocity to zero before resetting CAN port.")
                        for drive in self.motors:
                            this_pos, this_vel_cmd = calc[drive.name]
                            this_vel_cmd = 0
                            try:
                                drive.update_actuator(steering_pos_deg=this_pos, drive_velocity=this_vel_cmd)
                            except:
                                pass
                        print("Resetting can1")
                        self.reset_can_port("can1")
                    self.voltages = []
                    for drive in self.motors:
                        self.voltages.append(drive.controller.voltage)


                if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                    print("COMMS LOST SLOWING ACTUATOR. ~~ {}".format(
                        time.time()))
                    for drive in self.motors:
                        try:
                            drive.slow_actuator(0.50)
                        except RuntimeError as e:
                            print("Error updating actuator.")
                            print(e)
                        except OSError as e:
                            print("Error updating actuator.")
                            print(e)
                    corner_actuator.toggling_sleep(self.GPIO, 0.15)





        except KeyboardInterrupt:
            for drive in self.motors:
                drive.stop_actuator()

    def setup_GPIO(self, GPIO):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(corner_actuator.ESTOP_PIN, GPIO.OUT, initial=GPIO.LOW)
        # This trick stores state for the estop line in a new variable
        # attached to our GPIO object, though it probably doesnt need to be
        # done this way.
        GPIO.estop_state = GPIO.LOW


def e_stop_square_wave(GPIO):
    delay = 0.01
    while True:
        # ESTOP approx 100Hz square wave.
        time.sleep(delay)
        GPIO.output(corner_actuator.ESTOP_PIN, GPIO.LOW)
        time.sleep(delay)
        GPIO.output(corner_actuator.ESTOP_PIN, GPIO.HIGH)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run the Acorn motors process.')
    parser.add_argument('--manual', dest='manual_control',
                        default=False, action='store_true')
    parser.add_argument('--simulated_hardware',
                        dest='simulated_hardware', default=False, action='store_true')
    args = parser.parse_args()
    control = AcornMotorInterface(args.manual_control, args.simulated_hardware)
    control.run_main()
