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
import os
import fibre
import numpy as np

import corner_actuator
from corner_actuator import OdriveConnection
import model
from model import CORNER_NAMES

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

        self.odrive_connections = [
            OdriveConnection(name=CORNER_NAMES['front_right'], serial="335E31483536",
                             path="/dev/ttySC0", enable_steering=True, enable_traction=True),
            OdriveConnection(name=CORNER_NAMES['front_left'], serial="335B314C3536",
                             path="/dev/ttySC1", enable_steering=True, enable_traction=True),
            OdriveConnection(name=CORNER_NAMES['rear_right'], serial="3352316E3536",
                             path="/dev/ttySC2", enable_steering=True, enable_traction=True),
            OdriveConnection(name=CORNER_NAMES['rear_left'], serial="205F3882304E",
                             path="/dev/ttySC3", enable_steering=True, enable_traction=True)
        ]

        self.setup_shared_memory()

        self.odrives_connected = False
        self.motors_initialized = False
        self.steering_adjusted = False
        self.manual_control = manual_control
        self.simulated_hardware = simulated_hardware
        self.odrives = []
        self.voltages = [-1,-1,-1,-1]
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
            drive = self.odrives[index]
            print("(Use arrow keys or d if done. w/s drives wheel) Adjusting Odrive: {} with home position: {}, pot: {}".format(
                list(CORNER_NAMES)[drive.name], drive.home_position, drive.rotation_sensor_val))
            c = click.getchar()
            click.echo()
            if c == 'd':
                click.echo('Done adjusting steering.')
                self.steering_adjusted = True
                return
            elif c == 'w':
                speed += 100
                print("SPEED UP: {}".format(speed))
            elif c == 's':
                speed -= 100
                print("SPEED DOWN: {}".format(speed))
            elif c == _UP_KEYCODE:
                print("UP")
                speed = 0.0
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
                speed = 0.0
                index -= 1
                if index < 0:
                    index = len(self.odrives) - 1
            else:
                print(repr(c))
                speed = 0.0

            drive.update_actuator(0.0, speed)
            drive.sample_steering_pot()

    def connect_to_motors(self, enable_steering=True, enable_traction=True):
        for drive in self.odrive_connections:
            corner = corner_actuator.CornerActuator(GPIO=self.GPIO,
                                                    connection_definition=drive,
                                                    enable_steering=drive.enable_steering,
                                                    enable_traction=drive.enable_traction,
                                                    simulated_hardware=self.simulated_hardware,
                                                    disable_steering_limits=self.manual_control)
            self.odrives.append(corner)
        self.odrives_connected = True

    def initialize_motors(self):
        for drive in self.odrives:
            drive.print_errors(clear_errors=True)
        for drive in self.odrives:
            if drive.enable_steering:
                steering = 0.0
                if "rear" in list(CORNER_NAMES)[drive.name]:
                    drive.initialize_steering(
                        steering_flipped=True, skip_homing=self.manual_control)
                else:
                    drive.initialize_steering(
                        steering_flipped=False, skip_homing=self.manual_control)
                drive.update_actuator(steering, 0.0)
        for drive in self.odrives:
            if drive.enable_traction:
                if "rear_left" in list(CORNER_NAMES)[drive.name]:
                    drive.enable_thermistor()
                    continue
                drive.initialize_traction()
        self.motors_initialized = True

    def check_odrive_errors(self):
        for drive in self.odrives:
            try:
                drive.check_errors()
            except fibre.protocol.ChannelBrokenException as e:
                print("Exception in {} odrive.".format(list(CORNER_NAMES)[drive.name]))
                raise e
            except RuntimeError:
                print("RuntimeError in {} odrive.".format(list(CORNER_NAMES)[drive.name]))
                return True
        return False

    def communicate_message(self, state):
        try:
            if len(self.odrives) != 4:
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
                                    self.odrives[0].encoder_estimates,
                                    self.odrives[1].encoder_estimates,
                                    self.odrives[2].encoder_estimates,
                                    self.odrives[3].encoder_estimates])
            self.motor_output_values[:] = send_vals[:]
            return received_values
        except Exception as e:
            raise e

    def run_debug_control(self, enable_steering=True, enable_traction=True):
        print("RUN_MAIN_MOTORS")
        while True:
            try:
                if not self.odrives_connected:
                    self.connect_to_motors(
                        enable_steering=enable_steering, enable_traction=enable_traction)
                    print("CONNECTED TO MOTORS")
                    print(self.odrives)
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
        print("RUN_MAIN_MOTORS")
        debug_time = time.time()
        debug_tick = 0
        loop_tick = 0
        try:
            while True:
                # Due to a bug in python, closing another process with access
                # to this shared memory will close it. So if it gets closed,
                # just make it again.
                # https://github.com/python/cpython/issues/82300
                if not os.path.exists(f'/dev/shm/{self.input_shm_name}') or \
                    not os.path.exists(f'/dev/shm/{self.output_shm_name}'):
                    self.setup_shared_memory()
                if not self.odrives_connected:
                    self.communicate_message(model.MOTOR_DISCONNECTED)
                    try:
                        self.connect_to_motors()
                    except fibre.protocol.ChannelBrokenException:
                        GPIO.output(VOLT_OUT_PIN, GPIO.LOW)
                        time.sleep(5)
                        self.setup_GPIO(self.GPIO)
                elif not self.motors_initialized:
                    try:
                        self.initialize_motors()
                    except ValueError as e:
                        print("Unrecoverable error initializing steering.")
                        raise e
                    except RuntimeError:
                        print("Motor problem while initializing steering.")
                elif not self.steering_adjusted:
                    try:
                        self.steering_adjusted = True
                        if self.manual_control:
                            self.ask_if_adjust_steering()
                        print("Initialized Steering")
                    except RuntimeError:
                        print("Motor problem while adjusting steering.")


                # for vehicle_corner in self.odrives:
                #     corner_actuator.gpio_toggle(self.GPIO)
                #     try:
                #         vehicle_corner.update_voltage()
                #         vehicle_corner.update_thermistor_temperature_C()
                #     except Exception as e:
                #         print("Error in vehicle corner {}".format(
                #             list(CORNER_NAMES)[vehicle_corner.name]))
                #         raise e
                #     corner_actuator.gpio_toggle(self.GPIO)
                #     voltages.append(vehicle_corner.voltage)
                #     bus_currents.append(
                #         abs(vehicle_corner.ibus_0) + abs(vehicle_corner.ibus_1))
                #     temperatures.append(vehicle_corner.temperature_c)

                if existing_motor_error:
                    for drive in self.odrives:
                        drive.steering_error = drive.steering_axis.error
                        drive.traction_error = drive.traction_axis.error

                error_detected = False
                for drive in self.odrives:
                    # print(f"{drive.traction_error} {drive.steering_error}")
                    if drive.traction_error or drive.steering_error:
                        error_detected = True
                        break

                if error_detected:
                    if not existing_motor_error:
                        # If we have not yet set existing_motor_error, then
                        # intentionally break the e-stop loop to stop all motors.
                        time.sleep(1)
                    existing_motor_error = True
                    time.sleep(0.5)
                    corner_actuator.toggling_sleep(self.GPIO, 0.2)
                    print("ERROR DETECTED IN ODRIVES.")
                    for drive in self.odrives:
                        print("Drive: {}:".format(list(CORNER_NAMES)[drive.name]))
                        start = time.time()
                        drive.print_errors(clear_errors=True)
                        print("clear_errors_duration {}".format(
                            start - time.time()))
                    print("ERROR DETECTED IN ODRIVES.")
                    corner_actuator.toggling_sleep(
                        self.GPIO, _ERROR_RECOVERY_DELAY_S)
                    recv = self.communicate_message(model.MOTOR_DISABLED)
                    if recv is not None and len(recv) > 0 and recv[-1][0] == model.FRESH_MESSAGE:
                        print("Got motor command but motors are in error state.")
                        print("Motor command was {}".format(recv))
                else:
                    if existing_motor_error:
                        for odrive in self.odrives:
                            odrive.recover_from_estop()
                        if not self.check_odrive_errors():
                            print("Recovered from e-stop.")
                            existing_motor_error = False

                    calc = self.communicate_message(model.MOTOR_ENABLED)
                    corner_actuator.gpio_toggle(self.GPIO)

                    if calc is not None and len(calc) > 0 and calc[-1][0] == model.FRESH_MESSAGE:
                        if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                            print("Regained Motor Comms")
                        tick_time = time.time()

                        loop_tick += 1
                        if loop_tick == 6:
                            loop_tick = 0
                        update_amps = loop_tick == 3
                        update_volts = loop_tick == 5
                        update_errors = loop_tick == 1
                        # print(f"{loop_tick} UPDATE ERRORS {update_errors}")
                        # print("UPDATE ERRORS")

                        for drive in self.odrives:
                            this_pos, this_vel_cmd = calc[drive.name]
                            this_pos = math.degrees(this_pos)

                            try:
                                this_vel_cmd *= 1000
                                drive.update_actuator_async(steering_pos_deg=this_pos, drive_velocity=this_vel_cmd, update_amps=update_amps, update_volts=update_volts, update_errors=update_errors)

                            except RuntimeError as e:
                                print("Error updating actuator.")
                                print(e)
                            except AttributeError as e:
                                print("Attribute Error while updating actuator.")
                                print(e)
                                print("self.motors_initialized {}".format(
                                    self.motors_initialized))
                                print("self {}".format(self))
                                raise e

                        if update_amps:
                            self.bus_currents = []
                        if update_volts:
                            self.voltages = []
                        for drive in self.odrives:
                            if drive.retrieve_async_results(update_amps=update_amps, update_volts=update_volts, update_errors=update_errors):
                                if update_amps:
                                    self.bus_currents.append(abs(drive.ibus_0) + abs(drive.ibus_1))
                                if update_volts:
                                    self.voltages.append(drive.voltage)
                                if drive.errors > 0:
                                    drive.errors -= 1
                            else:
                                drive.errors +=1
                                if drive.errors > 5:
                                    raise RuntimeError(f"Too many consecutive read errors on drive {drive.name}")
                        debug_tick += 1
                        if time.time() - debug_time > 1.0:
                            # print(debug_tick)
                            debug_tick = 0
                            debug_time = time.time()
                    if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                        print("COMMS LOST SLOWING ACTUATOR. ~~ {}".format(
                            time.time()))
                        for drive in self.odrives:
                            try:
                                drive.slow_actuator(0.50)
                            except RuntimeError as e:
                                print("Error updating actuator.")
                                print(e)
                        corner_actuator.toggling_sleep(self.GPIO, 0.15)

        except KeyboardInterrupt:
            for drive in self.odrives:
                drive.stop_actuator()

    def setup_GPIO(self, GPIO):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(corner_actuator.ESTOP_PIN, GPIO.OUT, initial=GPIO.LOW)
        # This trick stores state for the estop line in a new variable
        # attached to our GPIO object, though it probably doesnt need to be
        # done this way.
        GPIO.estop_state = GPIO.LOW
        GPIO.setup(VOLT_OUT_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(VOLT_OUT_PIN, GPIO.LOW)
        time.sleep(1)

        if BOARD_VERSION == 1:
            for _ in range(100):
                GPIO.output(VOLT_OUT_PIN, GPIO.LOW)
                time.sleep(0.01)
                GPIO.output(VOLT_OUT_PIN, GPIO.HIGH)
                time.sleep(0.01)
        elif BOARD_VERSION == 2:
            GPIO.output(VOLT_OUT_PIN, GPIO.HIGH)


def e_stop_square_wave(GPIO):
    delay = 0.01
    while True:
        # ESTOP approx 1kHz square wave.
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
