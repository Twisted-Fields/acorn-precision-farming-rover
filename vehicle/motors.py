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
import zmq
import pickle
import click
import argparse
from multiprocessing import Process
import os
import fibre

import corner_actuator
from corner_actuator import OdriveConnection
import model


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
            OdriveConnection(name='front_right', serial="335E31483536",
                             path="/dev/ttySC0", enable_steering=True, enable_traction=True),
            OdriveConnection(name='front_left', serial="335B314C3536",
                             path="/dev/ttySC1", enable_steering=True, enable_traction=True),
            OdriveConnection(name='rear_right', serial="3352316E3536",
                             path="/dev/ttySC2", enable_steering=True, enable_traction=True),
            OdriveConnection(name='rear_left', serial="205F3882304E",
                             path="/dev/ttySC3", enable_steering=True, enable_traction=True)
        ]

        self.command_socket = self.create_socket()
        self.odrives_connected = False
        self.motors_initialized = False
        self.steering_adjusted = False
        self.manual_control = manual_control
        self.simulated_hardware = simulated_hardware

        if self.simulated_hardware:
            self.GPIO = simulated_GPIO()
        else:
            self.GPIO = GPIO
            self.setup_GPIO(self.GPIO)

    def create_socket(self, port=5590):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:{}".format(port))
        return socket

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
            print("(Use arrow keys or d if done) Adjusting Odrive: {} with home position: {}".format(
                drive.name, drive.home_position))
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

    def connect_to_motors(self, enable_steering=True, enable_traction=True):
        self.odrives = []
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
                if "rear" in drive.name:
                    drive.initialize_steering(
                        steering_flipped=True, skip_homing=self.manual_control)
                else:
                    drive.initialize_steering(
                        steering_flipped=False, skip_homing=self.manual_control)
                drive.update_actuator(steering, 0.0)
        for drive in self.odrives:
            if drive.enable_traction:
                if "rear_left" in drive.name:
                    drive.enable_thermistor()
                    continue
                drive.initialize_traction()
        self.motors_initialized = True

    def check_odrive_errors(self):
        for drive in self.odrives:
            try:
                drive.check_errors()
            except fibre.protocol.ChannelBrokenException as e:
                print("Exception in {} odrive.".format(drive.name))
                raise e
            except RuntimeError:
                print("RuntimeError in {} odrive.".format(drive.name))
                return True
        return False

    def communicate_message(self, state, voltages=None, ibus=None, temperatures=None):
        # print("TRYING TO SEND STATE: {}, Voltages {}".format(state, voltages))
        try:
            corner_actuator.gpio_toggle(self.GPIO)
            while self.command_socket.poll(timeout=20):
                corner_actuator.gpio_toggle(self.GPIO)
                recv = pickle.loads(self.command_socket.recv_pyobj())
                self.command_socket.send_pyobj(pickle.dumps(
                    (state, voltages, ibus, temperatures)), flags=zmq.DONTWAIT)
                corner_actuator.gpio_toggle(self.GPIO)
                return recv
            # print("no incoming messages")
        except zmq.error.ZMQError as e:
            print("Error with motor command socket: {}".format(e))
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
        motor_error = False
        tick_time = time.time()
        print("RUN_MAIN_MOTORS")
        try:
            while True:
                if not self.odrives_connected:
                    self.communicate_message(model.MOTOR_DISCONNECTED)
                    self.connect_to_motors()
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

                voltages = []
                bus_currents = []
                temperatures = []
                for vehicle_corner in self.odrives:
                    corner_actuator.gpio_toggle(self.GPIO)
                    try:
                        vehicle_corner.update_voltage()
                        vehicle_corner.update_thermistor_temperature_C()
                    except Exception as e:
                        print("Error in vehicle corner {}".format(
                            vehicle_corner.name))
                        raise e
                    corner_actuator.gpio_toggle(self.GPIO)
                    voltages.append(vehicle_corner.voltage)
                    bus_currents.append(
                        abs(vehicle_corner.ibus_0) + abs(vehicle_corner.ibus_1))
                    temperatures.append(vehicle_corner.temperature_c)


                if self.check_odrive_errors():
                    if not motor_error:
                        # Intentionally break the e-stop loop to stop all motors.
                        time.sleep(1)
                    motor_error = True
                    time.sleep(0.5)
                    corner_actuator.toggling_sleep(self.GPIO, 0.2)
                    print("ERROR DETECTED IN ODRIVES.")
                    for drive in self.odrives:
                        print("Drive: {}:".format(drive.name))
                        start = time.time()
                        drive.print_errors(clear_errors=True)
                        print("clear_errors_duration {}".format(
                            start - time.time()))
                    print("ERROR DETECTED IN ODRIVES.")
                    corner_actuator.toggling_sleep(
                        self.GPIO, _ERROR_RECOVERY_DELAY_S)
                    # time.sleep(_ERROR_RECOVERY_DELAY_S)
                    recv = self.communicate_message(
                        model.MOTOR_DISABLED, voltages, bus_currents, temperatures)
                    if recv:
                        print("Got motor command but motors are in error state.")
                        print("Motor command was {}".format(recv))
                else:
                    if motor_error:
                        for odrive in self.odrives:
                            odrive.recover_from_estop()
                        if not self.check_odrive_errors():
                            print("Recovered from e-stop.")
                            motor_error = False

                    calc = self.communicate_message(
                        model.MOTOR_ENABLED, voltages, bus_currents, temperatures)
                    corner_actuator.gpio_toggle(self.GPIO)
                    if calc:
                        if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                            print("Regained Motor Comms")
                        tick_time = time.time()
                        for drive in self.odrives:
                            this_pos, this_vel_cmd = calc[drive.name]
                            this_pos = math.degrees(this_pos)

                            try:
                                drive.update_actuator(
                                    this_pos, this_vel_cmd * 1000)
                            except RuntimeError as e:
                                print("Error updating actuator.")
                                print(e)
                            except AttributeError as e:
                                print("Attribute Error while updating actuator.")
                                print(e)
                                print("self.motors_initialized {}".format(
                                    self.motors_initialized))
                                print("self {}".format(self))
                            # time.sleep(0.02)
                            corner_actuator.toggling_sleep(self.GPIO, 0.02)

                    if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                        print("COMMS LOST SLOWING ACTUATOR. ~~ {}".format(
                            time.time()))
                        for drive in self.odrives:
                            try:
                                drive.slow_actuator(0.95)
                            except RuntimeError as e:
                                print("Error updating actuator.")
                                print(e)
                        # time.sleep(0.05)
                        corner_actuator.toggling_sleep(self.GPIO, 0.05)

        except KeyboardInterrupt:
            for drive in self.odrives:
                drive.stop_actuator()

    def setup_GPIO(self, GPIO):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(corner_actuator.ESTOP_PIN, GPIO.OUT, initial=GPIO.LOW)
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
