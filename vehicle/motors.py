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
import click
import argparse
from multiprocessing import Process
import fibre
import logging
from functools import partial
import weakref

from corner_actuator import OdriveConnection, CornerActuator, estop_nudge, toggling_sleep
from model import MotorStatus, PubsubTopic
from model import MOTORS_TO_REMOTE_CONTROL_IPC, REMOTE_CONTROL_MOTORS_TO_IPC
import estop
import ipc
import utils

_UP_KEYCODE = '\x1b[A'
_LEFT_KEYCODE = '\x1b[D'
_RIGHT_KEYCODE = '\x1b[C'
_DOWN_KEYCODE = '\x1b[B'

_SHUT_DOWN_MOTORS_COMMS_DELAY_S = 1.0
_ERROR_RECOVERY_DELAY_S = 5
_ACCELERATION_COUNTS_SEC = 0.5

logger = logging.getLogger("motors")
utils.config_logging(logger, debug=False)


class AcornMotorInterface():

    def __init__(self, manual_control=False, simulated_hardware=False):

        self.odrive_connections = [
            OdriveConnection(name='front_right', serial="335E31483536",
                             path="/dev/ttySC1", enable_steering=True, enable_traction=True),
            OdriveConnection(name='front_left', serial="335B314C3536",
                             path="/dev/ttySC0", enable_steering=True, enable_traction=True),
            OdriveConnection(name='rear_right', serial="3352316E3536",
                             path="/dev/ttySC2", enable_steering=True, enable_traction=True),
            OdriveConnection(name='rear_left', serial="205F3882304E",
                             path="/dev/ttySC3", enable_steering=True, enable_traction=True)
        ]

        publisher = ipc.ZMQPub(MOTORS_TO_REMOTE_CONTROL_IPC)
        weakref.finalize(self, publisher.close)
        self.to_remote_control = partial(publisher.pub, PubsubTopic.MOTORS_TO_REMOTE_CONTROL)
        subscriber = ipc.LeakyZMQSub()
        weakref.finalize(self, subscriber.close)
        self.from_remote_control = subscriber.sub(REMOTE_CONTROL_MOTORS_TO_IPC, PubsubTopic.REMOTE_CONTROL_TO_MOTORS)
        self.odrives_connected = False
        self.motors_initialized = False
        self.steering_adjusted = False
        self.manual_control = manual_control
        self.simulated_hardware = simulated_hardware
        self.motor_error = False
        self.tick_time = time.time()

        if not self.simulated_hardware:
            estop.setup()

    def ask_if_adjust_steering(self):
        self.square_wave = Process(target=estop.e_stop_square_wave)
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
            corner = CornerActuator(connection_definition=drive,
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
                logger.error("Exception in {} odrive.".format(drive.name))
                raise e
            except RuntimeError:
                logger.error("RuntimeError in {} odrive.".format(drive.name))
                return True
        return False

    def communicate_message(self, state, voltages=None, ibus=None, temperatures=None):
        try:
            self.to_remote_control((state, voltages, ibus, temperatures))
            recv = self.from_remote_control()
            return recv
        except Exception as e:
            logger.error("Error communicating with remote control process: {}".format(e))
            raise e

    def run_debug_control(self, enable_steering=True, enable_traction=True):
        logger.info("RUN_MAIN_MOTORS")
        while True:
            try:
                if not self.odrives_connected:
                    self.connect_to_motors(
                        enable_steering=enable_steering, enable_traction=enable_traction)
                    logger.info("CONNECTED TO MOTORS")
                    logger.info(self.odrives)
                elif not self.motors_initialized:
                    try:
                        logger.info("BEGIN INTIALIZE MOTORS")
                        self.initialize_motors()
                        logger.info("SUCCESS INTIALIZE MOTORS")
                    except ValueError as e:
                        logger.error(f"Unrecoverable error initializing steering: {e}")
                        raise e
                    except RuntimeError as e:
                        logger.error(f"Motor problem while initializing steering: {e}")
                elif not self.steering_adjusted:
                    try:
                        self.steering_adjusted = True
                        self.ask_if_adjust_steering()
                        logger.info("Initialized Steering")
                    except RuntimeError as e:
                        logger.error(f"Motor problem while adjusting steering: {e}")
            except Exception as e:
                logger.error(f"Exception: {e}")
                time.sleep(1)

    def run_main(self):
        logger.error("RUN_MAIN_MOTORS")
        # estop activates if it doesn't see a level flip every 50ms. To leave enough room for Linux scheduling, we set the interval so that the level is flipped every 20ms.
        interval = 1 / 50
        try:
            while True:
                start = time.time()
                self.tick()
                elapsed = time.time() - start
                if (left := interval - elapsed) > 0:
                    time.sleep(left)
                estop_nudge()
        except KeyboardInterrupt:
            for drive in self.odrives:
                drive.stop_actuator()

    def tick(self):
        if not self.odrives_connected:
            self.communicate_message(MotorStatus.DISCONNECTED)
            self.connect_to_motors()
        elif not self.motors_initialized:
            try:
                self.initialize_motors()
            except ValueError as e:
                logger.error(f"Unrecoverable error initializing steering: {e}")
                raise e
            except RuntimeError as e:
                logger.error(f"Motor problem while initializing steering: {e}")
        elif not self.steering_adjusted:
            try:
                self.steering_adjusted = True
                if self.manual_control:
                    self.ask_if_adjust_steering()
                logger.error("Initialized Steering")
            except RuntimeError as e:
                logger.error(f"Motor problem while adjusting steering: {e}")

        voltages = []
        bus_currents = []
        temperatures = []
        for vehicle_corner in self.odrives:
            try:
                vehicle_corner.update_voltage()
                vehicle_corner.update_thermistor_temperature_C()
            except Exception as e:
                logger.error(f"Error in vehicle corner {vehicle_corner.name}: {e}")
                raise e
            voltages.append(vehicle_corner.voltage)
            bus_currents.append(
                abs(vehicle_corner.ibus_0) + abs(vehicle_corner.ibus_1))
            temperatures.append(vehicle_corner.temperature_c)

        # velocities = []
        # for vehicle_corner in self.odrives:
        #     velocities.append(vehicle_corner.odrv0.axis1.encoder.vel_estimate)
        #     corner_actuator.estop_nudge()
        #
        # logger.error(velocities)

        if self.check_odrive_errors():
            if not self.motor_error:
                # Intentionally break the e-stop loop to stop all motors.
                time.sleep(1)
            self.motor_error = True
            time.sleep(0.5)
            toggling_sleep(0.2)
            logger.warning("ERROR DETECTED IN ODRIVES.")
            for drive in self.odrives:
                logger.warning("Drive: {}:".format(drive.name))
                start = time.time()
                drive.print_errors(clear_errors=True)
                logger.warning("clear_errors_duration {}".format(
                    start - time.time()))
            logger.warning("ERROR DETECTED IN ODRIVES.")
            toggling_sleep(_ERROR_RECOVERY_DELAY_S)
            recv = self.communicate_message(
                MotorStatus.DISABLED, voltages, bus_currents, temperatures)
            if recv:
                logger.warning("Got motor command but motors are in error state.")
                logger.warning("Motor command was {}".format(recv))
        else:
            if self.motor_error:
                for odrive in self.odrives:
                    odrive.recover_from_estop()
                if not self.check_odrive_errors():
                    logger.info("Recovered from e-stop.")
                    self.motor_error = False

            calc = self.communicate_message(
                MotorStatus.ENABLED, voltages, bus_currents, temperatures)
            if calc:
                logger.info(f"calc: {calc}")
                if time.time() - self.tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                    logger.info("Regained Motor Comms")
                self.tick_time = time.time()
                for drive in self.odrives:
                    this_pos, this_vel_cmd = calc[drive.name]
                    this_pos = math.degrees(this_pos)
                    # if "front_right" in drive.name:
                    #     # this_pos
                    #     this_vel_cmd = 0
                    # else:
                    #     this_pos = 0
                    #     this_vel_cmd = 0

                #    if "rear_left" in drive.name:
                    # this_vel_cmd *= 0.63
                    #    this_vel_cmd *= 0.70
                    #     drive.odrv0.axis1.controller.config.vel_gain = 0 # 0.02
                    #     drive.odrv0.axis1.controller.config.vel_integrator_gain = 0 # 0.1
                    # else:
                    #     this_vel_cmd = 0
                    # this_pos = 0
                    try:
                        drive.update_actuator(this_pos, this_vel_cmd * 1000)
                    except RuntimeError as e:
                        logger.error(f"Error updating actuator: {e}")
                    except AttributeError as e:
                        logger.error(f"Attribute Error while updating actuator: {e}")
                        logger.error(f"self.motors_initialized {self.motors_initialized}")
                        logger.error(f"self: {self}")

            if time.time() - self.tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                logger.error(f"COMMS LOST SLOWING ACTUATOR. ~~ {time.time()}")
                for drive in self.odrives:
                    try:
                        drive.slow_actuator(0.95)
                    except RuntimeError as e:
                        logger.error(f"Error updating actuator: {e}")


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
