
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
from multiprocessing import Process
import os
import fibre

# This file gets imported by server but we should only import GPIO on raspi.
if "arm" in os.uname().machine:
    import RPi.GPIO as GPIO

VOLT_OUT_PIN = 5

_UP_KEYCODE = '\x1b[A'
_LEFT_KEYCODE = '\x1b[D'
_RIGHT_KEYCODE = '\x1b[C'
_DOWN_KEYCODE = '\x1b[B'

STATE_DISCONNECTED = "Not connected."
_STATE_DISABLED = "Motor error."
_STATE_ENABLED = "Motors enabled."


_SHUT_DOWN_MOTORS_COMMS_DELAY_S = 1.0
_ERROR_RECOVERY_DELAY_S = 5
_ACCELERATION_COUNTS_SEC = 0.5

_USE_JOYSTICK = True


class AcornMotorInterface():

    def __init__(self, manual_control):
        self.odrive_devices = {
        "front_right":"335E31483536",
        "front_left":"335B314C3536",
        "rear_right":"3352316E3536",
        "rear_left":"205332784D4B"
        }

        self.command_socket = self.create_socket()
        self.odrives_connected = False
        self.motors_initialized = False
        self.steering_adjusted = False
        self.manual_control=manual_control


        self.setup_GPIO(GPIO)
        # self.square_wave = Process(target=e_stop_square_wave, args=(GPIO,))
        # self.square_wave.start()
        #p.join()

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
            print("(Use arrow keys or d if done) Adjusting Odrive: {} with home position: {}".format(drive.name, drive.home_position))
            c = click.getchar()
            click.echo()
            if c == 'd':
                click.echo('Done adjusting steering.')
                self.steering_adjusted = True
                return
            elif c == 'w':
                speed+= 100
                print("SPEED UP: {}".format(speed))
            elif c == 's':
                speed-= 100
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

    def connect_to_motors(self):
        front_right = corner_actuator.CornerActuator(serial_number=self.odrive_devices['front_right'], name='front_right', path="/dev/ttySC1", GPIO=GPIO)
        front_left = corner_actuator.CornerActuator(serial_number=self.odrive_devices['front_left'], name='front_left', path="/dev/ttySC0", GPIO=GPIO)
        rear_right = corner_actuator.CornerActuator(serial_number=self.odrive_devices['rear_right'], name='rear_right', path="/dev/ttySC2", GPIO=GPIO)
        rear_left = corner_actuator.CornerActuator(serial_number=self.odrive_devices['rear_left'], name='rear_left', path="/dev/ttySC3", GPIO=GPIO)

        self.odrives = [front_left, front_right, rear_right, rear_left]
        self.odrives_connected = True

    def initialize_motors(self):
        for drive in self.odrives:
            drive.print_errors(clear_errors=True)
        for drive in self.odrives:
            steering = 0.0
            if "rear" in drive.name:
                drive.initialize_steering(steering_flipped=True, skip_homing=self.manual_control)
            else:
                drive.initialize_steering(steering_flipped=False, skip_homing=self.manual_control)
            drive.update_actuator(steering, 0.0)
        for drive in self.odrives:
            if "rear_left" in drive.name:
                drive.enable_thermistor()
                continue
            drive.initialize_traction()
        self.motors_initialized = True

    def check_errors(self):
        for drive in self.odrives:
            try:
                #if "rear_left" in drive.name and drive.odrv0.axis0.error==False:
                #    continue
                if drive.odrv0.axis0.error or drive.odrv0.axis1.error:
                    return True
            except fibre.protocol.ChannelBrokenException as e:
                print("Exception in {} odrive.".format(drive.name))
                raise e
        return False

    def communicate_message(self, state, voltages=None, ibus=None, temperatures=None):
        # print("TRYING TO SEND STATE: {}, Voltages {}".format(state, voltages))
        try:
            corner_actuator.gpio_toggle(GPIO)
            while self.command_socket.poll(timeout=20):
                corner_actuator.gpio_toggle(GPIO)
                recv = pickle.loads(self.command_socket.recv_pyobj())
                self.command_socket.send_pyobj(pickle.dumps((state, voltages, ibus, temperatures)),flags=zmq.DONTWAIT)
                corner_actuator.gpio_toggle(GPIO)
                return recv
            # print("no incoming messages")
        except zmq.error.ZMQError as e:
            print("Error with motor command socket: {}".format(e))
            raise e

    def run_main(self):
        motor_error = False
        tick_time = time.time()
        print("RUN_MAIN_MOTORS")
        try:
            while True:
                if not self.odrives_connected:
                    self.communicate_message(STATE_DISCONNECTED)
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
                        if self.manual_control:
                            self.ask_if_adjust_steering()
                        print("Initialized Steering")
                    except RuntimeError as e:
                        print("Motor problem while adjusting steering.")


                voltages = []
                bus_currents = []
                temperatures = []
                for vehicle_corner in self.odrives:
                    corner_actuator.gpio_toggle(GPIO)
                    vehicle_corner.update_voltage()
                    corner_actuator.gpio_toggle(GPIO)
                    vehicle_corner.update_thermistor_temperature_C()
                    voltages.append(vehicle_corner.voltage)
                    bus_currents.append(abs(vehicle_corner.ibus_0) + abs(vehicle_corner.ibus_1))
                    temperatures.append(vehicle_corner.temperature_c)

                # velocities = []
                # for vehicle_corner in self.odrives:
                #     velocities.append(vehicle_corner.odrv0.axis1.encoder.vel_estimate)
                #     corner_actuator.gpio_toggle(GPIO)
                #
                # print(velocities)

                if self.check_errors():
                    if motor_error != True:
                        time.sleep(1) # Intentionally break the e-stop loop to stop all motors.
                    motor_error = True
                    print("ERROR DETECTED IN ODRIVES.")
                    for drive in self.odrives:
                        print("Drive: {}:".format(drive.name))
                        start = time.time()
                        drive.print_errors(clear_errors=True)
                        print("clear_errors_duration {}".format(start - time.time()))
                    print("ERROR DETECTED IN ODRIVES.")
                    corner_actuator.toggling_sleep(GPIO, _ERROR_RECOVERY_DELAY_S)
                    #time.sleep(_ERROR_RECOVERY_DELAY_S)
                    recv = self.communicate_message(_STATE_DISABLED, voltages, bus_currents, temperatures)
                    if recv:
                        print("Got motor command but motors are in error state.")
                        print("Motor command was {}".format(recv))
                else:
                    if motor_error:
                        for odrive in self.odrives:
                            odrive.recover_from_estop()
                        if not self.check_errors():
                            print("Recovered from e-stop.")
                            motor_error = False

                    calc = self.communicate_message(_STATE_ENABLED, voltages, bus_currents, temperatures)
                    corner_actuator.gpio_toggle(GPIO)
                    if calc:
                        if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                            print("Regained Motor Comms")
                        tick_time = time.time()
                        for drive in self.odrives:
                            this_pos, this_vel_cmd = calc[drive.name]
                            this_pos = math.degrees(this_pos)
                            # if "front_right" in drive.name:
                            #     # this_pos
                            #     this_vel_cmd = 0
                            # else:
                            #     this_pos = 0
                            #     this_vel_cmd = 0


                            if "rear_left" in drive.name:
                                # this_vel_cmd *= 0.63
                                this_vel_cmd *= 0.70
                            #     drive.odrv0.axis1.controller.config.vel_gain = 0 # 0.02
                            #     drive.odrv0.axis1.controller.config.vel_integrator_gain = 0 # 0.1
                            # else:
                            #     this_vel_cmd = 0
                            # this_pos = 0
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
                            #time.sleep(0.02)
                            corner_actuator.toggling_sleep(GPIO, 0.02)

                    if time.time() - tick_time > _SHUT_DOWN_MOTORS_COMMS_DELAY_S:
                        print("COMMS LOST SLOWING ACTUATOR. ~~ {}".format(time.time()))
                        for drive in self.odrives:
                            try:
                                drive.slow_actuator(0.95)
                            except RuntimeError as e:
                                print("Error updating actuator.")
                                print(e)
                        #time.sleep(0.05)
                        corner_actuator.toggling_sleep(GPIO, 0.05)

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

        for _ in range(100):
            time.sleep(0.001)
            GPIO.output(VOLT_OUT_PIN, GPIO.LOW)
            time.sleep(0.001)
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
    #import sys
    #sys.exit()
    parser = argparse.ArgumentParser(description='Run the Acorn motors process.')
    parser.add_argument('--manual', dest='manual_control', default=False, action='store_true')
    args = parser.parse_args()
    control = AcornMotorInterface(args.manual_control)
    control.run_main()
