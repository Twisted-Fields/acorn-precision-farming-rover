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

"""
Control one corner of the Acorn robot via the motor controller.
"""
import time
import math
import sys
import random
import collections
import traceback
import isotp
import motor_controller
import model
from model import CORNER_NAMES


THERMISTOR_ADC_CHANNEL = 3
_ADC_PORT_STEERING_POT = 5
_ADC_PORT_STEERING_HOME = 6


_VCC = 3.3
_VOLTAGE_MIDPOINT = _VCC/2
_POT_VOLTAGE_LOW = _VCC/6
_POT_VOLTAGE_HIGH = 5*_VCC/6
_HOMING_VELOCITY_STEERING_COUNTS = 800
_HOMING_POT_DEADBAND_VOLTS = 0.2
_HOMING_POT_HALF_DEADBAND_V = _HOMING_POT_DEADBAND_VOLTS/2
_FAST_POLLING_SLEEP_S = 0.01
_SLOW_POLLING_SLEEP_S = 0.5

_MAX_HOMING_ATTEMPTS = 4
_ZERO_VEL_COUNTS_THRESHOLD = 2


COMMAND_VALUE_MINIMUM = 0.001


STEERING_RADIANS_TO_COUNTS = (0.5*41/8)/(2.0*math.pi)

ESTOP_PIN = 6

MotorConnection = collections.namedtuple(
    'MotorConnection', 'name id port enable_steering enable_traction')

# class PowerSample:
#     def init(self, volts, amps, duration):
#
#
# class PowerTracker:
#     def init(self, interval):
#         self.recent_samples = []
#         self.collapse_samples_interval = interval



class CornerActuator:

    def __init__(self, name=None, path=None, GPIO=None,
                 connection_definition=None, enable_steering=True, enable_traction=True,
                 simulated_hardware=False, disable_steering_limits=False):

        if simulated_hardware:
            import sim_socket
            socket = sim_socket.SimulatedSocket()
        else:
            socket = isotp.socket()
        socket.set_opts(isotp.socket.flags.WAIT_TX_DONE)
        socket.bind("can1", isotp.Address(rxid=0x1, txid=connection_definition.id))

        self.GPIO = GPIO
        self.encoder_estimates = (0.0,0.0,0.0,0.0)
        self.enable_steering = enable_steering
        self.enable_traction = enable_traction
        self.controller = motor_controller.MotorController(id=connection_definition.id)
        self.name = name
        self.disable_steering_limits = disable_steering_limits
        self.zero_vel_timestamp = time.time()
        self.socket = socket
        self.home_position = 0
        self.rotation_sensor_val = 0
        self.steering_flipped = False
        self.steering_initialized = False
        self.traction_initialized = False



    def enable_thermistor(self):
        self.has_thermistor = True

    # def idle_wait(self):
    #     while self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
    #         toggling_sleep(self.GPIO, 0.1)

    def print_errors(self, clear_errors=False):
        # self.dump_errors(clear_errors)
        gpio_toggle(self.GPIO)
        if clear_errors:
            self.position = 0
            self.velocity = 0

    def sample_home_sensor(self):
        gpio_toggle(self.GPIO)
        return 0

    def request_reply(self, request_packet, error_limit=10):
        send_ok = False
        errors = 0
        while not send_ok:
            gpio_toggle(self.GPIO)
            try:
                # hex_string = "".join(" 0x%02x" % b for b in request_packet)
                # print(f"Send: {address} {hex_string}")
                self.socket.send(request_packet)
                send_ok = True
                # print("Send okay.")
            except KeyboardInterrupt as e:
                raise e
            except Exception as e:
                # raise e
                traceback.print_exc()
                print(f"Send Error, controller ID {self.controller.id}")
                time.sleep(0.004)
                errors += 1
                if errors > error_limit:
                    return False
        return self.get_reply(error_limit)

    def get_reply(self, error_limit=10):
        errors = 0
        data = None
        # print("===============================================")
        while not data:
            gpio_toggle(self.GPIO)
            try:
                data=self.socket.recv()
                # if data:
                    # hex_string = "".join(" 0x%02x" % b for b in data)
                    # print(f"{self.controller.id} recieve returned {hex_string}")
                if not data:
                    print(f"Recieve error (no data) with controller {self.controller.id}")
                    errors += 1
                    if errors > error_limit:
                        return False
                elif data[0] != self.controller.id:
                    # print("Skipping wrong packet")
                    data = None
            except KeyboardInterrupt as e:
                raise e
            except Exception:
                traceback.print_exc()
                print(f"Recieve error (exception raised) with controller {self.controller.id}")
                return False
        return data

    def ping_request(self):
        gpio_toggle(self.GPIO)
        reply = self.request_reply(self.controller.simple_ping())
        gpio_toggle(self.GPIO)
        if reply:
            return self.controller.decode_ping_reply(reply, print_result=True)
        else:
            return (False, False)

    def log_request(self):
        gpio_toggle(self.GPIO)
        reply = self.request_reply(self.controller.log_request())
        gpio_toggle(self.GPIO)
        if reply:
            return self.controller.decode_log_reply(reply)
        else:
            return None

    def sample_sensors(self, request_send=True):
        gpio_toggle(self.GPIO)
        data = None
        if request_send:
            data = self.request_reply(self.controller.sensor_request())
        else:
            data = self.get_reply()
        gpio_toggle(self.GPIO)
        if data and self.controller.decode_sensor_reply(data):
            self.rotation_sensor_val = self.controller.adc1 * 3.3/1024
        else:
            print(f"ERROR: No data on sample_sensors read, controller {self.controller.id}")

    def initialize_traction(self):
        self.traction_initialized = True
        gpio_toggle(self.GPIO)


    def check_steering_limits(self):
        if self.disable_steering_limits:
            print("{} steering sensor {}".format(
                list(CORNER_NAMES)[self.name], self.rotation_sensor_val))
        if self.rotation_sensor_val < _POT_VOLTAGE_LOW or self.rotation_sensor_val > _POT_VOLTAGE_HIGH:
            if self.disable_steering_limits:
                print("WARNING POTENTIOMETER VOLTAGE OUT OF RANGE DAMAGE " +
                      "MAY OCCUR: {}".format(self.rotation_sensor_val))
            else:
                raise ValueError(
                    "POTENTIOMETER VOLTAGE OUT OF RANGE: {}".format(self.rotation_sensor_val))

    def recover_from_estop(self):
        gpio_toggle(self.GPIO)
        # reset contol values
        self.initialize_traction()

    def initialize_steering(self, steering_flipped=False, skip_homing=False):
        gpio_toggle(self.GPIO)
        self.sample_sensors()
        self.check_steering_limits()
        self.enable_steering_control()
        gpio_toggle(self.GPIO)

        # do some homing

        self.steering_flipped = steering_flipped
        self.steering_initialized = True

    def enable_steering_control(self):
        # after reset, send the steering system initialization values here
        pass


    def check_errors(self, read_errors=True):
        # gpio_toggle(self.GPIO)
        # check system errors here
        pass

    def update_encoder_data(self):
        # self.encoder_estimates = (self.steering_axis.encoder.pos_estimate,
        #                             self.steering_axis.encoder.vel_estimate,
        #                             self.traction_axis.encoder.pos_estimate,
        #                             self.traction_axis.encoder.vel_estimate)
        # gpio_toggle(self.GPIO)
        pass


    def update_actuator(self, steering_pos_deg, drive_velocity, simple_request=False):
        #print("Update {}".format(self.name))
        # self.check_steering_limits()
        gpio_toggle(self.GPIO)
        if self.steering_flipped:
            drive_velocity *= -1
        self.position = steering_pos_deg
        self.velocity = drive_velocity
        self.update_encoder_data()
        self.log_request()


        # TODO: on previous robot this code reset integrator current after
        # a few seconds. May not be needed on V2 robot.
        # if abs(self.velocity) > _ZERO_VEL_COUNTS_THRESHOLD:
        #     self.zero_vel_timestamp = None
        # else:
        #     if self.zero_vel_timestamp is None:
        #         self.zero_vel_timestamp = time.time()
        #     else:
        #         if time.time() - self.zero_vel_timestamp > 5.0:
        #             # could reset integrator current here if needed
        #             pass

        self.controller.motor1.setpoint = STEERING_RADIANS_TO_COUNTS * self.position + self.home_position
        self.controller.motor2.setpoint = self.velocity
        if(self.enable_traction or self.enable_steering):
            try:
                # print(self.controller.serialize_motors())
                # print(f"pos: {self.position}, vel:{self.velocity}")
                gpio_toggle(self.GPIO)
                if simple_request:
                    self.socket.send(self.controller.serialize_basic(reply_requested=True))
                    self.sample_sensors(request_send=False)
                else:
                    self.socket.send(self.controller.serialize_motors())
                    self.sample_sensors(request_send=True)
            except Exception as e:
                traceback.print_exc()
                print(f"Send Error in corner {list(CORNER_NAMES)[self.name]}")
                raise e


    def slow_actuator(self, fraction):
        gpio_toggle(self.GPIO)
        if not (self.steering_initialized and self.traction_initialized):
            return
        position = fraction * self.position
        velocity = fraction * self.velocity
        if self.steering_flipped:
            velocity *= -1
        if math.fabs(self.position) < COMMAND_VALUE_MINIMUM:
            position = 0.0
        if math.fabs(self.velocity) < COMMAND_VALUE_MINIMUM:
            velocity = 0.0
        self.update_actuator(position, velocity)

    def stop_actuator(self):
        gpio_toggle(self.GPIO)
        # self.odrv0.axis0.controller.vel_setpoint = 0
        # self.odrv0.axis1.controller.vel_setpoint = 0

    def update_thermistor_temperature_C(self, adc_channel=THERMISTOR_ADC_CHANNEL):
        if not self.has_thermistor:
            return
        try:
            gpio_toggle(self.GPIO)
            # value = self.odrv0.get_adc_voltage(adc_channel)
            resistance = 10000 / (3.3/value)
            self.temperature_c = self.thermistor_steinhart_temperature_C(
                resistance)
        except ZeroDivisionError:
            pass

    def thermistor_steinhart_temperature_C(self, r, Ro=10000.0, To=25.0, beta=3900.0):
        """ Via: https://learn.adafruit.com/thermistor/circuitpython """
        steinhart = math.log(r / Ro) / beta      # log(R/Ro) / beta
        steinhart += 1.0 / (To + 273.15)         # log(R/Ro) / beta + 1/To
        steinhart = (1.0 / steinhart) - 273.15   # Invert, convert to C
        return steinhart


def toggling_sleep(GPIO, duration):
    start = time.time()
    while time.time() - start < duration:
        gpio_toggle(GPIO)
        time.sleep(_FAST_POLLING_SLEEP_S)


def gpio_toggle(GPIO):
    GPIO.estop_state = not GPIO.estop_state
    GPIO.output(ESTOP_PIN, GPIO.estop_state)
