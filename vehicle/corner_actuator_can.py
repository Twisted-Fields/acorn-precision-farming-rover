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


INDUCTION_STEERING_SENSOR = True
SHOULD_SYSTEM_USE_HOMING = INDUCTION_STEERING_SENSOR == False

THERMISTOR_ADC_CHANNEL = 3
_ADC_PORT_STEERING_POT = 5
_ADC_PORT_STEERING_HOME = 6


_VCC = 3.3
_VOLTAGE_MIDPOINT = _VCC/2
_POT_VOLTAGE_LOW = _VCC/8
_POT_VOLTAGE_HIGH = 7*_VCC/8
_HOMING_VELOCITY_STEERING_COUNTS = 800
_HOMING_POT_DEADBAND_VOLTS = 0.2
_HOMING_POT_HALF_DEADBAND_V = _HOMING_POT_DEADBAND_VOLTS/2
_FAST_POLLING_SLEEP_S = 0.01
_SLOW_POLLING_SLEEP_S = 0.5

_SYSTEM_VOLTAGE_HOMING_THRESHOLD = 30

_MAX_HOMING_ATTEMPTS = 4
_ZERO_VEL_COUNTS_THRESHOLD = 2

# _HOME_SENSOR_ACTIVATION_THRESHOLD = 100
# _HOME_SENSOR_SKIP_HOMING_THRESHOLD = 380
# _HOME_SENSOR_ACTIVATION_THRESHOLD = 50
_HOME_SENSOR_HOMING_THRESHOLD = 120

COMMAND_VALUE_MINIMUM = 0.001

# STEERING_COUNTS_TO_RADIANS = (1024.0*0.43)
# STEERING_RADIANS_TO_COUNTS = 1.0/STEERING_COUNTS_TO_RADIANS

STEERING_RADIANS_TO_COUNTS = (1024.0*0.43)
STEERING_COUNTS_TO_RADIANS = 1.0/STEERING_RADIANS_TO_COUNTS

DEFAULT_CAN_REPLY_ERROR_LIMIT = 10

ESTOP_PIN = 6

MotorConnection = collections.namedtuple(
    'MotorConnection', 'name id port enable_steering enable_traction reverse_drive')

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
                 simulated_hardware=False, disable_steering_limits=False, reverse_drive=False,
                 logger=None):

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
        self.needs_homing = SHOULD_SYSTEM_USE_HOMING
        self.rotation_sensor_val = 0
        self.steering_flipped = False
        self.reverse_drive = reverse_drive
        self.steering_initialized = False
        self.traction_initialized = False
        self.voltage_ok = False
        self.logger=logger
        self.last_voltage_warning_logging_time = time.time()
        self.external_voltage_ok_signal = False



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

    def request_reply(self, request_packet, error_limit=DEFAULT_CAN_REPLY_ERROR_LIMIT):
        send_ok = False
        errors = 0
        while not send_ok:
            gpio_toggle(self.GPIO)
            try:
                # hex_string = "".join(" 0x%02x" % b for b in request_packet)
                # self.logger.info(f"Send: {address} {hex_string}")
                self.socket.send(request_packet)
                send_ok = True
                # self.logger.info("Send okay.")
            except KeyboardInterrupt as e:
                raise e
            except Exception as e:
                # raise e
                traceback.print_exc()
                self.logger.info(f"Send Error, controller ID {self.controller.id}")
                time.sleep(0.004)
                errors += 1
                if errors > error_limit:
                    return False
        return self.get_reply(error_limit)

    def get_reply(self, error_limit=10):
        errors = 0
        data = None
        # self.logger.info("===============================================")
        while not data:
            gpio_toggle(self.GPIO)
            try:
                data=self.socket.recv()
                # if data:
                    # hex_string = "".join(" 0x%02x" % b for b in data)
                    # self.logger.info(f"{self.controller.id} recieve returned {hex_string}")
                if not data:
                    self.logger.info(f"Recieve error (no data) with controller {self.controller.id}")
                    errors += 1
                    if errors > error_limit:
                        return False
                elif data[0] != self.controller.id:
                    # self.logger.info("Skipping wrong packet")
                    data = None
            except KeyboardInterrupt as e:
                raise e
            except Exception:
                traceback.print_exc()
                self.logger.info(f"Recieve error (exception raised) with controller {self.controller.id}")
                return False
        return data

    def request_set_steering_zero(self, error_limit=DEFAULT_CAN_REPLY_ERROR_LIMIT):
        gpio_toggle(self.GPIO)
        reply = self.request_reply(self.controller.set_steering_zero(), error_limit)
        gpio_toggle(self.GPIO)
        if reply:
            return self.controller.decode_ping_reply(reply, logger=self.logger)
        else:
            return (False, False)

    def ping_request(self, error_limit=DEFAULT_CAN_REPLY_ERROR_LIMIT):
        gpio_toggle(self.GPIO)
        reply = self.request_reply(self.controller.simple_ping(), error_limit)
        gpio_toggle(self.GPIO)
        if reply:
            return self.controller.decode_ping_reply(reply, logger=self.logger)
        else:
            return (False, False)

    def log_request(self, error_limit=DEFAULT_CAN_REPLY_ERROR_LIMIT):
        gpio_toggle(self.GPIO)
        reply = self.request_reply(self.controller.log_request(),error_limit)
        gpio_toggle(self.GPIO)
        if reply:
            return self.controller.decode_log_reply(reply)
        else:
            return None

    def sample_sensors(self, request_send=True, error_limit=DEFAULT_CAN_REPLY_ERROR_LIMIT):
        gpio_toggle(self.GPIO)
        data = None
        if request_send:
            data = self.request_reply(self.controller.sensor_request(), error_limit)
        else:
            data = self.get_reply()
        gpio_toggle(self.GPIO)
        if data and self.controller.decode_sensor_reply(data):
            self.rotation_sensor_val = self.controller.adc1 * 3.3/1024
        else:
            self.logger.info(f"ERROR: No data on sample_sensors read, controller {self.controller.id}")

    def set_home_position(self, home_position, error_limit=2):
        # intetionally not running GPIO toggle so this can be set without enabling motors
        reply = self.request_reply(self.controller.set_steering_home(home_position), error_limit)
        if reply:
            return self.controller.decode_ping_reply(reply, print_result=False)
        else:
            return (False, False)

    def initialize_traction(self):
        self.traction_initialized = True
        gpio_toggle(self.GPIO)


    def check_steering_limits(self):
        # if self.disable_steering_limits:
        #     self.logger.info("{} steering sensor {}".format(
        #         list(CORNER_NAMES)[self.name], self.rotation_sensor_val))
        if self.rotation_sensor_val < _POT_VOLTAGE_LOW or self.rotation_sensor_val > _POT_VOLTAGE_HIGH:
            if self.disable_steering_limits:
                self.logger.info("WARNING POTENTIOMETER VOLTAGE OUT OF RANGE DAMAGE " +
                      "MAY OCCUR: {}".format(self.rotation_sensor_val))
            else:
                raise ValueError(
                    "POTENTIOMETER VOLTAGE OUT OF RANGE: {}".format(self.rotation_sensor_val))

    def recover_from_estop(self):
        gpio_toggle(self.GPIO)
        # reset contol values
        self.initialize_traction()
        self.check_homing()

    def check_homing(self):
        if self.needs_homing == False:
            return
        self.sample_sensors()
        self.voltage_ok = self.controller.voltage > _SYSTEM_VOLTAGE_HOMING_THRESHOLD
        if self.external_voltage_ok_signal:
            self.voltage_ok = True
        if self.controller.motion_allowed and self.voltage_ok:
            self.home_corner()


    def initialize_steering(self, steering_flipped=False, skip_homing=False):
        gpio_toggle(self.GPIO)
        self.sample_sensors()
        self.check_steering_limits()
        self.enable_steering_control()
        gpio_toggle(self.GPIO)

        self.voltage_ok = True #self.controller.voltage > _SYSTEM_VOLTAGE_HOMING_THRESHOLD
        if self.external_voltage_ok_signal:
            self.voltage_ok = True

        if self.controller.motion_allowed and self.voltage_ok:
            if not skip_homing and self.needs_homing:
                self.home_corner()
        else:
            if self.voltage_ok:
                self.logger.info("SKIPPING HOMING UNTIL MOTION ALLOWED")
            else:
                self.logger.info("SKIPPING HOMING UNTIL SYSTEM VOLTAGE IS HIGHER")
            self.needs_homing = SHOULD_SYSTEM_USE_HOMING


        self.steering_flipped = steering_flipped
        self.steering_initialized = True

    def home_corner(self, manual=False):
        if not SHOULD_SYSTEM_USE_HOMING:
            return
        self.needs_homing = False # Set first so update_actuator works.
        max_home_sensor_value = 0
        max_home_position = 0
        self.sample_sensors()
        self.check_steering_limits()
        OFFSET_DISTANCE_RADIANS = math.radians(45)
        home_value = abs(512-self.controller.adc2)
        drive_position = 0
        offset_value = 0
        if home_value > _HOME_SENSOR_HOMING_THRESHOLD and not manual:
            self.logger.info(f"Sensor value {home_value} is sufficient for homing. ")
            return
        # if home_value > _HOME_SENSOR_ACTIVATION_THRESHOLD:
        #     drive_position += OFFSET_DISTANCE_RADIANS/2
        #     self.update_actuator(drive_position, 0.0)
        #     self.steering_wait()
        #     offset_value = -OFFSET_DISTANCE_RADIANS
        if self.rotation_sensor_val < _VOLTAGE_MIDPOINT:
            drive_position -= OFFSET_DISTANCE_RADIANS/2
            self.update_actuator(drive_position, 0.0)
            self.steering_wait(timeout=6.0, error_threshold=0.02)
            offset_value = OFFSET_DISTANCE_RADIANS
        else:
            drive_position += OFFSET_DISTANCE_RADIANS/2
            self.update_actuator(drive_position, 0.0)
            self.steering_wait(timeout=6.0, error_threshold=0.02)
            offset_value = -OFFSET_DISTANCE_RADIANS

        while True:
            drive_position += offset_value
            self.logger.info(f"Homing {list(CORNER_NAMES)[self.name]}")
            self.logger.info(f"Drive Position moving to {drive_position}")
            if abs(drive_position) > 3.14*1.25 + offset_value:
                self.logger.info("ERROR: Full homing rotation with no sensor detected.")
                raise RuntimeError("Full homing rotation with no sensor detected.")
            self.update_actuator(drive_position, 0.0)

            tick = 0
            while not self.is_steering_at_setpoint(error_threshold=0.02):
                time.sleep(0.01)
                self.sample_sensors()
                self.check_steering_limits()
                home_value = abs(512-self.controller.adc2)
                if home_value > max_home_sensor_value:
                    self.logger.info(f"{list(CORNER_NAMES)[self.name]} FOUND_NEW_MAX")
                    max_home_sensor_value = home_value
                    max_home_position = self.controller.motor1.encoder_counts * STEERING_COUNTS_TO_RADIANS
                # self.logger.info(f"Motion Allowed: {self.controller.motion_allowed}")
                tick+=1
                if tick>10:
                    self.logger.info(f"{list(CORNER_NAMES)[self.name]} Home Value: {str(home_value)}, Max Home: {str(max_home_sensor_value)}, Home Position: {str(max_home_position)}")
                    tick = 0
            self.logger.info(f"{list(CORNER_NAMES)[self.name]} COMPLETED ROTATION, max sensor val: {max_home_sensor_value}")
            if max_home_sensor_value > _HOME_SENSOR_HOMING_THRESHOLD:
                self.logger.info("FOUND HOME")
                self.home_position = max_home_position
                self.update_actuator(0.0, 0.0)
                self.steering_wait(timeout=6.0, error_threshold=0.02)
                if all(self.request_set_steering_zero()):
                    self.logger.info("SET HOME POSITION SUCCESS")
                    self.home_position = 0.0
                    self.update_actuator(0.0, 0.0)
                else:
                    self.logger.info("FAILED TO SET HOME POSITION!")
                return
            time.sleep(1)


    def enable_steering_control(self):
        # after reset, send the steering system initialization values here
        pass


    def check_errors(self, read_errors=True):
        # gpio_toggle(self.GPIO)
        # check system errors here
        pass

    def is_steering_at_setpoint(self, error_threshold=0.005):
        steering_error = abs(self.controller.motor1.encoder_counts * STEERING_COUNTS_TO_RADIANS - self.controller.motor1.setpoint)
        self.logger.info("Steering enc counts: " + str(self.controller.motor1.encoder_counts) + ", Radians: " + str(self.controller.motor1.encoder_counts * STEERING_COUNTS_TO_RADIANS) + ", Setpoint: " + str(self.controller.motor1.setpoint) + f", System voltage {self.controller.voltage}")
        return steering_error < error_threshold

    def steering_wait(self, timeout=2.0, error_threshold=0.005):
        start_time = time.time()
        while not self.is_steering_at_setpoint(error_threshold):
            gpio_toggle(self.GPIO)
            time.sleep(0.05)
            self.sample_sensors()
            if time.time() - start_time > timeout:
                raise TimeoutError(f"Steering wait timeout exceeded for corner {list(CORNER_NAMES)[self.name]}")
        return

    def update_actuator(self, steering_pos_deg, drive_velocity, simple_request=False):
        #self.logger.info("Update {}".format(self.name))
        # self.check_steering_limits()
        gpio_toggle(self.GPIO)
        if self.needs_homing:
            self.sample_sensors()
            if time.time() - self.last_voltage_warning_logging_time > 2.0:
                self.logger.info(f"Corner needs homing, ignoring update_actuator commands. System voltage {self.controller.voltage}")
                self.last_voltage_warning_logging_time = time.time()
            return
        if self.steering_flipped:
            # TODO: shouldn't we also flip steering here?
            drive_velocity *= -1
        if self.reverse_drive:
            drive_velocity *= -1
        self.position = steering_pos_deg
        self.velocity = drive_velocity
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

        self.controller.motor1.setpoint = self.position + self.home_position
        self.controller.motor2.setpoint = self.velocity
        if(self.enable_traction or self.enable_steering):
            try:
                # self.logger.info(self.controller.serialize_motors())
                # self.logger.info(f"pos: {self.position}, vel:{self.velocity}")
                gpio_toggle(self.GPIO)
                if simple_request:
                    self.socket.send(self.controller.serialize_basic(reply_requested=True))
                    self.sample_sensors(request_send=False)
                else:
                    self.socket.send(self.controller.serialize_motors())
                    self.sample_sensors(request_send=True)
                if self.controller.error_codes != 0:
                    self.logger.info(f"Corner {list(CORNER_NAMES)[self.name]} returned error codes {self.controller.error_codes}")
                    if self.controller.error_codes & motor_controller.ERROR_CODE_INDUCTION_ENCODER_OFFLINE != 0:
                        self.logger.error(f"Corner {list(CORNER_NAMES)[self.name]} reports encoder error! Cannot initialize corner.")
                    else:
                        self.socket.send(self.controller.clear_error_codes())
            except Exception as e:
                traceback.print_exc()
                self.logger.info(f"Send Error in corner {list(CORNER_NAMES)[self.name]}")
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
