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
import random
import collections
import logging
import odrive_manager
import fake_odrive_manager
from odrive.utils import _VT100Colors
from odrive.enums import ENCODER_MODE_INCREMENTAL
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_ENCODER_INDEX_SEARCH

import estop
import utils

logger = logging.getLogger(__name__)
utils.config_logging(logger, debug=False)

try:
    from odrive.enums import CTRL_MODE_POSITION_CONTROL, CTRL_MODE_VELOCITY_CONTROL
except ImportError:
    logger.info("WARNING: potentially incompatible odrive library version!")
    from odrive.enums import CONTROL_MODE_POSITION_CONTROL, CONTROL_MODE_VELOCITY_CONTROL
    CTRL_MODE_POSITION_CONTROL = CONTROL_MODE_POSITION_CONTROL
    CTRL_MODE_VELOCITY_CONTROL = CONTROL_MODE_VELOCITY_CONTROL

THERMISTOR_ADC_CHANNEL = 3
_ADC_PORT_STEERING_POT = 5
_ADC_PORT_STEERING_HOME = 6


_VCC = 3.3
_VOLTAGE_MIDPOINT = _VCC / 2
_POT_VOLTAGE_LOW = _VCC / 6
_POT_VOLTAGE_HIGH = 5 * _VCC / 6
_HOMING_VELOCITY_STEERING_COUNTS = 800
_HOMING_POT_DEADBAND_VOLTS = 0.2
_HOMING_POT_HALF_DEADBAND_V = _HOMING_POT_DEADBAND_VOLTS / 2
_FAST_POLLING_SLEEP_S = 0.01
_SLOW_POLLING_SLEEP_S = 0.5
_ODRIVE_CONNECT_TIMEOUT = 75
_MAX_HOMING_ATTEMPTS = 10
_ZERO_VEL_COUNTS_THRESHOLD = 2

COMMAND_VALUE_MINIMUM = 0.001

COUNTS_PER_REVOLUTION = 9797.0
COUNTS_PER_REVOLUTION_NEW_STEERING = COUNTS_PER_REVOLUTION * 520.0 / 2000.0

OdriveConnection = collections.namedtuple(
    'OdriveConnection', 'name serial path enable_steering enable_traction')

# class PowerSample:
#     def init(self, volts, amps, duration):
#
#
# class PowerTracker:
#     def init(self, interval):
#         self.recent_samples = []
#         self.collapse_samples_interval = interval

_last_nudge = time.time()


def estop_nudge():
    """wraps estop.nudge with error print to help spot functions taking longer than expected."""
    global _last_nudge
    now = time.time()
    if (elapsed := now - _last_nudge) > 1 / 20:
        logger.warning(f"{elapsed*1000}ms has passed since the last estop nudge.", stack_info=True)
    # uncomment below to find out unnecessary calls to nudge():
    # elif elapsed < 1 / 100:
    #     logger.warning(f"it took only {elapsed*1000}ms since the last estop nudge.", stack_info=True)
    _last_nudge = now
    estop.nudge()


def toggling_sleep(duration):
    start = time.time()
    while time.time() - start < duration:
        estop_nudge()
        time.sleep(_FAST_POLLING_SLEEP_S)


class CornerActuator:

    def __init__(self, serial_number=None, name=None, path=None,
                 connection_definition=None, enable_steering=True, enable_traction=True,
                 simulated_hardware=False, disable_steering_limits=False):
        if connection_definition:
            serial_number = connection_definition.serial
            name = connection_definition.name
            path = connection_definition.path
        if serial_number and not isinstance(serial_number, str):
            raise ValueError("serial_number must be of type str but got type: {}".format(
                type(serial_number)))

        if simulated_hardware:
            self.odrv0 = fake_odrive_manager.FakeOdriveManager(
                path=path, serial_number=serial_number).find_odrive()
        else:
            self.odrv0 = odrive_manager.OdriveManager(
                path=path, serial_number=serial_number).find_odrive()

        self.name = name
        self.enable_steering = enable_steering
        self.enable_traction = enable_traction
        self.steering_axis = self.odrv0.axis0
        self.traction_axis = self.odrv0.axis1
        self.steering_initialized = False
        self.traction_initialized = False
        self.position = 0.0
        self.velocity = 0.0
        self.voltage = 0.0
        self.home_position = 0
        self.steering_flipped = False
        self.has_thermistor = False
        self.temperature_c = None
        self.zero_vel_timestamp = None
        self.disable_steering_limits = disable_steering_limits
        self.simulated_hardware = simulated_hardware

        if self.simulated_hardware:
            self.odrv0.vbus_voltage = 45.5 + random.random() * 2.0

    def enable_thermistor(self):
        self.has_thermistor = True

    def idle_wait(self):
        while self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
            toggling_sleep(0.1)

    def print_errors(self, clear_errors=False):
        self.dump_errors(clear_errors)
        if clear_errors:
            self.position = 0
            self.velocity = 0

    def sample_home_sensor(self):
        estop_nudge()
        return self.odrv0.get_adc_voltage(_ADC_PORT_STEERING_HOME) < _VOLTAGE_MIDPOINT

    def sample_steering_pot(self):
        estop_nudge()
        return self.odrv0.get_adc_voltage(_ADC_PORT_STEERING_POT)

    def initialize_traction(self):
        estop_nudge()
        self.odrv0.axis1.controller.vel_integrator_current = 0
        self.odrv0.axis1.controller.config.vel_integrator_gain = 0.5
        self.odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.traction_initialized = True

    def check_steering_limits(self):
        rotation_sensor_val = self.sample_steering_pot()
        if self.disable_steering_limits:
            logger.info("{} steering sensor {}".format(
                self.name, rotation_sensor_val))
        if rotation_sensor_val < _POT_VOLTAGE_LOW or rotation_sensor_val > _POT_VOLTAGE_HIGH:
            if self.disable_steering_limits:
                logger.info("WARNING POTENTIOMETER VOLTAGE OUT OF RANGE DAMAGE " +
                            "MAY OCCUR: {}".format(rotation_sensor_val))
            else:
                raise ValueError(
                    "POTENTIOMETER VOLTAGE OUT OF RANGE: {}".format(rotation_sensor_val))

    def recover_from_estop(self):
        estop_nudge()
        self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.initialize_traction()

    def initialize_steering(self, steering_flipped=False, skip_homing=False):
        estop_nudge()
        self.check_steering_limits()
        self.enable_steering_control()

        self.steering_flipped = steering_flipped

        self.home_position = self.odrv0.axis0.encoder.pos_estimate
        if not skip_homing:
            last_home_sensor_val = self.sample_home_sensor()
            rotation_sensor_val = self.sample_steering_pot()
            logger.info("Rotation sensor voltage: {}".format(rotation_sensor_val))

            _HOMING_DISPLACEMENT_DEGREES = 25.0
            # _HOMING_DISPLACEMENT_RADIANS = math.radians(_HOMING_DISPLACEMENT_DEGREES)
            # positions = [_HOMING_DISPLACEMENT_DEGREES, -_HOMING_DISPLACEMENT_DEGREES]
            position = _HOMING_DISPLACEMENT_DEGREES

            tick_time_s = 3
            last_tick_time = 0
            transitions = []
            attempts = 0
            while True:
                toggling_sleep(_FAST_POLLING_SLEEP_S)
                home_sensor_val = self.sample_home_sensor()
                rotation_sensor_val = self.sample_steering_pot()
                logger.info("{} Home: {}, Rotation: {}".format(
                    self.name, home_sensor_val, rotation_sensor_val))
                if home_sensor_val:
                    self.home_position = self.odrv0.axis0.encoder.pos_estimate
                    break
                if home_sensor_val != last_home_sensor_val:
                    transitions.append(self.odrv0.axis0.encoder.pos_estimate)
                    logger.info(transitions)
                    attempts += 1
                # rotation_sensor_val = self.sample_steering_pot()
                # logger.info("{} Home: {}, Rotation: {}".format(self.name, home_sensor_val, rotation_sensor_val))

                # logger.info(home_sensor_val)

                if time.time() - last_tick_time > tick_time_s:
                    last_tick_time = time.time()
                    position *= -1
                    if self.name == 'rear_left':
                        pos_counts = self.home_position + position * \
                            COUNTS_PER_REVOLUTION_NEW_STEERING / 360.0 * -1.0
                    else:
                        pos_counts = self.home_position + position * COUNTS_PER_REVOLUTION / 360.0
                    self.odrv0.axis0.controller.move_to_pos(pos_counts)
                    # self.odrv0.axis0.controller.pos_setpoint = pos_counts

                if len(transitions) > 4:
                    self.home_position = sum(transitions) / len(transitions)
                    break

                if attempts > _MAX_HOMING_ATTEMPTS:
                    raise RuntimeError("Exceeded max homing attempts.")
                try:
                    self.check_errors()
                except RuntimeError:
                    raise RuntimeError("ODrive threw error during homing.")

                last_home_sensor_val = home_sensor_val
                try:
                    self.check_steering_limits()
                except Exception as e:
                    raise e  # Finally will be called before the raise.
                finally:
                    self.stop_actuator()

            # Save values.
            # self.home_position = (transitions[0] + transitions[1])/2
        else:
            self.home_position = self.odrv0.axis0.encoder.pos_estimate
        self.odrv0.axis0.controller.move_to_pos(self.home_position)

        self.steering_initialized = True

    def enable_steering_control(self):
        self.odrv0.axis0.encoder.config.use_index = False
        self.odrv0.axis0.motor.config.direction = 1
        self.odrv0.axis0.motor.config.motor_type = 4  # MOTOR_TYPE_BRUSHED_VOLTAGE
        self.odrv0.axis0.motor.config.current_lim = 20.0
        estop_nudge()
        if self.name == 'rear_left':
            self.odrv0.axis0.controller.config.vel_gain = 0.020
            self.odrv0.axis0.controller.config.pos_gain = -40
            self.odrv0.axis0.encoder.config.cpr = 520
            self.odrv0.axis0.controller.config.vel_ramp_rate = 80
            self.odrv0.axis0.trap_traj.config.vel_limit = 32000
            self.odrv0.axis0.trap_traj.config.accel_limit = 12000
            self.odrv0.axis0.trap_traj.config.decel_limit = 12000
            self.odrv0.axis0.trap_traj.config.A_per_css = 0
        else:
            self.odrv0.axis0.controller.config.vel_gain = 0.005
            self.odrv0.axis0.controller.config.pos_gain = -10
            self.odrv0.axis0.encoder.config.cpr = 2000
            self.odrv0.axis0.controller.config.vel_ramp_rate = 80
            self.odrv0.axis0.trap_traj.config.vel_limit = 16000
            self.odrv0.axis0.trap_traj.config.accel_limit = 6000
            self.odrv0.axis0.trap_traj.config.decel_limit = 6000
            self.odrv0.axis0.trap_traj.config.A_per_css = 0
        self.odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
        self.odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

        estop_nudge()
        self.odrv0.axis0.encoder.config.use_index = False
        err_count = 0
        while True:
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            toggling_sleep(_SLOW_POLLING_SLEEP_S)
            self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis0.controller.vel_ramp_enable = True
            # self.odrv0.axis0.controller.config.vel_limit_tolerance = 2.5
            toggling_sleep(_SLOW_POLLING_SLEEP_S)
            errors = self.odrv0.axis0.error
            if errors == 0:
                break
            if not self.odrv0.axis0.encoder.is_ready:
                raise RuntimeError(
                    "Could not initialize steering. Encoder is not ready.")

            # logger.info("axis0.current_state: {}".format(self.odrv0.axis0.current_state))
            # self.odrv0.axis0.controller.vel_integrator_current = 0
            # self.odrv0.axis1.controller.vel_integrator_current = 0
            # self.odrv0.axis0.controller.vel_setpoint = 0
            # self.odrv0.axis1.controller.vel_setpoint = 0
            err_count += 1
            error_message = self.dump_errors(True)
            estop_nudge()
            if err_count > 3:
                raise RuntimeError(
                    "Could not initialize steering. Error was: {}".format(error_message))
            # TODO: Is this sleep time reasonable? Should also make it a variable.
            toggling_sleep(5)

    def check_errors(self):
        if (self.enable_traction and self.traction_axis.error
                or self.enable_steering and self.steering_axis.error):
            estop_nudge()
            self.dump_errors()
            self.update_voltage()
            raise RuntimeError("Odrive traction motor error state detected.")

    def update_voltage(self):
        estop_nudge()
        if self.simulated_hardware:
            self.odrv0.vbus_voltage = 45.5 + random.random() * 2.0
        self.voltage = self.odrv0.vbus_voltage
        self.ibus_0 = self.odrv0.axis0.motor.current_control.Ibus
        estop_nudge()
        self.ibus_1 = self.odrv0.axis1.motor.current_control.Ibus

    def update_actuator(self, steering_pos_deg, drive_velocity):
        # logger.info("Update {}".format(self.name))
        self.check_steering_limits()
        estop_nudge()
        if self.steering_flipped:
            drive_velocity *= -1
        self.position = steering_pos_deg
        self.velocity = drive_velocity
        # logger.info(drive_velocity)
        self.update_voltage()
        # if self.steering_flipped:
        #     pos_counts = self.home_position + (180 + steering_pos_deg) * COUNTS_PER_REVOLUTION / 360.0
        # else:
        if self.name == 'rear_left':
            pos_counts = self.home_position + steering_pos_deg * \
                COUNTS_PER_REVOLUTION_NEW_STEERING / 360.0 * -1.0
        else:
            pos_counts = self.home_position + steering_pos_deg * COUNTS_PER_REVOLUTION / 360.0
        # if self.name=='rear_left':
        #    logger.info("pos_counts: {}, shadow_count: {}, count_in_cpr: {}".format(pos_counts, self.odrv0.axis0.encoder.shadow_count,self.odrv0.axis0.encoder.count_in_cpr))
        # self.odrv0.axis0.controller.pos_setpoint = pos_counts
        self.odrv0.axis0.controller.move_to_pos(pos_counts)
        estop_nudge()
        # TODO: Setting vel_integrator_current to zero every time we update
        # means we just don't get integrator control. But that would be nice.

        if abs(self.velocity) > _ZERO_VEL_COUNTS_THRESHOLD:
            self.zero_vel_timestamp = None
        else:
            if self.zero_vel_timestamp is None:
                self.zero_vel_timestamp = time.time()
            else:
                if time.time() - self.zero_vel_timestamp > 5.0:
                    self.odrv0.axis1.controller.vel_integrator_current = 0
        self.odrv0.axis1.controller.vel_setpoint = self.velocity
        self.check_errors()

    def slow_actuator(self, fraction):
        estop_nudge()
        if not (self.steering_initialized and self.traction_initialized):
            return
        self.position = fraction * self.position
        self.velocity = fraction * self.velocity
        if math.fabs(self.position) < COMMAND_VALUE_MINIMUM:
            self.position = 0.0
        if math.fabs(self.velocity) < COMMAND_VALUE_MINIMUM:
            self.velocity = 0.0
        self.update_actuator(self.position, self.velocity)

    def stop_actuator(self):
        estop_nudge()
        self.odrv0.axis0.controller.vel_setpoint = 0
        self.odrv0.axis1.controller.vel_setpoint = 0

    def update_thermistor_temperature_C(self, adc_channel=THERMISTOR_ADC_CHANNEL):
        if not self.has_thermistor:
            return
        try:
            estop_nudge()
            value = self.odrv0.get_adc_voltage(adc_channel)
            resistance = 10000 / (3.3 / value)
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

    def dump_errors(self, clear=False):
        """A copy of dump_errors from odrive utils.py but with estop_nudge added."""
        axes = [(name, axis) for name,
                axis in self.odrv0._remote_attributes.items() if 'axis' in name]
        axes.sort()
        for name, axis in axes:
            logger.info(name)

            # Flatten axis and submodules
            # (name, remote_obj, errorcode)
            module_decode_map = [
                ('axis', axis, errors.axis),
                ('motor', axis.motor, errors.motor),
                ('encoder', axis.encoder, errors.encoder),
                ('controller', axis.controller, errors.controller),
            ]

            # Module error decode
            for name, remote_obj, errorcodes in module_decode_map:
                prefix = ' ' * 2 + name + ": "
                if (remote_obj.error != errorcodes.ERROR_NONE):
                    logger.info(prefix + _VT100Colors['red'] +
                                "Error(s):" + _VT100Colors['default'])
                    errorcodes_tup = [
                        (name, val) for name, val in errorcodes.__dict__.items() if 'ERROR_' in name]
                    for codename, codeval in errorcodes_tup:
                        if remote_obj.error & codeval != 0:
                            logger.info("    " + codename)
                    if clear:
                        remote_obj.error = errorcodes.ERROR_NONE
                else:
                    logger.info(prefix + _VT100Colors['green'] +
                                "no error" + _VT100Colors['default'])
