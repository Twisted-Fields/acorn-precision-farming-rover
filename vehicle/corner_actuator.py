#!/usr/bin/env python3
"""
Control one corner of the Acorn robot via the Odrive motor controller.
"""

from __future__ import print_function

import odrive_manager
from odrive.enums import *
from odrive.utils import dump_errors
import time
import math
import sys
import fibre

_ADC_PORT_STEERING_POT = 4
_ADC_PORT_STEERING_HOME = 3
_VCC = 3.3
_VOLTAGE_MIDPOINT = _VCC/2
_POT_VOLTAGE_LOW = _VCC/6
_POT_VOLTAGE_HIGH = 5*_VCC/6
_POT_VOLTAGE_LOW_WARN = _VCC/5
_POT_VOLTAGE_HIGH_WARN = 4*_VCC/5
_HOMING_VELOCITY_STEERING_COUNTS = 800
_HOMING_POT_DEADBAND_VOLTS = 0.2
_HOMING_POT_HALF_DEADBAND_V = _HOMING_POT_DEADBAND_VOLTS/2
_FAST_POLLING_SLEEP_S = 0.01
_SLOW_POLLING_SLEEP_S = 0.5
_ODRIVE_CONNECT_TIMEOUT = 75

COMMAND_VALUE_MINIMUM = 0.001

COUNTS_PER_REVOLUTION = 9797.0


# od = OdriveManager(path='/dev/ttySC3', serial_number='336B31643536').find_odrive()

class CornerActuator:

    def __init__(self, serial_number=None, name=None, path=None):
        if serial_number and not isinstance(serial_number, str):
            raise ValueError("serial_number must be of type str but got type: {}".format(type(serial_number)))

        self.odrv0 = odrive_manager.OdriveManager(path=path, serial_number=serial_number).find_odrive()

        # if path:
        #     self.odrv0 = odrive.find_any(path=path, timeout=_ODRIVE_CONNECT_TIMEOUT)
        # elif not serial_number:
        #     self.odrv0 = odrive.find_any(timeout=_ODRIVE_CONNECT_TIMEOUT)
        # else:
        #     self.odrv0 = odrive.find_any(serial_number=serial_number, timeout=_ODRIVE_CONNECT_TIMEOUT)
        #     #self.odrv0 = odrive.find_any(path="usb:1-1.1.3.4", timeout=_ODRIVE_CONNECT_TIMEOUT)
        self.name = name
        self.steering_initialized = False
        self.traction_initialized = False
        self.position = 0.0
        self.velocity = 0.0

    def idle_wait(self):
        while self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

    def print_errors(self, clear_errors=False):
        dump_errors(self.odrv0, clear_errors)
        if clear_errors:
            self.position = 0
            self.velocity = 0

    def sample_home_sensor(self):
        return self.odrv0.get_adc_voltage(_ADC_PORT_STEERING_HOME) < _VOLTAGE_MIDPOINT

    def sample_steering_pot(self):
        return self.odrv0.get_adc_voltage(_ADC_PORT_STEERING_POT)

    def initialize_traction(self):
        self.odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.traction_initialized = True

    def recover_from_estop(self):
        self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.initialize_traction()

    def initialize_steering(self, steering_flipped=False, skip_homing=False):
        rotation_sensor_val = self.odrv0.get_adc_voltage(_ADC_PORT_STEERING_POT)
        if rotation_sensor_val < _POT_VOLTAGE_LOW or rotation_sensor_val > _POT_VOLTAGE_HIGH:
            raise ValueError("POTENTIOMETER VOLTAGE OUT OF RANGE: {}".format(rotation_sensor_val))
        self.odrv0.axis0.encoder.config.use_index=False
        self.odrv0.axis0.motor.config.direction = 1
        self.odrv0.axis0.motor.config.motor_type = 4 # MOTOR_TYPE_BRUSHED_VOLTAGE
        self.odrv0.axis0.motor.config.current_lim = 18.0
        self.odrv0.axis0.controller.config.vel_gain = 0.005
        self.odrv0.axis0.controller.config.pos_gain = -5
        self.odrv0.axis0.encoder.config.cpr = 2000
        self.odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
        self.odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

        self.odrv0.axis0.encoder.config.use_index=False
        self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.steering_flipped = steering_flipped
        self.steering_initialized = True


        home_position = None
        steering_midpoint = None
        if not skip_homing:
            last_home_sensor_val = self.sample_home_sensor()
            if last_home_sensor_val:
                print("STARTED ON HOME SENSOR")
            ignore_first_transition = last_home_sensor_val
            transitions = []
            print("START HOME")

            rotation_sensor_val = self.sample_steering_pot()
            print(rotation_sensor_val)
            if rotation_sensor_val < _VOLTAGE_MIDPOINT:
                self.odrv0.axis0.controller.vel_integrator_current = 0
                self.odrv0.axis0.controller.vel_setpoint = _HOMING_VELOCITY_STEERING_COUNTS
            else:
                self.odrv0.axis0.controller.vel_integrator_current = 0
                self.odrv0.axis0.controller.vel_setpoint = -_HOMING_VELOCITY_STEERING_COUNTS

            while True:
                time.sleep(_FAST_POLLING_SLEEP_S)
                home_sensor_val = self.sample_home_sensor()
                rotation_sensor_val = self.sample_steering_pot()
                #print(rotation_sensor_val)
                if self.odrv0.axis0.error or self.odrv0.axis1.error:
                    print("Error on {}".format(self.name))
                    dump_errors(self.odrv0)
                    self.stop_actuator()
                    raise RuntimeError("ODrive threw error during homing.")

                if home_sensor_val != last_home_sensor_val:
                  if ignore_first_transition:
                      # Skip if it started activated.
                      ignore_first_transition = False
                  else:
                      print("TRANSITION")
                      if not steering_midpoint and len(transitions) >= 2:
                          # If we got another transition but havent hit steering
                          # midpoint, empty the transitions. This ensures we home
                          # near the steering midpoint.
                          transitions = []
                      transitions.append(self.odrv0.axis0.encoder.shadow_count)
                last_home_sensor_val = home_sensor_val
                if _VOLTAGE_MIDPOINT-_HOMING_POT_HALF_DEADBAND_V < rotation_sensor_val < _VOLTAGE_MIDPOINT + _HOMING_POT_HALF_DEADBAND_V:
                    steering_midpoint = self.odrv0.axis0.encoder.shadow_count
                if rotation_sensor_val < _POT_VOLTAGE_LOW_WARN or rotation_sensor_val > _POT_VOLTAGE_HIGH_WARN:
                  self.stop_actuator()
                  raise ValueError("POTENTIOMETER VOLTAGE OUT OF RANGE: {}".format(rotation_sensor_val))
                if len(transitions) >= 2:
                    if not steering_midpoint:
                        steering_midpoint = self.odrv0.axis0.encoder.shadow_count
                    break

            print(transitions)
            # Save values.
            self.steering_midpoint = steering_midpoint
            self.home_position = (transitions[0] + transitions[1])/2
        else:
            if self.steering_flipped:
                self.home_position = self.odrv0.axis0.encoder.shadow_count - COUNTS_PER_REVOLUTION / 2.0
            else:
                self.home_position = self.odrv0.axis0.encoder.shadow_count
            self.steering_midpoint = self.odrv0.axis0.encoder.shadow_count


        err_count = 0
        while True:
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            time.sleep(_SLOW_POLLING_SLEEP_S)
            self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(_SLOW_POLLING_SLEEP_S)
            errors = self.odrv0.axis0.error
            if errors == 0:
                break
            if not self.odrv0.axis0.encoder.is_ready:
                raise RuntimeError("Could not initialize steering. Encoder is not ready.")

            # print("axis0.current_state: {}".format(self.odrv0.axis0.current_state))
            # self.odrv0.axis0.controller.vel_integrator_current = 0
            # self.odrv0.axis1.controller.vel_integrator_current = 0
            # self.odrv0.axis0.controller.vel_setpoint = 0
            # self.odrv0.axis1.controller.vel_setpoint = 0
            err_count += 1
            error_message = dump_errors(self.odrv0,True)
            if err_count > 3:
                raise RuntimeError("Could not initialize steering. Error was: {}".format(error_message))
            time.sleep(5)

    def check_errors(self):
        if self.odrv0.axis0.error or self.odrv0.axis1.error:
            dump_errors(self.odrv0)
            raise RuntimeError("odrive error state detected.")

    def update_actuator(self, steering_pos_deg, drive_velocity):
        #print("Update {}".format(self.name))
        self.position = steering_pos_deg
        self.velocity = drive_velocity
        if self.steering_flipped:
            pos_counts = self.home_position + (180 + steering_pos_deg) * COUNTS_PER_REVOLUTION / 360.0
        else:
            pos_counts = self.home_position + steering_pos_deg * COUNTS_PER_REVOLUTION / 360.0
        self.odrv0.axis0.controller.pos_setpoint = pos_counts
        # TODO: Setting vel_integrator_current to zero every time we update
        # means we just don't get integrator control. But that would be nice.
        self.odrv0.axis1.controller.vel_integrator_current = 0
        self.odrv0.axis1.controller.vel_setpoint = self.velocity
        self.check_errors()

    def slow_actuator(self, fraction):
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
        self.odrv0.axis0.controller.vel_setpoint = 0
        self.odrv0.axis1.controller.vel_setpoint = 0
