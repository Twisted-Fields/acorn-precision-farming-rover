#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import math
import sys
import fibre

DO_FULL_SETUP = False
#DO_FULL_SETUP = True

USE_BRUSHED_MOTOR = False


def idle_wait():
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    print(dump_errors(odrv0))

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
odrv0 = odrive.find_any()

print(dump_errors(odrv0,True))

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#odrv0 = odrive.find_any("serial:/dev/ttyUSB0")




SETUP_1 = False
#SETUP_1 = True

if SETUP_1 or DO_FULL_SETUP:

    odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
    odrv0.axis1.motor.config.requested_current_range = 25 #Requires config save and reboot

    odrv0.save_configuration()
    try:
        odrv0.reboot()
    except fibre.protocol.ChannelBrokenException:
        pass
    odrv0 = odrive.find_any()
    print("Step 1 complete.")

SETUP_2 = False
#SETUP_2 = True

if SETUP_2 or DO_FULL_SETUP:
    odrv0.axis1.motor.config.current_control_bandwidth = 100
    odrv0.config.brake_resistance = 0

    odrv0.axis1.motor.config.pole_pairs = 4

    odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
    odrv0.axis1.encoder.config.cpr = 24

    odrv0.axis1.motor.config.requested_current_range = 25

    odrv0.axis1.encoder.config.bandwidth = 100
    odrv0.axis1.controller.config.pos_gain = 1
    odrv0.axis1.controller.config.vel_gain = 0.02
    odrv0.axis1.controller.config.vel_integrator_gain = 0.1
    odrv0.axis1.controller.config.vel_limit = 1000
    odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    #
    odrv0.save_configuration()
    try:
        odrv0.reboot()
    except fibre.protocol.ChannelBrokenException:
        pass
    odrv0 = odrive.find_any()
    print("Step 2 complete.")


# # Calibrate motor and wait for it to finish
SETUP_3 = False
#SETUP_3 = True
if SETUP_3 or DO_FULL_SETUP:
    print("starting calibration...")
    odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    idle_wait()
    print(odrv0.axis1.motor)
    odrv0.axis1.motor.config.pre_calibrated = True
    odrv0.save_configuration()
    try:
        odrv0.reboot()
    except fibre.protocol.ChannelBrokenException:
        pass
    odrv0 = odrive.find_any()
    print("Step 3 complete.")

SETUP_4 = False
#SETUP_4 = True
if SETUP_4 or DO_FULL_SETUP:
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    idle_wait()
    print(odrv0.axis1.encoder)
    odrv0.axis1.encoder.config.pre_calibrated = True


    #odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #idle_wait()
    odrv0.save_configuration()
    try:
        odrv0.reboot()
    except fibre.protocol.ChannelBrokenException:
        pass
    odrv0 = odrive.find_any()
    print("Step 4 complete.")


SETUP_5 = False
#SETUP_5 = True
if SETUP_5:
    odrv0.axis0.encoder.config.use_index = True
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    idle_wait()
    odrv0.axis0.encoder.config.pre_calibrated = True
    odrv0.axis0.config.startup_encoder_index_search = True
    odrv0.save_configuration()
    try:
        odrv0.reboot()
    except fibre.protocol.ChannelBrokenException:
        pass
    odrv0 = odrive.find_any()


if USE_BRUSHED_MOTOR:
    odrv0.axis0.motor.config.direction = 1
    #odrv0.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
    odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_BRUSHED_VOLTAGE
    #odrv0.axis0.requested_state = AXIS_STATE_BRUSHED_VOLTAGE_CONTROL
    odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# To read a value, simply read the property
print("Bus voltage is " + str(odrv0.vbus_voltage) + "V")

# # Or to change a value, just assign to the property
# odrv0.axis1.controller.pos_setpoint = 3.14
# print("Position setpoint is " + str(odrv0.axis1.controller.pos_setpoint))

# # And this is how function calls are done:
# for i in [1,2,3,4]:
#     print('voltage on GPIO{} is {} Volt'.format(i, odrv0.get_adc_voltage(i)))

# A sine wave to test
t0 = time.monotonic()
try:
    while True:
        setpoint = 1000.0 * math.sin(((time.monotonic() - t0)/4)*2)
        #print("goto " + str(int(setpoint)))
        #setpoint = 200
        odrv0.axis1.controller.vel_setpoint = setpoint

        odrv0.axis1.controller.vel_setpoint = 2000
        time.sleep(0.2)
        odrv0.axis1.controller.vel_setpoint = -2000
        time.sleep(0.2)
        if USE_BRUSHED_MOTOR:
            odrv0.axis0.controller.current_setpoint = setpoint / 1000.0 * 24.0
        #print(setpoint / 1000.0)
        if odrv0.axis0.error:
            print("error! Setpoint: {}, axis0: {}, motor: {}".format(setpoint / 1000.0, odrv0.axis0.error,odrv0.axis0.motor.error))
            print(dump_errors(odrv0,True))
            break
        if odrv0.axis1.error:
            print("error! Setpoint: {}, axis1: {}, motor: {}".format(setpoint / 1000.0, odrv0.axis1.error,odrv0.axis1.motor.error))
            print(dump_errors(odrv0,True))
            break
        #time.sleep(0.05)
except KeyboardInterrupt:
    odrv0.axis1.controller.vel_setpoint = 0
    odrv0.axis0.controller.current_setpoint = 0.0
