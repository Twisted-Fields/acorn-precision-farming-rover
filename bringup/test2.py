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
from odrive_manager import OdriveManager


def idle_wait():
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    print(dump_errors(odrv0))


# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")

odrv0 = OdriveManager(path='/dev/ttySC0',
                      serial_number='336B31643536').find_odrive()
# odrv0 = odrive.find_any()


print(dump_errors(odrv0, True))
print(dump_errors(odrv0, True))

odrv0.axis1.requested_state = AXIS_STATE_IDLE
odrv0.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#odrv0 = odrive.find_any("serial:/dev/ttyUSB0")


odrv0.axis1.motor.config.current_control_bandwidth = 20
odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.motor.current_control.p_gain = 0.3
odrv0.axis1.motor.current_control.i_gain = 0
odrv0.axis1.motor.current_control.final_v_beta = 0.1  # Voltage Ramp Rate


odrv0.axis1.requested_state = 11
odrv0.axis1.motor.current_control.p_gain = 50.0
odrv0.axis1.motor.current_control.i_gain = 0.0


odrv0.axis1.motor.current_control.final_v_beta = 0.1  # Voltage Ramp Rate


# max_vel = 0
# reverse = False
# reverse_start_time = 0
# reverse_duration_allowed_sec = 4
#
# start_time = time.time()
# reverse_position = 0

print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)

# odrv0.axis1.controller.current_setpoint = 6
# time.sleep(0.1)
# odrv0.axis1.controller.current_setpoint = 2
#
# time.sleep(0.2)


odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
idle_wait()


odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.controller.config.vel_ramp_rate = 10
odrv0.axis1.controller.config.vel_gain = -0.003
odrv0.axis1.controller.config.vel_integrator_gain = -0.005
odrv0.axis1.controller.vel_integrator_current = 0
odrv0.axis1.requested_state = AXIS_STATE_IDLE
odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.motor.config.current_lim = 20.0
odrv0.axis1.controller.config.vel_limit_tolerance = 2.5
odrv0.axis1.controller.config.vel_limit = 2000


backoff = True
# backoff = False

if backoff:
    odrv0.axis1.controller.vel_setpoint = -500
    time.sleep(0.5)
    odrv0.axis1.controller.vel_setpoint = 0

odrv0.axis1.controller.vel_setpoint = 500

loop = True
while loop:
    value = odrv0.get_adc_voltage(3)
    pos = odrv0.axis1.encoder.pos_estimate
    vel = odrv0.axis1.encoder.vel_estimate
    print("pos: {}, vel {}".format(pos, vel))
    # print(value)
    if value < 1.5:
        odrv0.axis1.controller.vel_setpoint = 0
        loop = False
    time.sleep(0.01)


print(dump_errors(odrv0, True))
print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
idle_wait()
print("Index Search complete")
print(dump_errors(odrv0, True))

print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)


#
print("Bus voltage is " + str(odrv0.vbus_voltage) + "V")


timer = time.time()

# time.sleep(1)


# odrv0.axis1.encoder.config.bandwidth = 100
# odrv0.axis1.controller.config.vel_ramp_rate = 10
# odrv0.axis1.controller.config.vel_gain = -0.002
# odrv0.axis1.controller.config.vel_integrator_gain = 0#-0.001
# odrv0.axis1.controller.vel_integrator_current = 0
# odrv0.axis1.requested_state = AXIS_STATE_IDLE
# odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
# odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# odrv0.axis1.motor.config.current_lim = 20.0
# odrv0.axis1.controller.config.vel_limit_tolerance = 2.5
# odrv0.axis1.controller.config.vel_limit = 2000


# odrv0.axis1.controller.vel_setpoint = -500
# time.sleep(1)
# # odrv0.axis1.controller.vel_setpoint = 0
# # odrv0.axis1.controller.current_setpoint = 0
# print(dump_errors(odrv0,True))
# sys.exit()
odrv0.axis1.controller.config.vel_gain = -0.002
odrv0.axis1.controller.config.vel_integrator_gain = 0

ctrl = odrv0.axis1.motor.current_control

if True:
    odrv0.axis1.encoder.config.bandwidth = 1000
    # odrv0.axis1.controller.config.vel_ramp_rate = 3000
    # odrv0.axis1.controller.config.vel_gain = -0.03
    # odrv0.axis1.controller.config.vel_integrator_gain = 0
    # odrv0.axis1.controller.vel_integrator_current = 0
    odrv0.axis1.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.pos_gain = 70
    odrv0.axis1.controller.vel_ramp_enable = True
    odrv0.axis1.controller.config.vel_ramp_rate = 10

    # odrv0.axis1.controller.config.vel_limit_tolerance = 2.5
    # odrv0.axis1.controller.config.vel_limit = 8000
    odrv0.axis1.trap_traj.config.vel_limit = 8000
    odrv0.axis1.trap_traj.config.accel_limit = 1500
    odrv0.axis1.trap_traj.config.decel_limit = 1500
    odrv0.axis1.trap_traj.config.A_per_css = 0

    #base_position = 1130/2
    base_position = -280
    offset = 450
    # odrv0.axis1.controller.pos_setpoint = base_position
    # odrv0.axis1.controller.move_to_pos(position)
    delay = 2
    positions = [base_position, base_position -
                 offset, base_position, base_position + offset]
    while True:
        for position in positions:
            try:
                time.sleep(delay)
                odrv0.axis1.controller.move_to_pos(position)
                print("For setpoint {}, estimate is {}, current: {}, bus_voltage {}".format(
                    odrv0.axis1.controller.pos_setpoint, odrv0.axis1.encoder.pos_estimate, ctrl.Iq_measured, odrv0.vbus_voltage))
            except KeyboardInterrupt:
                odrv0.axis1.requested_state = AXIS_STATE_IDLE
                sys.exit()

    sys.exit()
    min = base_position - offset
    count = 0
    steps = 6
    # sys.exit()
    while True:
        print("For setpoint {}, estimate is {}, current: {}, bus_voltage {}".format(
            odrv0.axis1.controller.pos_setpoint, odrv0.axis1.encoder.pos_estimate, ctrl.Iq_measured, odrv0.vbus_voltage))
        # odrv0.axis1.encoder.pos_estimate
        # print(dump_errors(odrv0))
        time.sleep(0.1)
        if time.time() - timer > 0.5:
            timer = time.time()
            count += 1
            #offset *= -1
            #odrv0.axis1.controller.move_to_pos(base_position + offset)
            if count > steps:
                count = 0
            odrv0.axis1.controller.move_to_pos(min + (2*offset)*count/steps)
            if count == 0:
                timer += 1.0
                # time.sleep(1)
        if odrv0.axis1.error:
            print(dump_errors(odrv0, True))
            print("Gate Driver: {}".format(odrv0.axis1.motor.gate_driver))
            print("For setpoint {}, estimate is {}, current: {}, bus_voltage {}".format(
                odrv0.axis1.controller.pos_setpoint, odrv0.axis1.encoder.pos_estimate, ctrl.Iq_measured, odrv0.vbus_voltage))
            sys.exit()
