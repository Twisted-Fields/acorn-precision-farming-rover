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
    while odrv0.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    print(dump_errors(odrv0))

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")

#odrv0 = OdriveManager(path='/dev/ttyACM0', serial_number='336B31643536').find_odrive()
odrv0 = odrive.find_any()

odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#odrv0 = odrive.find_any("serial:/dev/ttyUSB0")

# odrv0.axis0.motor.DC_calib_phB = 0.0
# odrv0.axis0.motor.DC_calib_phC = 0.0
print(dump_errors(odrv0,True))
odrv0.axis0.motor.config.current_control_bandwidth = 20
odrv0.axis0.encoder.config.bandwidth = 100

print(dump_errors(odrv0,True))
odrv0.axis0.requested_state = 11 # AXIS_STATE_BRUSHED_CURRENT_CONTROL
#odrv0.axis0.requested_state = 12 # AXIS_STATE_BRUSHED_CURRENT_CONTROL
odrv0.axis0.motor.current_control.p_gain = 10.0
odrv0.axis0.motor.current_control.i_gain = 0.1
odrv0.axis0.motor.config.phase_resistance = 2.5
odrv0.axis0.motor.config.phase_inductance = 0.001

print(odrv0.axis0)
print()
print("motor.current_control:")
print(odrv0.axis0.motor.current_control)
print()
print("motor.config:")
print(odrv0.axis0.motor.config)



odrv0.axis0.motor.current_control.final_v_beta = 1.0 # Voltage Ramp Rate


max_vel = 0
reverse = False
reverse_start_time = 0
reverse_duration_allowed_sec = 4
ctrl = odrv0.axis0.motor.current_control
start_time = time.time()
reverse_position = 0

print(odrv0.axis0.encoder.pos_estimate)
print(odrv0.axis0.encoder.pos_estimate)

# odrv0.axis0.controller.current_setpoint = 3.0
# time.sleep(2)
# odrv0.axis0.controller.current_setpoint = 1
# time.sleep(2)
# odrv0.axis0.controller.current_setpoint = 0
# sys.exit()


odrv0.axis0.controller.current_setpoint = 0

while odrv0.axis0.controller.current_setpoint < 3.0:
    odrv0.axis0.controller.current_setpoint += 0.01
    time.sleep(0.01)




try:
    while True:
        print("current setpoint {:0.2f}, voltage: {:0.2f}, PH B: {:0.2f}, PH C: {:0.2f}, Iq_measured {:0.2f}, bus voltage {:0.2f}, vel estimate {:0.2f}, max_vel {:0.2f}.".format(odrv0.axis0.controller.current_setpoint, ctrl.final_v_alpha, odrv0.axis0.motor.current_meas_phB, odrv0.axis0.motor.current_meas_phC, ctrl.Iq_measured, odrv0.vbus_voltage, odrv0.axis0.encoder.vel_estimate, max_vel))
        odrv0.axis0.controller.current_setpoint *=1.2
        if odrv0.axis0.error:
            print(dump_errors(odrv0,True))
            print("Gate Driver: {}".format(odrv0.axis0.motor.gate_driver))
            print("current setpoint {:0.2f}, voltage: {:0.2f}, PH B: {:0.2f}, PH C: {:0.2f}, Iq_measured {:0.2f}, bus voltage {:0.2f}, vel estimate {:0.2f}, max_vel {:0.2f}.".format(odrv0.axis0.controller.current_setpoint, ctrl.final_v_alpha, odrv0.axis0.motor.current_meas_phB, odrv0.axis0.motor.current_meas_phC, ctrl.Iq_measured, odrv0.vbus_voltage, odrv0.axis0.encoder.vel_estimate, max_vel))
            sys.exit()
        time.sleep(1)
except KeyboardInterrupt:
    odrv0.axis0.controller.current_setpoint = 0.0
    odrv0.axis0.controller.current_setpoint = 0.0
    odrv0.axis0.requested_state = 1
    odrv0.axis0.requested_state = 1
    time.sleep(1)
    sys.exit()
# time.sleep(100)

# This is a current-based hard block homing routine.
while True:
    print("current setpoint {}, voltage: {}, Iq_measured {}, bus voltage {}, vel estimate {}, max_vel {}.".format(odrv0.axis0.controller.current_setpoint, ctrl.final_v_alpha, ctrl.Iq_measured, odrv0.vbus_voltage, odrv0.axis0.encoder.vel_estimate, max_vel))
    # if reverse:
    #     if odrv0.axis0.encoder.pos_estimate - reverse_position > 400 or time.time() - reverse_start_time > reverse_duration_allowed_sec:
    #         reverse = False
    #         odrv0.axis0.controller.current_setpoint *= -1
    #         max_vel = 0
    #         start_time = time.time()
    #         print(odrv0.axis0.encoder.pos_estimate)
    #         print(odrv0.axis0.encoder.pos_estimate)
    # else:
    if abs(odrv0.axis0.encoder.vel_estimate) > max_vel:
        max_vel = abs(odrv0.axis0.encoder.vel_estimate)
    if max_vel > 100 and abs(odrv0.axis0.encoder.vel_estimate) < 10:
        print("Contact")
        time.sleep(10)
        break
    if max_vel < 400 and time.time()-start_time > 2.0:
        odrv0.axis0.controller.current_setpoint *= 1.1
        #reverse = True
        reverse_start_time = time.time()
        reverse_position = odrv0.axis0.encoder.pos_estimate
    #if max_vel >
    time.sleep(0.1)


print(dump_errors(odrv0,True))
print(odrv0.axis0.encoder.pos_estimate)
print(odrv0.axis0.encoder.pos_estimate)
print(odrv0.axis0.encoder.pos_estimate)
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
idle_wait()
print("Index Search complete")
print(dump_errors(odrv0,True))

print(odrv0.axis0.encoder.pos_estimate)
print(odrv0.axis0.encoder.pos_estimate)



print("Bus voltage is " + str(odrv0.vbus_voltage) + "V")


timer = time.time()

time.sleep(2)

if True:
    odrv0.axis0.encoder.config.bandwidth = 100
    odrv0.axis0.controller.config.vel_ramp_rate = 30
    odrv0.axis0.controller.config.vel_gain = -0.03
    odrv0.axis0.controller.config.vel_integrator_gain = 0
    odrv0.axis0.controller.vel_integrator_current = 0
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.pos_gain = 2
    odrv0.axis0.controller.vel_ramp_enable = True
    odrv0.axis0.controller.config.vel_ramp_rate = 1

    odrv0.axis0.motor.config.current_lim = 3.0
    odrv0.axis0.controller.config.vel_limit_tolerance = 2.5
    odrv0.axis0.controller.config.vel_limit = 200
    odrv0.axis0.trap_traj.config.vel_limit = 200
    odrv0.axis0.trap_traj.config.accel_limit = 50
    odrv0.axis0.trap_traj.config.decel_limit = 50
    odrv0.axis0.trap_traj.config.A_per_css = 0

    #base_position = 1130/2
    base_position = 0
    offset = 400
    odrv0.axis0.controller.pos_setpoint = base_position
    min = base_position - offset
    count = 0
    steps = 6
    #sys.exit()
    while True:
        print("For setpoint {}, estimate is {}, current: {}, bus_voltage {}".format(odrv0.axis0.controller.pos_setpoint, odrv0.axis0.encoder.pos_estimate, ctrl.Iq_measured, odrv0.vbus_voltage))
        #odrv0.axis0.encoder.pos_estimate
        #print(dump_errors(odrv0))
        time.sleep(0.1)
        if time.time() - timer > 0.5:
            timer = time.time()
            count += 1
            #offset *= -1
            #odrv0.axis0.controller.move_to_pos(base_position + offset)
            if count > steps:
                count = 0
            odrv0.axis0.controller.move_to_pos(min + (2*offset)*count/steps)
            if count == 0:
                timer += 1.0
                #time.sleep(1)
        if odrv0.axis0.error:
            print(dump_errors(odrv0,True))
            print("Gate Driver: {}".format(odrv0.axis0.motor.gate_driver))
            print("For setpoint {}, estimate is {}, current: {}, bus_voltage {}".format(odrv0.axis0.controller.pos_setpoint, odrv0.axis0.encoder.pos_estimate, ctrl.Iq_measured, odrv0.vbus_voltage))
            sys.exit()
