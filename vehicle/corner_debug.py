
import time
import math
import pickle
import click
import argparse
from multiprocessing import Process, shared_memory
import subprocess
import os
import numpy as np
import traceback
import corner_actuator_can as corner_actuator
import model
from model import CORNER_NAMES
import isotp
import motor_controller as mot
import logging
from utils import config_logging
import random
import traceback
import sys
import RPi.GPIO as GPIO

import voltage_monitor
from pca9536 import PCA9536
from smbus2 import SMBus
from pca9536 import PinMode


bus = SMBus(6)
device = PCA9536(bus=bus)
pin = device[0]
pin.mode = PinMode.output  # or use "output"
pin.write(True)
time.sleep(1)
pin.write(False)
time.sleep(1)

disable_steering_limits = False

logger = logging.getLogger('motors')

config_logging(logger, debug=False)

connection = corner_actuator.MotorConnection(name=CORNER_NAMES['rear_left'], id=0x8,
                 port="can1", enable_steering=True, enable_traction=True, reverse_drive=False)


GPIO.setmode(GPIO.BCM)
GPIO.setup(corner_actuator.ESTOP_PIN, GPIO.OUT, initial=GPIO.LOW)
# This trick stores state for the estop line in a new variable
# attached to our GPIO object, though it probably doesnt need to be
# done this way.
GPIO.estop_state = GPIO.LOW

print(connection)
print(dir(connection))
drive = corner_actuator.CornerActuator(GPIO=GPIO,
                                        name=connection.name,
                                        connection_definition=connection,
                                        enable_steering=connection.enable_steering,
                                        enable_traction=connection.enable_traction,
                                        simulated_hardware = False,
                                        disable_steering_limits=disable_steering_limits,
                                        reverse_drive=connection.reverse_drive,
                                        logger=logger)

drive.socket.send(drive.controller.clear_error_codes())

drive.socket.send(drive.controller.clear_error_codes())

result = drive.set_home_position(1.57)

position = float(sys.argv[1])

if position > 8 or position < -8:
    sys.exit()

while True:
    drive.update_actuator(position, 0.0)
    error = position - drive.controller.motor1.steering_angle_radians
    print(error)
    # if abs(error) < 0.1:
    #     break
    time.sleep(0.02)
    # print("loop")
