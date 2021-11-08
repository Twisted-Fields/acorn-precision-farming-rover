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
import motors
import fibre
import os
from multiprocessing import Process
import argparse
import click
import pickle
import zmq
from steering import calculate_steering
from evdev import InputDevice, list_devices, categorize, ecodes, KeyEvent
from odrive.utils import dump_errors
import math
import time
import serial
from corner_actuator import COUNTS_PER_REVOLUTION, OdriveConnection
import corner_actuator
import sys
sys.path.append('../vehicle')


control = motors.AcornMotorInterface(manual_control=True)
control.odrive_connections = [
    OdriveConnection(name='front_right',
                     serial="335E31483536", path="/dev/ttySC1"),
    #OdriveConnection(name='front_left', serial="335B314C3536", path="/dev/ttySC0"),
    #OdriveConnection(name='rear_right', serial="3352316E3536", path="/dev/ttySC2"),
    #OdriveConnection(name='rear_left', serial="205F3882304E", path="/dev/ttySC3")
]

control.run_debug_control(enable_steering=True, enable_traction=False)
