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

import RPi.GPIO as GPIO
import time
import board
import busio
import digitalio
import sys


NVIDIA_ENABLE_PIN = 16

GPIO.setmode(GPIO.BCM)

GPIO.setup(NVIDIA_ENABLE_PIN, GPIO.OUT, initial=GPIO.LOW)

if len(sys.argv) > 1:
    # Turn off computer
    GPIO.output(NVIDIA_ENABLE_PIN, GPIO.HIGH)
else:
    # Turn on computer
    GPIO.output(NVIDIA_ENABLE_PIN, GPIO.LOW)

sys.exit()
