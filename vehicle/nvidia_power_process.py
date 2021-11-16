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
import sys
import os

# This file gets imported by server but we should only import GPIO on raspi.
if "arm" in os.uname().machine:
    import RPi.GPIO as GPIO
    import board
    import busio
    import digitalio
    from adafruit_mcp230xx.mcp23017 import MCP23017

NVIDIA_ENABLE_PIN = 16


class NvidiaPowerProcess:

    def __init__(self, master_conn, simulated_hardware):
        self.simulated_hardware = simulated_hardware
        self.connection = master_conn

    def process_loop(self):
        if self.simulated_hardware == False:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(NVIDIA_ENABLE_PIN, GPIO.OUT, initial=GPIO.HIGH)
            # Turn off computer
            GPIO.output(NVIDIA_ENABLE_PIN, GPIO.HIGH)
            time.sleep(5)
            # Turn on computer
            GPIO.output(NVIDIA_ENABLE_PIN, GPIO.LOW)
        while True:
            # TODO(tlalexander): Whatever we need to do on loop goes here.
            time.sleep(1)


def nvidia_power_loop(master_conn, simulated_hardware=False):
    power_process = NvidiaPowerProcess(master_conn, simulated_hardware)
    power_process.process_loop()


if __name__ == "__main__":
    nvidia_power_loop(None)
