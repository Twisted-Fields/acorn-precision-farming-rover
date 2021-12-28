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

import os
import time

_VOLT_OUT_PIN = 5
_ESTOP_PIN = 6
_FAST_POLLING_SLEEP_S = 0.01

# This file gets imported by server but we should only import GPIO on raspi.
if "arm" in os.uname().machine:
    import RPi.GPIO as GPIO

    def setup():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(_ESTOP_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.estop_state = GPIO.LOW
        GPIO.setup(_VOLT_OUT_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(_VOLT_OUT_PIN, GPIO.LOW)
        time.sleep(1)

        for _ in range(100):
            time.sleep(0.001)
            GPIO.output(_VOLT_OUT_PIN, GPIO.LOW)
            time.sleep(0.001)
            GPIO.output(_VOLT_OUT_PIN, GPIO.HIGH)
else:
    class simulated_GPIO():
        def __init__(self):
            self.estop_state = True

        def output(self, *args):
            pass
    GPIO = simulated_GPIO()

    def setup():
        pass


def toggling_sleep(duration):
    start = time.time()
    while time.time() - start < duration:
        toggle()
        time.sleep(_FAST_POLLING_SLEEP_S)


def toggle():
    GPIO.estop_state = not GPIO.estop_state
    GPIO.output(_ESTOP_PIN, GPIO.estop_state)


def e_stop_square_wave():
    while True:
        # ESTOP approx 100Hz square wave.
        time.sleep(_FAST_POLLING_SLEEP_S)
        toggle()
