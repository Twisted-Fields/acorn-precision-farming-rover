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

try:
    import RPi.GPIO as GPIO
except RuntimeError as e:
    if str(e) != 'This module can only be run on a Raspberry Pi!':
        raise

    class simulated_GPIO():
        def __init__(self):
            self.estop_state = True

        def setmode(self, *args):
            pass

        def output(self, *args):
            pass
    GPIO = simulated_GPIO()
