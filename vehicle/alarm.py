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
import board
import busio
import digitalio
import sys

from adafruit_mcp230xx.mcp23017 import MCP23017


i2c = busio.I2C(board.SCL, board.SDA)

mcp = MCP23017(i2c)  # , address=0x20)  # MCP23017


alarm1 = mcp.get_pin(0)
alarm2 = mcp.get_pin(1)
alarm3 = mcp.get_pin(2)

alarm1.switch_to_output(value=False)
alarm2.switch_to_output(value=False)
alarm3.switch_to_output(value=False)


while True:
    # Seven Alarm modes plus off.
    alarm1.value = True
    alarm2.value = False
    alarm3.value = True
    time.sleep(2.0)
    alarm1.value = False
    alarm2.value = False
    alarm3.value = False
    time.sleep(2.0)
    sys.exit()
    alarm1.value = True
    alarm2.value = True
    alarm3.value = False
    time.sleep(2.0)
    alarm1.value = True
    alarm2.value = True
    alarm3.value = True
    time.sleep(2.0)
    alarm1.value = True
    alarm2.value = False
    alarm3.value = True
    time.sleep(2.0)
    alarm1.value = False
    alarm2.value = True
    alarm3.value = True
    time.sleep(2.0)
    alarm1.value = False
    alarm2.value = False
    alarm3.value = True
    time.sleep(2.0)
    alarm1.value = False
    alarm2.value = True
    alarm3.value = False
    time.sleep(2.0)
