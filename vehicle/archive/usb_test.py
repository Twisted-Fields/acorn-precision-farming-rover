
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import serial
import re
import time

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import math
import sys
import fibre


#Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
odrv0 = odrive.find_any(
odrv0 = odrive.find_any("serial:/dev/ttyUSB0"))

print(dump_errors(odrv0,True))
