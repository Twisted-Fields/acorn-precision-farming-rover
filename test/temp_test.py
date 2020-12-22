# import sys
# import time
# sys.path.append('../vehicle')
# import odrive_manager
#
# #odrv0 = odrive_manager.OdriveManager(path=path, serial_number=serial_number).find_odrive()
# odrv0 = odrive_manager.OdriveManager(path='/dev/ttyACM0', serial_number='336B31643536').find_odrive()
#
# while(True):
#     print(odrv0.get_adc_voltage(3))
#     time.sleep(0.2)


#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math


def steinhart_temperature_C(r, Ro=10000.0, To=25.0, beta=3900.0):
    steinhart = math.log(r / Ro) / beta      # log(R/Ro) / beta
    steinhart += 1.0 / (To + 273.15)         # log(R/Ro) / beta + 1/To
    steinhart = (1.0 / steinhart) - 273.15   # Invert, convert to C
    return steinhart

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
odrv0 = odrive.find_any()


# To read a value, simply read the property
print("Bus voltage is " + str(odrv0.vbus_voltage) + "V")

while(True):
    value = odrv0.get_adc_voltage(3)
    R = 10000 / (3.3/value)
    print('Voltage: {}, Thermistor resistance: {} ohms, Temperature {}'.format(value, R, steinhart_temperature_C(R)))
    time.sleep(0.2)
