import time
import board
import busio
import digitalio
import sys

from adafruit_mcp230xx.mcp23017 import MCP23017


i2c = busio.I2C(board.SCL, board.SDA)

mcp = MCP23017(i2c)#, address=0x20)  # MCP23017

alarm1 = mcp.get_pin(0)
alarm2 = mcp.get_pin(1)
alarm3 = mcp.get_pin(2)

alarm1.switch_to_output(value=False)
alarm2.switch_to_output(value=False)
alarm3.switch_to_output(value=False)



while True:
    # Seven Alarm modes plus off.
    alarm1.value = False
    alarm2.value = False
    alarm3.value = False
    time.sleep(2.0)
    alarm1.value = True
    alarm2.value = False
    alarm3.value = False
    time.sleep(2.0)
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
