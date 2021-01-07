import RPi.GPIO as GPIO
import time
import board
import busio
import digitalio
import sys

from adafruit_mcp230xx.mcp23017 import MCP23017

ESTOP_PIN = 6
VOLT_OUT_PIN = 5

GPIO.setmode(GPIO.BCM)

GPIO.setup(ESTOP_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(VOLT_OUT_PIN, GPIO.OUT, initial=GPIO.LOW)

#
#
if len(sys.argv) > 1:
    GPIO.output(VOLT_OUT_PIN, GPIO.LOW)
    sys.exit()
else:
    for _ in range(100):
        GPIO.output(VOLT_OUT_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(VOLT_OUT_PIN, GPIO.HIGH)
        time.sleep(0.01)

delay = 0.005

start = time.time()

while True:
    # ESTOP approx 1kHz square wave.
    time.sleep(delay)
    GPIO.output(ESTOP_PIN, GPIO.LOW)
    time.sleep(delay)
    GPIO.output(ESTOP_PIN, GPIO.HIGH)
    # if time.time() - start > 5:
    #     delay = delay/2.0
    #     start = time.time()
    #     print("Cycle rate is {} Hz".format(1.0/(2.0*delay)))


# while True:
#
#     time.sleep(1000)
#     GPIO.output(VOLT_OUT_PIN, GPIO.LOW)
#     sys.exit()


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
