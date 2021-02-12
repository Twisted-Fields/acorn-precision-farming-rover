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
