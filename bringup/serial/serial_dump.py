import serial
import time


# sudo sh -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
# cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# 1500000

BAUD = 115200

ser0 = serial.Serial('/dev/ttyAMA2', BAUD, timeout=1.5)
print(ser0.name)

try:
    while True:
        msg = ser0.readline()
        print(msg)
except Exception as e:
    raise e
    print("SERIAL EXCEPTION")

ser0.close()
