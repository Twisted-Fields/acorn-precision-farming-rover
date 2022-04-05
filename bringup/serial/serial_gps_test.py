import serial
import time


# sudo sh -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
# cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# 1500000


BAUD = 115200

ser0 = serial.Serial('/dev/ttyACM0', BAUD, timeout=0.5)
print(ser0.name)
ser1 = serial.Serial('/dev/ttySC4', BAUD, timeout=0.5)
print(ser1.name)
ser2 = serial.Serial('/dev/ttySC5', BAUD, timeout=0.5)
print(ser2.name)
# ser3 = serial.Serial('/dev/ttySC3', BAUD, timeout=1)
# print(ser3.name)
ser0.reset_input_buffer()
ser0.reset_output_buffer()
ser1.reset_input_buffer()
ser1.reset_output_buffer()
ser2.reset_input_buffer()
ser2.reset_output_buffer()


try:
    while True:
        toread = ser0.in_waiting
        if toread > 20:
            msg = ser0.read(toread)
            ser1.write(msg)
            # print(msg)
        toread = ser2.in_waiting
        if toread > 20:
            msg = ser2.read(toread)
            print(msg)

except Exception as e:
    raise e
    print("SERIAL EXCEPTION")

ser0.close()
ser1.close()
ser2.close()
