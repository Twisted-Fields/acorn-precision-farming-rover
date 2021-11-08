import serial
import time


# sudo sh -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
# cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# 1500000


BAUD = 115200

ser0 = serial.Serial('/dev/ttySC1', BAUD, timeout=0.5)
print(ser0.name)
# ser1 = serial.Serial('/dev/ttySC1', BAUD, timeout=1)
# print(ser1.name)
# ser2 = serial.Serial('/dev/ttySC2', BAUD, timeout=1)
# print(ser2.name)
# ser3 = serial.Serial('/dev/ttySC3', BAUD, timeout=1)
# print(ser3.name)
loop = True
while loop:
    start = time.time()
    for _ in range(100):
        # time.sleep(0.001)
        ser0.write(b'r vbus_voltage\n')
        # ser1.write(b'hababababa\n')
        # ser2.readline()
        # ser3.readline()
        # ser2.write(b'nananananan\n')
        # ser3.write(b'nananananan\n')
        try:
            print(ser0.readline())
        except KeyboardInterrupt:
            loop = False
            break
        except:
            print("SERIAL EXCEPTION")
        # ser1.readline()
    #duration = time.time() - start
    #print("100 exchanges took {} milliseconds, a rate of {} Hz.".format(duration * 1000, 1.0/duration * 100))
# data_r = ser.read(5)  # Read 5 bytes
ser0.close()
# ser1.close()
# ser2.close()
# ser3.close()
