import serial
import time



# sudo sh -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
# cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# 1500000


BAUD=500000

ser0 = serial.Serial('/dev/ttyACM0', BAUD, timeout=0.5)
print(ser0.name)

loop = True
while loop:
    start = time.time()
    for value in range(100):
        value = 10
        ser0.write(bytes('{}\n'.format(value), 'utf8'))
        time.sleep(1)
        try:
            print(ser0.readline())
        except KeyboardInterrupt:
            loop = False
            break
        except:
            print("SERIAL EXCEPTION")
ser0.close()

