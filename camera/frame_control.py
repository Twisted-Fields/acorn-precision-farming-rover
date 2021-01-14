import serial
import time



# sudo sh -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
# cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# 1500000

_POLLING_DELAY_S = 0.01

def run_frame_control(parent_conn):

    BAUD=500000

    ser0 = serial.Serial('/dev/ttyACM0', BAUD, timeout=0.5)
    print(ser0.name)

    loop = True
    while loop:
        start = time.time()
        if parent_conn is not None:
            new_value = False
            while parent_conn.poll():
                value = parent_conn.recv()
                new_value = True
            if new_value:
                try:
                    ser0.write(bytes('{}\n'.format(value), 'utf8'))
                except Exception as e:
                    print("Exception writing to shutter serial connection:")
                    print(e)
        try:
            time.sleep(_POLLING_DELAY_S)
            # print(ser0.readline())
        except KeyboardInterrupt:
            loop = False
            break
        except:
            print("SERIAL EXCEPTION")
    ser0.close()
