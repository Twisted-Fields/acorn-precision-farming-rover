"""
*********************************************************************
                     This file is part of:
                       The Acorn Project
             https://wwww.twistedfields.com/research
*********************************************************************
Copyright (c) 2019-2021 Taylor Alexander, Twisted Fields LLC
Copyright (c) 2021 The Acorn Project contributors (cf. AUTHORS.md).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*********************************************************************
"""
import serial
import time

# sudo sh -c "echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
# cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# 1500000

_POLLING_DELAY_S = 0.01


def run_frame_control(parent_conn):

    BAUD = 500000

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
