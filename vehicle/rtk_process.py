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

import socket
import subprocess
import time
import os
import pickle
import math
from datetime import datetime
import select
import multiprocessing as mp
import gps_tools
import sys
import psutil
import struct
import random

TCP_IP = "127.0.0.1"
TCP_PORT1 = 10001
TCP_PORT2 = 10002
TCP_BUFFER_SIZE = 1024
VEHICLE_AZIMUTH_OFFSET_DEG = 90 + 45.0
_FAST_POLLING_DELAY_S = 0.05
SOCKET_TIMEOUT_SECONDS = 1
_GPS_ERRORS_ALLOWED = 4


if sys.maxsize > 2**32:
  TCP_TIMEOUT = struct.pack(str("ll"), int(SOCKET_TIMEOUT_SECONDS), int(0))
else:
  TCP_TIMEOUT = struct.pack(str("ii"), int(SOCKET_TIMEOUT_SECONDS), int(0))



RTK_CMD1 = ['/usr/local/bin/rtkrcv','-s','-d','/dev/null','-o','/home/pi/vehicle/twisted.conf']
RTK_CMD2 = ['/usr/local/bin/rtkrcv','-s','-d','/dev/null','-o','/home/pi/vehicle/twisted2.conf']

#/usr/local/bin/rtkrcv -s -d /dev/null -o /home/pi/vehicle/twisted.conf
#/usr/local/bin/rtkrcv -s -o /home/pi/vehicle/twisted.conf


"""
In this file, the RTK programs are launched as subprocesses. When both processes
have returned data, the vehicle position is calculated and then sent through the
pipe given when this code was launched.

"""


def digest_data(buffer):
    #print(data)
    # print(buffer)
    line_buffer = buffer.splitlines(keepends=True)
    # print(line_buffer)

    errors = 0
    index = len(line_buffer) - 1
    while True:
        try:
            data = line_buffer[index]
            data = data.splitlines()[0]
            data = data.split(' ')
            #print(data)
            data = [a for a in data if a] # Remove empty string entries.
            # print(data)
            # print(index)
            if len(data) <= 13:
                raise ValueError("Data incomplete.")
            lat = float(data[2])
            lon = float(data[3])
            height_m = float(data[4].replace('\'',''))
            status = "fix" if data[5] is "1" else "no fix"
            num_sats = data[6]
            base_rtk_age = float(data[13])
            #print("Good GPS data recieved: {}".format(data))
            sample = gps_tools.GpsSample(lat, lon, height_m, status, num_sats, None, time.time(), base_rtk_age)
            if index < len(line_buffer) - 1:
                buffer = "".join(line_buffer[index+1:])
                #print(buffer)
            else:
                buffer = ""
            if index > 2:
                print("Warning, skipping {} GPS samples.".format(index))
            # print("Returning")
            return buffer, sample
        except Exception as e:
            # print(repr(e))
            errors += 1
            index -= 1
            if errors > _GPS_ERRORS_ALLOWED or index < 0:
                break
            else:
                continue
    else:
        print("Incorrect GPS data length. Recieved: {}".format(data))
        return buffer, None

def run_rtk_system(master_conn):
    launch_rtk_sub_procs()
    tcp_sock1, tcp_sock2 = connect_rtk_procs()
    print_gps_counter = 0
    latest_sample = None
    buffers = ["", ""]
    while True:
        buffers, latest_sample = rtk_loop_once(tcp_sock1, tcp_sock2, buffers, print_gps=True, last_sample=latest_sample)
        # buffers, latest_sample = rtk_loop_once(tcp_sock1, tcp_sock2, buffers, print_gps=print_gps_counter % 40 == 0, last_sample=latest_sample)
        print_gps_counter += 1
        if master_conn is not None:
            master_conn.send(latest_sample)


def run_rtk_system_single():
    launch_rtk_sub_procs(single=True)
    tcp_sock1, tcp_sock2 = connect_rtk_procs(single=True)
    print_gps_counter = 0
    latest_sample = None
    while True:
        latest_sample = rtk_loop_once_single_receiver(tcp_sock1, print_gps=(print_gps_counter % 10 == 0), last_sample=latest_sample)
        print_gps_counter += 1
        # if master_conn is not None:
        #     master_conn.send(latest_sample)


def kill_rtk_sub_procs():
    for proc in psutil.process_iter():
    # check whether the process name matches
        if proc.name() == 'rtkrcv':
            print(proc.cmdline())
            proc.kill()

def check_for_rtk_sub_procs(single=False):
    match_count = 0
    for proc in psutil.process_iter():
    # check whether the process name matches
        if proc.name() == 'rtkrcv':
        #if proc.cmdline == RTK_CMD1 or proc.cmdline == RTK_CMD2:
            match_count+=1
    if match_count == 2 - single*1:
        print("RTK processes already running so will connect to those.")
        return True
    kill_rtk_sub_procs()
    return False

def launch_rtk_sub_procs(single=False):
    # Launch rtklib process.

    if check_for_rtk_sub_procs(single):
        return

    # This comes from:
    # https://www.oreilly.com/library/view/python-cookbook/0596001673/ch06s08.html
    try:
        pid = os.fork()
        if pid > 0:
            # Exit first parent
            #os.waitid(os.P_PID, pid, os.WEXITED)
            return
            #sys.exit(0)
    except OSError as e:
        sys.exit(1)

    # Decouple from parent environment
    os.chdir("/")
    os.setsid()
    os.umask(0)

    # Do second fork
    try:
        pid = os.fork()
        if pid > 0:
            sys.exit(0)
    except OSError as e:
        sys.exit(1)

    cwd = "/home/pi/vehicle/"

    proc1 = subprocess.Popen(RTK_CMD1, cwd=cwd, stdin=None, stdout=None, stderr=None, close_fds=True, shell=False)
    if single== False:
        proc2 = subprocess.Popen(RTK_CMD2, cwd=cwd, stdin=None, stdout=None, stderr=None, close_fds=True, shell=False)

    while True:
        time.sleep(0.001)


def reset_and_reconnect_rtk(sock1, sock2, single=False):
    kill_rtk_sub_procs()
    time.sleep(5)
    sock1.shutdown(socket.SHUT_RDWR)
    sock2.shutdown(socket.SHUT_RDWR)
    # sock1.shutdown()
    # sock2.shutdown()
    time.sleep(5)
    sock1.close()
    sock2.close()
    time.sleep(5)
    return connect_rtk_procs(single)

def connect_rtk_procs(single=False):
    tcp_sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock1.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, TCP_TIMEOUT)
    # tcp_sock1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if single == False:
        tcp_sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_sock2.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, TCP_TIMEOUT)
        # tcp_sock2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    else:
        tcp_sock2 = None

    connected = False
    attempts = 0
    while True:
      try:
        # Connect to socket.
        tcp_sock1.connect((TCP_IP, TCP_PORT1))
        if single == False:
            print("SINGLE IS FALSE")
            tcp_sock2.connect((TCP_IP, TCP_PORT2))
        print('Connected to RTK subprocesses.')
        break
      except Exception as e:
        print(e)
        time.sleep(0.500 + attempts)
        print('Attempt failed.')
        attempts += 1
        if attempts > 5:
            raise RuntimeError("Could not connect to rtkrcv TCP socket.")

    return tcp_sock1, tcp_sock2


def rtk_loop_once_single_receiver(tcp_sock, print_gps=False, last_sample=None):
    print_gps_counter = 0
    while True:
        data1 = tcp_sock.recv(TCP_BUFFER_SIZE)
        try:
            if data1:
                latest_sample = digest_data(data1)
                if last_sample:
                    period = latest_sample.time_stamp - last_sample.time_stamp
                else:
                    period = None
                #print("GPS DEBUG: FIX reads... : {}".format(latest_sample.status))
                if print_gps:
                    try:
                        print("Lat: {:.10f}, Lon: {:.10f}, Height M: {:.4f}, Fix: {}, Period: {:.4f}".format(latest_sample.lat, latest_sample.lon, latest_sample.height_m, latest_sample.status, period))
                    except:
                        pass
                return latest_sample
            else:
                print("Missing GPS Data")
                time.sleep(1)
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            print(e)
        # TODO: Close sockets when needed.


# Good GPS data recieved: ["b'2138", '345203', '37.353039224', '-122.333725667', '79.7368', '1', '16', '0.0055', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0039', '0.40', "280.7'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060376', '-122.333742916', '79.8650', '1', '16', '0.0055', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.50', "288.4'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039236', '-122.333725665', '79.7345', '1', '16', '0.0055', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.60', "280.0'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060388', '-122.333742931', '79.8652', '1', '16', '0.0055', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.60', "288.4'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039233', '-122.333725682', '79.7360', '1', '16', '0.0056', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.70', "279.7'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060378', '-122.333742933', '79.8635', '1', '16', '0.0056', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.69', "288.4'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039228', '-122.333725683', '79.7289', '1', '16', '0.0056', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.80', "279.2'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060389', '-122.333742924', '79.8602', '1', '16', '0.0056', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.80', "288.5'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039231', '-122.333725678', '79.7309', '1', '16', '0.0056', '0.0054', '0.0153', '-0.0032', '0.0049', '-0.0040', '0.90', "278.7'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060379', '-122.333742930', '79.8641', '1', '16', '0.0056', '0.0054', '0.0153', '-0.0032', '0.0049', '-0.0040', '0.89', "288.6'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039225', '-122.333725655', '79.7303', '1', '16', '0.0056', '0.0055', '0.0153', '-0.0032', '0.0049', '-0.0040', '1.00', "278.5'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060380', '-122.333742935', '79.8615', '1', '16', '0.0056', '0.0055', '0.0153', '-0.0032', '0.0049', '-0.0040', '1.00', "288.6'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039211', '-122.333725705', '79.7376', '1', '16', '0.0055', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.10', "278.0'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060376', '-122.333742936', '79.8621', '1', '16', '0.0055', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0040', '0.09', "288.7'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039224', '-122.333725703', '79.7352', '1', '16', '0.0055', '0.0054', '0.0151', '-0.0031', '0.0048', '-0.0039', '0.20', "277.3'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060374', '-122.333742933', '79.8648', '1', '16', '0.0055', '0.0054', '0.0151', '-0.0031', '0.0048', '-0.0039', '0.19', "288.7'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039220', '-122.333725716', '79.7365', '1', '16', '0.0055', '0.0054', '0.0151', '-0.0031', '0.0048', '-0.0039', '0.30', "276.9'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353060366', '-122.333742965', '79.8674', '1', '16', '0.0055', '0.0054', '0.0151', '-0.0031', '0.0048', '-0.0039', '0.29', "288.8'"]
# Good GPS data recieved: ["b'2138", '345204', '37.353039214', '-122.333725704', '79.7374', '1', '16', '0.0055', '0.0054', '0.0152', '-0.0031', '0.0048', '-0.0039', '0.40', "276.4'"]



# def read_to_newline(sock, retries=3):
#     errors = 0
#     data = []
#     result = None
#     while True:
#         next_byte = sock.recv(1)
#         data.append(next_byte)
#         # print(data)
#
#         if next_byte == b'\n':
#             try:
#                 result = digest_data(b''.join(data))
#             except Exception as e:
#                 print("GPS Parse failed: {}".format(e)) # Todo: logger trace
#             if result:
#                 return result
#             else:
#                 data = []
#                 errors += 1
#                 if errors > retries:
#                     return None



def fake_data(print_gps):
    lat = 37.3540865905 #+ random() * 0.0001
    lon = -122.333349927 #+ random() * 0.0001
    height_m = 80 + random.random() * 10
    azimuth_degrees = -22 + random.random() * 360.0
    rtk_age = 1.0
    latest_sample = gps_tools.GpsSample(lat, lon, height_m, ("fix", "fix"), (21, 23), azimuth_degrees, time.time(), rtk_age)
    d = 2.8
    if print_gps:
        print("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fix1: {}, Fix2: {}, Period: {}".format(latest_sample.lat, latest_sample.lon, azimuth_degrees, d, "Fix", "Fix", 0.121))

    return [], latest_sample



def rtk_loop_once(tcp_sock1, tcp_sock2, buffers, print_gps=False, last_sample=None, retries=3, return_fake_data=False):
    if return_fake_data:
        return fake_data(print_gps)

    print_gps_counter = 0
    errors = 0
    #print(buffers)
    blocking_exception = None
    while True:
        try:
            start_time = time.time()
            data0 = tcp_sock1.recv(TCP_BUFFER_SIZE).decode('utf-8')
            data1 = tcp_sock2.recv(TCP_BUFFER_SIZE).decode('utf-8')
            buffers[0] += data0
            buffers[1] += data1
            buffers[0], data1 = digest_data(buffers[0])
            buffers[1], data2 = digest_data(buffers[1])
            #print("Read GPS duration {}".format(time.time() - start_time))
            break
        except BlockingIOError as e:
            blocking_exception = e
            #print("GPS BlockingIOError")
        except Exception as e:
            print("GPS ERROR DURING READ: {}".format(e))
        errors += 1
        if errors > retries:
            if blocking_exception is not None:
                print("ERROR GPS DATA NOT AVAILABLE")
            print("TOO MANY GPS ERRORS")
            return buffers, None

    try:
        if data1 and data2:
            azimuth_degrees = gps_tools.get_heading(data1, data2) + VEHICLE_AZIMUTH_OFFSET_DEG
            d = gps_tools.get_distance(data1,data2)
            lat = (data1.lat + data2.lat) / 2.0
            lon = (data1.lon + data2.lon) / 2.0
            #print("HEIGHT DIFFERENCE {}".format(data1.height_m - data2.height_m))
            height_m = (data1.height_m + data2.height_m) / 2.0
            rtk_age = max([data1.rtk_age, data2.rtk_age])
            latest_sample = gps_tools.GpsSample(lat, lon, height_m, (data1.status, data2.status), (data1.num_sats, data2.num_sats), azimuth_degrees, data1.time_stamp, rtk_age)
            if last_sample:
                period = latest_sample.time_stamp - last_sample.time_stamp
            else:
                period = None
            #print("GPS DEBUG: FIX reads... : {}".format(latest_sample.status))
            if print_gps:
                print("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fix1: {}, Fix2: {}, Period: {}".format(latest_sample.lat, latest_sample.lon, azimuth_degrees, d, data1.status, data2.status, period))
            return buffers, latest_sample
        else:
            print("Missing GPS Data")
            return buffers, None
    except KeyboardInterrupt as e:
        raise e
    except BlockingIOError:
        return buffers, None
    except Exception as e:
        print("GPS ERROR DURING PROCESSING: {}".format(e))
        return buffers, None
    # TODO: Close sockets when needed.


def start_gps(master_conn):
    rtk_proc = mp.Process(target=run_rtk_system, args=(master_conn,))
    rtk_proc.start()


if __name__=="__main__":
    #run_rtk_system_single()
    run_rtk_system(None)
