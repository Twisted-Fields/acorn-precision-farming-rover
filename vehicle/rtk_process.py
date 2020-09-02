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

TCP_IP = "127.0.0.1"
TCP_PORT = 10001
TCP_PORT2 = 10002
BUFFER_SIZE = 1024
VEHICLE_AZIMUTH_OFFSET_DEG = 90 + 35.0
_FAST_POLLING_DELAY_S = 0.05


"""
In this file, the RTK programs are launched as subprocesses. When both processes
have returned data, the vehicle position is calculated and then sent through the
pipe given when this code was launched.

"""


def digest_data(data):
    data = str(data.splitlines()[0])
    #print(data)
    data = data.split(' ')
    data = [a for a in data if a]

    if len(data) > 13:
        lat = float(data[2])
        lon = float(data[3])
        height_m = float(data[4])
        status = "fix" if data[5] is "1" else "no fix"
        num_sats = data[6]
        #age = data[13]
        return gps_tools.GpsSample(lat, lon, height_m, status, num_sats, None, time.time())
    else:
        return None


def launch_rtk_sub_procs(master_conn):
    # Launch rtklib process.
    cmd = 'rtkrcv -s -d /dev/null -o /home/pi/vehicle/twisted.conf'
    proc1 = subprocess.Popen(cmd, shell=True)
    cmd2 = 'rtkrcv -s -d /dev/null -o /home/pi/vehicle/twisted2.conf'
    proc2 = subprocess.Popen(cmd2, shell=True)

    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    connected = False
    attempts = 0
    while True:
      try:
        # Connect to socket.
        tcp_sock.connect((TCP_IP, TCP_PORT))
        tcp_sock2.connect((TCP_IP, TCP_PORT2))
        print('Connected to RTK subprocesses.')
        break
      except:
        time.sleep(0.500)
        #print('Attempt failed.')
        attempts += 1
        if attempts > 5:
            raise RuntimeError("Could not connect to rtkrcv TCP socket.")
    tick_time = time.time()
    print_gps_counter = 0
    while True:
        data = tcp_sock.recv(BUFFER_SIZE)
        data2 = tcp_sock2.recv(BUFFER_SIZE)

        period = time.time() - tick_time
        # print("{} sec gps sample period".format(period))
        tick_time = time.time()
        try:
            if data and data2:
                data = digest_data(data)
                data2 = digest_data(data2)
                azimuth_degrees = gps_tools.get_heading(data, data2) + VEHICLE_AZIMUTH_OFFSET_DEG
                d = gps_tools.get_distance(data,data2)
                lat = (data.lat + data2.lat) / 2.0
                lon = (data.lon + data2.lon) / 2.0
                height_m = (data.height_m + data2.height_m) / 2.0
                latest_sample = gps_tools.GpsSample(lat, lon, height_m, (data.status, data2.status), (data.num_sats, data2.num_sats), azimuth_degrees, data.time_stamp)
                #print("GPS DEBUG: FIX reads... : {}".format(latest_sample.status))
                print_gps_counter += 1
                if print_gps_counter%10 == 0:
                    print("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fix1: {}, Fix2: {}, Period: {}".format(latest_sample.lat, latest_sample.lon, azimuth_degrees, d, data.status, data2.status, period))

                master_conn.send(latest_sample)
            else:
                print("Missing GPS Data")
        except Exception as e:
            print(e)
        # else:
        #     print("Await data...")
        #     time.sleep(0.1)

        # TODO: Close sockets when needed.


def start_gps(master_conn):
    rtk_proc = mp.Process(target=launch_rtk_sub_procs, args=(master_conn,))
    rtk_proc.start()


if __name__=="__main__":
    pass
