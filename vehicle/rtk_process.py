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

TCP_IP = "127.0.0.1"
TCP_PORT1 = 10001
TCP_PORT2 = 10002
BUFFER_SIZE = 1024
VEHICLE_AZIMUTH_OFFSET_DEG = 90 + 39.0
_FAST_POLLING_DELAY_S = 0.05


RTK_CMD1 = ['/usr/local/bin/rtkrcv','-s','-d','/dev/null','-o','/home/pi/vehicle/twisted.conf']
RTK_CMD2 = ['/usr/local/bin/rtkrcv','-s','-d','/dev/null','-o','/home/pi/vehicle/twisted2.conf']


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
        height_m = float(data[4].replace('\'',''))
        status = "fix" if data[5] is "1" else "no fix"
        num_sats = data[6]
        base_rtk_age = float(data[13])
        return gps_tools.GpsSample(lat, lon, height_m, status, num_sats, None, time.time(), base_rtk_age)
    else:
        return None

def run_rtk_system(master_conn):
    launch_rtk_sub_procs()
    tcp_sock1, tcp_sock2 = connect_rtk_procs()
    print_gps_counter = 0
    latest_sample = None
    while True:
        latest_sample = rtk_loop_once(tcp_sock1, tcp_sock2, print_gps=print_gps_counter % 40 == 0, last_sample=latest_sample)
        print_gps_counter += 1
        if master_conn is not None:
            master_conn.send(latest_sample)


def kill_rtk_sub_procs():
    for proc in psutil.process_iter():
    # check whether the process name matches
        if proc.name() == 'rtkrcv':
            print(proc.cmdline())
            proc.kill()

def check_for_rtk_sub_procs():
    match_count = 0
    for proc in psutil.process_iter():
    # check whether the process name matches
        if proc.name() == 'rtkrcv':
        #if proc.cmdline == RTK_CMD1 or proc.cmdline == RTK_CMD2:
            match_count+=1
    if match_count == 2:
        print("RTK processes already running so will connect to those.")
        return True
    kill_rtk_sub_procs()
    return False

def launch_rtk_sub_procs():
    # Launch rtklib process.

    if check_for_rtk_sub_procs():
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
    proc2 = subprocess.Popen(RTK_CMD2, cwd=cwd, stdin=None, stdout=None, stderr=None, close_fds=True, shell=False)

    while True:
        time.sleep(0.001)


def connect_rtk_procs():
    tcp_sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    connected = False
    attempts = 0
    while True:
      try:
        # Connect to socket.
        tcp_sock1.connect((TCP_IP, TCP_PORT1))
        tcp_sock2.connect((TCP_IP, TCP_PORT2))
        print('Connected to RTK subprocesses.')
        break
      except:
        time.sleep(0.500)
        #print('Attempt failed.')
        attempts += 1
        if attempts > 5:
            raise RuntimeError("Could not connect to rtkrcv TCP socket.")

    return tcp_sock1, tcp_sock2


def rtk_loop_once(tcp_sock1, tcp_sock2, print_gps=False, last_sample=None):
    print_gps_counter = 0
    while True:
        data1 = tcp_sock1.recv(BUFFER_SIZE)
        data2 = tcp_sock2.recv(BUFFER_SIZE)
        try:
            if data1 and data2:
                data1 = digest_data(data1)
                data2 = digest_data(data2)
                azimuth_degrees = gps_tools.get_heading(data1, data2) + VEHICLE_AZIMUTH_OFFSET_DEG
                d = gps_tools.get_distance(data1,data2)
                lat = (data1.lat + data2.lat) / 2.0
                lon = (data1.lon + data2.lon) / 2.0
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
                return latest_sample
            else:
                print("Missing GPS Data")
                time.sleep(1)
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            print(e)
        # TODO: Close sockets when needed.


def start_gps(master_conn):
    rtk_proc = mp.Process(target=run_rtk_system, args=(master_conn,))
    rtk_proc.start()


if __name__=="__main__":
    run_rtk_system(None)
