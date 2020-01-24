import socket
import subprocess
import time
import os
import math
from random import random
import gps_tools

TCP_IP = "127.0.0.1"
TCP_PORT = 10001
TCP_PORT2 = 10002
BUFFER_SIZE = 1024
VEHICLE_AZIMUTH_OFFSET_DEG = 90 + 45.0
_FAST_POLLING_DELAY_S = 0.05


"""
In this file, fake RTK data is sent over the pipe given when the file was launched.

"""


def start_gps(master_conn):
        deg = -200
        while True:
            lat = 37.3540865905 #+ random() * 0.0001
            lon = -122.333349927 #+ random() * 0.0001
            height_m = 80 + random() * 10
            azimuth_degrees = -22 + deg #random() * 360.0
            deg += 10
            if deg > 400:
                deg = -200
            latest_sample = gps_tools.GpsSample(lat, lon, height_m, ("fix", "fix"), (21, 23), azimuth_degrees, time.time())
            #print("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fix1: {}, Fix2: {}".format(latest_sample.lat, latest_sample.lon, azimuth, 0, "fix","fix"))
            master_conn.send(latest_sample)
            print("UPDATED AZIMUTH TO {} degrees".format(azimuth_degrees))
            time.sleep(1)

if __name__=="__main__":
    pass
