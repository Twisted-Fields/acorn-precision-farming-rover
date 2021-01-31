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
            deg += 1
            if deg > 400:
                deg = -200
            latest_sample = gps_tools.GpsSample(lat, lon, height_m, ("fix", "fix"), (21, 23), azimuth_degrees, time.time())
            #print("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fix1: {}, Fix2: {}".format(latest_sample.lat, latest_sample.lon, azimuth, 0, "fix","fix"))
            master_conn.send(latest_sample)
            #print("UPDATED AZIMUTH TO {} degrees".format(azimuth_degrees))
            time.sleep(0.1)

if __name__=="__main__":
    pass
