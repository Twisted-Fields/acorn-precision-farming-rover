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

import redis
import time
import pickle
from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mp_colors
import sys
from datetime import datetime

from scipy.interpolate import splprep, splev
sys.path.append('../vehicle')
from remote_control_process import EnergySegment

_SMOOTH_MULTIPLIER = 0.00000000001

r = redis.Redis(
    host='192.168.1.170',
    port=6379)


for key in r.scan_iter():
    # print(key)
    if 'energy_segment' in str(key):
        orig_x = []
        orig_y1 = []
        orig_y2 = []
        orig_y3 = []
        orig_y4 = []
        orig_z = []
        colors = []
        min_colorval = 9999999
        max_colorval = -9999999
        print(key)
        list_length = r.llen(key)
        print("List Length {}".format(list_length))
        first_stamp = pickle.loads(r.lindex(key, 0)).start_gps.time_stamp
        colorby = ""
        watt_seconds = False
        now = time.time()
        today = time.localtime(now)
        power_vals = [[], [], [], []]
        total_meters_traveled = 0
        while True:
            print("loop")
            day_index = 0
            last_total = total_meters_traveled
            total_meters_traveled = 0
            list_length = r.llen(key)
            today = datetime.now()
            last_sequence_num = 0
            for idx in range(list_length-1, 0, -1):
                # print(idx)
                segment = pickle.loads(r.lindex(key, idx))
                if segment.sequence_num == last_sequence_num:
                    continue
                last_sequence_num = segment.sequence_num
                this_stamp = segment.start_gps.time_stamp
                # stamp_localtime = time.localtime(this_stamp)
                if this_stamp.year == today.year and this_stamp.day == today.day:
                    total_meters_traveled += segment.distance_sum
                else:
                    print(total_meters_traveled)
                    if total_meters_traveled > last_total:
                        print("Still rollin.")
                    else:
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                        print(
                            "##############################################################")
                    time.sleep(5)
                    break
