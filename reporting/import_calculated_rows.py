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

from scipy.interpolate import splprep, splev
sys.path.append('../vehicle')
from remote_control_process import EnergySegment
import spline_lib
import gps_tools


_SMOOTH_MULTIPLIER = 0.00000000001

# r = redis.Redis(
#     host='acornserver.local',
#     port=6379)

r = redis.Redis(
    host='192.168.1.170',
    port=6379)



# self.sequence_num = sequence_num
# self.time_stamp = end_gps.time_stamp
# self.start_gps = start_gps
# self.end_gps = end_gps
# self.duration = end_gps.time_stamp - start_gps.time_stamp
# self.distance_sum = distance_sum
# self.meters_per_second = distance_sum / self.duration
# self.watt_seconds_per_meter = total_watt_seconds/distance_sum
# self.height_change = end_gps.height_m - start_gps.height_m
# self.avg_watts = avg_watts
colorlist = ["#0000FF", "#00FF00", "#FF0066"]
idx = 0
orig_x = []
orig_y = []
colors = []
path_cuts = [(0,0), (23,0), (0,48)]
final_path = []
path1 = []
path2 = []
path3 = []
paths = [path1, path2, path3]
for key in r.scan_iter():
    #print(key)
    if 'gpspath' in str(key):
        #if "row_cross" in str(key) or "new_big_field_row_shortened" in str(key) or "row_2_full" in str(key):
        if "test" in str(key):
            path = pickle.loads(r.get(key))
            # for item in path:
            #     print(type(item))
            print(path)

print("%%%%%%%%%%%%%%%%%%%%%%%%")

lat_lon_tracks = pickle.load(open('lat_lon_tracks_3.pickle', 'rb'))

# print(lat_lon_tracks)
tracknum = 1
track_set_complete = []

for track in lat_lon_tracks:
    # print(len(track))
    if len(track) > 60:
        print(tracknum)
        next_track = []
        for point in track:
            #point = gps_tools.GpsPoint(point[0], point[1])
            point = {'lat':point[0], 'lon':point[1]}
            print(point)
            next_track.append(point)
            track_set_complete.append(point)
        print(next_track)
        r.set('twistedfields:gpspath:autogen_1_row_{}:key'.format(tracknum), pickle.dumps(next_track))
        tracknum += 1
    print("#################")

r.set('twistedfields:gpspath:autogen_1_complete:key', pickle.dumps(track_set_complete))
