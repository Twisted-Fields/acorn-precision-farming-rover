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


_SMOOTH_MULTIPLIER = 0.00000000001

r = redis.Redis(
    host='192.168.1.170',
    port=6379)


for key in r.scan_iter():
    #print(key)
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
        power_vals = [[],[],[],[]]
        total_meters_traveled = 0
        while True:
            print("loop")
            day_index = 0
            last_total = total_meters_traveled
            total_meters_traveled = 0
            list_length = r.llen(key)
            for idx in range(list_length-1, 0, -1):
                # print(idx)
                segment = pickle.loads(r.lindex(key, idx))
                this_stamp = segment.start_gps.time_stamp
                stamp_localtime = time.localtime(this_stamp)
                if stamp_localtime.tm_year == today.tm_year and stamp_localtime.tm_yday == today.tm_yday:
                    total_meters_traveled += segment.distance_sum
                else:
                    print(total_meters_traveled)
                    if total_meters_traveled > last_total:
                        print("Still rollin.")
                    else:
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                        print("##############################################################")
                    time.sleep(5)
                    break
