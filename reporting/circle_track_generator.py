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
import matplotlib.path as path
import sys
import scipy
import utm
import geomdl.fitting as fitting
from geomdl.visualization import VisMPL
import open3d
import math
import random
import gps_mesh_utils as meshing
import traceback
import curver
import datetime


from scipy.interpolate import splprep, splev
sys.path.append('../vehicle')
from remote_control_process import EnergySegment
from spline_lib import GpsSpline
import gps_tools


_SMOOTH_MULTIPLIER = 0.00000000001

_MIN_ALLOWED_PATH_ANGLE = 120

# r = redis.Redis(
#     host='acornserver.local',
#     port=6379)

r = redis.Redis(
    host='127.0.0.1',
    port=6379)



def calculate_distance(point1, point2):
    p1 = np.array([point1[0], point1[1], point1[2]])
    p2 = np.array([point2[0], point2[1], point2[2]])
    squared_dist = np.sum((p1-p2)**2, axis=0)
    return(np.sqrt(squared_dist))

def get_angle(last, current, nextp):
    """Get the 2D planar angle between three grid points."""
    #  (l) *
    #      a  c
    #  (c) * b  * (n)

    dist_a = calculate_distance(last, current)
    dist_b = calculate_distance(current, nextp)
    dist_c = calculate_distance(last, nextp)

    angle = math.degrees(
        np.arccos(
         (dist_a ** 2 + dist_b ** 2 - dist_c ** 2) / (2 * dist_a * dist_b))
        )
    if math.isnan(angle):
        # Check for colinearity by confirming the distances sum as
        # expected.
        distances = [dist_a, dist_b, dist_c]
        distances.sort()
        calculated_diff = distances[0] + distances[1] - distances[2]
        if math.isclose(0, calculated_diff, abs_tol=0.001):
            # Points are colinear.
            if last[0] < current[0] < nextp[0] or \
              last[0] > current[0] > nextp[0] or \
              last[1] < current[1] < nextp[1] or \
              last[1] > current[1] > nextp[1]:
                return 180
            else:
                return 0
    return angle

def get_dense_path(redis_client=None, robot_key=None):
    key = path_key = bytes(str(robot_key)[2:-1].replace(":key", ":energy_segment:key"), encoding='ascii')
    list_length = redis_client.llen(key)
    path = []
    today = datetime.datetime.today()
    for idx in range(list_length - 1, 0, -1):
        segment = pickle.loads(redis_client.lindex(key, idx))
        # print(segment.subsampled_points)
        try:
            if (today - segment.start_gps.time_stamp.replace(tzinfo=None)).total_seconds() > 12*60*60:
                if (today - segment.start_gps.time_stamp.replace(tzinfo=None)).total_seconds() < 24*60*60:
                    path.append(segment.start_gps)
                    path_length = len(segment.subsampled_points)
            # if (today - segment.start_gps.time_stamp.replace(tzinfo=None)).days > 2:
            #     return path


            # path.append(segment.subsampled_points[int(path_length/2)])
            # path.append(segment.subsampled_points[int(2 * path_length/3)])
            # for point in segment.subsampled_points:
            #     path.append(point)
        except Exception as e:
            print(e)
            break

    return path

# https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302
def rotate_origin_only(xy, radians):
    """Only rotate a point around the origin (0, 0)."""
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return xx, yy

from area import area

_SQUARE_METERS_PER_ACRE = 4046.86

poly_path = None
for key in r.scan_iter():
    #print(key)
    if 'gpspolygon' in str(key):
        print(key)
        polygon = pickle.loads(r.get(key))
        # print(polygon["geometry"])
        polygon_area = area(polygon["geometry"])
        print("Polygon is {} acres".format(polygon_area/_SQUARE_METERS_PER_ACRE))
        #print(polygon["geometry"]["coordinates"][0])
        polygon = polygon["geometry"]["coordinates"][0]
        poly_path = path.Path(polygon, closed=True)
        print(polygon)
        # sys.exit()

    # if poly_path.contains_point((point['lon'], point['lat']),radius=0.0):
    #     x, y, zone1, zone2 = utm.from_latlon(latitude=point['lat'], longitude=point['lon'])



center_point = (polygon[0][1], polygon[0][0])




total_paths_length = []

CIRCLE_RADIUS_M = 3

circle_points = []
for angle_degrees in range(0,360,10):
    circle_points.append(gps_tools.project_point(center_point,angle_degrees, CIRCLE_RADIUS_M))


print(circle_points)

pickle.dump(circle_points, open('circle_tracks2.pickle', 'wb'))













#h
