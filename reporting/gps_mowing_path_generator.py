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

tracks = []

converted_points = []
for item in polygon:
    point = item
    point = gps_tools.GpsPoint(point[1], point[0])
    converted_points.append(point)

NUM_ROW_POINTS = 30

initial_row = gps_tools.interpolate_points([converted_points[1], converted_points[2]], NUM_ROW_POINTS, make_dict=False)
# print(initial_row)
tracks.append(initial_row)

row_spacing = 1
print(converted_points)
total_horizontal_distance = gps_tools.get_distance(converted_points[1], converted_points[0])
horizontal_direction = gps_tools.get_heading(converted_points[1], converted_points[0])
print(total_horizontal_distance)
# sys.exit()

rows_required = int(0.5*total_horizontal_distance/row_spacing)*2
row_counter = 0

while row_counter <= rows_required:
    next_row = gps_tools.offset_row(tracks[row_counter], row_spacing, horizontal_direction, copy_data=False, make_dict=False)
    tracks.append(next_row)
    row_counter += 1

print(row_counter)
# sys.exit()


total_paths_length = []

# https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302
def rotate_origin_only(xy, radians):
    """Only rotate a point around the origin (0, 0)."""
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return xx, yy

# print(total_paths_length)
# total = 0
# for path_length in total_paths_length:
#     print(path_length)
#     total += path_length
# print("TOTAL ROWS LENGTH: {}".format(total))

mesh_array = []
colors = [[1,0,0],[0,1,0],[0,0,1]]
count = 0
lat_lon_tracks = []
start_x = None
start_y = None
for track in tracks:
    if count < len(colors):
        row_color = colors[count]
    else:
        row_color = [random.random(), random.random(), random.random()]
    count += 1
    for point in track:
        mesh_box = open3d.geometry.TriangleMesh.create_box(width=0.8, height=0.8, depth=0.8)
        mesh_box.compute_vertex_normals()
        mesh_box.paint_uniform_color(row_color)
        # print(type(point))
        # print(dir(point))
        x, y, zone1, zone2 = utm.from_latlon(latitude=point.lat, longitude=point.lon)
        translation = [x, y, 0.0]
        if not start_x:
            start_x = x
        if not start_y:
            start_y = y
        # track_lat_lon.append(latlon_point)
        mesh_box.translate(translation)
        mesh_array.append(mesh_box)
    # lat_lon_tracks.append(track_lat_lon)
pickle.dump(tracks, open('mowing_tracks.pickle', 'wb'))




pcd = open3d.geometry.PointCloud()
# np_points = np.random.rand(100, 3)

# print(np.array(point_cloud))



# From numpy to Open3D
# pcd.points = open3d.utility.Vector3dVector(gps_mesh.pcd)
# pcd.points = open3d.utility.Vector3dVector(gps_mesh.slice_points)

mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(
    size=10, origin=[start_x, start_y, 0])

mesh_array.append(pcd)
mesh_array.append(mesh_frame)

open3d.visualization.draw_geometries(mesh_array)
