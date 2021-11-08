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
from remote_control_process import NavigationParameters, PathControlValues, PathSection, Direction
from area import area
import matplotlib.path as path
import gps_tools
import spline_lib
from remote_control_process import EnergySegment
import redis
import time
import pickle
from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mp_colors
import sys
import utm

import scipy

import geomdl.fitting as fitting
from geomdl.visualization import VisMPL
# import open3d
import math
import random


from scipy.interpolate import splprep, splev
sys.path.append('../vehicle')

_SMOOTH_MULTIPLIER = 0.00000000001

# r = redis.Redis(
#     host='acornserver.local',
#     port=6379)

r = redis.Redis(
    host='0.0.0.0',
    port=6379)

_ROW_POINTS_CUT_OFF = 8

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
path_cuts = [(0, 0), (23, 0), (0, 48)]
final_path = []

print("%%%%%%%%%%%%%%%%%%%%%%%%")

_SQUARE_METERS_PER_ACRE = 4046.86

poly_path = None
row = None
for key in r.scan_iter():
    if "parking_row" in str(key):
        row = pickle.loads(r.get(key))
        # row.reverse()
        break


projection_distance_meters = 1.0

projected_points = []

rows_in_polygon = []


heading_1 = gps_tools.get_heading(row[10], row[0])
new_row = gps_tools.offset_row(
    row, projection_distance_meters, heading_1 + 90, copy_data=True, make_dict=True)


print("%%%%%%%%%%%")

# for point in row:
#     print(point)
for point in new_row:
    print(point)

# sys.exit()


#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)
forward_navigation_parameters = NavigationParameters(
    travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=False)
connector_navigation_parameters = NavigationParameters(
    travel_speed=0.0, path_following_direction=Direction.EITHER, vehicle_travel_direction=Direction.FORWARD, loop_path=False)

#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)


_MAXIMUM_ALLOWED_DISTANCE_METERS = 8
_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES = 140

path_control_vals = PathControlValues(
    angular_p=0.9, lateral_p=-0.25, angular_d=0.3, lateral_d=-0.05)
turn_control_vals = PathControlValues(
    angular_p=0.9, lateral_p=-0.25, angular_d=0.3, lateral_d=-0.05)
nav_path = PathSection(points=[],
                       control_values=path_control_vals,
                       navigation_parameters=forward_navigation_parameters,
                       max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                       max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                       end_dist=1.0,
                       end_angle=45)


print(len(row))
# sys.exit()
subject_rows = [row[2:8], new_row[2:8], row[2:8]]
starting_direction = -1
# rows_in_polygon_save = gps_tools.chain_rows(subject_rows, row[0], "three_pt", forward_navigation_parameters, connector_navigation_parameters, turn_control_vals, nav_path, asdict=False)
rows_in_polygon = gps_tools.chain_rows(subject_rows, row[0], starting_direction, "three_pt",
                                       forward_navigation_parameters, connector_navigation_parameters, turn_control_vals, nav_path, asdict=True)

rows_in_polygon = rows_in_polygon[:-1]


print("%%%%%%%%%%%%")
print("%%%%%%%%%%%%")
print("%%%%%%%%%%%%")
print("%%%%%%%%%%%%")
print("%%%%%%%%%%%%")
for single_row in rows_in_polygon:
    print(len(single_row.points))
    # for point in single_row.points:
    #     print(point)
#
r.set('twistedfields:gpspath:three_pt_test:key', pickle.dumps(rows_in_polygon))
sys.exit()

min_x = 0
first_x = 0
min_y = 0
first_y = 0

mesh_array = []
colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
count = 0
lat_lon_tracks = []
for track in rows_in_polygon:
    if count < len(colors):
        row_color = colors[count]
    else:
        row_color = [random.random(), random.random(), random.random()]
    count += 1
    track_lat_lon = []
    # track = track[3:-4]
    if len(track.points) > 2:
        big_point = True
    for point in track.points:
        if big_point:
            mesh_box = open3d.geometry.TriangleMesh.create_box(
                width=1.8, height=1.8, depth=1.8)
            big_point = False
        else:
            mesh_box = open3d.geometry.TriangleMesh.create_box(
                width=0.8, height=0.8, depth=0.8)
        mesh_box.compute_vertex_normals()
        mesh_box.paint_uniform_color(row_color)
        translation = [point["lat"] * 100000 - 3735387,
                       point["lon"] * 100000 + 12233156, 0]
        print(translation)
        #print("{} {}".format(point["lat"] + min_x + first_x, point["lon"] + min_y + first_y))
        #latlon_point = utm.to_latlon(point["lat"] + min_x + first_x, point["lon"] + min_y + first_y, ut_zone[0], ut_zone[1])
        # print(latlon_point)
        # track_lat_lon.append(latlon_point)
        mesh_box.translate(translation)
        mesh_array.append(mesh_box)
    # lat_lon_tracks.append(track_lat_lon)

        pcd = open3d.geometry.PointCloud()
        # np_points = np.random.rand(100, 3)

        # print(np.array(point_cloud))


# From numpy to Open3D
# pcd.points = open3d.utility.Vector3dVector(gps_mesh.pcd)
# # pcd.points = open3d.utility.Vector3dVector(gps_mesh.slice_points)
#
mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(
    size=10, origin=[0, 0, 0])
#
# mesh_array.append(pcd)
# mesh_array.append(mesh_frame)

open3d.visualization.draw_geometries(mesh_array)

# print(row_list[row_key])

# for row_key in sorted(row_list.keys()):
#     print(str(row_key))
#     # print(row_key[0])


# obj = {'type':'Polygon','coordinates':[[[-180,-90],[-180,90],[180,90],[180,-90],[-180,-90]]]}
# area(obj)

# sys.exit()
#
# lat_lon_tracks = pickle.load(open('lat_lon_tracks_3.pickle', 'rb'))
#
# # print(lat_lon_tracks)
# tracknum = 1
# track_set_complete = []
#
# for track in lat_lon_tracks:
#     # print(len(track))
#     if len(track) > 60:
#         print(tracknum)
#         next_track = []
#         for point in track:
#             #point = gps_tools.GpsPoint(point[0], point[1])
#             point = {'lat':point[0], 'lon':point[1]}
#             print(point)
#             next_track.append(point)
#             track_set_complete.append(point)
#         print(next_track)
#         # r.set('twistedfields:gpspath:autogen_1_row_{}:key'.format(tracknum), pickle.dumps(next_track))
#         tracknum += 1
#     print("#################")
#
# # r.set('twistedfields:gpspath:autogen_1_complete:key', pickle.dumps(track_set_complete))
