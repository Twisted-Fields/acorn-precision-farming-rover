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
import sys
sys.path.append('../vehicle')
import copy
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

import utm

import scipy

import geomdl.fitting as fitting
from geomdl.visualization import VisMPL
# import open3d
import math
import random


from scipy.interpolate import splprep, splev


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
path1 = []
path2 = []
path3 = []
paths = [path1, path2, path3]


print("%%%%%%%%%%%%%%%%%%%%%%%%")

_SQUARE_METERS_PER_ACRE = 4046.86

poly_path = None
row_list = {}
for key in r.scan_iter():
    # print(key)
    if 'gpspolygon' in str(key):
        print(key)
        polygon = pickle.loads(r.get(key))
        print(polygon["geometry"])
        polygon_area = area(polygon["geometry"])
        print("Polygon is {} acres".format(
            polygon_area/_SQUARE_METERS_PER_ACRE))
        print(polygon["geometry"]["coordinates"][0])
        polygon = polygon["geometry"]["coordinates"][0]
        poly_path = path.Path(polygon, closed=True)
        # print(poly_path)
    if "twistedfields:gpspath:autogen_01_row_" in str(key):
        row = pickle.loads(r.get(key))
        row_list[str(key)] = row
# print(row_list.keys())

rows_in_polygon = []

for row_number in range(len(row_list)):
    row_key = "b'twistedfields:gpspath:autogen_01_row_{:02d}:key'".format(
        row_number+1)
    row = row_list[row_key]
    # print(row)
    row_points_in_polygon = []
    for point in row:
        if poly_path.contains_point((point["lon"], point["lat"]), radius=0.0):
            # print(point)
            row_points_in_polygon.append(point)
        elif len(row_points_in_polygon) > 0:
            if len(row_points_in_polygon) > _ROW_POINTS_CUT_OFF:
                rows_in_polygon.append(row_points_in_polygon)
                print(len(row_points_in_polygon))
            break

# def calculate_distance(point1, point2):
#     p1 = np.array([point1[0], point1[1]])
#     p2 = np.array([point2[0], point2[1]])
#     squared_dist = np.sum((p1-p2)**2, axis=0)
#     return(np.sqrt(squared_dist))
#
#
# def calculate_projection(point1, point2, distance):
#     """Return a point a given distance past point2."""
#     delta_x = utm_points[0][0] - utm_points[1][0]
#     delta_y = utm_points[0][1] - utm_points[1][1]
#     distance = calculate_distance(utm_points[0], utm_points[1])
#     new_x = (projection_distance_meters * delta_x)/distance
#     new_y = (projection_distance_meters * delta_y)/distance
#     return point2[0] + new_x, point2[1] + new_y

projection_distance_meters = 2.0

print("$")


#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)
forward_navigation_parameters = NavigationParameters(
    travel_speed=0.4, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.FORWARD, repeat_path=False)
connector_navigation_parameters = NavigationParameters(
    travel_speed=0.2, path_following_direction=Direction.EITHER, vehicle_travel_direction=Direction.FORWARD, repeat_path=False)

#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)


_MAXIMUM_ALLOWED_DISTANCE_METERS = 8
_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES = 140

# path_control_vals = PathControlValues(angular_p=0.9, lateral_p=-0.25, angular_d=0.3, lateral_d=-0.2)
# turn_control_vals = PathControlValues(angular_p=0.9, lateral_p=-0.25, angular_d=0.3, lateral_d=-0.2)

path_control_vals = PathControlValues(
    angular_p=0.7, lateral_p=0.15, angular_d=0.4, lateral_d=0.1)
turn_control_vals = PathControlValues(
    angular_p=0.7, lateral_p=0.15, angular_d=0.4, lateral_d=0.1)
nav_path = PathSection(points=[],
                       control_values=path_control_vals,
                       navigation_parameters=forward_navigation_parameters,
                       max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                       max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                       end_dist=1.0,
                       end_angle=45)


starting_direction = -1

rows_in_polygon = gps_tools.chain_rows(rows_in_polygon, rows_in_polygon[0][0], starting_direction, "three_pt",
                                       forward_navigation_parameters, connector_navigation_parameters, turn_control_vals, nav_path, asdict=True)


interpolate_list = []

row = rows_in_polygon[-1].points
start_points = row[-2], row[-1]


heading = gps_tools.get_heading(start_points[0], start_points[1])
row_aligned_away_pt = gps_tools.project_point(start_points[1], heading, 1.5)
latlon_point1 = gps_tools.project_point(row_aligned_away_pt, heading + 90, 0.5)
latlon_point2 = gps_tools.project_point(row_aligned_away_pt, heading + 90, 1.0)
new_turn = [latlon_point1._asdict(), latlon_point2._asdict()]

interpolate_list.append(latlon_point2._asdict())

turn1_path = copy.deepcopy(nav_path)
turn1_path.points = new_turn
turn1_path.navigation_parameters = connector_navigation_parameters
turn1_path.end_dist = 1.0
turn1_path.end_angle = 20
turn1_path.control_values = turn_control_vals

rows_in_polygon.append(turn1_path)


row = rows_in_polygon[0].points
start_points = row[1], row[0]

heading = gps_tools.get_heading(start_points[0], start_points[1])
row_aligned_away_pt = gps_tools.project_point(start_points[1], heading, 1.5)
latlon_point1 = gps_tools.project_point(
    row_aligned_away_pt, heading + -90, 1.0)
latlon_point2 = gps_tools.project_point(
    row_aligned_away_pt, heading + -90, 0.5)
interpolate_list.append(latlon_point2._asdict())
new_turn = [latlon_point1._asdict(), latlon_point2._asdict()]

turn1_path = copy.deepcopy(nav_path)
turn1_path.points = new_turn
turn1_path.navigation_parameters = connector_navigation_parameters
turn1_path.end_dist = 1.0
turn1_path.end_angle = 20
turn1_path.control_values = turn_control_vals

# print(interpolate_list)
interpolated_path_points = gps_tools.interpolate_points(interpolate_list, 25)

print(interpolated_path_points)


interpolated_path = copy.deepcopy(nav_path)
interpolated_path.points = interpolated_path_points
interpolated_path.navigation_parameters = forward_navigation_parameters
interpolated_path.end_dist = 1.0
interpolated_path.end_angle = 20
interpolated_path.control_values = path_control_vals


rows_in_polygon.append(interpolated_path)

rows_in_polygon.append(turn1_path)


row = rows_in_polygon[0].points
start_points = row[0], row[1]

turn1_path = copy.deepcopy(nav_path)
turn1_path.points = start_points
turn1_path.navigation_parameters = connector_navigation_parameters
turn1_path.end_dist = 1.0
turn1_path.end_angle = 20
turn1_path.control_values = turn_control_vals

rows_in_polygon.append(turn1_path)

# rows_in_polygon = rows_in_polygon[-8:]


r.set('twistedfields:gpspath:aaa_test3:key', pickle.dumps(rows_in_polygon))

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
    for point in track.points:
        if len(track.points) == 2:
            mesh_box = open3d.geometry.TriangleMesh.create_box(
                width=0.8, height=0.8, depth=0.8)
        else:
            mesh_box = open3d.geometry.TriangleMesh.create_box(
                width=0.7, height=0.7, depth=0.7)

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
mesh_array.append(mesh_frame)

open3d.visualization.draw_geometries(mesh_array)
