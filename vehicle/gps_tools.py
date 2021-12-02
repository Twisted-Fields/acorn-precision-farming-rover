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

import geopy
from geopy.distance import distance
from geographiclib.geodesic import Geodesic
import math
from collections import namedtuple
import numpy as np
import copy


_SMOOTH_MULTIPLIER = 0.00000000001
_GPS_DISTANCE_SCALAR = 100000

GpsSample = namedtuple(
    'GpsSample', 'lat lon height_m status num_sats azimuth_degrees time_stamp rtk_age')

GpsPoint = namedtuple('GpsPoint', 'lat lon')

GpsPoint3D = namedtuple('GpsPoint3D', 'lat lon height_m')

geod = Geodesic.WGS84  # define the WGS84 ellipsoid


def get_distance(point1, point2):
    """Gets distance in meters."""
    point1 = check_point(point1)
    point2 = check_point(point2)
    return distance((point1.lat, point1.lon), (point2.lat, point2.lon)).m


def check_point(point):
    if isinstance(point, dict):
        point = GpsPoint(point['lat'], point['lon'])
    return point


def is_dual_fix(gps_sample):
    if not gps_sample:
        return False
    if len(gps_sample.status) == 2:
        if gps_sample.status[0] == 'fix' and gps_sample.status[1] == 'fix':
            return True
    return False


def get_closest_point(point, point_list):
    closest_point = None
    closest_index = -1
    min_distance = math.inf
    for idx in range(len(point_list)):
        d = get_distance(point_list[idx], point)
        if d < min_distance:
            min_distance = d
            closest_point = check_point(point_list[idx])
            closest_index = idx
    return closest_point, closest_index


def determine_point_move_sign(line_points, pt3):
    """
    Determine which side of pt1 is pt3.
    """
    pt1 = line_points[0]
    pt2 = line_points[1]
    pt4 = find_closest_pt_on_line(pt1, pt2, pt3)
    dist1 = get_distance(pt1, pt4)
    dist2 = get_distance(pt2, pt4)
    dist12 = get_distance(pt1, pt2)
    if dist1 < dist2 and dist2 > dist12:
        return 1
    return -1


def find_closest_pt_on_line(pt1, pt2, pt3):
    """
    For a line defined by tuples pt1 and pt2, find the point on the line
    closest to pt3.
    via https://stackoverflow.com/questions/47177493
    """
    x1, y1 = pt1
    x2, y2 = pt2
    x3, y3 = pt3
    dx, dy = x2 - x1, y2 - y1
    det = dx * dx + dy * dy
    a = (dy * (y3 - y1) + dx * (x3 - x1)) / det
    return GpsPoint(x1 + a * dx, y1 + a * dy)


def get_heading(start_pt, second_pt):
    start_pt = check_point(start_pt)
    second_pt = check_point(second_pt)
    g = geod.Inverse(start_pt.lat, start_pt.lon, second_pt.lat, second_pt.lon, Geodesic.AZIMUTH)
    return float(g['azi1'])


def project_point(point, bearing, distance_meters):
    point = check_point(point)
    # Convert to geopy point for calculation.
    point = geopy.Point(point.lat, point.lon)

    d = geopy.distance.geodesic(kilometers=distance_meters / 1000.0)

    # Use the `destination` method with a bearing of bearing degrees
    geo_point = d.destination(point=point, bearing=bearing)

    return GpsPoint(geo_point.latitude, geo_point.longitude)


def get_closest_points_at_distance(point_index, distance_meters, path):
    """Finds points some distance from a known point on the path"""
    min_err_up = math.inf
    best_index_up = point_index
    min_err_down = math.inf
    best_index_down = point_index
    index_down = point_index
    index_up = point_index
    while True:
        if index_down > 0:
            index_down -= 1
        if index_up < len(path) - 1:
            index_up += 1
        dist_up = get_distance(path[point_index], path[index_up])
        this_err_up = math.fabs(distance_meters - dist_up)
        if this_err_up < min_err_up:
            best_index_up = index_up
            min_err_up = this_err_up
        dist_down = get_distance(path[point_index], path[index_down])
        this_err_down = math.fabs(distance_meters - dist_down)
        if math.fabs(distance_meters - dist_down) < min_err_down:
            best_index_down = index_down
            min_err_down = this_err_down
        if index_down == 0 and index_up == len(path) - 1:
            return path[best_index_up], path[best_index_down]


def get_approx_distance_point_from_line(point, line_pt1, line_pt2):
    """Note that this is not the real distance in meters but a relative measure."""
    p1 = np.asarray((line_pt1.lat, line_pt1.lon))
    p2 = np.asarray((line_pt2.lat, line_pt2.lon))
    p3 = np.asarray((point.lat, point.lon))

    # https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points#
    d1 = np.cross(p2 - p1, p1 - p3) / np.linalg.norm(p2 - p1) * -1
    return d1 * _GPS_DISTANCE_SCALAR


def offset_row(row, distance, direction, copy_data=False, make_dict=True):
    new_row = []
    for pt in row:
        new_pt = None
        offset_pt = project_point(pt, direction, distance)
        if copy_data:
            try:
                new_pt = GpsSample(offset_pt.lat, offset_pt.lon, pt['height_m'], pt['status'],
                                   pt['num_sats'], pt['azimuth_degrees'], pt['time_stamp'], 0)
                print("copy_ok")
            except BaseException:
                try:
                    new_pt = GpsPoint3D(
                        offset_pt.lat, offset_pt.lon, pt['height_m'])
                except BaseException:
                    pass
        if not new_pt:
            new_pt = offset_pt
        if make_dict:
            new_pt = new_pt._asdict()
        new_row.append(new_pt)
    return new_row


def interpolate_points(points, num_points):
    total_distance = get_distance(points[0], points[1])
    increment_distance = total_distance / num_points
    heading = get_heading(points[0], points[1])
    last_point = points[0]
    interpolated_points = [points[0]]
    for _ in range(num_points):
        new_point = project_point(last_point, heading, increment_distance)
        interpolated_points.append(new_point._asdict())
        last_point = new_point
    return interpolated_points


def three_point_turn(points, angle, distance_away, turn_radius, robot_length=2.0, asdict=False):
    heading = get_heading(points[0], points[1])
    row_aligned_away_pt = project_point(points[1], heading, distance_away)
    latlon_point1 = project_point(row_aligned_away_pt, heading + angle, 0.5)
    latlon_point2 = project_point(row_aligned_away_pt, heading + angle, 1.0)
    # latlon_point1 = project_point(row_aligned_away_pt, heading + angle, turn_radius)
    # latlon_point2 = project_point(row_aligned_away_pt, heading + angle, turn_radius)
    if asdict:
        return [latlon_point1._asdict(), latlon_point2._asdict()]
    return [check_point(latlon_point1), check_point(latlon_point2)]


def chain_rows(row_list, starting_point, starting_direction, connection_type, forward_navigation_parameters,
               connector_navigation_parameters, turn_control_vals, nav_path, asdict=False):
    """
    Given a list of GPS rows, connect each row with a u-turn command.
    """
    row_chain = []
    last_point = starting_point

    for idx in range(len(row_list)):
        row = row_list[idx]
        if get_distance(last_point, row[0]) > get_distance(last_point, row[-1]):
            row.reverse()

        row_path = copy.deepcopy(nav_path)
        row_path.points = row
        row_path.navigation_parameters = forward_navigation_parameters
        row_chain.append(row_path)

        if idx + 1 >= len(row_list):
            return row_chain
        next_row = row_list[idx + 1]
        angle_reverse = starting_direction
        if get_distance(row[-1], next_row[0]) > get_distance(row[-1], next_row[-1]):
            next_row.reverse()
            if len(row_list) > 3:
                angle_reverse *= -1
        if connection_type == "three_pt":
            start_points = row[-2], row[-1]
            turn1 = three_point_turn(
                start_points, angle=90 * angle_reverse, distance_away=1.5, turn_radius=0.1, asdict=asdict)

            if asdict:
                final_connector = next_row[0], next_row[1]
            else:
                final_connector = check_point(
                    next_row[0]), check_point(next_row[1])

            turn1_path = copy.deepcopy(nav_path)
            turn1_path.points = turn1
            turn1_path.navigation_parameters = connector_navigation_parameters
            turn1_path.end_dist = 3.0
            turn1_path.end_angle = 45
            turn1_path.control_values = turn_control_vals
            row_chain.append(turn1_path)

            connector_path = copy.deepcopy(nav_path)
            connector_path.points = final_connector
            connector_path.navigation_parameters = connector_navigation_parameters
            connector_path.end_dist = 2.0
            connector_path.end_angle = 30
            connector_path.control_values = turn_control_vals
            row_chain.append(connector_path)
            last_point = next_row[0]

    return row_chain
