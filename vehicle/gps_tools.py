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
from scipy.interpolate import splprep, splev


_SMOOTH_MULTIPLIER = 0.00000000001
_GPS_DISTANCE_SCALAR = 100000

GpsSample = namedtuple('GpsSample', 'lat lon height_m status num_sats azimuth_degrees time_stamp rtk_age')

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

def get_heading(start_pt, second_pt):
    start_pt = check_point(start_pt)
    second_pt = check_point(second_pt)
    g = geod.Inverse(start_pt.lat, start_pt.lon, second_pt.lat, second_pt.lon)
    return float(g['azi1'])

def project_point(point, bearing, distance_meters):
    point = check_point(point)
    # Convert to geopy point for calculation.
    point = geopy.Point(point.lat, point.lon)

    d = geopy.distance.VincentyDistance(kilometers = distance_meters/1000.0)

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
            index_down -=1
        if index_up < len(path)-1:
            index_up +=1
        dist_up = get_distance(path[point_index], path[index_up])
        this_err_up = math.fabs(distance_meters - dist_up)
        if  this_err_up < min_err_up:
            best_index_up = index_up
            min_err_up = this_err_up
        dist_down = get_distance(path[point_index], path[index_down])
        this_err_down = math.fabs(distance_meters - dist_down)
        if math.fabs(distance_meters - dist_down) < min_err_down:
            best_index_down = index_down
            min_err_down = this_err_down
        if index_down == 0 and index_up == len(path)-1:
            return path[best_index_up], path[best_index_down]

def get_approx_distance_point_from_line(point, line_pt1, line_pt2):
    """Note that this is not the real distance in meters but a relative measure."""
    p1 = np.asarray((line_pt1.lat, line_pt1.lon))
    p2 = np.asarray((line_pt2.lat, line_pt2.lon))
    p3 = np.asarray((point.lat, point.lon))

    # https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points#
    d1 = np.cross(p2-p1, p1-p3) / np.linalg.norm(p2-p1) * -1
    return d1 * _GPS_DISTANCE_SCALAR
