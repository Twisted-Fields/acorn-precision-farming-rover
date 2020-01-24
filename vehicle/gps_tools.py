import geopy
from geopy.distance import distance
from geographiclib.geodesic import Geodesic
import math
from collections import namedtuple

GpsSample = namedtuple('GpsSample', 'lat lon height_m status num_sats azimuth_degrees time_stamp')

GpsPoint = namedtuple('GpsPoint', 'lat lon')

geod = Geodesic.WGS84  # define the WGS84 ellipsoid

def get_distance(point1, point2):
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
            closest_point = point_list[idx]
            closest_index = idx
    return closest_point, closest_index

def get_heading(start_pt, second_pt):
    start_pt = check_point(start_pt)
    second_pt = check_point(second_pt)
    g = geod.Inverse(start_pt.lat, start_pt.lon, second_pt.lat, second_pt.lon)
    return float(g['azi1'])

def project_point(point, heading):
    """This is some junk code. Not real math!"""
    point = check_point(point)
    new_lat = point.lat + (math.sin(math.radians(heading)) * 0.00004)
    new_lon = point.lon + (math.cos(math.radians(heading)) * 0.00004)
    return GpsSample(new_lat, new_lon, 0, "", 0, 0)
