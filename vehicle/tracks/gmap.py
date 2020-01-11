import gmplot
import socket
import subprocess
import time
import os
from collections import namedtuple
import pickle
import math
from datetime import datetime
import geopy
from geopy.distance import distance
from geographiclib.geodesic import Geodesic

AZ_ADJ = -45


GpsSample = namedtuple('GpsSample', 'lat lon height_m status num_sats azimuth')


geod = Geodesic.WGS84  # define the WGS84 ellipsoid

filename = "gps_track_08-11-2019_11-25-47_PM.pkl"
gps_data = pickle.load( open( filename, "rb" ) )

latitude_list = []
longitude_list = []
combined_list = []



index = 0
for item in gps_data:
    if (index > 900 and index < 920):
        latitude_list.append(item.lat)
        longitude_list.append(item.lon)
        combined_list.append(item)
        #print("[{},{}]".format(item.lat, item.lon))
        print("{{lat: {}, lon: {}}},".format(item.lat, item.lon))
    index+=1


point = gps_data[680]

point = GpsSample(point.lat+.0001, point.lon-.0001, 0, "", 0, point.azimuth)

def project_point(point, heading):
    new_lat = point.lat + (math.sin(math.radians(heading)) * 0.00004)
    new_lon = point.lon + (math.cos(math.radians(heading)) * 0.00004)
    return GpsSample(new_lat, new_lon, 0, "", 0, 0)

#print(point.azimuth)

point2 = project_point(point, -point.azimuth + AZ_ADJ)

lats2 = [point.lat, point2.lat]
lons2 = [point.lon, point2.lon]


def get_distance(point1, point2):
    return distance((point1.lat, point1.lon), (point2.lat, point2.lon)).m


def get_closest_point(point, point_list):
    closest_point = None
    min_distance = math.inf
    for item in point_list:
        d = get_distance(item, point)
        if d < min_distance:
            min_distance = d
            closest_point = item
    return closest_point

def get_heading(start_pt, second_pt):
    g = geod.Inverse(start_pt.lat, start_pt.lon, second_pt.lat, second_pt.lon)
    return float(g['azi1'])


closest_point = get_closest_point(point, combined_list)

heading = 90 - get_heading(point, closest_point)
print(heading)
heading_proj = project_point(point, heading)

diff = -point.azimuth + AZ_ADJ - heading
print(diff)





lats3 = [point.lat, heading_proj.lat]
lons3 = [point.lon, heading_proj.lon]

gmap3 = gmplot.GoogleMapPlotter(point.lat, point.lon, 21)
gmap3.apikey = 'AIzaSyAPS-BUmKaCfHnQRVx3UYfv5OfiFb0RIgU'

# scatter method of map object
# scatter points on the google map
gmap3.scatter(latitude_list, longitude_list, '# FF0000',
                              size = 0.5, marker = False )

# gmap3.scatter(lats2, lons2, '# FF0000',
#                             size = 0.5, marker = False )

gmap3.marker(closest_point.lat, closest_point.lon, 'cornflowerblue')

gmap3.plot(lats2, lons2,
           'DarkRed', edge_width = 12.5)
gmap3.plot(lats3, lons3,
          'DarkGreen', edge_width = 5.5)

print(gps_data[680])

# Plot method Draw a line in
# between given coordinates
gmap3.plot(latitude_list, longitude_list,
           'cornflowerblue', edge_width = 2.5)

gmap3.draw( "map13.html" )
