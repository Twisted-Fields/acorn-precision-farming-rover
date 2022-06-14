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
import surface_test
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
    host='192.168.1.170',
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


# r.set('foo', 'bar')

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
        # print(poly_path)


# obj = {'type':'Polygon','coordinates':[[[-180,-90],[-180,90],[180,90],[180,-90],[-180,-90]]]}
# area(obj)

LOAD_RECOVERED_PATH = False
new_path = []
if LOAD_RECOVERED_PATH:
    key = 'twistedfields:gpspath:recovered_points:key'
    new_path = pickle.loads(r.get(key))
else:
    for key in r.scan_iter():
        if 'twistedfields:robot:acorn1:key' in str(key):
            # robot_key = key
            path = get_dense_path(redis_client=r, robot_key=key)
            path = [point._asdict() for point in path]
            # path = path[45:2120]
            last_p = None

            for p in path:
                if p != last_p:
                    new_path.append(p)
                    last_p = p
                    # print(p)
            print(len(new_path))



#
# with open('5-30-2022-path_vals.pickle', 'wb') as handle:
#     pickle.dump(new_path, handle, protocol=pickle.HIGHEST_PROTOCOL)

# with open('5-30-2022-path_vals.pickle', 'rb') as handle:
#     new_path = pickle.load(handle)

# sys.exit()

orig_x = []
orig_y = []
orig_y1 = []
orig_y2 = []
orig_y3 = []
orig_y4 = []
orig_z = []
colors = []

values = []
min_colorval = 9999999
max_colorval = -9999999

print(new_path[0])

# first_stamp = new_path[0]['time_stamp']
colorby = ""
watt_seconds = False
now = time.time()
today = time.localtime(now)
power_vals = [[],[],[],[]]

day_index = 0
total_daily_meters = [0]

first_x = None
first_y = None

min_x = 999999
min_y = 999999
max_x = -999999
max_y = -999999

points = np.empty((0,3))

ut_zone = None

# report distance
total_meters_traveled = 0

max_z = 0
print(new_path[40])
for point in new_path:

    if point['time_stamp'].day == 9 and point['time_stamp'].hour == 19:
        if point['time_stamp'].minute in [22,20,19,17,12,38,53]:
            continue
    if point['time_stamp'].day == 9 and point['time_stamp'].hour == 20:
        if point['time_stamp'].minute in [46]:
            continue

    if poly_path.contains_point((point['lon'], point['lat']),radius=0.0):#0000001):
        # print("{}, {}, {}".format(point.lat, point.lon, point.height_m))
        #print("{}, {}, {}".format(point.lat, point.lon, point.height_m))
        x, y, zone1, zone2 = utm.from_latlon(latitude=point['lat'], longitude=point['lon'])
        # print("{}, {}".format(x,y))
        if (zone1, zone2) != ut_zone:
            print("NEW UT ZONE")
            print((zone1, zone2))
        ut_zone = (zone1, zone2)
        if first_x == None:
            first_x = x
            first_y = y
        x -= first_x
        y -= first_y

        if x < min_x:
            min_x = x
        if y < min_y:
            min_y = y
        if x > max_x:
            max_x = x
        if y > max_y:
            max_y = y


        points = np.append(points, np.array([[x, y, point['height_m']]]), axis=0)

# print(max_z)
# sys.exit()

print(min_x)
print(min_y)
print(max_x)
print(max_y)
sys.exit()
# time.sleep(0.5)

# print(points[0])
# print(points[1])
# print(points[3])
# # points = points[-8:]
# # points = np.sort(points)
# # for point in points:
# #     print(point)
# # print("%%%")
# points = points[points[:, 2].argsort()]
#
# points_bad = points[-14:]
# points = points[:-14]
#
#
# point['time_stamp'].day == 9, point['time_stamp'].hour >= 19
#
#
# for point in new_path:
#     height = point['height_m']
#     for point_bad in points_bad:
#         if point_bad[2] == height:
#             # if point['time_stamp'].day == 9 and point['time_stamp'].hour >= 19:
#             #     print("True")
#             # else:
#             #     print("False")
#             print(point['time_stamp'])
#             # print(dir(point['time_stamp']))
# sys.exit()
# for point in points:
#     print(point)

# print(points.shape)
# sys.exit()


# points is np.array so this is elementwise
points = points - np.array([min_x, min_y, 0])

max_x -= min_x
max_y -= min_y



print("Begin meshing")
gps_mesh = meshing.Mesh(points, max_x, max_y)

print("interpolate to grid")
gps_mesh.interpolate_to_grid()
print("generate_point_cloud")
gps_mesh.generate_point_cloud(poly_path, min_x + first_x, min_y + first_y, ut_zone)



# print(gps_mesh.pcd)
#
# print(gps_mesh.slice_height_points(103.5, 104.5))

tracks = []
row_spacing = 1.2

sample_height = gps_mesh.get_max_height_point()[2] - 0.2
# height_test = sample_height - 0.5

total_paths_length = []

# https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302
def rotate_origin_only(xy, radians):
    """Only rotate a point around the origin (0, 0)."""
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return xx, yy

while True:
    try:
        mesh_subsample = gps_mesh.slice_height_points(sample_height, 0.12)
        if len(mesh_subsample) == 0:
            break
        #print(mesh_subsample)
        contours = [[mesh_subsample[0]]]
        mesh_subsample = np.delete(mesh_subsample, 0, axis=0)
        THRESHOLD = 8.0
        sub_index = 0

        while len(mesh_subsample) > 1:
            no_change = True
            for contour_index in range(len(contours)):
                while True:
                    added = False
                    for contour_point in contours[contour_index]:
                        sub_index = 0
                        while sub_index < len(mesh_subsample):
                            #print(contour_point)
                            p1 = (contour_point[0], contour_point[1])
                        #    print(mesh_subsample)
                            p2 = (mesh_subsample[sub_index][0], mesh_subsample[sub_index][1])
                            dist = gps_mesh.get_distance(p1, p2)
                            if dist < THRESHOLD:
                                contours[contour_index].append(mesh_subsample[sub_index])
                                mesh_subsample = np.delete(mesh_subsample, sub_index, axis=0)
                                added = True
                                no_change = False
                                #print("added")
                            else:
                                sub_index += 1
                            #print(sub_index)
                    #print("loop")
                    if added == False:
                        break
            if no_change == True:
            #    print("No change")
                contours.append([mesh_subsample[0]])
                np.delete(mesh_subsample, 0)

        for contour in contours:
            contour_subsample = np.array(contour)
            if len(contour_subsample) < 8:
                continue
            # print(contour)
            # tracks.append(contour)
            # continue
                            #print(len(contour))

        #    pickle.dump(contour_subsample, open( "save.p", "wb" ))
            #sys.exit()

            path_points = []
            points = []
            values = []
            blah = []
            for index in range(len(contour_subsample)):
                contour_subsample[index][0] += random.random() * 0.01
                #contour_subsample[index][1] += random.random() * 0.01
                #contour_subsample[index][2] += random.random() * 0.01
            # sorted_array = contour_subsample[np.argsort(contour_subsample[:, 0])]
            # curver_array = []
            for point in contour_subsample:
                path_points.append(gps_tools.GpsPoint3D(point[0], point[1], gps_mesh.get_height(point)))
            #     points.append(point[0])
            #     #print(point[0])
            #     values.append(point[1])
            #     blah.append(gps_mesh.get_height(point))
            #     curver_array.append([point[0], point[1]])
            #print(contour_subsample)
            #gps_spline = GpsSpline(path_points, 10, 10)
            # gps_spline = GpsSpline(gps_spline.points, 10, 10)
        #    gps_spline = GpsSpline(gps_spline.points, 10, 50)
            if len(path_points) < 20:
                continue
            #print(len(path_points))
            #print(path_points)
            gps_spline = GpsSpline(path_points, 1000, 20, degree=5)
            # curve_obj = curver.Curver(np.array(curver_array))
            # reconstructed = curve_obj.reconstruct(H=0.01,niter=5)
            # full_spline = []
            # for point in reconstructed:
            #     full_spline.append([point[0],point[1], sample_height])
            #print(reconstructed)
            # sys.exit()
            # from scipy.spatial.distance import pdist, squareform
            #
            # X = np.array(curver_array)
            # #print(X)
            #
            # k = 4
            # # matrix of pairwise Euclidean distances
            # distmat = squareform(pdist(X, 'euclidean'))
            # # select the kNN for each datapoint
            # neighbors = np.sort(np.argsort(distmat, axis=1)[:, 0:k])
            # newpoints = []
            # for i in np.arange(len(X)):
            #     for j in np.arange(k):
            #         x1 = np.array([X[i,:][0], X[neighbors[i, j], :][0]])
            #         x2 = np.array([X[i,:][1], X[neighbors[i, j], :][1]])
            #         #print("{}, {}, {}, {}".format(x1[0], x2[0], x1[1], x2[1]))
            #         newpoints.append([x1[0], x2[0]])

            #print(newpoints)


            # dist_sq = np.sum((X[:, np.newaxis, :] - X[np.newaxis, :, :]) ** 2, axis=-1)
            # print(dist_sq)
            # K = 2
            # nearest_partition = np.argpartition(dist_sq, K + 1, axis=1)
            # print(nearest_partition)
            #sys.exit()
            # newpoints = np.unique(newpoints,axis=0)
            # path_points = []
            # for point in newpoints:
            #     print(point)
            #     path_points.append(gps_tools.GpsPoint3D(point[0], point[1], sample_height))





            full_spline = gps_spline.filter_data(contour_subsample)
            full_spline_copy = full_spline.copy()

            if len(full_spline) < 3:
                continue

            rotation_rads = 0.0
            rotation_increment = 0.1
            spline_okay = False
            spline_failed = False
            while not spline_okay and not spline_failed:
                min_x = full_spline[0][0]
                for index in range(len(full_spline)):
                    point = full_spline[index]
                    if point[0] < min_x:
                        break
                    if point[0] > min_x:
                        min_x = point[0]
                    if index == len(full_spline)-1:
                        spline_okay = True
                if not spline_okay:
                    rotation_rads += rotation_increment
                    #print(rotation_rads)
                    if rotation_rads > 2.0 * math.pi:
                        spline_failed = True
                    for index in range(len(full_spline)):
                        xy = full_spline[index][0:2]
                        x, y = rotate_origin_only(xy, rotation_increment)
                        full_spline[index][0] = x
                        full_spline[index][1] = y
            if spline_failed:
                continue
            x_vals = []
            y_vals = []
            for point in full_spline:
                x_vals.append(point[0])
                y_vals.append(point[1])

            full_spline = gps_spline.get_2d_spline(x_vals, y_vals, blah)
            if rotation_rads > 0:
                for index in range(len(full_spline)):
                    xy = full_spline[index][0:2]
                    x, y = rotate_origin_only(xy, 2.0*math.pi-rotation_rads)
                    full_spline[index][0] = x
                    full_spline[index][1] = y
            # import csaps
            # sp_theta = csaps.MultivariateCubicSmoothingSpline(full_spline, theta, smooth=0.95)

            # print(filtered)

            # heights = np.full(len(spline), sample_height)
            # # print(spline)
            # # print(heights)
            # #full_spline = np.concatenate((spline[0], heights), axis=1)
            # full_spline = np.array(list(zip(spline[:,:1], spline[:,1:], heights)))
            #print(full_spline)
            #sys.exit()
            path_points = []
            for point in full_spline:
                path_points.append(gps_tools.GpsPoint3D(point[0], point[1], sample_height))

            path_length = 0
            previous_point = None
            for point in path_points:
                if previous_point:
                    path_length += gps_mesh.get_distance(point, previous_point)
                previous_point = point
            # total_paths_length.append(path_length)


            gps_spline = GpsSpline(path_points, 100, int(path_length))
            gps_spline.calculate_3D_factors(path_points, 100, int(path_length))
            new_points = gps_spline.get_array3D()
            gps_spline = GpsSpline(new_points, 1000, int(path_length/1.0))
            gps_spline.calculate_3D_factors(new_points, 1000, int(path_length/1.0))
            full_spline = gps_spline.get_array3D()

            row_okay = True
            for track in tracks:
                #print(path_points)
                #print(track)
                if not gps_mesh.check_path_to_path_distance(full_spline, track, row_spacing):
                    print("reject row")
                    #tracks = [full_spline, track]
                    #sample_height -= 0.01
                    row_okay = False
                    break
            if row_okay:
                print("PATH LENGTH: {}".format(path_length))
                total_paths_length.append(path_length)
                tracks.append(full_spline)
                #for index in len(full_spline_copy):
                #    full_spline_copy[index][2] = full_spline_copy[index][2] + 0.5
                #tracks.append(full_spline_copy)
                # if sample_height < 84.03334979723762:
                #     tracks.append(full_spline)
                #     tracks.append(contour_subsample)
                print("accept row")
        #else:
        #    raise Exception("n")
        sample_height -= 0.03
        #print(sample_height)
    except:# Exception as e:
        #print(e)
        #raise e
        #sys.exit()
        break
# print(gps_spline)
#sys.exit()



# tracks = [[max_z_position]]




#print("begin search")




_STARTING_HEIGHT_INCREMENT = 0.2
_HEIGHT_INCREMENT_ADJUSTEMENT  = 0.0002

height_increment  = _STARTING_HEIGHT_INCREMENT

min_height = gps_mesh.get_min_height_point()[2]

total_paths_length = []

max_distance = 0
max_distance_pair = None
count = 0
# while False: #len(gps_mesh.working_point_cloud) > 20:
#     try:
#         count +=1
#         #print("1")
#         max_height_point = gps_mesh.get_max_height_point()
#         #print("2")
#         current_height = max_height_point[2] - height_increment/2
#         if count == 1:
#             current_height -= 2.0
#         #print("3")
#         contour_subsample = gps_mesh.slice_height_points(current_height, height_increment)
#         #print("4")
#         #print(gps_mesh.slice_points)
#         # sys.exit()
#
#         max_distance = -1
#         max_distance_pair = None
#         for idx1 in range(0, len(contour_subsample), 5):
#             #print("5")
#             for idx2 in range(0, len(contour_subsample), 5):
#                 point1 = contour_subsample[idx1]
#                 point2 = contour_subsample[idx2]
#                 distance = calculate_distance(point1, point2)
#                 #print(distance)
#                 if distance > max_distance:
#                     max_distance = distance
#                     max_distance_pair = (point1, point2)
#             # grid_working_copy[point1[0]][point1[1]] = math.nan # remove points?
#         if not max_distance_pair:
#             height_increment += _HEIGHT_INCREMENT_ADJUSTEMENT
#             print("No max distance pair. increasing height increment to {}".format(height_increment))
#             continue
#
#         if len(tracks)  == 0:
#             last_track = []
#         else:
#             last_track = tracks[-1]
#         print("find_costs")
#         if not gps_mesh.find_path_costs(max_distance_pair[0], max_distance_pair[1], last_track, row_spacing):
#             height_increment += _HEIGHT_INCREMENT_ADJUSTEMENT
#             print("Path not returned. increasing height increment to {}".format(height_increment))
#             continue
#
#         #sys.exit()
#         # print("CAME FROM")
#         # #print(gps_mesh.came_from)
#         # debug_points = []
#         # for key, value in gps_mesh.came_from.items():
#         #     print("{} : {}".format(key,value))
#         #     debug_points.append((key[0], key[1], 110))
#         #
#         # print("CAME FROM")
#     #    sys.exit()
#
#         try:
#             gps_mesh.reconstruct_path()
#
#
#             # print(gps_mesh.path)
#             path_points = []
#
#             for point in gps_mesh.path:
#                 path_points.append(gps_tools.GpsPoint3D(point[0], point[1], gps_mesh.get_height(point)))
#
#
#             gps_spline = GpsSpline(path_points, 1, 150)
#
#             loop_okay = True
#             for point1 in last_track:
#                 for point2 in gps_spline.points:
#                     if gps_mesh.get_distance(point1, point2) < row_spacing * 0.9:
#                         loop_okay = False
#                         break
#                 if not loop_okay:
#                     break
#             if not loop_okay:
#                 height_increment += _HEIGHT_INCREMENT_ADJUSTEMENT
#                 print("Path too close to last one. increasing height increment to {}".format(height_increment))
#                 continue
#
#
#             height_average = 0
#             path_length = 0
#             previous_point = None
#             for point in gps_spline.points:
#                 height_average += point.height_m
#                 if previous_point:
#                     path_length += gps_mesh.get_distance(point, previous_point)
#                 previous_point = point
#             print("PATH LENGTH: {}".format(path_length))
#             total_paths_length.append(path_length)
#             height_average = height_average / len(gps_spline.points)
#             gps_mesh.remove_points_near_path(gps_spline.points, row_spacing, height_average + 0.2)
#             path_points.pop(0)
#             path_points.pop(-1)
#             gps_spline = GpsSpline(path_points, 1, 150)
#             #print(gps_spline)
#             tracks.append(gps_spline.points)
#
#
#             #tracks.append(path_points)
#         except Exception as e:
#             print(e)
#             traceback.print_exc()
#             print(height_increment)
#             height_increment += _HEIGHT_INCREMENT_ADJUSTEMENT
#             continue
#
#         print("Row completed")
#         height_increment  = _STARTING_HEIGHT_INCREMENT
#
#         # tracks = [contour_subsample]
#         # if count > 5:
#         #     break
#         #sys.exit()
#         # break
#
#     except: # Exception as e:
#     #    print(e)
#         #traceback.print_exc()
#         break
#     # gps_mesh.remove_points_ab_path(gps_spline.points, row_spacing)

print(total_paths_length)
total = 0
for path_length in total_paths_length:
    print(path_length)
    total += path_length
print("TOTAL ROWS LENGTH: {}".format(total))

mesh_array = []
colors = [[1,0,0],[0,1,0],[0,0,1]]
count = 0
lat_lon_tracks = []
for track in tracks:
    if count < len(colors):
        row_color = colors[count]
    else:
        row_color = [random.random(), random.random(), random.random()]
    count += 1
    track_lat_lon = []
    for point in track:
        mesh_box = open3d.geometry.TriangleMesh.create_box(width=0.8, height=0.8, depth=0.8)
        mesh_box.compute_vertex_normals()
        mesh_box.paint_uniform_color(row_color)
        translation = [point[0], point[1], point[2]]
        print("{} {}".format(point[0] + min_x + first_x, point[1] + min_y + first_y))
        latlon_point = utm.to_latlon(point[0] + min_x + first_x, point[1] + min_y + first_y, ut_zone[0], ut_zone[1])
        print(latlon_point)
        track_lat_lon.append(latlon_point)
        mesh_box.translate(translation)
        mesh_array.append(mesh_box)
    lat_lon_tracks.append(track_lat_lon)
pickle.dump(lat_lon_tracks, open('lat_lon_tracks.pickle', 'wb'))


# for point in gps_spline.points:
#     mesh_box = open3d.geometry.TriangleMesh.create_box(width=0.3, height=0.3, depth=0.3)
#     mesh_box.compute_vertex_normals()
#     mesh_box.paint_uniform_color([.6,.5,0])
#     translation = [point[0], point[1], gps_mesh.get_height(point)+.5]
#     mesh_box.translate(translation)
#     mesh_array.append(mesh_box)
    # break


# print('RENDERING POINTS')
# points = gps_mesh.source_points
# print(len(points))
# mesh_array = []
# idx = 0
# while idx < len(points):
#     mesh_box = open3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
#     mesh_box.compute_vertex_normals()
#     mesh_box.paint_uniform_color([0.9, 0.1, 0.1])
#     translation = [points[idx][0], points[idx][1], points[idx][2]]
#     mesh_box.translate(translation)
#     mesh_array.append(mesh_box)
#     # if len(points) > 1000:
#     #     idx = int(idx + len(points)/1000.0)
#     # else:
#     idx += 1
#     print(".", end="")





# max_distance_pair[0] = (point1, point2)

# try:
#
#     mesh_box = open3d.geometry.TriangleMesh.create_box(width=0.8, height=0.8, depth=0.8)
#     mesh_box.compute_vertex_normals()
#     mesh_box.paint_uniform_color([0.0, 0.1, 0.9])
#     translation = [max_height_point[0], max_height_point[1], max_height_point[2]]
#     mesh_box.translate(translation)
#     mesh_array.append(mesh_box)
#
#     mesh_box = open3d.geometry.TriangleMesh.create_box(width=0.8, height=0.8, depth=0.8)
#     mesh_box.compute_vertex_normals()
#     mesh_box.paint_uniform_color([0.0, 0.9, 0.1])
#     translation = [max_distance_pair[0][0], max_distance_pair[0][1], max_distance_pair[0][2]]
#     mesh_box.translate(translation)
#     mesh_array.append(mesh_box)
#
#     mesh_box = open3d.geometry.TriangleMesh.create_box(width=0.8, height=0.8, depth=0.8)
#     mesh_box.compute_vertex_normals()
#     mesh_box.paint_uniform_color([0.0, 0.9, 0.1])
#     translation = [max_distance_pair[1][0], max_distance_pair[1][1], max_distance_pair[1][2]]
#     mesh_box.translate(translation)
#     mesh_array.append(mesh_box)
# except:
#     pass


# print(point_cloud)

pcd = open3d.geometry.PointCloud()
# np_points = np.random.rand(100, 3)

# print(np.array(point_cloud))



# From numpy to Open3D
pcd.points = open3d.utility.Vector3dVector(gps_mesh.pcd)
# pcd.points = open3d.utility.Vector3dVector(gps_mesh.slice_points)

mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(
    size=10, origin=[0, 0, min_height])

mesh_array.append(pcd)
mesh_array.append(mesh_frame)

open3d.visualization.draw_geometries(mesh_array)



# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt
# import numpy as np
#
# # x = np.append(0, (radii*np.cos(angles)).flatten())
# # y = np.append(0, (radii*np.sin(angles)).flatten())
# #
# # # Compute z to make the pringle surface.
# # z = np.sin(-x*y)
# # print(z)
#
# fig = plt.figure()
# ax = fig.gca(projection='3d')
#
# ax.plot_trisurf(grid_x.flatten(), grid_y.flatten(), grid_z2.flatten(), linewidth=0.2, antialiased=True)
#
# plt.show()

# fig = plt.figure()
# #ax = fig.add_subplot(111, projection = '3d')
# ax = fig.add_subplot(111)
#
# ax.set_title('Acorn energy usage')
# if watt_seconds:
#     ax.set_ylabel('watt seconds per meter')
# else:
#     ax.set_ylabel('average watts over one meter per motor')
# ax.set_xlabel('height change (m)')
#
# ax.scatter(orig_x, orig_y)
# ax.scatter(orig_x, orig_y1, c = colors)
# ax.plot(orig_y1)
# ax.plot(orig_y2)
# ax.plot(orig_y3)
# ax.plot(orig_y4)
# plt.show()

# sys.exit()
#
#
# def f(x,y):
#     return grid_z2

# fig = plt.figure(figsize=(6,6))
# ax = fig.add_subplot(111, projection='3d')
#
# ax.plot_surface(grid_x, grid_y, grid_z2, cmap='viridis')
# # ax.set_aspect(1)
#
#
# # plt.imshow(grid_z2.T)
# plt.show()

# plt.imshow(grid_z2.T, extent=(0,max_x,0,max_y), origin='lower')
#
# plt.title('Cubic')
#
#
# plt.show()

# CS = plt.contour(grid_z2, grid_x, grid_y, 15, linewidths=0.5, colors='k')
# CS = plt.contourf(grid_z2, grid_x, grid_y, 15)
# plt.colorbar()  # draw colorbar
# # plot data points.
# # plt.scatter(x, y, marker='o', s=5, zorder=10)
# plt.xlim(0, max_x)
# plt.ylim(0, max_y)
# #plt.title('griddata test (%d points)' % npts)
# plt.show()





# Plot the control points grid and the evaluated surface
# surface.vis = VisMPL.VisSurface()
# surface.render(colormap=cm.cool)

# initialParameters = [1.0, 1.0, 1.0] # these are the same as scipy default values in this example
#
# # here a non-linear surface fit is made with scipy's curve_fit()
# fittedParameters, pcov = scipy.optimize.curve_fit(surface_test.func, [orig_x, orig_y], orig_z, p0 = initialParameters)
#
# #ScatterPlot(data)
# surface_test.SurfacePlot(surface_test.func, data, fittedParameters)
# #surface_test.ContourPlot(surface_test.func, data, fittedParameters)
#
# print('fitted prameters', fittedParameters)

# modelPredictions = func(data, *fittedParameters)
#
# absError = modelPredictions - zData
#
# SE = numpy.square(absError) # squared errors
# MSE = numpy.mean(SE) # mean squared errors
# RMSE = numpy.sqrt(MSE) # Root Mean Squared Error, RMSE
# Rsquared = 1.0 - (numpy.var(absError) / numpy.var(zData))
# print('RMSE:', RMSE)
# print('R-squared:', Rsquared)

    # this_stamp = segment.start_gps.time_stamp
    # stamp_localtime = time.localtime(this_stamp)
    # if stamp_localtime.tm_year == today.tm_year and stamp_localtime.tm_yday == today.tm_yday - day_index:
    #     #print(stamp_localtime)
    #     total_meters_traveled += segment.distance_sum
    #     total_daily_meters[day_index]+= segment.distance_sum
    # else:
    #     if day_index > 5:
    #         print(total_daily_meters)
    #         print(sum(total_daily_meters))
    #         sys.exit()
    #     day_index += 1
    #     total_daily_meters.append(0)
        #print(total_daily_meters)
        #print("Traveled {} meters today!".format(total_meters_traveled))
        #sys.exit()



    #total_meters_traveled += segment.distance_sum
    #print("Traveled {} meters today!".format(total_meters_traveled))
    # if now - this_stamp < 3600:
    #     print(this_stamp)
    # else:
    #     sys.exit()
    # continue






#
#
#
#
#     # print((segment.per_motor_watt_average))
#     #print("sequence, {}, watt_seconds_per_meter, {}, meters_per_second, {}".format(segment.sequence_num, segment.watt_seconds_per_meter, segment.meters_per_second))
#     # orig_x.append(segment.start_gps.lat)
#     # orig_y.append(segment.start_gps.lon)
#     # print(segment.start_gps.lat, segment.start_gps.lon)
#     if segment.height_change > -0.15 and segment.watt_seconds_per_meter < 1000 and segment.meters_per_second < 2:
#         this_stamp = segment.start_gps.time_stamp
#         if now - this_stamp > 1000:
#             print(idx)
#             continue
#
#         try:
#             print(segment.per_motor_watt_average)
#         except:
#             continue
#
#         for idx in range(4):
#             power_vals[idx].append(segment.per_motor_watt_average[idx])
#
#         if len(power_vals[0])<10:
#             continue
#         else:
#             orig_y1.append(sum(power_vals[0])/len(power_vals[0]))
#             orig_y2.append(sum(power_vals[1])/len(power_vals[1]))
#             orig_y3.append(sum(power_vals[2])/len(power_vals[2]))
#             orig_y4.append(sum(power_vals[3])/len(power_vals[3]))
#             power_vals = [[],[],[],[]]
#
#         # orig_y1.append(segment.per_motor_watt_average[0])
#         # orig_y2.append(segment.per_motor_watt_average[1])
#         # orig_y3.append(segment.per_motor_watt_average[2])
#         # orig_y4.append(segment.per_motor_watt_average[3])
#
#         #orig_x.append(segment.meters_per_second)
#         orig_x.append(segment.height_change)
#         orig_z.append(segment.meters_per_second)
#         colors.append(segment.meters_per_second)
#         if segment.meters_per_second < min_colorval:
#             min_colorval = segment.meters_per_second
#         if segment.meters_per_second > max_colorval:
#             max_colorval = segment.meters_per_second
#
#
# print("min_colorval {}, max_colorval {}".format(min_colorval, max_colorval))
# cNorm  = mp_colors.Normalize(vmin=min_colorval, vmax=max_colorval)
# scalarMap = cm.ScalarMappable(norm=cNorm, cmap=cm.jet)
#
# for idx in range(len(colors)):
#     colors[idx] = scalarMap.to_rgba(colors[idx])
#
# fig = plt.figure()
# #ax = fig.add_subplot(111, projection = '3d')
# ax = fig.add_subplot(111)
#
# ax.set_title('Acorn energy usage')
# if watt_seconds:
#     ax.set_ylabel('watt seconds per meter')
# else:
#     ax.set_ylabel('average watts over one meter per motor')
# # ax.set_xlabel('height change (m)')
#
# #ax.scatter(orig_x, orig_y, orig_z, c = colors)
# #ax.scatter(orig_x, orig_y1, c = colors)
# ax.plot(orig_y1)
# ax.plot(orig_y2)
# ax.plot(orig_y3)
# ax.plot(orig_y4)
# plt.show()
#


# #     newkey = str(key).replace('-key\'',':key')
# #     newkey = newkey.replace('b\'','')
# # #     print(newkey)
# # #     # #print(bytes(newkey, encoding='ascii'))
# # #     # # #newkey = "twistedfields:gpspath:{}-key".format(str(key))
# #     r.delete(key)
#  gps_coords = pickle.loads(r.get(key))
#  spline = spline_lib.GpsSpline(gps_coords, smooth_factor=1, num_points=500)
#  #lat_smooth, lon_smooth, orig_x, orig_y = smooth_track(gps_coords, smooth_factor=10, num_points=200)
#  #print(list(zip(lat_smooth, lon_smooth)))
#  #plt.plot(dat[:,0], dat[:,1],'ro')
#  lat_smooth = []
#  lon_smooth = []
#  for p in spline.points:
#      lat_smooth.append(p.lat)
#      lon_smooth.append(p.lon)

#
#
#  point_of_interest = {'lat':37.35409860533507, 'lon':-122.33325479993744}
#  point_of_interest = {'lat':37.35398195436689, 'lon':-122.33308312401907}
#  point_of_interest = {'lat': 37.3540842425, 'lon': -122.3333173125}
#  point_of_interest = {'lat': 37.35402, 'lon': -122.3334}
#
#      #37.3540842425, -122.3333173125
#  start = time.time()
#  for _ in range(1000):
#      closeu = spline.closestUOnSpline(point_of_interest)
#  time1 = time.time()-start
#  print("closeu {}, time {}".format(closeu, time1))
#  start = time.time()
#  for _ in range(1000):
#      closeu2 = spline.closestUOnSplinePoints(point_of_interest)
#  time2 = time.time()-start
#  print("closeu2 {}, time {}".format(closeu2, time2))
#  coord = spline.coordAtU(closeu)
#  coord2 = spline.coordAtU(closeu2)
#  mag = spline.slopeRadiansAtU(closeu)
#  import math
#  mag = math.degrees(mag)
#  #mag = mag[0]/mag[1] * 90
#  print("closeu {}, coord {}, mag {}".format(closeu, coord, mag))
#
#





# plt.plot(orig_x, orig_y, 'ro')
#    plt.plot(lat_smooth, lon_smooth, 'bo')
#    plt.plot(point_of_interest['lat'],point_of_interest['lon'], 'go', markersize=20)
#    plt.plot(coord.lat, coord.lon, 'mo', markersize=20)
#    plt.plot(coord2.lat, coord2.lon, 'yo', markersize=20)
# plt.title(str(key))
# plt.show()
 #   print(value)
 #   point_data = []
 #   lats = []
 #   lons = []
 #   utm_x = []
 #   utm_y = []
 # #  try:
 #
 #
 #   for line in value:
 #       lats.append(line['lat'])
 #       lons.append(line['lon'])
 #       point_data.append((line['lat'], line['lon']))
 #       utm_coord = utm.from_latlon(line['lat'], line['lon'])
 #       utm_x.append(utm_coord[0])
 #       utm_x.append(utm_coord[1])
 #   x, y = np.array(lats), np.array(lons)
 #   #simple_coords = rdp(point_data, epsilon=1e-4)
 #   #print("{} points reduced to {}!".format(coords.shape[0], simple_coords.shape[0]))
 #   #plt.plot(simple_coords[:, 0], simple_coords[:, 1], 'ro')
 #   #plt.show()
 #
 #   smooth_factor = 1
 #
 #
 #
 #   dat = np.array([(x,y) for x,y in zip(lats, lons)])
 #    #dat = np.array([(x,y) for x,y in zip(coords.lon[::18], coords.lat[::18])])
 #   tck, u = splprep(dat.T, u=None, s=smooth_factor * _SMOOTH_MULTIPLIER, per=0, t=10)
 #   u_new = np.linspace(u.min(), u.max(), 200)
 #   x_new, y_new = splev(u_new, tck, der=0)
 #   #print(x_new)








   # print(point_data)
   # plt.plot(x, y, 'ro', ms=5)
   # cs = CubicSpline(x, y)
   # xs = 2 * np.pi * np.linspace(0, 1, 100)
   # ax.plot(xs, cs(xs), label="S")
   # plt.show()
   # spl = UnivariateSpline(x, y)
   # xs = np.linspace(-3, 3, 1000)
   # plt.plot(xs, spl(xs), 'g', lw=3)
  # except:
    #   print('exception unpickling key {}'.format(key))
       #r.delete(key)



# while True:
#     value = r.get('foo')
#     print(value)
#     time.sleep(0.1)
