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

import numpy as np
import scipy.interpolate as si
import scipy.spatial.distance as ssd
import scipy.integrate
import math
import gps_tools
from csaps import csaps

"""
Do some work with splines. Modified from original copy.

Originally found on github with no license attached. Considering free to use.
https://gist.github.com/lcrs/ea0001d3542372b2eced
"""

_SPLINE_HALFWAY = 0.5

_SMOOTH_MULTIPLIER = 0.00000000001

# Return distance from 3d point p to a point on the spline at spline parameter u
# Must be static function so it can be optimized in closest_point_on_spline


def distToP(u, tck, p):
    s = si.splev(u, tck)
    result = ssd.euclidean(p, s)
    return result


class GpsSpline():

    def __init__(self, gps_coords, smooth_factor, num_points, degree=3):
        self.tck, self.u = self.smooth_track(gps_coords, smooth_factor, degree)
        self.points = self.get_array(num_points)
        self.num_points = num_points

    def calculate_3D_factors(self, gps_coords, smooth_factor, num_points, degree=3):
        self.tck_3D, self.u_3D = self.smooth_track3D(
            gps_coords, smooth_factor, degree)

    def calc_distance_2D(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def filter_data(self, data, radius=5.0):
        filtered = []
        sample_point = data[0]
        filtered.append(sample_point)
        while len(data) > 2:
            # Remove close points
            for index in range(len(data)-1, -1, -1):
                if self.calc_distance_2D(sample_point[:2], data[index][:2]) < radius:
                    # data.pop(index)
                    data = np.delete(data, index, axis=0)
            # Find a nearby point:
            adjust = 0.1
            next_point = None
            done = False
            while not done and len(data) > 2:
                for index in range(len(data)-1, -1, -1):
                    if self.calc_distance_2D(sample_point, data[index]) < (radius + adjust):
                        next_point = data[index]
                        filtered.append(next_point)
                        done = True
                        break
                adjust += 0.2
                # print(adjust)
            sample_point = next_point
        return filtered

    def get_2d_spline(self, x, y, z):
        # print("BOOP")

        # try:
        # fit = np.polyfit()
        # fit = np.polynomial.polynomial.Polynomial.fit(x,y,7)
        # #print(fit.linspace(100))
        # spline = np.stack(fit.linspace(50), axis=1)
        # return spline
        # except Exception as e:
        #     print(e)
        #     raise e

        # spline = si.SmoothBivariateSpline(x,y,z)
        # xs = np.linspace(np.array(x).min(), np.array(x).max(), 150)
        # ys = spline(xs)
        # return xs, ys

        # bbox = (np.array(x).min(), np.array(x).max())
        #
        # spline = si.UnivariateSpline(x,y,bbox=bbox,k=5)
        # xs = np.linspace(np.array(x).min(), np.array(x).max(), 150)
        # ys = spline(xs)
        # return xs, ys

        sp = csaps(x, y, smooth=0.05)
        xs = np.linspace(np.array(x).min(), np.array(x).max(), 150)
        ys = sp(xs)
        # return xs, ys
        spline = np.stack((xs, ys), axis=1)
        return spline
        # return si.CloughTocher2DInterpolator(points, values, tol=1e-06)
        # rbfi = si.Rbf(x, y, z, function='thin_plate', smooth = 1)
        # xi = yi = zi = np.linspace(0, 1, 100)
        # print(xi.shape)
        # print(zi.shape)
        # di = rbfi(xi, yi, zi) # interpolated values
        # return di

        # return x_new, y_new
        # np_points = np.empty((len(points), 2))
        # # print(points[:,:1][0])
        # # print(points[:,1:2][0])
        # #return si.splrep(points[:,:1][0], points[:,1:2][0])
        # # for idx in range(len(points)):
        # #     # print(points[0])
        # #     np_points[idx] = (points[idx][0], points[idx][1])
        # return si.SmoothBivariateSpline(points[:,:1][0], points[:,1:2][0])

    def smooth_track(self, gps_coords, smooth_factor, degree=3):
        """ Calculated a spline based on a gps track.
        Args:
            gps_coords: A list of GpsSample or GpsPoint objects
            smooth_factor: Any float, but recommend 1-10.
            num_points: The number of points
        Returns: A list of GpsPoints representing the smoothed track.
        """
        np_points = np.empty((len(gps_coords), 2))
        for idx in range(len(gps_coords)):
            line = gps_tools.check_point(gps_coords[idx])
            np_points[idx] = (line.lat, line.lon)
        return si.splprep(np_points.T, u=None, s=smooth_factor * _SMOOTH_MULTIPLIER, per=0, t=10, k=degree)

    def smooth_track3D(self, gps_coords, smooth_factor, degree=3):
        """ Calculated a spline based on a gps track.
        Args:
            gps_coords: A list of GpsSample or GpsPoint objects
            smooth_factor: Any float, but recommend 1-10.
            num_points: The number of points
        Returns: A list of GpsPoints representing the smoothed track.
        """
        np_points = np.empty((len(gps_coords), 3))
        for idx in range(len(gps_coords)):
            line = gps_tools.check_point(gps_coords[idx])
            np_points[idx] = (line.lat, line.lon, line.height_m)
        return si.splprep(np_points.T, u=None, s=smooth_factor * _SMOOTH_MULTIPLIER, per=0, t=10, k=degree)

    def get_array(self, num_points):
        u_new = np.linspace(self.u.min(), self.u.max(), num_points)
        lat_smooth, lon_smooth = si.splev(u_new, self.tck, der=0)
        smoothed = []
        for lat, lon in zip(lat_smooth, lon_smooth):
            smoothed.append(gps_tools.GpsPoint(lat, lon))
        return smoothed

    def get_array3D(self, num_points=None):
        if not num_points:
            num_points = self.num_points
        u_new = np.linspace(self.u_3D.min(), self.u_3D.max(), num_points)
        lat_smooth, lon_smooth, h_smooth = si.splev(u_new, self.tck_3D, der=0)
        smoothed = []
        for lat, lon, height in zip(lat_smooth, lon_smooth, h_smooth):
            smoothed.append(gps_tools.GpsPoint3D(lat, lon, height))
        return smoothed

    # Return the angle in radians of the spline at u
    def slopeRadiansAtU(self, u):
        x, y = si.splev(u, self.tck, 1)
        return math.atan2(y, x)
        # return ssd.euclidean(0, si.splev(u, self.tck, 1))

    # Return the distance from u to v along the spline
    def distAlong(self, u, v):
        return scipy.integrate.quad(self.slopeMag, u, v)[0]

    # Return the distance from u along the spline to the halfway point
    def distAlongToHalf(self, u):
        return abs(self.distAlong(0.0, u) - (self.distAlong(0.0, 1.0)/2))

    def coordAtU(self, u):
        coord = si.splev(u, self.tck)
        coord = gps_tools.GpsPoint(float(coord[0]), float(coord[1]))
        return coord

    def closestUOnSpline(self, point):
        return self.closestUOnSplinePoints(point)
        # #print("POINT {}".format(point))
        # point = gps_tools.check_point(point)
        # point = (point.lat, point.lon)
        # #https://stackoverflow.com/questions/52077459/python-pass-extra-arguments-to-callable
        # dist_partial = functools.partial(distToP, tck=self.tck, p=point)
        # # THIS IS NOT WORKING. BUG IN MINIMIZE function
        # # https://github.com/scipy/scipy/issues/4240
        # retval = so.minimize_scalar(dist_partial, method='bounded', bounds=[0.0,1.00]).x
        # return retval

    def closestUOnSplinePoints(self, point):
        point = gps_tools.check_point(point)
        min_distance = math.inf
        min_dist_index = -1
        for idx in range(len(self.points)):
            p = self.points[idx]
            # Using naive x-y distance calculation as it was found to be ~100x
            # faster than gps distance calc.
            dist = math.sqrt(math.pow(p.lat - point.lat, 2) + math.pow(p.lon - point.lon, 2))
            if dist < min_distance:
                min_dist_index = idx
                min_distance = dist
        return min_dist_index/len(self.points)

    def closestUOnSplinePointsNearU(self, point, previous_u, search_range=0.05):
        point = gps_tools.check_point(point)
        min_distance = math.inf
        min_dist_index = -1

        if previous_u > 0:
            min_u = previous_u - search_range
            max_u = previous_u + search_range
            if min_u < 0:
                min_u = 0
            if max_u > 1.0:
                max_u = 1.0
        else:
            min_u = 0.0
            max_u = 1.0

        start = int(len(self.points) * min_u)
        end = int(len(self.points) * max_u)
        for idx in range(start, end):
            p = self.points[idx]
            # Using naive x-y distance calculation as it was found to be ~100x
            # faster than gps distance calc.
            dist = math.sqrt(math.pow(p.lat - point.lat, 2) + math.pow(p.lon - point.lon, 2))
            if dist < min_distance:
                min_dist_index = idx
                min_distance = dist
        return min_dist_index/len(self.points)

    def closest_point_on_spline(self, point):
        return self.coordAtU(self.closestUOnSpline(point))

# # Find the closest point on the spline to our 3d point p
# # We do this by finding a value for the spline parameter u which
# # gives the minimum distance in 3d to p
# closestu = so.fmin(distToP, 0.5)
# closest = si.splev(closestu, tck)
#
# # Find out where on the spline the halfway point is
# # Spline parameter u = 0.5 isn't necessarily halfway along the curve
# halfwayu = so.fmin(distAlongToHalf, 0.5)
# halfway = si.splev(halfwayu, tck)
# s = "distance along spline to halfway point is %f" % distAlong(halfwayu, closestu)
#
# # Build a list of 3d points along the spline
# spline = si.splev(np.linspace(0,1,100), tck)
#
# # Plot things!
# ax = mp.Axes3D(pp.figure(figsize=(8,8)))
# ax.plot((p[0],), (p[1],), (p[2],), 'o', label='input point')
# ax.plot(closest[0], closest[1], closest[2], 'o', label='closest point on spline')
# ax.plot(halfway[0], halfway[1], halfway[2], 'o', label='halfway along spline')
# ax.plot(data[0], data[1], data[2], label='raw data points')
# ax.plot(spline[0], spline[1], spline[2], label='spline fit to data')
# pp.legend(loc='lower right')
# pp.title(s)
# pp.show()
