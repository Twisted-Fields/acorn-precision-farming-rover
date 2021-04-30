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

import numpy as np, matplotlib.pyplot as pp, mpl_toolkits.mplot3d as mp
import scipy.interpolate as si, scipy.optimize as so, scipy.spatial.distance as ssd, scipy.integrate
import functools
import math
import gps_tools

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

    def __init__(self, gps_coords, smooth_factor, num_points):
        self.tck, self.u = self.smooth_track(gps_coords, smooth_factor)
        self.points = self.get_array(num_points)

    def smooth_track(self, gps_coords, smooth_factor):
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

        return si.splprep(np_points.T, u=None, s=smooth_factor * _SMOOTH_MULTIPLIER, per=0, t=10)


    def get_array(self, num_points):
        u_new = np.linspace(self.u.min(), self.u.max(), num_points)
        lat_smooth, lon_smooth = si.splev(u_new, self.tck, der=0)
        smoothed = []
        for lat, lon in zip(lat_smooth, lon_smooth):
            smoothed.append(gps_tools.GpsPoint(lat, lon))
        return smoothed

    # Return the magnitude of the gradient of the spline at u
    def slopeRadiansAtU(self, u):
        x, y = si.splev(u, self.tck, 1)
        return math.atan2(y,x)
        #return ssd.euclidean(0, si.splev(u, self.tck, 1))

    # Return the distance from u to v along the spline
    def distAlong(self, u, v):
        return scipy.integrate.quad(slopeMag, u, v)[0]

    # Return the distance from u along the spline to the halfway point
    def distAlongToHalf(self, u):
        return abs(distAlong(0.0, u) - (distAlong(0.0, 1.0)/2))

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

    def closestUOnSplinePointsNearU(self, point, previous_u, search_range=0.1):
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
