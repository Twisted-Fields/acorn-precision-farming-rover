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

from scipy.interpolate import splprep, splev
sys.path.append('../vehicle')
from remote_control_process import NavigationParameters, PathControlValues, PathSection, Direction
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



_SMOOTH_MULTIPLIER = 0.00000000001

# r = redis.Redis(
#     host='acornserver.local',
#     port=6379)

r = redis.Redis(
    host='127.0.0.1',
    port=6379)


SPEED = 0.2

print("%%%%%%%%%%%%%%%%%%%%%%%%")

# lat_lon_tracks = pickle.load(open('lat_lon_tracks_before_reset_jun1.pickle', 'rb'))
circle_tracks = pickle.load(open('circle_tracks2.pickle', 'rb'))

#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)
forward_navigation_parameters = NavigationParameters(
    travel_speed=SPEED, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.EITHER, repeat_path=False)


#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.FORWARD, vehicle_travel_direction=Direction.FORWARD, loop_path=True)
#self.default_navigation_parameters = NavigationParameters(travel_speed=0.0, path_following_direction=Direction.BACKWARD, vehicle_travel_direction=Direction.BACKWARD, loop_path=True)


_MAXIMUM_ALLOWED_DISTANCE_METERS = 1
_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES = 40

# path_control_vals = PathControlValues(angular_p=0.9, lateral_p=-0.25, angular_d=0.3, lateral_d=-0.2)
# turn_control_vals = PathControlValues(angular_p=0.9, lateral_p=-0.25, angular_d=0.3, lateral_d=-0.2)

path_control_vals = PathControlValues(
    angular_p=5, lateral_p=0.6, angular_d=0.4, lateral_d=0.1)
nav_path = PathSection(points=circle_tracks,
                       control_values=path_control_vals,
                       navigation_parameters=forward_navigation_parameters,
                       max_dist=_MAXIMUM_ALLOWED_DISTANCE_METERS,
                       max_angle=_MAXIMUM_ALLOWED_ANGLE_ERROR_DEGREES,
                       end_dist=2.0,
                       end_angle=60,
                       closed_loop=True)
nav_path = [nav_path]

starting_direction = -1

# print(lat_lon_tracks[0][0])
# sys.exit()


# print(rows_in_polygon)


r.set('twistedfields:gpspath:aa_new_circle5:key',
      pickle.dumps(nav_path))
