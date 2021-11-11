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
import math
import numpy as np

steering_track = 1.5
wheel_base_ = 1.830
wheel_radius_ = 0.4
M_PI_2 = math.pi/2.0


def normalize_values(angle, throttle, angle_limit_deg):
    if angle > angle_limit_deg:
        angle -= 180
        throttle *= -1
    if angle < -angle_limit_deg:
        angle += 180
        throttle *= -1
    return angle, throttle


def steering_to_numpy(calculated_values):
    values = []
    corners = ["front_left", "front_right", "rear_left", "rear_right"]
    for corner in corners:
        values.append(calculated_values[corner][0])
        values.append(calculated_values[corner][1])
    return np.array(values)


def calculate_steering(steer, throttle, strafe, angle_limit_deg):
    """
    Returns the steering and velocity of all 4 wheels given the required motion of the vehicle.

    Math given from the following page:
    https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    Specifically in the documents:
    https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
    and
    https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
    """
    L = wheel_base_
    W = steering_track
    R = math.sqrt(L * L + W * W)

    steer = steer * throttle

    throttle_factor = 2.5

    RCW = steer
    FWD = throttle
    STR = strafe

    A = STR - RCW * (L/R)
    B = STR + RCW * (L/R)
    C = FWD - RCW * (W/R)
    D = FWD + RCW * (W/R)

    ws1 = math.sqrt(B * B + C * C) * throttle_factor
    ws2 = math.sqrt(B * B + D * D) * throttle_factor
    ws3 = math.sqrt(A * A + D * D) * throttle_factor
    ws4 = math.sqrt(A * A + C * C) * throttle_factor

    wa1 = math.atan2(B, C)*180.0/math.pi
    wa2 = math.atan2(B, D)*180.0/math.pi
    wa3 = math.atan2(A, D)*180.0/math.pi
    wa4 = math.atan2(A, C)*180.0/math.pi

    wa1, ws1 = normalize_values(wa1, ws1, angle_limit_deg)
    wa2, ws2 = normalize_values(wa2, ws2, angle_limit_deg)
    wa3, ws3 = normalize_values(wa3, ws3, angle_limit_deg)
    wa4, ws4 = normalize_values(wa4, ws4, angle_limit_deg)

    front_left_steering = math.radians(wa2)
    front_right_steering = math.radians(wa1)
    rear_right_steering = math.radians(wa4)
    rear_left_steering = math.radians(wa3)
    vel_left_front = ws2
    vel_right_front = ws1
    vel_right_rear = ws4
    vel_left_rear = ws3

    return {"front_left": (front_left_steering, vel_left_front),
            "front_right": (front_right_steering, vel_right_front),
            "rear_left": (rear_left_steering, vel_left_rear),
            "rear_right": (rear_right_steering, vel_right_rear)}


def compare_steering_values(old_val, new_val, steering_limit=1.0, velocity_limit=0.5):
    corners = ["front_left", "front_right", "rear_left", "rear_right"]
    error_string = ""
    for corner in corners:
        steering_delta = old_val[corner][0] - new_val[corner][0]
        velocity_delta = old_val[corner][1] - new_val[corner][1]
        if abs(steering_delta) > steering_limit:
            error_string += "{} angle delta exceeded | ".format(corner)
        if abs(velocity_delta) > velocity_limit:
            error_string += "{} velocity delta exceeded | ".format(corner)
    if len(error_string) > 0:
        error_string = "Steering deltas exceeded! {}".format(error_string)
        return False, error_string
    return True, ""
