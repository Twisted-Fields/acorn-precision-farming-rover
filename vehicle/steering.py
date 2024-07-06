"""
*********************************************************************
                     This file is part of:
                       The Acorn Project
             https://wwww.twistedfields.com/research
*********************************************************************
Copyright (c) 2019-2021 Taylor Alexander, Twisted Fields LLC
Copyright (c) 2021 The Acorn Project contributors (cf. AUTHORS.md).

Licensed under the Apache License, Version 2.0 (the "License")
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
import gps_tools
from enum import Enum
import model
from model import Direction
import time
from utils import AppendFIFO


# Woody
wheel_base_width = 1.83
wheel_base_length = 2.2

# Acorn
# wheel_base_width = 1.5
# wheel_base_length = 1.830
wheel_radius_ = 0.4
M_PI_2 = math.pi/2.0
_ABSOLUTE_STEERING_LIMIT = math.pi * 2.0
_PROJECTED_POINT_DISTANCE_METERS = 1.0
_ERROR_RATE_AVERAGING_COUNT = 3


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


def calculate_steering(steer, throttle, strafe, angle_limit_deg=360):
    """
    Returns the steering and velocity of all 4 wheels given the required motion of the vehicle.

    Math given from the following page:
    https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    Specifically in the documents:
    https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
    and
    https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf

    Copies of these documents are in this git repo in the /docs/steering folder.
    """
    L = wheel_base_length
    W = wheel_base_width
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

    return {
            "front_right": (front_right_steering, vel_right_front),
            "front_left": (front_left_steering, vel_left_front),
            "rear_right": (rear_right_steering, vel_right_rear),
            "rear_left": (rear_left_steering, vel_left_rear)
            }


def recalculate_steering_values(calc, last_calc):
    for key in calc.keys():
        vals = [calc[key][0], calc[key][1]]
        while True:
            angle_diff = vals[0] - last_calc[key][0]
            if angle_diff > math.pi/2.0:
                vals[0] -= math.pi
                vals[1] *= -1
            elif angle_diff < -math.pi/2.0:
                vals[0] += math.pi
                vals[1] *= -1
            else:
                # TODO: handle this without crashing.
                if abs(vals[0]) > _ABSOLUTE_STEERING_LIMIT:
                    raise ValueError("Steering value exceeds limit of {}".format(_ABSOLUTE_STEERING_LIMIT))
                calc[key] = vals
                break
    return calc


def compare_steering_values(old_val, new_val, steering_limit=0.5, velocity_limit=0.2):
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



def steering_calc(controller):
    """
    Determine the steering commands for the robot.
    TODO: Break this out to a separate module.
    """

    if not controller.nav_path.points:
        return (gps_tools.GpsPoint(0, 0),  # projected_path_tangent_point,
                None,  # closest_path_point,
                math.inf,  # absolute_path_distance,
                None,  # drive_reverse,
                None,  # calculated_rotation_degrees,
                None)  # calculated_strafe)
    closest_path_point, path_point_heading = controller.calc_closest_path_point_and_heading()
    absolute_path_distance = gps_tools.get_distance(controller.gps.last_sample(), closest_path_point)
    calculated_rotation_degrees = path_point_heading - controller.gps.last_sample().azimuth_degrees
    # Truncate values to between 0 and 360
    calculated_rotation_degrees %= 360
    # Set value to +/- 180
    if calculated_rotation_degrees > 180:
        calculated_rotation_degrees -= 360
    controller.logger.debug("calculated_rotation_degrees: {}, distance to path: {}".format(
        calculated_rotation_degrees, abs(absolute_path_distance)))
    controller.logger.debug("robot heading {}, path heading {}".format(
        controller.gps.last_sample().azimuth_degrees, path_point_heading))

    projected_path_tangent_point = gps_tools.project_point(closest_path_point, path_point_heading, _PROJECTED_POINT_DISTANCE_METERS)
    gps_lateral_distance_error = gps_tools.get_approx_distance_point_from_line(
        controller.gps.last_sample(), closest_path_point, projected_path_tangent_point)
    calculated_strafe = -1 * gps_lateral_distance_error

    # Truncate values to between 0 and 360
    calculated_rotation_degrees %= 360

    # Set value to +/- 180
    if calculated_rotation_degrees > 180:
        calculated_rotation_degrees -= 360

    controller.logger.debug("calculated_rotation_degrees: {}".format(calculated_rotation_degrees))

    _MAXIMUM_ROTATION_ERROR_DEGREES = 140

    vehicle_dir = controller.nav_path.navigation_parameters.vehicle_travel_direction
    path_dir = controller.nav_path.navigation_parameters.path_following_direction
    drive_solution_okay = (vehicle_dir == Direction.EITHER or
                           path_dir == Direction.EITHER or
                           (vehicle_dir == path_dir and
                            abs(calculated_rotation_degrees) <= _MAXIMUM_ROTATION_ERROR_DEGREES))

    if vehicle_dir == Direction.BACKWARD:
        controller.driving_direction = -1
    else:
        controller.driving_direction = 1
    if vehicle_dir == Direction.EITHER:
        if abs(calculated_rotation_degrees) > 90:
            controller.driving_direction = -1
            calculated_rotation_degrees -= math.copysign(180, calculated_rotation_degrees)
        controller.driving_direction *= controller.nav_direction
        calculated_strafe *= controller.nav_direction * controller.driving_direction
    elif vehicle_dir == Direction.FORWARD:
        if (abs(calculated_rotation_degrees) > _MAXIMUM_ROTATION_ERROR_DEGREES and
                (path_dir in (Direction.EITHER, Direction.BACKWARD))):
            calculated_rotation_degrees -= math.copysign(180, calculated_rotation_degrees)
            calculated_strafe *= -1
            controller.nav_direction = -1
    elif vehicle_dir == Direction.BACKWARD:
        if abs(calculated_rotation_degrees) > _MAXIMUM_ROTATION_ERROR_DEGREES:
            if (path_dir in (Direction.EITHER, Direction.FORWARD)):
                calculated_rotation_degrees -= math.copysign(180, calculated_rotation_degrees)
                controller.nav_direction = 1
        elif path_dir == Direction.BACKWARD:
            calculated_strafe *= -1

    drive_reverse = 1.0
    if (len(controller.nav_path.points) == 2 and path_dir == Direction.EITHER):
        if abs(calculated_rotation_degrees) > 20:
            original = calculated_strafe
            if abs(calculated_rotation_degrees) > 40:
                calculated_strafe = 0
                controller.gps_lateral_error_rate_averaging_list = []
            else:
                calculated_strafe *= (40 - abs(calculated_rotation_degrees)) / 20.0
            controller.logger.debug("Reduced strafe from {}, to: {}".format(original, calculated_strafe))
        else:
            vehicle_position = (controller.gps.last_sample().lat, controller.gps.last_sample().lon)
            use_second_pt = True
            if controller.nav_direction == -1:
                use_second_pt = False
            drive_reverse = gps_tools.determine_point_move_sign(
                controller.nav_path.points, vehicle_position, use_second_pt)
            # Figure out if we're close to aligned with the
            # target point and reduce forward or reverse
            # velocity if so.
            closest_pt_on_line = gps_tools.find_closest_pt_on_line(
                controller.nav_path.points[0], controller.nav_path.points[1],
                vehicle_position)
            dist_along_line = gps_tools.get_distance(controller.nav_path.points[0], closest_pt_on_line)
            controller.logger.debug("dist_along_line {}".format(dist_along_line))
            if gps_lateral_distance_error > 1.0:
                # Reduce forward/reverse direction command
                # if we are far from the line but also
                # aligned to the target point.
                if dist_along_line < 1.0:
                    drive_reverse = 0
                elif dist_along_line < 2.0:
                    drive_reverse *= 0.1
                elif dist_along_line < 4.0:
                    drive_reverse *= 0.25
        controller.logger.debug("rotation {}, strafe: {} direction {}, drive_reverse {}".
                          format(calculated_rotation_degrees, calculated_strafe,
                                 controller.driving_direction, drive_reverse))
    else:
        # controller.logger.info(f"gps_lateral_distance_error: {gps_lateral_distance_error}")
        if abs(gps_lateral_distance_error) > controller.nav_path.strafe_priority_distance_m:
            # Don't drive forward until we are close enough to path.
            drive_reverse *= 0.1
            calculated_strafe *= 2.0 # Increase strafe value outside of path bounds.




    if not drive_solution_okay:
        controller.autonomy_hold = True
        controller.activate_autonomy = False
        controller.control_state = model.CONTROL_NO_STEERING_SOLUTION
        controller.logger.error("Could not find drive solution. Disabling autonomy.")
        controller.logger.error("calculated_rotation_degrees: {}, vehicle_travel_direction {}, path_following_direction {}"
                          .format(calculated_rotation_degrees, controller.nav_path.
                                  navigation_parameters.vehicle_travel_direction,
                                  controller.nav_path.navigation_parameters.
                                  path_following_direction))

    controller.logger.debug(
        f"calculated_rotation_degrees: {calculated_rotation_degrees}, "
        f"vehicle_travel_direction {controller.nav_path.navigation_parameters.vehicle_travel_direction}, "
        f"path_following_direction {controller.nav_path.navigation_parameters.path_following_direction}, "
        f"controller.nav_direction {controller.nav_direction}, "
        f"controller.driving_direction {controller.driving_direction}")

    gps_path_angle_error = calculated_rotation_degrees
    # Accumulate a list of error values for angular and
    # lateral error. This allows averaging of errors
    # and also determination of their rate of change.
    time_delta = time.time() - controller.gps_error_update_time
    controller.gps_error_update_time = time.time()
    if not controller.reloaded_path:
        AppendFIFO(controller.gps_lateral_error_rate_averaging_list,
                   (gps_lateral_distance_error - controller.gps_path_lateral_error) / time_delta,
                   _ERROR_RATE_AVERAGING_COUNT)
        AppendFIFO(controller.gps_angle_error_rate_averaging_list,
                   (gps_path_angle_error - controller.gps_path_angular_error) / time_delta,
                   _ERROR_RATE_AVERAGING_COUNT)
        controller.logger.debug(
            "gps_lateral_distance_error: {}, controller.gps_path_lateral_error: {}"
            .format(gps_lateral_distance_error, controller.gps_path_lateral_error))
    controller.gps_path_angular_error = gps_path_angle_error
    controller.gps_path_lateral_error = gps_lateral_distance_error

    # Evaluate end conditions.

    pt = controller.nav_path.points[0] if controller.nav_direction == -1 else controller.nav_path.points[-1]
    end_distance = gps_tools.get_distance(controller.gps.last_sample(), pt)

    if controller.nav_path.closed_loop:
        pt = controller.nav_path.points[-1] if controller.nav_direction == -1 else controller.nav_path.points[0]
        start_distance = gps_tools.get_distance(controller.gps.last_sample(), pt)
        if end_distance < controller.nav_path.end_distance_m or start_distance < controller.nav_path.end_distance_m:
            # this resets the path point tracking variable to ensure
            # proper track looping
            controller.last_closest_path_u = -1


    if abs(calculated_rotation_degrees) < controller.nav_path.end_angle_degrees and \
                        end_distance < controller.nav_path.end_distance_m and \
                        not controller.nav_path.closed_loop:
        controller.logger.info("MET END CONDITIONS {} {}".format(calculated_rotation_degrees, absolute_path_distance))
        if controller.nav_path.navigation_parameters.repeat_path:
            controller.load_path(controller.nav_path, simulation_teleport=False, generate_spline=False)
            controller.load_path_time = time.time()
        else:
            controller.nav_path_index += 1
            if controller.nav_path_index < len(controller.nav_path_list):
                controller.load_path(controller.nav_path_list[controller.nav_path_index],
                               simulation_teleport=False, generate_spline=True)
            else:
                controller.nav_path_index = 0
                controller.load_path(controller.nav_path_list[controller.nav_path_index],
                               simulation_teleport=True, generate_spline=True)

        controller.gps_lateral_error_rate_averaging_list = []
        controller.gps_angle_error_rate_averaging_list = []
        controller.reloaded_path = True
        return (projected_path_tangent_point, closest_path_point, absolute_path_distance,
                drive_reverse, calculated_rotation_degrees, calculated_strafe)
    else:
        controller.logger.debug(f"END CONDITIONS NOT MET: Path end distance: {end_distance} < controller.nav_path.end_distance_m {controller.nav_path.end_distance_m}, Calculated rotation: {abs(calculated_rotation_degrees)} > nav_path.end_angle_degrees {controller.nav_path.end_angle_degrees}")

    controller.reloaded_path = False
    controller.logger.debug("controller.gps_path_lateral_error_rate {}, {} / {}".format(
        controller.gps_path_lateral_error_rate(),
        sum(controller.gps_lateral_error_rate_averaging_list),
        len(controller.gps_lateral_error_rate_averaging_list)))

    controller.next_point_heading = calculated_rotation_degrees
    return (projected_path_tangent_point, closest_path_point, absolute_path_distance,
            drive_reverse, calculated_rotation_degrees, calculated_strafe)









#
# def forwardKinematics(calc):
# {
# 	L = wheel_base_length
# 	W = wheel_base_width
#
# 	FR_B = sin(frRotationDelta) * frDriveDelta
# 	FR_C = cos(frRotationDelta) * frDriveDelta
#
# 	FL_B = sin(flRotationDelta) * flDriveDelta
# 	FL_D = cos(flRotationDelta) * flDriveDelta
#
# 	BR_A = sin(brRotationDelta) * brDriveDelta
# 	BR_C = cos(brRotationDelta) * brDriveDelta
#
# 	BL_A = sin(blRotationDelta) * blDriveDelta
# 	BL_D = cos(blRotationDelta) * blDriveDelta
#
# 	A = (BR_A + BL_A) / 2.0
# 	B = (FR_B + FL_B) / 2.0
# 	C = (FR_C + BR_C) / 2.0
# 	D = (FL_D + BL_D) / 2.0
#
# 	omega1 = (B - A) / L
# 	omega2 = (C - D) / W
# 	omega = (omega1 + omega2) / 2.0
#
# 	STR, FWD, STR1, STR2, FWD1, FWD2
# 	STR1 = omega * (L / 2.0) + A
# 	STR2 = -omega * (L / 2.0) + B
# 	FWD1 = omega * (W / 2.0) + C
# 	FWD2 = -omega * (W / 2.0) + D
#
# 	STR = (STR1 + STR2) / 2.0
# 	FWD = (FWD1 + FWD2) / 2.0
#
# 	return FWD, STR
# }
