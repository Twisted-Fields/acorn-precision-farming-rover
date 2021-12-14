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

import pickle
from model import RobotCommand


def get_energy_segment_key(robot_key):
    return bytes(str(robot_key)[2:-1].replace(":key", ":energy_segment:key"), encoding='ascii')


def get_robot_command_key(robot_key):
    return bytes(str(robot_key)[2:-1].replace(":key", ":command:key"), encoding='ascii')


def is_robot_key(key):
    if ':robot:' in key:
        if ':command:' not in key and ':energy_segment' not in key:
            return True
    return False


def get_robot_keys(redis_client):
    robot_keys = []
    for key in redis_client.scan_iter():
        if is_robot_key(str(key)):
            robot_keys.append(key)
    # print(robot_keys)
    return robot_keys


def get_command_object_from_robot_key(redis_client, robot_key):
    command_key = get_robot_command_key(robot_key)
    return get_valid_command_object(redis_client, command_key)


def save_command_object_from_robot_key(redis_client, robot_key, command_object):
    command_key = get_robot_command_key(robot_key)
    redis_client.set(command_key, pickle.dumps(command_object))


def get_valid_command_object(redis_client, command_key):
    if redis_client.exists(command_key):
        robot_command = pickle.loads(redis_client.get(command_key))
        # Create a new command object if the definition has changed.
        if len(dir(RobotCommand())) != len(dir(robot_command)):
            robot_command = RobotCommand()
    else:
        robot_command = RobotCommand()
    return robot_command


def clear_autonomy_hold(redis_client=None, vehicle_name=None, value=None, active_site=None):
    if not all((vehicle_name, value, redis_client)):
        return "Missing info for clear_autonomy_hold. No changes made."
    if len(active_site) == 0:
        return "Active site not set. Please load a path."
    # TODO: We have two different versions of getting a command key string now.
    vehicle_command_key = "{}:robot:{}:command:key".format(
        active_site, vehicle_name)
    robot_command = get_valid_command_object(redis_client, vehicle_command_key)
    robot_command.clear_autonomy_hold = value
    redis_client.set(vehicle_command_key, pickle.dumps(robot_command))
    print("Clear hold vehicle {}".format(vehicle_command_key))
    return "Clear hold vehicle {}".format(vehicle_command_key)


def set_vehicle_autonomy(redis_client=None, vehicle_name=None, speed=None, enable=None, active_site=None):
    if not all((vehicle_name, speed, redis_client)):
        return "Missing something. No vehicle autonomy set.  {}  {}  {}  {}".format(
            vehicle_name, speed, enable, redis_client)
    if len(active_site) == 0:
        return "Active site not set. Please load a path."
    # TODO: We have two different versions of getting a command key string now.
    vehicle_command_key = "{}:robot:{}:command:key".format(
        active_site, vehicle_name)
    robot_command = get_valid_command_object(redis_client, vehicle_command_key)
    robot_command.activate_autonomy = enable
    robot_command.autonomy_velocity = float(speed)
    redis_client.set(vehicle_command_key, pickle.dumps(robot_command))
    return "Set vehicle {} autonomy to {}".format(vehicle_command_key, (speed, enable))


def get_dense_path(redis_client=None, robot_key=None):
    key = get_energy_segment_key(robot_key)
    list_length = redis_client.llen(key)
    path = []
    for idx in range(list_length - 1, 0, -1):
        segment = pickle.loads(redis_client.lindex(key, idx))
        # print(segment.subsampled_points)
        try:
            segment.subsampled_points
            for point in segment.subsampled_points:
                path.append(point)
        except:
            break
            # path.append(segment.start_gps)
        #     break
        # for point in segment.subsampled_points:
        #     path.append(point)
    return path
