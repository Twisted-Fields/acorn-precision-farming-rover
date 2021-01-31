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
# coding: utf-8

# Loop the imports so that temporary syntax errors in imported files does not kill server.
while True:
    try:
        import time
        from flask import Flask, render_template, request, send_from_directory, jsonify
        from flask_redis import FlaskRedis
        import sys
        from svgpathtools import svg2paths, paths2svg
        import re
        import pickle
        import json
        import datetime
        sys.path.append('../vehicle')
        from master_process import Robot, RobotCommand
        break
    except Exception as e:
        print(e)
        print("Server had some error. Restarting...")
        time.sleep(5)


app = Flask(__name__, template_folder="templates")
app.config['REDIS_URL'] = "redis://:@localhost:6379/0"
redis_client = FlaskRedis(app)


volatile_path = [] # When the robot is streaming a path, save it in RAM here.
active_site = "twistedfields"

date_handler = lambda obj: (
    obj.isoformat() + "-07:00"
    if isinstance(obj, (datetime.datetime, datetime.date))
    else None
)

def send_herd_data():
    keys = get_robot_keys()
    robots = robots_to_json(keys)
    return jsonify(robots)

def get_robot_keys():
    robot_keys = []
    for key in redis_client.scan_iter():
        if ':robot:' in str(key):
            if ':command:' not in str(key):
                robot_keys.append(key)
    return robot_keys


def load_first_robot(redis_client):
    keys = get_robot_keys()
    for key in keys:
        robot = pickle.loads(redis_client.get(key))
        time_stamp = json.dumps(robot.time_stamp, default=date_handler)
        return robot

def robots_to_json(keys):
    robots_list = []
    for key in keys:
        robot = pickle.loads(redis_client.get(key))
        time_stamp = json.dumps(robot.time_stamp, default=date_handler)
        live_path_data = robot.live_path_data
        gps_path_data = robot.gps_path_data
        debug_points = robot.debug_points
        #print(debug_points)
        robot_entry = { 'name': robot.name, 'lat': robot.location.lat, 'lon':
        robot.location.lon, 'heading': robot.location.azimuth_degrees,
        'speed': robot.speed, 'turn_intent_degrees': robot.turn_intent_degrees,
        'voltage': robot.voltage, 'control_state': robot.control_state,
        'motor_state': robot.motor_state, 'time_stamp': time_stamp,
        'loaded_path_name': "" if not robot.loaded_path_name else robot.loaded_path_name.split('gpspath:')[1].split(':key')[0],
        'live_path_data' : live_path_data,
        'gps_path_data' : gps_path_data,
        'debug_points' : debug_points,
        'autonomy_hold' : robot.autonomy_hold,
        'activate_autonomy' : robot.activate_autonomy,
        'access_point_name' : robot.wifi_ap_name,
        'wifi_signal' : robot.wifi_strength,
        'gps_distances' : robot.gps_distances,
        'gps_angles' : robot.gps_angles,
        'gps_distance_rates' : robot.gps_path_lateral_error_rates,
        'gps_angle_rates' : robot.gps_path_angular_error_rates,
        'strafe' : robot.strafe,
        'rotation' : robot.rotation,
        'strafeD' : robot.strafeD,
        'steerD' : robot.steerD
        # 'front_lat': debug_points[0].lat,
        # 'front_lon': debug_points[0].lat,
        # 'rear_lat': debug_points[1].lat,
        # 'rear_lon': debug_points[1].lat,
        # 'front_close_lat': debug_points[2].lat,
        # 'front_close_lon': debug_points[2].lat,
        # 'rear_close_lat': debug_points[3].lat,
        # 'rear_close_lon': debug_points[3].lat,
        }
        robots_list.append(robot_entry)
    return robots_list


if __name__ == "__main__":
    oldlist = None
    while True:
        robot = load_first_robot(redis_client)
        newlist = robot.gps_path_lateral_error_rates
        if oldlist:
            offset = 0
            counter = 0
            match_found = False
            while match_found == False:
            #for value in old_list:
                if oldlist[offset+counter] == newlist[counter]:
                    counter+=1
                    if counter > 50:
                        print("Found Match, offset: {}".format(offset))
                        match_found = True
                else:
                    offset += 1
                    counter = 0
        oldlist = newlist

        #print(robot.gps_path_lateral_error_rates)
        time.sleep(1)
