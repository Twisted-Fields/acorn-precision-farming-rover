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
        import re
        import pickle
        import datetime
        import redis_utils
        from model import RobotCommand
        break
    except Exception as e:
        print(e)
        print("Server had some error. Restarting...")
        time.sleep(5)

app = Flask(__name__, template_folder="templates")
app.config['REDIS_URL'] = "redis://:@localhost:6379/0"
redis_client = FlaskRedis(app)

volatile_path = []  # When the robot is streaming a path, save it in RAM here.
active_site = "twistedfields"


def get_svg_path(filename):
    p = re.compile(r'\s+[d][=]["]([M][^"]+)["]')
    with open(filename) as f:
        for line in f:
            matches = p.match(line)
            if matches:
                return matches.group(1).replace(' ', ', ')


robot_icon_path = get_svg_path("robot.svg")
arrow_icon_path = get_svg_path("arrow.svg")


@app.route('/')
def map_test():
    return render_template('acorn.html')


def date_handler(obj):
    if isinstance(obj, (datetime.datetime, datetime.date)):
        return obj.isoformat() + "+00:00"
    else:
        return None


@app.route('/api/get_herd_data')
def send_herd_data():
    keys = redis_utils.get_robot_keys(redis_client)
    robots = robots_to_json(keys)
    # print(robots)
    # print(jsonify(robots))
    return jsonify(robots)


def robots_to_json(keys):
    robots_list = []
    for key in keys:
        robot = pickle.loads(redis_client.get(key))
        if robot is None:
            print("Database returned None for key {}".format(key))
        time_stamp = date_handler(robot.time_stamp)
        live_path_data = [point._asdict() for point in robot.live_path_data]
        gps_path_data = [point._asdict() for point in robot.gps_path_data]
        try:
            debug_points = [point._asdict() for point in robot.debug_points]
        except BaseException:
            debug_points = []
        # print(json.dumps(robot.gps_path_data[0]._asdict()))
        loaded_path_name = ""
        if robot.loaded_path_name:
            loaded_path_name = robot.loaded_path_name.split('gpspath:')[1].split(':key')[0]

        robot_entry = {
            'name': robot.name,
            'lat': robot.location.lat,
            'lon': robot.location.lon,
            'heading': robot.location.azimuth_degrees,
            'speed': robot.speed,
            'turn_intent_degrees': robot.turn_intent_degrees,
            'voltage': robot.voltage,
            'control_state': robot.control_state,
            'motor_state': robot.motor_state,
            'time_stamp': time_stamp,
            'loaded_path_name': loaded_path_name,
            'live_path_data': live_path_data,
            'gps_path_data': gps_path_data,
            'debug_points': debug_points,
            'autonomy_hold': robot.autonomy_hold,
            'activate_autonomy': robot.activate_autonomy,
            'access_point_name': robot.wifi_ap_name,
            'wifi_signal': robot.wifi_strength,
            'gps_distances': robot.gps_distances,
            'gps_angles': robot.gps_angles,
            'gps_distance_rates': robot.gps_path_lateral_error_rates,
            'gps_angle_rates': robot.gps_path_angular_error_rates,
            'strafeP': robot.strafeP,
            'steerP': robot.steerP,
            'strafeD': robot.strafeD,
            'steerD': robot.steerD,
            'autonomy_steer_cmd': robot.autonomy_steer_cmd,
            'autonomy_strafe_cmd': robot.autonomy_strafe_cmd,
            'simulated_data': robot.simulated_data
            # 'front_lat': debug_points[0].lat,
            # 'front_lon': debug_points[0].lat,
            # 'rear_lat': debug_points[1].lat,
            # 'rear_lon': debug_points[1].lat,
            # 'front_close_lat': debug_points[2].lat,
            # 'front_close_lon': debug_points[2].lat,
            # 'rear_close_lat': debug_points[3].lat,
            # 'rear_close_lon': debug_points[3].lat,
        }
        # print(robot_entry)
        robots_list.append(robot_entry)

    return robots_list


@app.route('/api/save_path/', methods=['POST'])
@app.route('/api/save_path/<pathname>', methods=['POST'])
def save_current_path(pathname=None):
    if request.method == 'POST':
        pathdata = request.json
        print(pathdata)
    if not pathdata:
        return "Missing something. No path saved."
    if not pathname:
        # volatile_path = pathdata
        return "Updated volatile_path"
    key = get_path_key(pathname)
    redis_client.set(key, pickle.dumps(pathdata))
    return "Saved Path {}".format(key)


@app.route('/api/save_polygon', methods=['POST'])
@app.route('/api/save_polygon/<polygon_name>', methods=['POST'])
def save_polygon(polygon_name="default"):
    if request.method == 'POST':
        polygon_data = request.json
        print(polygon_data)
    if not polygon_data:
        return "Missing something. No polygon saved."
    key = get_polygon_key(polygon_name)
    redis_client.set(key, pickle.dumps(polygon_data))
    return "Saved Polygon {}".format(key)


@app.route('/api/delete_path/<pathname>')
def delete_path(pathname=None):
    if not pathname:
        return "Missing something. No path deleted."
    redis_client.delete(get_path_key(pathname))
    return "Deleted path {}".format(pathname)


@app.route('/api/set_vehicle_path/<pathname>/<vehicle_name>')
def set_vehicle_path(pathname=None, vehicle_name=None):
    if not vehicle_name or not pathname:
        return "Missing something. No vehicle path set."
    vehicle_command_key = "{}:robot:{}:command:key".format(
        active_site, vehicle_name)
    if redis_client.exists(vehicle_command_key):
        robot_command = pickle.loads(redis_client.get(vehicle_command_key))
    else:
        robot_command = RobotCommand()
    robot_command.load_path = get_path_key(pathname)
    print(vehicle_command_key)
    redis_client.set(vehicle_command_key, pickle.dumps(robot_command))
    return "Set vehicle {} path to {}".format(vehicle_command_key, robot_command.load_path)


@app.route('/api/set_gps_recording/<vehicle_name>/<record_gps_path>')
def set_gps_recording(vehicle_name=None, record_gps_path=None):
    if not vehicle_name or not record_gps_path:
        return "Missing something. No gps command set."
    if len(active_site) == 0:
        return "Active site not set. Please load a path."
    vehicle_command_key = "{}:robot:{}:command:key".format(
        active_site, vehicle_name)
    if redis_client.exists(vehicle_command_key):
        robot_command = pickle.loads(redis_client.get(vehicle_command_key))
    else:
        robot_command = RobotCommand()
    robot_command.record_gps_path = record_gps_path
    print(robot_command.record_gps_path)
    redis_client.set(vehicle_command_key, pickle.dumps(robot_command))
    return "Set vehicle {} record gps command to {}".format(vehicle_command_key, record_gps_path)


@app.route('/api/set_vehicle_autonomy/<vehicle_name>/<speed>/<enable>')
def set_vehicle_autonomy(vehicle_name=None, speed=None, enable=None):
    enable = enable == "true"
    return redis_utils.set_vehicle_autonomy(redis_client=redis_client,
                                            vehicle_name=vehicle_name,
                                            speed=speed,
                                            enable=enable,
                                            active_site=active_site)


@app.route('/api/modify_autonomy_hold/<vehicle_name>/<value>')
def clear_autonomy_hold(vehicle_name=None, value=None):
    value = value == "true"
    return redis_utils.clear_autonomy_hold(redis_client=redis_client,
                                           vehicle_name=vehicle_name,
                                           value=value,
                                           active_site=active_site)


def get_path_key(pathname):
    print(active_site)
    return "{}:gpspath:{}:key".format(active_site, pathname)


def get_polygon_key(polygon_name):
    print(active_site)
    return "{}:gpspolygon:{}:key".format(active_site, polygon_name)


@app.route('/api/get_path/')
@app.route('/api/get_path/<pathname>')
def show_path(pathname=None):
    if not pathname:
        return jsonify(volatile_path)
    else:
        print(get_path_key(pathname))
        path = pickle.loads(redis_client.get(get_path_key(pathname)))
        print(type(path))
        print(type(path[0]))
        if 'PathSection' in str(type(path[0])):
            new_path = []
            for row in path:
                for point in row.points:
                    new_path.append(point)
            path = new_path

        return jsonify(path)


@app.route('/api/get_path_names')
def send_path_names():
    names = []
    for key in redis_client.scan_iter():
        if ':gpspath:' in str(key):
            pathname = str(key).split(":")[2]
            names.append(pathname)
            global active_site
            active_site = str(str(key).split(":")[0]).replace('b\'', '')
            print(active_site)
    names.sort()
    return jsonify(names)


@app.route('/api/getRobotIcon')
def send_robot_paths():
    return jsonify(robot_icon_path)


@app.route('/api/getArrowIcon')
def send_arrow_paths():
    return jsonify(arrow_icon_path)


@app.route('/api/robot.svg')
def send_robot_icon():
    return send_from_directory('../', "robot.svg")


# @app.route('/api/get_dense_path/<start>/<end>')


@app.route('/api/get_dense_path')
def get_dense_path():
    robot_keys = redis_utils.get_robot_keys(redis_client)
    if len(robot_keys) > 0:
        # TODO(tlalexander): support multiple robots in database
        path = redis_utils.get_dense_path(redis_client=redis_client, robot_key=robot_keys[0])
        path = [point._asdict() for point in path]
        print(type(path))
        print(type(path[0]))
        return jsonify(path)
    return "No keys found"


if __name__ == "__main__":
    while True:
        try:
            app.run(debug=True,
                    use_reloader=True,
                    host="0.0.0.0",
                    port=int("80"))
        except BaseException:
            print("Server had some error. Restarting...")
            time.sleep(5)
