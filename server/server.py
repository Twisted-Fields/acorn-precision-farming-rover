# coding: utf-8

from flask import Flask, render_template, request, send_from_directory, jsonify
from flask_redis import FlaskRedis
import sys
from svgpathtools import svg2paths, paths2svg
import re
import pickle

sys.path.append('../vehicle')

from master_process import Robot


app = Flask(__name__, template_folder="templates")
app.config['REDIS_URL'] = "redis://:@localhost:6379/0"
redis_client = FlaskRedis(app)


volatile_path = [] # When the robot is streaming a path, save it in RAM here.
active_site = ""

def get_svg_path(filename):
    p = re.compile('\s+[d][=]["]([M][^"]+)["]')
    with open(filename) as f:
        for line in f:
            matches = p.match(line)
            if matches:
                return matches.group(1).replace(' ', ', ')

robot_icon_path = get_svg_path("robot.svg")
arrow_icon_path = get_svg_path("arrow.svg")


@app.route('/')
def map_canvas_flask():
    return render_template(
        'acorn_map.html'
    )

@app.route('/api/get_herd_data')
def send_herd_data():
    keys = get_robot_keys()
    robots = robots_to_json(keys)
    return jsonify(robots)


def get_robot_keys():
    robot_keys = []
    for key in redis_client.scan_iter():
        if ':robot:' in str(key):
            robot_keys.append(key)
    return robot_keys


def robots_to_json(keys):
    robots_list = []
    for key in keys:
        robot = pickle.loads(redis_client.get(key))
        robot_entry = { 'name': robot.name, 'lat': robot.location.lat, 'lon':
        robot.location.lon, 'heading': robot.location.azimuth_degrees,
        'speed': robot.speed, 'turn_intent': robot.turn_intent_degrees,
        'voltage': robot.voltage, 'state': robot.state}
        robots_list.append(robot_entry)
    return robots_list


@app.route('/api/save_path', methods = ['POST'])
@app.route('/api/save_path/<pathname>', methods = ['POST'])
def save_current_path(pathname=None):
    if request.method == 'POST':
        pathdata = request.json
        print(pathdata)
    if not pathdata:
        return "Missing something. No path saved."
    if not pathname:
        volatile_path = pathdata
        return "Updated volatile_path"
    key = get_path_key(pathname)
    redis_client.set(key, pickle.dumps(pathdata))
    return "Saved Path {}".format(key)


def get_path_key(pathname):
    print(active_site)
    return "{}:gpspath:{}:key".format(active_site, pathname)

@app.route('/api/get_path/')
@app.route('/api/get_path/<pathname>')
def show_path(pathname=None):
    if not pathname:
        return jsonify(volatile_path)
    else:
        print(get_path_key(pathname))
        path = pickle.loads(redis_client.get(get_path_key(pathname)))
        return jsonify(path)

@app.route('/api/get_path_names')
def send_path_names():
    names = []
    for key in redis_client.scan_iter():
        if ':gpspath:' in str(key):
            pathname = str(key).split(":")[2]
            names.append(pathname)
            global active_site
            active_site = str(str(key).split(":")[0]).replace('b\'','')
            print(active_site)
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


if __name__ == "__main__":
    app.run(debug=True, use_reloader=True, host="0.0.0.0")
