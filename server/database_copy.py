# coding: utf-8

from flask import Flask, render_template, request, send_from_directory, jsonify
import flask_shelve as shelve
# from flask_googlemaps import GoogleMaps
# from flask_googlemaps import Map, icons
from svgpathtools import svg2paths, paths2svg
import re
import sys
import pickle

import redis


r = redis.Redis(
    host='localhost',
    port=6379)


app = Flask(__name__, template_folder="templates")
app.config['SHELVE_FILENAME'] = 'shelve.db'
shelve.init_app(app)


@app.route('/')
def map_canvas_flask():
    return render_template(
        'acorn_map.html'
    )

@app.route('/api/get_herd_data')
def send_herd_data():
    db = shelve.get_shelve('c')
    for key in db.keys():
        print(db[key])
        r.set(key, pickle.dumps(db[key]))
    import time
    time.sleep(1000)


if __name__ == "__main__":
    app.run(debug=True, use_reloader=True, host="0.0.0.0")
