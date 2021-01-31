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
