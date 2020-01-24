import socket
import subprocess
import time
import os
from collections import namedtuple
import pickle
import math
from datetime import datetime
import yaml
import requests
from requests_futures.sessions import FuturesSession
import json
import select

import multiprocessing as mp

import gps_tools

TCP_IP = "127.0.0.1"
TCP_PORT = 10001
TCP_PORT2 = 10002
BUFFER_SIZE = 1024
VEHICLE_AZIMUTH_OFFSET_DEG = 90 + 45.0
_REQUEST_INTERVAL_S = 2

_YAML_NAME="/home/ubuntu/vehicle/server_config.yaml"

GpsSample = namedtuple('GpsSample', 'lat lon height_m status num_sats azimuth time_stamp')


def launch_rtk_procs(child_conn):
    # Launch rtklib process.
    cmd = 'rtkrcv -s -d /dev/null -o /home/ubuntu/vehicle/twisted.conf'
    proc1 = subprocess.Popen(cmd, shell=True)
    cmd2 = 'rtkrcv -s -d /dev/null -o /home/ubuntu/vehicle/twisted2.conf'
    proc2 = subprocess.Popen(cmd2, shell=True)

    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    connected = False
    attempts = 0
    while True:
      try:
        # Connect to socket.
        tcp_sock.connect((TCP_IP, TCP_PORT))
        tcp_sock2.connect((TCP_IP, TCP_PORT2))
        print('Connected.')
        break
      except:
        time.sleep(0.500)
        print('Attempt failed.')
        attempts += 1
        if attempts > 5:
            raise RuntimeError("Could not connect to rtkrcv TCP socket.")
    while True:
        data = tcp_sock.recv(BUFFER_SIZE)
        data2 = tcp_sock2.recv(BUFFER_SIZE)
        if data and data2:
            child_conn.send((data, data2))

        # TODO: Close sockets when needed.




class GpsSampler:
    """Loads GPS positions and sends them to the server."""
    def __init__(self):
        self.load_goat_config()
        self.requests_session = FuturesSession(max_workers=2)
        self.futures = []
        self.nav_path = []
        self.next_nav_point = None
        self.turn_intent_degrees = 0
        self.vehicle_positions = []
        parent_conn, child_conn = mp.Pipe()
        self.parent_conn = parent_conn
        self.rtk_proc = mp.Process(target=launch_rtk_procs, args=(child_conn,))
        self.rtk_proc.start()

    def load_goat_config(self):
        with open(_YAML_NAME, 'r') as stream:
            try:
                config = yaml.safe_load(stream)
                self.vehicle_num = str(config["vehiclenum"])
                self.server = str(config["server"])
            except yaml.YAMLError as exc:
                print("Error! Problem Loading Yaml File. Does it exist?/n"
                      "Actual error thrown was: {}".format(exc))

    def send_position_to_server(self):
        """Sends data to server from this vehicle."""
        if not self.latest_sample:
            return
        #first make the request URL string
        urlString = "http://{}:5000/api/set_vehicle_status/{}".format(
            self.server,
            self.vehicle_num
        )
        data = {
            "time_stamp":str(self.latest_sample.time_stamp),
            "lat" : str(self.latest_sample.lat),
            "lon" : str(self.latest_sample.lon),
            "azimuth": str(self.latest_sample.azimuth),
            "speed" : str(0),
            "turn_intent" : str(self.turn_intent_degrees)
            }
        return self.requests_session.get(urlString, data=json.dumps(data))

    def send_path_to_server(self):
        """Sends nav path to server from this vehicle."""
        if not self.latest_sample:
            return
        points = []
        for point in self.nav_path:
            points.append({'lat': point.lat, 'lon': point.lon})
        urlString = "http://{}:5000/api/save_path/".format(
        self.server,
        self.vehicle_num,
        )
        #print(urlString)
        return self.requests_session.get(urlString, data=json.dumps(points))

    def get_path_future(self):
        """Gets the robot path from the server."""
        urlString = ("http://{}:5000/api/get_path/".format(
            self.server))
        future = self.requests_session.get(urlString)
        future.add_done_callback(self.process_path)
        return future

    def process_path(self, future):
        try:
            r = future.result()
            point_list = r.json()
            print("=====")
            self.nav_path = []
            for p in point_list:
              print(p)
              s = GpsSample(p['lat'], p['lon'], 0, None, 0, None, 0)
              self.nav_path.append(s)
        except Exception:
            print("")
            print("##################  Connection Error!  ##################")
            print("")


    def digest_data(self, data):
        #print(data)
        data = str(data.splitlines()[0])
        data = data.split(' ')
        data = [a for a in data if a]

        if len(data) > 13:
            lat = float(data[2])
            lon = float(data[3])
            height_m = float(data[4])
            status = "fix" if data[5] is "1" else "no fix"
            num_sats = data[6]
            #age = data[13]
            return GpsSample(lat, lon, height_m, status, num_sats, None, time.time())
        else:
            return None


    def get_gps_path(self):
        self.futures.append(self.get_path_future())

    def run_once(self):
        #print("enter run once")
        if not self.parent_conn.poll():
            return

        data, data2 = self.parent_conn.recv()

        data = self.digest_data(data)
        data2 = self.digest_data(data2)

        azimuth = gps_tools.get_heading(data, data2) + VEHICLE_AZIMUTH_OFFSET_DEG

        d = gps_tools.get_distance(data,data2)

        lat = (data.lat + data2.lat) / 2.0
        lon = (data.lon + data2.lon) / 2.0
        height_m = (data.height_m + data2.height_m) / 2.0
        self.latest_sample = GpsSample(lat, lon, height_m, (data.status, data2.status), (data.num_sats, data2.num_sats), azimuth, data.time_stamp)
        self.vehicle_positions.append(self.latest_sample)


        if(len(self.nav_path)>0):
            goal, index = gps_tools.get_closest_point(self.latest_sample, self.nav_path)
            goal_dist = gps_tools.get_distance(self.latest_sample,goal)
            if goal_dist < 3.0:

                self.nav_path.remove(goal)
            else:
                self.next_nav_point = goal
                #self.turn_intent = 0
                self.turn_intent_degrees = gps_tools.get_heading(self.latest_sample, self.next_nav_point)
                self.turn_intent_degrees -= self.latest_sample.azimuth
                if self.turn_intent_degrees > 180:
                    self.turn_intent_degrees -= 360
                if self.turn_intent_degrees < -180:
                    self.turn_intent_degrees += 360
        else:
            self.turn_intent_degrees = -180

        #self.send_nav_path_to_server()
        self.send_position_to_server()
        print("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fix1: {}, Fix2: {}, Turn Intent: {:.2f}".format(self.latest_sample.lat, self.latest_sample.lon, azimuth, d, data.status, data2.status, self.turn_intent_degrees))


    def save_gps_track(self):
        filename = datetime.now().strftime("tracks/gps_track_%d-%m-%Y_%I-%M-%S_%p.pkl")
        pickle.dump(self.vehicle_positions, open(filename, "wb" ))


    def run_loop(self):
        request_time = time.time()



if __name__=="__main__":
    sampler = GpsSampler()
    sampler.get_gps_path()
    while True:
        try:
            sampler.run_once()
            #print(sampler.turn_intent)
            time.sleep(0.1)
            #print("loop")
        except KeyboardInterrupt:
            sampler.save_gps_track()
            # sampler.tcp_sock.close()
            # sampler.tcp_sock2.close()
            break
