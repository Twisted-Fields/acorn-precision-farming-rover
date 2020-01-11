import socket
import subprocess
import time
import os
import geopy
from geopy.distance import distance
from collections import namedtuple
import pickle
from geographiclib.geodesic import Geodesic
import math
from datetime import datetime

TCP_IP = "127.0.0.1"
TCP_PORT = 10001
TCP_PORT2 = 10002
BUFFER_SIZE = 1024

GpsSample = namedtuple('GpsSample', 'lat lon height_m status num_sats azimuth')

geod = Geodesic.WGS84  # define the WGS84 ellipsoid

def digest_data(data):
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
        return GpsSample(lat, lon, height_m, status, num_sats, None)
    else:
        return None


# Launch rtklib process.
cmd = 'rtkrcv -s -d /dev/null -o /home/ubuntu/odrive/twisted.conf'
subprocess.Popen(cmd, shell=True)
cmd2 = 'rtkrcv -s -d /dev/null -o /home/ubuntu/odrive/twisted2.conf'
subprocess.Popen(cmd2, shell=True)


tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

connected = False
attempts = 0
while not connected:
  try:
    # Connect to socket.
    tcp_sock.connect((TCP_IP, TCP_PORT))
    tcp_sock2.connect((TCP_IP, TCP_PORT2))
    connected = True
    print('Connected.')
  except:
    time.sleep(0.500)
    print('Attempt failed.')
    attempts += 1
    if attempts > 5:
        raise RuntimeError("Could not connect to rtkrcv TCP socket.")
vehicle_positions = []
while True:
  #print("Await Data")
  try:
      data = tcp_sock.recv(BUFFER_SIZE)
      data2 = tcp_sock2.recv(BUFFER_SIZE)
      if data and data2:
        data = digest_data(data)
        data2 = digest_data(data2)

        g = geod.Inverse(data.lat, data.lon, data2.lat, data2.lon)
        #print(g.azi1)
        azimuth = g['azi1']

        d = distance((data.lat, data.lon), (data2.lat, data2.lon)).m

        lat = (data.lat + data2.lat) / 2.0
        lon = (data.lon + data2.lon) / 2.0
        height_m = (data.height_m + data2.height_m) / 2.0
        vehicle_pos = GpsSample(lat, lon, height_m, (data.status, data2.status), (data.num_sats, data2.num_sats), azimuth)
        vehicle_positions.append(vehicle_pos)
    #    try:
        print("Lat: {:.10f}, Lon: {:.10f}, Azimuth: {:.2f}, Distance: {:.4f}, Fix1: {}, Fix2: {}".format(vehicle_pos.lat, vehicle_pos.lon, azimuth, d, data.status, data2.status))
        #print(vehicle_pos)
        time.sleep(0.1)
  except KeyboardInterrupt:
    filename = datetime.now().strftime("tracks/gps_track_%d-%m-%Y_%I-%M-%S_%p.pkl")
    pickle.dump(vehicle_positions, open(filename, "wb" ))
    break

tcp_sock.close()
tcp_sock2.close()
