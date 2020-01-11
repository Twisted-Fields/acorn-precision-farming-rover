import socket
import subprocess
import time
import os
from collections import namedtuple
import pickle
import math
from datetime import datetime


GpsSample = namedtuple('GpsSample', 'lat lon height_m status num_sats azimuth')

filename = "gps_track_08-11-2019_11-25-47_PM.pkl"
gps_data = pickle.load( open( filename, "rb" ) )
for item in gps_data:
    print(item)
