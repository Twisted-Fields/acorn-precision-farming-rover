import serial
import subprocess as sp
import time
import os
import shlex
import socket

HOST = "192.168.1.50"  # The server's hostname or IP address
PORT = 130  # The port used by the server

BAUD = 921600

ser_rtcm_out = serial.Serial('/dev/ttySC4', BAUD, timeout=0.5)
print(ser_rtcm_out.name)
# ser_gps_in = serial.Serial('/dev/ttySC5', BAUD, timeout=0.5)
# print(ser_gps_in.name)

ser_rtcm_out.reset_input_buffer()
ser_rtcm_out.reset_output_buffer()
# ser_gps_in.reset_input_buffer()
# ser_gps_in.reset_output_buffer()



with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as conn:
    conn.connect((HOST, PORT))
    while True:
        data = conn.recv(1024)
        if data:
            print("--->" + str(data))
            ser_rtcm_out.write(data)
