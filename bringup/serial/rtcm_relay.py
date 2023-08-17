import serial
import subprocess as sp
import time
import os
import shlex
import socket
import sys
import struct
import fcntl
from pyrtcm import RTCMReader

HOST = "192.168.1.50"  # The server's hostname or IP address
PORT = 130  # The port used by the server

BAUD = 921600
# BAUD = 1000000  # NOTE: Actual baud rate is 1M but must set as 921600

SOCKET_TIMEOUT_SECONDS = 1
if sys.maxsize > 2**32:
    TCP_TIMEOUT = struct.pack(str("ll"), int(SOCKET_TIMEOUT_SECONDS), int(0))
else:
    TCP_TIMEOUT = struct.pack(str("ii"), int(SOCKET_TIMEOUT_SECONDS), int(0))

while True:
    try:
        # ser_rtcm_out0 = serial.Serial('/dev/gps_0', BAUD, timeout=0.5)
        # ser_rtcm_out1 = serial.Serial('/dev/gps_1', BAUD, timeout=0.5)
        ser_rtcm_out0 = serial.Serial('/dev/ttySC0', BAUD, timeout=0.5)
        ser_rtcm_out1 = serial.Serial('/dev/ttySC2', BAUD, timeout=0.5)
        print(ser_rtcm_out0.name)
        print(ser_rtcm_out1.name)
        ser_rtcm_out0.reset_input_buffer()
        ser_rtcm_out0.reset_output_buffer()
        ser_rtcm_out1.reset_input_buffer()
        ser_rtcm_out1.reset_output_buffer()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as conn:
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, TCP_TIMEOUT)
            conn.connect((HOST, PORT))
            fcntl.fcntl(conn, fcntl.F_SETFL, os.O_NONBLOCK)
            last_data = time.time()
            while True:
                try:
                    rtr = RTCMReader(conn)
                    for (raw_data, parsed_data) in rtr:
                        last_data = time.time()
                        print("--->--->--->--->--->--->--->--->--->--->--->--->")
                        print(parsed_data)
                        ser_rtcm_out0.write(raw_data)
                        ser_rtcm_out1.write(raw_data)
                        print(f"Sent {len(raw_data)} bytes")
                    # continue
                    # data = conn.recv(1024)
                    # if data:
                    #     last_data = time.time()
                    #     print("--->" + str(data))
                    #     ser_rtcm_out0.write(data)
                    #     ser_rtcm_out1.write(data)
                    #     print(f"Sent {len(data)} bytes")
                except BlockingIOError:
                    print("------")
                    time.sleep(0.5)
                    if time.time() - last_data > 20:
                        raise TimeoutError("RTCM stream timed out, resetting socket.")
    except Exception as e:
        print(e)
        time.sleep(10)
