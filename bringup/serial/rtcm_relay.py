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
from pyubx2 import UBXMessage

HOST = "192.168.1.50"  # The server's hostname or IP address
PORT = 130  # The port used by the server

BAUD = 921600


RECOVER_BAUD = 36600

SOCKET_TIMEOUT_SECONDS = 1
if sys.maxsize > 2**32:
    TCP_TIMEOUT = struct.pack(str("ll"), int(SOCKET_TIMEOUT_SECONDS), int(0))
else:
    TCP_TIMEOUT = struct.pack(str("ii"), int(SOCKET_TIMEOUT_SECONDS), int(0))


PORT1 = '/dev/ttySC2'
PORT2 = '/dev/ttySC3'

gps_configured = False


def configure_gps():
    ports = [PORT1, PORT2]
    for port in ports:
        with serial.Serial(port, RECOVER_BAUD, timeout=0.5, write_timeout=2) as serial_conn:
            layers = 1
            transaction = 0
            cfgData = [('CFG_MSGOUT_UBX_NAV_PVT_UART2', 1), ('CFG_MSGOUT_UBX_NAV_PVT_USB', 1), ('CFG_MSGOUT_UBX_NAV2_PVT_USB', 10), ('CFG_RATE_MEAS', 200), ('CFG_RATE_NAV', 5), ('CFG_SIGNAL_BDS_ENA', 0), ('CFG_SIGNAL_GAL_ENA', 0), ('CFG_SIGNAL_QZSS_ENA', 0), ('CFG_SIGNAL_SBAS_ENA', 0), ('CFG_UART1_BAUDRATE', 921600), ('CFG_UART1OUTPROT_NMEA', 0), ('CFG_UART1OUTPROT_RTCM3X', 0), ('CFG_UART1OUTPROT_UBX', 0), ('CFG_UART2_BAUDRATE', 921600), ('CFG_UART2OUTPROT_RTCM3X', 0), ('CFG_UART2OUTPROT_UBX', 1), ('CFG_USBOUTPROT_NMEA', 0)]
            msg = UBXMessage.config_set(layers, transaction, cfgData)
            serial_conn.write(msg.serialize())


while True:
    try:
        if not gps_configured:
            configure_gps()
            gps_configured = True
        with serial.Serial(PORT1, BAUD, timeout=0.5, write_timeout=2) as ser_rtcm_out0:
            with serial.Serial(PORT2, BAUD, timeout=0.5, write_timeout=2) as ser_rtcm_out1:
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
                            if time.time() - last_data > 5:
                                print("--------------------------------------------------")
                                last_data = time.time()
                                sys.exit()
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
    except serial.SerialTimeoutException as e:
        print("Serial write timeout exception. Resetting connection.")
        time.sleep(2)
    except Exception as e:
        print(e)
        # raise e
        time.sleep(0.5)
