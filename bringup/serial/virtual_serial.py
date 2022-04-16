import serial
import subprocess as sp
import time
import os
import shlex
import socket

HOST = "192.168.1.50"  # The server's hostname or IP address
HOST = "192.168.1.86"  # The server's hostname or IP address
HOST = "192.168.1.71"
PORT = 65436  # The port used by the server

COMMAND = 'socat -d -d pty,raw,echo=0 pty,raw,echo=0'

LINKNAME = "/home/taylor/.wine/dosdevices/com1"
# LINKNAME = "/dev/ttyACM7"

proc = sp.Popen(shlex.split(COMMAND),stdout=sp.PIPE,stderr=sp.STDOUT)

# Dummy port gets symlinked so u-center can connect to it
dummy_port = "/dev" + str(proc.stdout.readline().split(b"dev")[1][:-1], 'utf-8')
# Comms port is used by this python program to read and write from the loopback
comms_port = "/dev" + str(proc.stdout.readline().split(b"dev")[1][:-1], 'utf-8')

# if os.path.exists(LINKNAME):
#     print("removing link")
os.remove(LINKNAME)
os.symlink(dummy_port, LINKNAME)

ser = serial.Serial(comms_port, 9600, timeout=1)


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as conn:
    conn.connect((HOST, PORT))
    ticktime = time.time()
    while True:
        bytelist = []
        while ser.in_waiting:
            bytelist.append(ser.read(ser.in_waiting))
        for item in bytelist:
            print("<---" + str(item))
            if len(item)>0:
                conn.sendall(item)
        data = conn.recv(1024)
        if data:
            print("--->" + str(data))
            ser.write(data)
        # time.sleep(0.2)
        if time.time() - 4 > ticktime:
            ticktime = time.time()
            print("%%%%")








# control, dependent = pty.openpty()
# s_name = os.ttyname(dependent)
#
# print(s_name)
# ser = serial.Serial(s_name)
# #
# # # To Write to the device
# # ser.write('Your text')
#
# idx = 0
# while True:
#     # To read from the device
#     time.sleep(1)
#     ser.write(bytes('Your text {}'.format(idx),'utf-8'))
#     idx += 1
#     #print(os.read(control,1000))
