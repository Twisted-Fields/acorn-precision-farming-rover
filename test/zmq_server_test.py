
import time
import sys
import math
import zmq
import pickle




_PUBLISHER_PORT = 5556


context = zmq.Context()

publisher_socket = context.socket(zmq.PUB)
publisher_socket.bind("tcp://*:{}".format(_PUBLISHER_PORT))

print(dir(publisher_socket))



class RobotSocket:
    port=None
    socket=None
    connected=False




worker_sockets = []
for i in range(10):
    port = _PUBLISHER_PORT + i + 1
    worker_sockets.append((port, context.socket(zmq.REP)))
    worker_sockets[i][1].bind("tcp://*:{}".format(port))


while True:

    ports = []
    for port, sock in worker_sockets:





    calc = None
    while command_socket.poll(timeout=0):
        calc = pickle.loads(command_socket.recv_pyobj())
        #tick_time = time.time()

    if calc:
        print(calc)
