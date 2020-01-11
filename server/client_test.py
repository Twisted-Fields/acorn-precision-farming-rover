import zmq
import sys
import threading
import time
from random import randint, random
import pickle


class Robot:
    def __init__(self, name):
        self.name = name
        self.location = {'lat': 37.3540865905, 'lon': -122.333349927}
        self.voltage = 24.0

# parent_conn, child_conn = mp.Pipe()
# self.parent_conn = parent_conn
# self.rtk_proc = mp.Process(target=launch_rtk_procs, args=(child_conn,))


acorn = Robot("acorn1")

context = zmq.Context()
socket = context.socket(zmq.DEALER)
name = bytes(acorn.name, encoding='ascii')
socket.identity = name
socket.connect('tcp://localhost:5570')

print('Client %s started' % (id))

poll = zmq.Poller()
poll.register(socket, zmq.POLLIN)
reqs = 0
while True:
    reqs = reqs + 1
    print('Req #%d sent..' % (reqs))

    acorn.voltage = 24 + random()*3

    socket.send(pickle.dumps(acorn))

    time.sleep(0.1)
    for i in range(5):
        sockets = dict(poll.poll(1000))
        if socket in sockets:
            msg = socket.recv()
            print('Client %s received: %s' % (id, msg))
            break

socket.close()
context.term()
