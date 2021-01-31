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
