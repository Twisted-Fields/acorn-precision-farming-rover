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

# Modified from example file 
# Paranoid Pirate queue by  Daniel Lundin <dln(at)eintr(dot)org>
#

from collections import OrderedDict
import time

import zmq

HEARTBEAT_LIVENESS = 3     # 3..5 is reasonable
HEARTBEAT_INTERVAL = 1.0   # Seconds

#  Paranoid Pirate Protocol constants
PPP_READY = b"\x01"      # Signals worker is ready
PPP_HEARTBEAT = b"\x02"  # Signals worker heartbeat


class Worker(object):
    def __init__(self, address):
        self.address = address
        self.expiry = time.time() + HEARTBEAT_INTERVAL * HEARTBEAT_LIVENESS

class WorkerQueue(object):
    def __init__(self):
        self.queue = OrderedDict()

    def ready(self, worker):
        self.queue.pop(worker.address, None)
        self.queue[worker.address] = worker

    def purge(self):
        """Look for & kill expired workers."""
        t = time.time()
        expired = []
        for address, worker in self.queue.items():
            if t > worker.expiry:  # Worker expired
                expired.append(address)
        for address in expired:
            print("W: Idle worker expired: %s" % address)
            self.queue.pop(address, None)

    def next(self):
        address, worker = self.queue.popitem(False)
        return address

context = zmq.Context(1)

frontend = context.socket(zmq.ROUTER) # ROUTER
backend = context.socket(zmq.ROUTER)  # ROUTER
frontend.bind("tcp://*:5570") # For clients
backend.bind("tcp://*:5569")  # For workers

poll_workers = zmq.Poller()
poll_workers.register(backend, zmq.POLLIN)

poll_both = zmq.Poller()
poll_both.register(frontend, zmq.POLLIN)
poll_both.register(backend, zmq.POLLIN)

workers = WorkerQueue()

heartbeat_at = time.time() + HEARTBEAT_INTERVAL

while True:
    if len(workers.queue) > 0:
        poller = poll_both
    else:
        poller = poll_workers
    socks = dict(poller.poll(HEARTBEAT_INTERVAL * 1000))

    # Handle worker activity on backend
    if socks.get(backend) == zmq.POLLIN:
        # Use worker address for LRU routing
        frames = backend.recv_multipart()
        if not frames:
            break

        address = frames[0]
        # print("Address: {}".format(address))
        workers.ready(Worker(address))

        # Validate control message, or return reply to client
        msg = frames[1:]
        if len(msg) == 1:
            if msg[0] not in (PPP_READY, PPP_HEARTBEAT):
                print("E: Invalid message from worker: %s" % msg)
        else:
            print(msg)
            tracker = frontend.send_multipart(msg)

        # Send heartbeats to idle workers if it's time
        if time.time() >= heartbeat_at:
            for worker in workers.queue:
                msg = [worker, PPP_HEARTBEAT]
                backend.send_multipart(msg)
            heartbeat_at = time.time() + HEARTBEAT_INTERVAL
    if socks.get(frontend) == zmq.POLLIN:
        frames = frontend.recv_multipart()
        if not frames:
            break

        frames.insert(0, workers.next())
        backend.send_multipart(frames)


    workers.purge()
