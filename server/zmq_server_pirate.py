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
# Paranoid Pirate Worker by  Daniel Lundin <dln(at)eintr(dot)org>

from master_process import Robot, RobotCommand, _CMD_WRITE_KEY, _CMD_READ_KEY, _CMD_UPDATE_ROBOT, _CMD_ROBOT_COMMAND, _CMD_ACK, _CMD_READ_KEY_REPLY, _CMD_READ_PATH_KEY
from random import randint
import time
import zmq
import redis
import pickle
import zmq_server
import sys

# Necessary so pickle can access class definitions from vehicle.
sys.path.append('../vehicle')

REDIS_PORT = 6379


HEARTBEAT_LIVENESS = 3
HEARTBEAT_INTERVAL = 1
INTERVAL_INIT = 1
INTERVAL_MAX = 32

#  Paranoid Pirate Protocol constants
PPP_READY = b"\x01"      # Signals worker is ready
PPP_HEARTBEAT = b"\x02"  # Signals worker heartbeat


def worker_socket(context, poller):
    """Helper function that returns a new configured socket
       connected to the Paranoid Pirate queue"""
    worker = context.socket(zmq.DEALER)  # DEALER
    identity = b"%04X-%04X" % (randint(0, 0x10000), randint(0, 0x10000))
    worker.setsockopt(zmq.IDENTITY, identity)
    poller.register(worker, zmq.POLLIN)
    worker.connect("tcp://localhost:5569")
    worker.send(PPP_READY)
    return worker


def main():

    r = redis.Redis(
        host='localhost',
        port=REDIS_PORT)

    context = zmq.Context(1)
    poller = zmq.Poller()

    liveness = HEARTBEAT_LIVENESS
    interval = INTERVAL_INIT

    heartbeat_at = time.time() + HEARTBEAT_INTERVAL

    worker = worker_socket(context, poller)
    cycles = 0
    while True:
        socks = dict(poller.poll(HEARTBEAT_INTERVAL * 1000))

        # Handle worker activity on backend
        if socks.get(worker) == zmq.POLLIN:
            #  Get message
            #  - 3-part envelope + content -> request
            #  - 1-part HEARTBEAT -> heartbeat
            frames = worker.recv_multipart()
            if not frames:
                break  # Interrupted

            if len(frames) >= 5:
                cycles += 1
                print("I: Normal reply")
                # print(len(frames))
                # print(frames)
                ident, zero_frame, idx, command, key, msg = frames
                return_command, reply = zmq_server.handle_command(
                    r, command, key, msg)
                worker.send_multipart(
                    [ident, zero_frame, idx, return_command, reply])

                # worker.send_multipart(frames)
                liveness = HEARTBEAT_LIVENESS
            elif len(frames) == 1 and frames[0] == PPP_HEARTBEAT:
                print("I: Queue heartbeat")
                liveness = HEARTBEAT_LIVENESS
            else:
                print("E: Invalid message: %s" % frames)
            interval = INTERVAL_INIT
        else:
            liveness -= 1
            if liveness == 0:
                print("W: Heartbeat failure, can't reach queue")
                print("W: Reconnecting in %0.2fs..." % interval)
                time.sleep(interval)

                if interval < INTERVAL_MAX:
                    interval *= 2
                poller.unregister(worker)
                worker.setsockopt(zmq.LINGER, 0)
                worker.close()
                worker = worker_socket(context, poller)
                liveness = HEARTBEAT_LIVENESS
        if time.time() > heartbeat_at:
            heartbeat_at = time.time() + HEARTBEAT_INTERVAL
            print("I: Worker heartbeat")
            worker.send(PPP_HEARTBEAT)


if __name__ == "__main__":
    main()
