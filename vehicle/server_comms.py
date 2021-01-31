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
# Lazy Pirate poll by  Daniel Lundin <dln(at)eintr(dot)org>

import itertools
import logging
import sys
import zmq

import time


_POLL_TIMEOUT_SEC = 2.5


def AcornServerComms(acorn_pipe, server_endpoint):

    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    REQUEST_TIMEOUT = 4500
    REQUEST_RETRIES = 30

    context = zmq.Context()

    logging.info("Connecting to server…")
    client = context.socket(zmq.REQ)
    client.connect(server_endpoint)

    for sequence in itertools.count():
        while not acorn_pipe.poll(timeout=_POLL_TIMEOUT_SEC):
            print("No data from master.")

        seq_string = str(sequence).encode()
        # logging.info("Sending (%s)", seq_string)
        request = acorn_pipe.recv()
        request.insert(0, seq_string)
        client.send_multipart(request)

        # time.sleep(0.5)

        retries_left = REQUEST_RETRIES
        while True:
            if (client.poll(REQUEST_TIMEOUT) & zmq.POLLIN) != 0:
                reply = client.recv_multipart()
                # print(reply)

                if int(reply[0]) == sequence:
                    #logging.info("Server replied OK (%s)", reply)
                    acorn_pipe.send(reply[1:])
                    break
                else:
                    logging.error("Malformed reply from server: %s", reply)
                    continue

            retries_left -= 1
            logging.warning("No response from server")
            # Socket is confused. Close and remove it.
            client.setsockopt(zmq.LINGER, 0)
            client.close()
            if retries_left == 0:
                logging.error("Server seems to be offline, abandoning")
                #sys.exit()

            logging.info("Reconnecting to server…")
            # Create new connection
            client = context.socket(zmq.REQ)
            client.connect(server_endpoint)
            logging.info("Resending (%s)", seq_string)
            client.send_multipart(request)
