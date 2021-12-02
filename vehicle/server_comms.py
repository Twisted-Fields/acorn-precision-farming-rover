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
import zmq
import utils


_POLL_TIMEOUT_SEC = 2.5


def AcornServerComms(stop_signal, acorn_pipe, server_endpoint, logging):
    logger = logging.getLogger('main.comms')
    utils.config_logging(logger, debug=False)

    REQUEST_TIMEOUT = 4500
    REQUEST_RETRIES = 30

    logger.info("Connecting to server at {} …".format(server_endpoint))
    context = zmq.Context()
    client = context.socket(zmq.REQ)
    client.connect(server_endpoint)

    for sequence in itertools.count():
        if stop_signal.is_set():
            break
        if not acorn_pipe.poll(timeout=_POLL_TIMEOUT_SEC):
            logger.error("No data from master.")
            continue

        request = acorn_pipe.recv()
        seq_string = str(sequence).encode()
        logger.debug("Sending (%s)", seq_string)
        request.insert(0, seq_string)
        client.send_multipart(request)

        # time.sleep(0.5)

        retries_left = REQUEST_RETRIES
        while not stop_signal.is_set() and retries_left > 0:
            if (client.poll(REQUEST_TIMEOUT) & zmq.POLLIN) != 0:
                reply = client.recv_multipart()
                # print(reply)

                if int(reply[0]) == sequence:
                    logger.debug("Server replied OK (%s...)", reply[:2])
                    acorn_pipe.send(reply[1:])
                    break
                else:
                    logger.error("Malformed reply from server: %s", reply)
                    continue

            retries_left -= 1
            logger.warning("No response from server")
            # Socket is confused. Close and remove it.
            client.setsockopt(zmq.LINGER, 0)
            client.close()
            logger.info("Reconnecting to server…")
            # Create new connection
            client = context.socket(zmq.REQ)
            client.connect(server_endpoint)
            logger.info("Resending (%s)", seq_string)
            client.send_multipart(request)
