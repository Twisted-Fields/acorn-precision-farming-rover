#
#  Lazy Pirate client
#  Use zmq_poll to do a safe request-reply
#  To run, start lpserver and then randomly kill/restart it
#
#   Author: Daniel Lundin <dln(at)eintr(dot)org>
#
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
            logging.info("Resending (%s)", request)
            client.send_multipart(request)
