#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq
import pickle

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.bind("tcp://*:5555")

SHUT_DOWN_MOTORS_COMMS_DELAY_S = 1

vel = 0
tick_time = time.time()
while True:
    #  Wait for next request from client
    if socket.poll(timeout=0):
        calc = pickle.loads(socket.recv_pyobj())
        print("Received request: {}".format(calc))
        tick_time = time.time()
        vel = calc["front_right"][1]

    #  Do some 'work'
    if time.time() - tick_time > SHUT_DOWN_MOTORS_COMMS_DELAY_S:
        vel *= 0.95
        time.sleep(0.05)
        print(vel)
    # print('loop')

    #  Send reply back to client
    #socket.send(b"World")
