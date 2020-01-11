#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import zmq
import time
import pickle

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.PUSH)
socket.connect("tcp://localhost:5555")

time.sleep(1)

#  Do 10 requests, waiting each time for a response
for request in range(10):

    message = ("0.8", "-0.177")
    print("Sending request {} …".format(message))
    socket.send_pyobj(pickle.dumps(message))

    time.sleep(1)

    #  Get the reply.
    #message = socket.recv()
    #print("Received reply %s [ %s ]" % (request, message))
