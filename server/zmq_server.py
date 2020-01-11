

import zmq
import sys
import threading
import time
from random import randint, random
import pickle
import redis

# Necessary so pickle can access class definitions from vehicle.
sys.path.append('../vehicle')

from master_process import Robot, _CMD_WRITE


def tprint(msg):
    """like print, but won't get newlines confused with multiple threads"""
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()

class ServerTask(threading.Thread):
    """ServerTask"""
    def __init__(self):
        threading.Thread.__init__ (self)

    def run(self):
        context = zmq.Context()
        frontend = context.socket(zmq.ROUTER)
        frontend.bind('tcp://*:5570')

        backend = context.socket(zmq.DEALER)
        backend.bind('inproc://backend')

        workers = []
        for i in range(5):
            worker = ServerWorker(context)
            worker.start()
            workers.append(worker)

        zmq.proxy(frontend, backend)

        frontend.close()
        backend.close()
        context.term()

class ServerWorker(threading.Thread):
    """ServerWorker"""
    def __init__(self, context):
        threading.Thread.__init__ (self)
        self.context = context

    def run(self):
        worker = self.context.socket(zmq.DEALER)
        worker.connect('inproc://backend')
        r = redis.Redis(
            host='localhost',
            port=6379)
        tprint('Worker started')
        while True:
            ident, command, key, msg = worker.recv_multipart()
            # msg = pickle.loads(msg)
            tprint('Command: {} {} from {}'.format(command, key, ident))
            reply = handle_command(r, ident, command, key, msg)
            worker.send_multipart([ident, reply])

        worker.close()

def handle_command(r, ident, command, key, msg):
    if command == _CMD_WRITE:
        #tprint(key)
        r.set(key, msg)
    else:
        tprint("NOPE")
        #print(type(command))


    message = bytes("ok", encoding='ascii')
    return message



def main():
    """main function"""
    server = ServerTask()
    server.start()
    server.join()

if __name__ == "__main__":
    main()
