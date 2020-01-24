

import zmq
import sys
import threading
import time
from random import randint, random
import pickle
import redis

# Necessary so pickle can access class definitions from vehicle.
sys.path.append('../vehicle')

from master_process import Robot, RobotCommand, _CMD_WRITE_KEY, _CMD_READ_KEY, _CMD_UPDATE_ROBOT, _CMD_ROBOT_COMMAND, _CMD_ACK, _CMD_READ_KEY_REPLY


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
            return_command, reply = handle_command(r, ident, command, key, msg)
            worker.send_multipart([ident, return_command, reply])

        worker.close()

def handle_command(r, ident, command, key, msg):
    command_reply = _CMD_ACK
    if command == _CMD_WRITE_KEY:
        #tprint(key)
        r.set(key, msg)
        message = bytes("ok", encoding='ascii')
    elif command == _CMD_READ_KEY:

        tprint("**************")
        tprint(str(key))
        tprint("**************")
        message = pickle.dumps((key, r.get(key)))
        command_reply = _CMD_READ_KEY_REPLY
    elif command == _CMD_UPDATE_ROBOT:
        #tprint(key)
        r.set(key, msg)
        command_key = get_robot_command_key(key)
        #print(command_key)
        #message = bytes("ok", encoding='ascii')
        message = r.get(command_key)
        command_reply = _CMD_ROBOT_COMMAND
    else:
        tprint("NOPE")
        tprint(command)
        message = bytes("BAD_COMMAND", encoding='ascii')
    return command_reply, message

def get_robot_command_key(robot_key):

    return bytes(str(robot_key)[2:-1].replace(":key", ":command:key"), encoding='ascii')



#bytes(str(robot_key).replace(":key", ":command:key"), encoding='ascii')


def main():
    """main function"""
    server = ServerTask()
    server.start()
    server.join()

if __name__ == "__main__":
    main()
