

import zmq
import sys
import threading
import time
from random import randint, random
import pickle
import redis

# Necessary so pickle can access class definitions from vehicle.
sys.path.append('../vehicle')

from master_process import Robot, RobotCommand, _CMD_WRITE_KEY, _CMD_READ_KEY, _CMD_UPDATE_ROBOT, _CMD_ROBOT_COMMAND, _CMD_ACK, _CMD_READ_KEY_REPLY,_CMD_READ_PATH_KEY


def tprint(msg):
    """like print, but won't get newlines confused with multiple threads"""
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()

class ServerTask(threading.Thread):
    """ServerTask"""
    def __init__(self):
        threading.Thread.__init__ (self)

    def run(self):
        while True:

            try:
                context = zmq.Context()
                frontend = context.socket(zmq.ROUTER)
                frontend.bind('tcp://*:5570')

                backend = context.socket(zmq.DEALER)
                backend.bind('inproc://backend')
            except zmq.error.ZMQError:
                print("ZMQ error when setting up sockets.")
                time.sleep(5)

            workers = []
            for i in range(3):
                worker = ServerWorker(context)
                worker.start()
                workers.append(worker)

            # while True:
            #     print(dir(frontend))
            #     time.sleep(0.5)
            try:
                zmq.proxy(frontend, backend)
            except Exception as e:
                print(e)
                print("Proxy Crashed. Restarting.")
                # cleanup if needed
            frontend.close()
            backend.close()
            #context.term()


class ServerWorker(threading.Thread):
    """ServerWorker"""
    def __init__(self, context):
        threading.Thread.__init__ (self)
        self.context = context
        self.last_active_time = time.time()

    def run(self):
        _POLL_MILLISECONDS = 50
        connection_active = False
        worker = self.context.socket(zmq.DEALER)
        worker.connect('inproc://backend')
        r = redis.Redis(
            host='localhost',
            port=6379)
        tprint('Worker started')
        while True:
            if connection_active and time.time() - self.last_active_time > 10:
                self.context.destroy()
                break
            #print(type(worker))
            try:
                if not self.context.closed and worker.poll(_POLL_MILLISECONDS):
                    ident, command, key, msg = worker.recv_multipart()
                    connection_active = True
                    self.last_active_time = time.time()
                    # msg = pickle.loads(msg)
                    tprint('Command: {} {} from {}'.format(command, key, ident))
                    _BACKOFF_DELAY = 0.0  # TODO: this was debug code and this delay could be removed.
                    return_command, reply = handle_command(r, ident, command, key, msg, _BACKOFF_DELAY)
                    worker.send_multipart([ident, return_command, reply])
            except zmq.error.ZMQError:
                print("Socket Error. Closing.")
                break

        worker.close()

def handle_command(r, ident, command, key, msg, delay):
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
    elif command == _CMD_READ_PATH_KEY:

        # Got the path request. Remove the path command from the command object
        # and send the path.
        command_key = get_robot_command_key(msg)
        command_object = pickle.loads(r.get(command_key))
        command_object.load_path = ""
        r.set(command_key, pickle.dumps(command_object))
        message = pickle.dumps((key, r.get(key)))
        command_reply = _CMD_READ_KEY_REPLY
    elif command == _CMD_UPDATE_ROBOT:
        #tprint(key)
        r.set(key, msg)
        command_key = get_robot_command_key(key)
        # if not command_key:
        #     print("Error. Command key is none. Key was:")
        command_object = pickle.loads(r.get(command_key))
        #print(dir(command_object))
        #print(command_key)
        #message = bytes("ok", encoding='ascii')
        message = r.get(command_key)
        command_reply = _CMD_ROBOT_COMMAND
    else:
        tprint("NOPE")
        tprint(command)
        message = bytes("BAD_COMMAND", encoding='ascii')
    #print(delay)
    #time.sleep(delay)
    return command_reply, message

def get_robot_command_key(robot_key):

    return bytes(str(robot_key)[2:-1].replace(":key", ":command:key"), encoding='ascii')





#bytes(str(robot_key).replace(":key", ":command:key"), encoding='ascii')


def main():
    """main function"""
    _BACKOFF_DELAY = 0.1
    server = ServerTask()
    server.start()
    server.join()

if __name__ == "__main__":
    main()
