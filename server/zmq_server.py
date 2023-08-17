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

import zmq
import sys
import threading
import time
import datetime
import pickle
import redis
import psutil
import redis_utils
import model
from model import RobotCommand
from main_process import _CMD_WRITE_KEY, _CMD_READ_KEY, _CMD_UPDATE_ROBOT, _CMD_ROBOT_COMMAND
from main_process import _CMD_ACK, _CMD_READ_KEY_REPLY, _CMD_READ_PATH_KEY

_ALLOWED_ACTIVITY_LAPSE_SEC = 120
_SOCKET_RESET_TIMEOUT_MIN = 60
_SOCKET_RESET_TIMEOUT_SEC = _SOCKET_RESET_TIMEOUT_MIN * 60


_PORT = 5570
# _PORT = 5579 # debug
# _SOCKET_RESET_TIMEOUT_SEC = 5 # debug


def tprint(msg):
    """like print, but won't get newlines confused with multiple threads"""
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()


class ServerTask(threading.Thread):
    """ServerTask"""

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while True:

            try:
                context = zmq.Context()
                frontend = context.socket(zmq.ROUTER)
                frontend.bind('tcp://*:{}'.format(_PORT))

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

            try:
                zmq.proxy(frontend, backend)
            except Exception as e:
                print(e)
                print("Proxy Crashed. Restarting.")
                # cleanup if needed
            try:
                frontend.setsockopt(zmq.LINGER, 0)
                backend.setsockopt(zmq.LINGER, 0)
                frontend.close()
                backend.close()
            except Exception as e:
                print("Closing sockets raised exception: {}".format(e))
            # context.term()


def REALLY_KILL():
    for proc in psutil.process_iter():
        # check whether the process name matches
        if 'zmq_server.py' in proc.cmdline():
            print(proc.cmdline())
            proc.kill()


class ServerWorker(threading.Thread):
    """ServerWorker"""

    def __init__(self, context):
        threading.Thread.__init__(self)
        self.context = context
        self.last_active_time = time.time()

    def run(self):
        _POLL_MILLISECONDS = 50
        connection_active = False
        worker = self.context.socket(zmq.DEALER)
        worker.connect('inproc://backend')
        r = redis.Redis(host='localhost', port=6379)
        tprint('Worker started')
        while True:
            if (connection_active and time.time() - self.last_active_time > _ALLOWED_ACTIVITY_LAPSE_SEC):
                print("Lost connection so killing socket.")
                break
            if time.time() - self.last_active_time > _SOCKET_RESET_TIMEOUT_SEC:
                print("NO RECENT ZMQ ACTIVITY SO RESTARTING PROGRAM.")
                break
            # print(type(worker))
            try:
                if not self.context.closed and worker.poll(_POLL_MILLISECONDS):
                    ident, command, key, msg = worker.recv_multipart()
                    connection_active = True
                    self.last_active_time = time.time()
                    # msg = pickle.loads(msg)
                    tprint('Command: {} {} from {}'.format(command, key, ident))
                    return_command, reply = handle_command(
                        r, command, key, msg)
                    worker.send_multipart([ident, return_command, reply])
            except zmq.error.ZMQError as e:
                print(e)
                print("Socket Error. Closing.")
                break

        print("worker close")
        worker.setsockopt(zmq.LINGER, 0)
        worker.close()
        print("worker closed")
        REALLY_KILL()
        # raise zmq.error.ZMQError


def handle_command(r, command, key, msg):
    # tprint("GOT COMMAND {}".format(command))
    command_reply = _CMD_ACK
    if command == _CMD_WRITE_KEY:
        # tprint(key)
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
        # command_key = get_robot_command_key(msg)
        # command_object = pickle.loads(r.get(command_key))
        # command_object.load_path = ""
        # r.set(command_key, pickle.dumps(command_object))
        message = pickle.dumps((key, r.get(key)))
        command_reply = _CMD_READ_KEY_REPLY
    elif command == _CMD_UPDATE_ROBOT:
        print(key)
        robot = pickle.loads(msg)
        robot = update_robot(r, key, robot)

        command_key = redis_utils.get_robot_command_key(key)
        # if not command_key:
        #     print("Error. Command key is none. Key was:")
        command_encoded = r.get(command_key)
        try:
            command_object = pickle.loads(command_encoded)
        except:
            print(f"Error loading command object with key {command_key}"+ 
                    "will generate a new command object.")
            command_object = None

        # If the object changed definition we need to create a new one.
        if len(dir(RobotCommand())) != len(dir(command_object)):
            command_object = RobotCommand()

        if robot.autonomy_hold:
            command_object.activate_autonomy = False
            command_object.autonomy_velocity = float(0.0)

        r.set(command_key, pickle.dumps(command_object))

        robot_pickle = pickle.dumps(robot)
        r.set(key, robot_pickle)
        print(dir(command_object))
        print(command_key)
        # message = bytes("ok", encoding='ascii')
        message = r.get(command_key)
        command_reply = _CMD_ROBOT_COMMAND
    else:
        tprint("NOPE")
        tprint(command)
        message = bytes("BAD_COMMAND", encoding='ascii')
    # print(delay)
    # time.sleep(delay)
    return command_reply, message

# bytes(str(robot_key).replace(":key", ":command:key"), encoding='ascii')


def update_robot(r, key, robot):
    if not robot.simulated_data and len(robot.energy_segment_list) > 0:
        key = redis_utils.get_energy_segment_key(key)
        print("UPDATE ENERGY SEGMENT")
        print(key)
        for segment in robot.energy_segment_list:
            this_stamp = segment.start_gps.time_stamp
            if isinstance(this_stamp, datetime.datetime):
                stamp_localtime = this_stamp.timetuple()
            else:
                stamp_localtime = time.localtime(this_stamp)
            print(stamp_localtime)
            # print(segment.time_stamp)
            r.rpush(key, pickle.dumps(segment))
        robot.energy_segment_list = []
    if robot.simulated_data:
        print("DUMPING SIMULATED ENERGY DATA")
        robot.energy_segment_list = []
    return robot


def main():
    """main function"""
    server = ServerTask()
    server.start()
    server.join()


if __name__ == "__main__":
    main()
