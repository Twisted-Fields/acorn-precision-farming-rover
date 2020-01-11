import zmq
import sys
import threading
import time
from random import randint, random
import pickle
import fake_rtk_process as rtk_process
import yaml
import multiprocessing as mp

_YAML_NAME="/home/pi/vehicle/server_config.yaml"

_CMD_WRITE = bytes('w', encoding='ascii')

class Robot:
    def __init__(self):
        self.key = ""
        self.location = {'lat': 0, 'lon': 0}
        self.voltage = 0.0
        self.name = ""
        self.server = ""
        self.site = ""
        self.turn_intent_degrees = 0
        self.speed = 0
        self.state = "Idle"

    def setup(self):
        self.load_yaml_config()
        self.key = bytes("{}:robot:{}:key".format(self.site, self.name), encoding='ascii')
        self.voltage = 24.0

    def load_yaml_config(self):
        with open(_YAML_NAME, 'r') as stream:
            try:
                config = yaml.safe_load(stream)
                self.name = str(config["vehicle_name"])
                self.server = str(config["server"])
                self.site = str(config["site"])
            except yaml.YAMLError as exc:
                print("Error! Problem Loading Yaml File. Does it exist?/n"
                      "Actual error thrown was: {}".format(exc))


class MasterProcess():
    def __init__(self):
        pass
    def run(self):
        gps_parent_conn, gps_child_conn = mp.Pipe()
        gps_proc = mp.Process(target=rtk_process.start_gps, args=(gps_child_conn,))
        gps_proc.start()

        acorn = Robot()
        acorn.setup()

        context = zmq.Context()
        remote_server_socket = context.socket(zmq.DEALER)
        remote_server_socket.identity = bytes(acorn.name, encoding='ascii')
        remote_server_socket.connect('tcp://192.168.1.170:5570')


        reqs = 0
        robot_id = bytes(acorn.name, encoding='ascii')
        while True:

            if gps_parent_conn.poll():
                acorn.location = gps_parent_conn.recv()
                acorn.voltage = 24 + random()*3
                remote_server_socket.send_multipart([_CMD_WRITE, acorn.key, pickle.dumps(acorn)])

            while remote_server_socket.poll(timeout=0):
                remote_id, msg = remote_server_socket.recv()
                print('Client %s received: %s' % (remote_id, msg))


        remote_server_socket.close()
        context.term()


def run_master():
    master = MasterProcess()
    master.run()


if __name__ == "__main__":
    run_master()
