import zmq
import sys
import threading
import time
from random import randint, random
import pickle
import yaml
import multiprocessing as mp
import remote_control_process
import voltage_monitor_process
import gps_tools

_USE_FAKE_GPS = False

if _USE_FAKE_GPS:
    import fake_rtk_process as rtk_process
else:
    import rtk_process

#_YAML_NAME="/home/pi/vehicle/server_config.yaml"
_YAML_NAME="/home/ubuntu/vehicle/server_config.yaml"

_CMD_WRITE_KEY = bytes('w', encoding='ascii')
_CMD_READ_KEY = bytes('readkey', encoding='ascii')
_CMD_READ_KEY_REPLY = bytes('readkeyreply', encoding='ascii')
_CMD_UPDATE_ROBOT = bytes('ur', encoding='ascii')
_CMD_ROBOT_COMMAND = bytes('rc', encoding='ascii')
_CMD_ACK = bytes('a', encoding='ascii')

class Robot:
    def __init__(self):
        self.key = ""
        self.location = gps_tools.GpsSample(0, 0, 0, ("", ""), (0, 0), 0, time.time())
        self.voltage = 0.0
        self.cell1 = 0.0
        self.cell2 = 0.0
        self.cell3 = 0.0
        self.name = ""
        self.server = ""
        self.site = ""
        self.turn_intent_degrees = 0
        self.speed = 0
        self.control_state = "Idle"
        self.motor_state = "Off"
        self.loaded_path_name = ""
        self.loaded_path = []
        self.live_path_data = []

    def __repr__(self):
        return 'Robot'

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

class RobotCommand:
    def __init__(self):
        self.key = ""
        self.load_path = ""
        self.activate_autonomy = False
        self.autonomy_velocity = 0
        self.record_gps_path = False


class MasterProcess():
    def __init__(self):
        pass
    def run(self):
        gps_parent_conn, gps_child_conn = mp.Pipe()
        gps_proc = mp.Process(target=rtk_process.start_gps, args=(gps_child_conn,))
        gps_proc.start()

        remote_control_parent_conn, remote_control_child_conn = mp.Pipe()
        remote_control_proc = mp.Process(target=remote_control_process.run_control, args=(remote_control_child_conn,))
        remote_control_proc.start()

        voltage_monitor_parent_conn, voltage_monitor_child_conn = mp.Pipe()
        voltage_proc = mp.Process(target=voltage_monitor_process.sampler_loop, args=(voltage_monitor_child_conn,))
        voltage_proc.start()

        acorn = Robot()
        acorn.setup()

        context = zmq.Context()
        self.remote_server_socket = context.socket(zmq.DEALER)
        self.remote_server_socket.identity = bytes(acorn.name, encoding='ascii')
        self.remote_server_socket.connect('tcp://192.168.1.170:5570')

        reqs = 0
        robot_id = bytes(acorn.name, encoding='ascii')
        updated_object = False
        while True:

            if gps_parent_conn.poll():
                acorn.location = gps_parent_conn.recv()
                updated_object = True

            if voltage_monitor_parent_conn.poll():
                cell1, cell2, cell3, total = voltage_monitor_parent_conn.recv()
                acorn.voltage = total
                acorn.cell1 = cell1
                acorn.cell2 = cell2
                acorn.cell3 = cell3
                updated_object = True

            #print("MAIN READS AZIMUTH AS {} degrees".format(acorn.location.azimuth_degrees))
            if len(acorn.loaded_path) > 0:
                # Clear read buffer.
                while remote_control_parent_conn.poll():
                    remote_control_parent_conn.recv()
                remote_control_parent_conn.send(acorn)

            if remote_control_parent_conn.poll(0.5):
                #print("DROP IN TO READ REMOTE PIPE")
                acorn.live_path_data, acorn.turn_intent_degrees = remote_control_parent_conn.recv()
                #print("FINISHED DROP IN TO READ REMOTE PIPE")
                updated_object = True

            if updated_object:
                #print(acorn.location)
                self.remote_server_socket.send_multipart([_CMD_UPDATE_ROBOT, acorn.key, pickle.dumps(acorn)])
                updated_object = False

            while self.remote_server_socket.poll(timeout=0):
                command, msg = self.remote_server_socket.recv_multipart()
                #print('Client received command {} with message {}'.format(command, msg))
                if command == _CMD_ROBOT_COMMAND:
                    robot_command = pickle.loads(msg)
                    if robot_command.load_path != acorn.loaded_path_name:
                        print("GETTING PATH DATA")
                        self.consume_messages()
                        path = self.get_path(robot_command.load_path)
                        if path:
                            acorn.loaded_path_name = robot_command.load_path
                            acorn.loaded_path = path
                            print(acorn.loaded_path_name)
                            updated_object = True
                            #print(path)


        self.remote_server_socket.close()
        context.term()

    def get_path(self, pathkey):
        #TODO: Boy this function sure got complicated. Is there a better way?
        print("SENT REQUEST FOR PATH DATA")
        while True:
            attempts = 0
            self.remote_server_socket.send_multipart([_CMD_READ_KEY, bytes(pathkey, encoding='ascii'), bytes('', encoding='ascii')])
            time.sleep(0.5)
            while attempts < 5:
                if self.remote_server_socket.poll():
                    print("READING PATH DATA")
                    command, msg = self.remote_server_socket.recv_multipart()
                    if command == _CMD_READ_KEY_REPLY:
                        msg = pickle.loads(msg)
                        if len(msg) == 2:
                            key = msg[0]
                            if key == bytes(pathkey, encoding='ascii'):
                                return pickle.loads(msg[1])
                            else:
                                print("{} and {} dont match".format(key, pathkey))
                        else:
                            print(msg)
                attempts+=1


    def consume_messages(self):
        while self.remote_server_socket.poll(timeout=0):
            self.remote_server_socket.recv_multipart()


def run_master():
    master = MasterProcess()
    master.run()

if __name__ == "__main__":
    run_master()
