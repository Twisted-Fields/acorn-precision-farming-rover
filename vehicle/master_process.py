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
from motors import STATE_DISCONNECTED
from datetime import datetime
import wifi
import os
import queue
import psutil
import pipe_relay
import fcntl
import server_comms

_USE_FAKE_GPS = False

if _USE_FAKE_GPS:
    import fake_rtk_process as rtk_process
else:
    import rtk_process

_YAML_NAME="/home/pi/vehicle/server_config.yaml"

_CMD_WRITE_KEY = bytes('w', encoding='ascii')
_CMD_READ_KEY = bytes('readkey', encoding='ascii')
_CMD_READ_PATH_KEY = bytes('readpathkey', encoding='ascii')
_CMD_READ_KEY_REPLY = bytes('readkeyreply', encoding='ascii')
_CMD_UPDATE_ROBOT = bytes('ur', encoding='ascii')
_CMD_ROBOT_COMMAND = bytes('rc', encoding='ascii')
_CMD_ACK = bytes('a', encoding='ascii')

_MAX_GPS_DISTANCES = 1000

_GPS_RECORDING_ACTIVATE = "Record"
_GPS_RECORDING_PAUSE = "Pause"
_GPS_RECORDING_CLEAR = "Clear"

_UPDATE_PERIOD = 2.0

_SERVER_REPLY_TIMEOUT_MILLISECONDS = 3000

_MAX_ALLOWED_SERVER_COMMS_OUTAGE_SEC = 60

_SERVER_CONNECT_TIME_LIMIT_MINUTES = 10

_SEC_IN_ONE_MINUTE = 60

_WIFI_SETTLING_SLEEP_SEC = 5
_SERVER_PING_DELAY_SEC = 2


def kill_master_procs():
    for proc in psutil.process_iter():
    # check whether the process name matches
        #print(proc.cmdline())
        for item in proc.cmdline():
            if 'master_process.py' in item and proc.pid != os.getpid() and proc.pid != os.getppid():
                print(proc.cmdline())
                proc.kill()


class Robot:
    def __init__(self):
        self.key = ""
        self.location = gps_tools.GpsSample(0, 0, 0, ("", ""), (0, 0), 0, time.time(), 0)
        self.voltage = 0.0
        self.cell1 = 0.0
        self.cell2 = 0.0
        self.cell3 = 0.0
        self.name = ""
        self.server = ""
        self.site = ""
        self.turn_intent_degrees = 0
        self.speed = 0
        self.control_state = remote_control_process.CONTROL_STARTUP
        self.motor_state = STATE_DISCONNECTED
        self.loaded_path_name = ""
        self.loaded_path = []
        self.live_path_data = []
        self.gps_path_data = []
        self.record_gps_command = _GPS_RECORDING_CLEAR
        self.activate_autonomy = False
        self.autonomy_velocity = 0
        self.time_stamp = datetime.now()
        self.debug_points = None
        self.wifi_strength = None
        self.wifi_ap_name = None
        self.last_server_communication_stamp = 0
        self.autonomy_hold = True
        self.clear_autonomy_hold = False
        self.gps_distances = []
        self.gps_angles = []
        self.gps_path_lateral_error_rates = []
        self.gps_path_angular_error_rates = []
        self.strafe = []
        self.rotation = []
        self.strafeD = []
        self.steerD = []
        self.cpu_temperature_c = 0.0
        self.energy_segment_list = []
        self.motor_temperatures = []

    def __repr__(self):
        return 'Robot'

    def setup(self):
        self.load_yaml_config()
        self.key = bytes("{}:robot:{}:key".format(self.site, self.name), encoding='ascii')
        self.voltage = 0.0

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
        self.clear_autonomy_hold = False
        self.autonomy_velocity = 0
        self.record_gps_path = _GPS_RECORDING_CLEAR


def AppendFIFO(list, value, max_values):
    list.append(value)
    while len(list) > max_values:
        list.pop(0)
    return list

class MasterProcess():
    def __init__(self):
        pass
    def run(self):

        # Initialize robot object.
        acorn = Robot()
        acorn.setup()
        connected = False

        # Setup and start wifi config and monitor process.
        wifi_parent_conn, wifi_child_conn = mp.Pipe()
        wifi_proc = mp.Process(target=wifi.wifi_process, args=(wifi_child_conn,))
        wifi_proc.start()

        # Setup and start server communications process.
        self.server_comms_parent_conn, server_comms_child_conn = mp.Pipe()
        server_comms_proc = mp.Process(target=server_comms.AcornServerComms, args=(server_comms_child_conn, 'tcp://{}:5570'.format(acorn.server), ))
        server_comms_proc.start()


        # Let wifi settle before moving to ping test.
        time.sleep(_WIFI_SETTLING_SLEEP_SEC)

        while True:
            # Check to make sure we can at least reach the server.
            print("trying to ping server...")
            if os.system("ping -c 1 " + acorn.server) == 0:
                print("Ping Successful")
                break
            print("Ping failed. Will wait and retry.")
            time.sleep(_SERVER_PING_DELAY_SEC)

        acorn.last_server_communication_stamp = time.time()

        port = "5996"
        context = zmq.Context()
        remote_control_parent_conn = context.socket(zmq.PAIR)
        remote_control_parent_conn.bind("tcp://*:%s" % port)

        remote_control_proc = mp.Process(target=remote_control_process.run_control, args=())
        remote_control_proc.start()


        voltage_monitor_parent_conn, voltage_monitor_child_conn = mp.Pipe()
        voltage_proc = mp.Process(target=voltage_monitor_process.sampler_loop, args=(voltage_monitor_child_conn,))
        voltage_proc.start()

        self.message_tracker = []

        reqs = 0
        robot_id = bytes(acorn.name, encoding='ascii')
        updated_object = False
        gps_count = 0
        send_robot_object = True

        while True:

            # print("3333")

            if send_robot_object:
                remote_control_parent_conn.send_pyobj(pickle.dumps(acorn))
                send_robot_object = False

            # print("4444")

            if voltage_monitor_parent_conn.poll():
                cell1, cell2, cell3, total = voltage_monitor_parent_conn.recv()
                #acorn.voltage = total
                acorn.cell1 = cell1
                acorn.cell2 = cell2
                acorn.cell3 = cell3
                updated_object = True

            while wifi_parent_conn.poll():
                acorn.wifi_strength, acorn.wifi_ap_name, acorn.cpu_temperature_c = wifi_parent_conn.recv()

            # print("5555")

            read_okay = False
            try:
                acorn_location, acorn.live_path_data, acorn.turn_intent_degrees, acorn.debug_points, acorn.control_state, acorn.motor_state, acorn.autonomy_hold, gps_distance, gps_angle, gps_lateral_rate, gps_angular_rate, strafe, rotation, strafeD, steerD, gps_fix, acorn.voltage, energy_segment, acorn.motor_temperatures = pickle.loads(remote_control_parent_conn.recv_pyobj(flags=zmq.NOBLOCK))
                read_okay = True
                send_robot_object = True
                if acorn_location != None:
                    acorn.location = acorn_location
                # print("44444")
            except Exception as e:
                pass
                # time.sleep(2)
                # print(e)
            if read_okay:
                if gps_fix:
                    #print("GPS_FIX")
                    acorn.gps_distances = AppendFIFO(acorn.gps_distances, gps_distance, _MAX_GPS_DISTANCES)
                    acorn.gps_angles = AppendFIFO(acorn.gps_angles, gps_angle, _MAX_GPS_DISTANCES)
                    acorn.gps_path_lateral_error_rates = AppendFIFO(acorn.gps_path_lateral_error_rates, gps_lateral_rate, _MAX_GPS_DISTANCES)
                    acorn.gps_path_angular_error_rates = AppendFIFO(acorn.gps_path_angular_error_rates, gps_angular_rate, _MAX_GPS_DISTANCES)
                    acorn.strafe = AppendFIFO(acorn.strafe, strafe, _MAX_GPS_DISTANCES)
                    acorn.rotation = AppendFIFO(acorn.rotation, rotation, _MAX_GPS_DISTANCES)
                    acorn.strafeD = AppendFIFO(acorn.strafeD, strafeD, _MAX_GPS_DISTANCES)
                    acorn.steerD = AppendFIFO(acorn.steerD, steerD, _MAX_GPS_DISTANCES)

                if energy_segment != None:
                    acorn.energy_segment_list.append(energy_segment)

                updated_object = True
                gps_count += 1
                if gps_count % 10 == 0:
                    updated_object = True
                    if acorn.record_gps_command == _GPS_RECORDING_ACTIVATE:
                        acorn.gps_path_data.append(acorn.location)
                        print("APPEND GPS. TEMP PATH LENGTH {}".format(len(acorn.gps_path_data)))
                        #print(acorn.gps_path_data)
                if acorn.record_gps_command == _GPS_RECORDING_PAUSE:
                    pass
                if acorn.record_gps_command == _GPS_RECORDING_CLEAR:
                    acorn.gps_path_data = []



            # print("6666")
            seconds_since_update = (datetime.now() - acorn.time_stamp).total_seconds()
            if updated_object and seconds_since_update > _UPDATE_PERIOD:
                acorn.time_stamp = datetime.now()
                #print(acorn.time_stamp)
                try:
                    self.server_comms_parent_conn.send([_CMD_UPDATE_ROBOT, acorn.key, pickle.dumps(acorn)])
                    acorn.energy_segment_list = []
                except zmq.error.Again as e:
                    print("Remote server unreachable.")
                updated_object = False

            # print("$$$$$$")


            while self.server_comms_parent_conn.poll():
                #print("recv_multipart")
                command, msg = self.server_comms_parent_conn.recv()
                #print('Client received command {} with message {}'.format(command, msg))
                acorn.last_server_communication_stamp = time.time()

            #    print("7777")
                if command == _CMD_ROBOT_COMMAND:
                    robot_command = pickle.loads(msg)
                    #print("GOT COMMAND: {}".format(robot_command))
                    if robot_command.load_path != acorn.loaded_path_name and len(robot_command.load_path)>0:
                        print("GETTING PATH DATA")
                        path = self.get_path(robot_command.load_path, acorn)
                    #    print("8888")
                        if path:
                            acorn.loaded_path_name = robot_command.load_path
                            acorn.loaded_path = path
                            print(acorn.loaded_path_name)
                            updated_object = True
                            #print(path)

                    if robot_command.record_gps_path:
                        acorn.record_gps_command = robot_command.record_gps_path
                    acorn.activate_autonomy = robot_command.activate_autonomy
                    acorn.autonomy_velocity = robot_command.autonomy_velocity
                    acorn.clear_autonomy_hold = robot_command.clear_autonomy_hold
                    # if acorn.activate_autonomy == True:
                    #     acorn.request_autonomy_at_startup = False
                    print("GPS Path: {}, Autonomy Hold: {}, Activate Autonomy: {}, Autonomy Velocity: {}, Clear Autonomy Hold: {}".format(robot_command.record_gps_path, acorn.autonomy_hold, robot_command.activate_autonomy, robot_command.autonomy_velocity, acorn.clear_autonomy_hold ))

            #print(time.time())

        #    print("((((()))))")
            # if time.time() - acorn.last_server_communication_stamp > _MAX_ALLOWED_SERVER_COMMS_OUTAGE_SEC:
            #     print("RESET SERVER CONNECTION")
                # acorn.last_server_communication_stamp = time.time()
        #    print("((8888))")



        context.term()

    def get_path(self, pathkey, robot):
        #TODO: Boy this function sure got complicated. Is there a better way?
        print("SEND REQUEST FOR PATH DATA")
        while True:
            attempts = 0
            self.server_comms_parent_conn.send([_CMD_READ_PATH_KEY, bytes(pathkey, encoding='ascii'), robot.key])
            time.sleep(0.5)
            while attempts < 5:
                if self.server_comms_parent_conn.poll(timeout=_SERVER_REPLY_TIMEOUT_MILLISECONDS / 1000.0):
                    print("READING PATH DATA")
                    command, msg = self.server_comms_parent_conn.recv()
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


def run_master():
    kill_master_procs()
    #sys.exit()
    master = MasterProcess()
    master.run()

if __name__ == "__main__":
    run_master()
