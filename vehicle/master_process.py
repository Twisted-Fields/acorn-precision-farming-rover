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
import rtk_process
import argparse
import nvidia_power_process
from ctypes import c_char_p
import coloredlogs
import logging
import subprocess

_YAML_NAME_LOCAL="vehicle/server_config.yaml"
_YAML_NAME_RASPBERRY="/home/pi/vehicle/server_config.yaml"
_YAML_NAME_DOCKER="/acorn/vehicle/server_config.yaml"

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


_SIMULATION_UPDATE_PERIOD = 0.1
_SIMULATION_SERVER_REPLY_TIMEOUT_MILLISECONDS = 300

_UPDATE_PERIOD = 2.0
_SERVER_REPLY_TIMEOUT_MILLISECONDS = 3000

_MAX_ALLOWED_SERVER_COMMS_OUTAGE_SEC = 60

_SERVER_CONNECT_TIME_LIMIT_MINUTES = 10

_SEC_IN_ONE_MINUTE = 60

_WIFI_SETTLING_SLEEP_SEC = 5
_SERVER_PING_DELAY_SEC = 2

_SIMULATION_IGNORE_YAML_SERVER_IP = True
_SIMULATION_SERVER_IP_ADDRESS = "127.0.0.1"

_LOGGER_FORMAT_STRING = '%(asctime)s - %(name)-11s - %(levelname)-4s - %(message)s'
_LOGGER_DATE_FORMAT = '%m/%d/%Y %I:%M:%S %p'
_LOGGER_LEVEL = 'INFO'

logging_details = (_LOGGER_FORMAT_STRING, _LOGGER_DATE_FORMAT, _LOGGER_LEVEL)


def kill_main_procs():
    for proc in psutil.process_iter():
    # check whether the process name matches
        #print(proc.cmdline())
        for item in proc.cmdline():
            if 'master_process.py' in item and proc.pid != os.getpid() and proc.pid != os.getppid():
                print(proc.cmdline())
                proc.kill()


class Robot:
    def __init__(self, simulated_data=False, logger=None):
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
        self.strafeP = []
        self.steerP = []
        self.strafeD = []
        self.steerD = []
        self.autonomy_steer_cmd = []
        self.autonomy_strafe_cmd = []
        self.cpu_temperature_c = 0.0
        self.energy_segment_list = []
        self.motor_temperatures = []
        self.simulated_data = simulated_data
        self.logger = logger

    def __repr__(self):
        return 'Robot'

    def setup(self, yaml_path):
        self.load_yaml_config(yaml_path)
        self.key = bytes("{}:robot:{}:key".format(self.site, self.name), encoding='ascii')
        self.voltage = 0.0

    def load_yaml_config(self, yaml_path):
        self.logger.info("Opening yaml file: {}".format(yaml_path))
        with open(yaml_path, 'r') as stream:
            try:
                config = yaml.safe_load(stream)
                self.name = str(config["vehicle_name"])
                self.server = str(config["server"])
                self.site = str(config["site"])
                self.logger.info("Using server from yaml {}".format(self.server))
            except yaml.YAMLError as exc:
                self.logger.error("Error! Problem Loading Yaml File. Does it exist?/n"
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

class MainProcess():
    def __init__(self, simulation):
        self.simulation = simulation
        if self.simulation:
            self.yaml_path = _YAML_NAME_LOCAL
        else:
            if os.path.isfile(_YAML_NAME_DOCKER):
                self.yaml_path = _YAML_NAME_DOCKER
            else:
                self.yaml_path = _YAML_NAME_RASPBERRY

    def run(self):

        self.logger = logging.getLogger('main')
        coloredlogs.install(fmt=_LOGGER_FORMAT_STRING,
                            datefmt=_LOGGER_DATE_FORMAT,
                            level=_LOGGER_LEVEL,
                            logger=self.logger)

        # This module throws out debug messages so we change its logger level.
        i2c_logger = logging.getLogger('Adafruit_I2C')
        i2c_logger.setLevel(logging.CRITICAL)

        for _ in range(10):
            self.logger.info("")

        banner = r"""
                                  _____           _
     /\                          / ____|         | |
    /  \   ___ ___  _ __ _ __   | (___  _   _ ___| |_ ___ _ __ ___
   / /\ \ / __/ _ \| '__| '_ \   \___ \| | | / __| __/ _ \ '_ ` _ \
  / ____ \ (_| (_) | |  | | | |  ____) | |_| \__ \ ||  __/ | | | | |
 /_/    \_\___\___/|_|  |_| |_| |_____/ \__, |___/\__\___|_| |_| |_|
                                         __/ |
                                        |___/
                      _
                    _(_)_                          wWWWw   _
        @@@@       (_)@(_)   vVVVv     _     @@@@  (___) _(_)_
       @@()@@ wWWWw  (_)\    (___)   _(_)_  @@()@@   Y  (_)@(_)
        @@@@  (___)     `|/    Y    (_)@(_)  @@@@   \|/   (_)\
         /      Y       \|    \|/    /(_)    \|      |/      |
      \ |     \ |/       | / \ | /  \|/       |/    \|      \|/
  jgs \\|//   \\|///  \\\|//\\\|/// \|///  \\\|//  \\|//  \\\|//
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        """
        # Flower art by joan stark via https://www.asciiart.eu/plants/flowers

        for line in banner.split('\n'):
            self.logger.info(line)


        # Initialize robot object.
        acorn = Robot(self.simulation, self.logger)
        acorn.setup(self.yaml_path)
        connected = False

        # Setup and start wifi config and monitor process.
        wifi_parent_conn, wifi_child_conn = mp.Pipe()
        wifi_proc = mp.Process(target=wifi.wifi_process,
                               args=(wifi_child_conn, logging, logging_details,))
        wifi_proc.start()

        # Setup and start vision config and monitor process.
        vision_parent_conn, vision_child_conn = mp.Pipe()
        vision_proc = mp.Process(target=nvidia_power_process.nvidia_power_loop,
                                 args=(vision_child_conn,
                                       self.simulation,))
        vision_proc.start()

        if self.simulation and _SIMULATION_IGNORE_YAML_SERVER_IP:
            acorn.server = _SIMULATION_SERVER_IP_ADDRESS

        # Setup and start server communications process.
        self.server_comms_parent_conn, server_comms_child_conn = mp.Pipe()
        self.logger.warning("Using custom logging level for server comms process.")
        server_comms_proc = mp.Process(target=server_comms.AcornServerComms, args=(server_comms_child_conn, 'tcp://{}:5570'.format(acorn.server), logging, (_LOGGER_FORMAT_STRING, _LOGGER_DATE_FORMAT, "INFO"), ))
        server_comms_proc.start()

        acorn.last_server_communication_stamp = time.time()

        remote_control_manager = mp.Manager()
        remote_to_main_lock = remote_control_manager.Lock()
        main_to_remote_lock = remote_control_manager.Lock()

        remote_to_main_string = remote_control_manager.dict()
        main_to_remote_string = remote_control_manager.dict()

        main_to_remote_string["value"] = pickle.dumps(acorn)

        remote_control_proc = mp.Process(target=remote_control_process.run_control, args=(remote_to_main_lock, main_to_remote_lock, remote_to_main_string, main_to_remote_string, logging, logging_details, self.simulation))
        remote_control_proc.start()


        # Let wifi settle before moving to ping test.
        time.sleep(_WIFI_SETTLING_SLEEP_SEC)

        while True:
            # Check to make sure we can at least reach the server.
            self.logger.info("trying to ping server...")
            ping = subprocess.run("ping -c 1 " + acorn.server, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            for line in ping.stdout.split(b'\n'):
                self.logger.info(line.decode("utf-8"))
            if ping.returncode == 0:
                self.logger.info("Ping Successful")
                break
            self.logger.error("Ping failed. Will wait and retry.")
            time.sleep(_SERVER_PING_DELAY_SEC)

        voltage_monitor_parent_conn, voltage_monitor_child_conn = mp.Pipe()
        voltage_proc = mp.Process(target=voltage_monitor_process.sampler_loop, args=(voltage_monitor_child_conn, self.simulation, ))
        voltage_proc.start()

        self.message_tracker = []

        reqs = 0
        robot_id = bytes(acorn.name, encoding='ascii')
        updated_object = False
        gps_count = 0
        send_robot_object = False

        while True:

            #print("3333")

            # if send_robot_object:
            #     remote_control_parent_conn.send_pyobj(pickle.dumps(acorn))
            #     send_robot_object = False
            with main_to_remote_lock:
                main_to_remote_string["value"] = pickle.dumps(acorn)

            #print("4444")

            if voltage_monitor_parent_conn.poll():
                cell1, cell2, cell3, total = voltage_monitor_parent_conn.recv()
                #acorn.voltage = total
                acorn.cell1 = cell1
                acorn.cell2 = cell2
                acorn.cell3 = cell3
                updated_object = True

            while wifi_parent_conn.poll():
                acorn.wifi_strength, acorn.wifi_ap_name, acorn.cpu_temperature_c = wifi_parent_conn.recv()

            #print("5555")

            read_okay = False
            try:
                with remote_to_main_lock:
                    #acorn_location, acorn.live_path_data, acorn.turn_intent_degrees, acorn.debug_points, acorn.control_state, acorn.motor_state, acorn.autonomy_hold, gps_distance, gps_angle, gps_lateral_rate, gps_angular_rate, strafeP, steerP, strafeD, steerD, autonomy_steer_cmd, autonomy_strafe_cmd, gps_fix, acorn.voltage, energy_segment, acorn.motor_temperatures = pickle.loads(remote_control_parent_conn.recv_pyobj(flags=zmq.NOBLOCK))
                    acorn_location, acorn.live_path_data, acorn.turn_intent_degrees, acorn.debug_points, acorn.control_state, acorn.motor_state, acorn.autonomy_hold, gps_distance, gps_angle, gps_lateral_rate, gps_angular_rate, strafeP, steerP, strafeD, steerD, autonomy_steer_cmd, autonomy_strafe_cmd, gps_fix, acorn.voltage, energy_segment, acorn.motor_temperatures = pickle.loads(remote_to_main_string["value"])
                read_okay = True
                send_robot_object = True
                if acorn_location != None:
                    acorn.location = acorn_location
                #print("44444")
            except Exception as e:
                pass
                # time.sleep(2)
                # print(e)
            if read_okay:
                if gps_fix:
                    #print("GPS_FIX")
                    acorn.autonomy_steer_cmd = AppendFIFO(acorn.autonomy_steer_cmd, autonomy_steer_cmd, _MAX_GPS_DISTANCES)
                    acorn.autonomy_strafe_cmd = AppendFIFO(acorn.autonomy_strafe_cmd, autonomy_strafe_cmd, _MAX_GPS_DISTANCES)
                    acorn.gps_distances = AppendFIFO(acorn.gps_distances, gps_distance, _MAX_GPS_DISTANCES)
                    acorn.gps_angles = AppendFIFO(acorn.gps_angles, gps_angle, _MAX_GPS_DISTANCES)
                    acorn.gps_path_lateral_error_rates = AppendFIFO(acorn.gps_path_lateral_error_rates, gps_lateral_rate, _MAX_GPS_DISTANCES)
                    acorn.gps_path_angular_error_rates = AppendFIFO(acorn.gps_path_angular_error_rates, gps_angular_rate, _MAX_GPS_DISTANCES)
                    acorn.strafeP = AppendFIFO(acorn.strafeP, strafeP, _MAX_GPS_DISTANCES)
                    acorn.steerP = AppendFIFO(acorn.steerP, steerP, _MAX_GPS_DISTANCES)
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
                        self.logger.info("APPEND GPS. TEMP PATH LENGTH {}".format(len(acorn.gps_path_data)))
                        #print(acorn.gps_path_data)
                if acorn.record_gps_command == _GPS_RECORDING_PAUSE:
                    pass
                if acorn.record_gps_command == _GPS_RECORDING_CLEAR:
                    acorn.gps_path_data = []



            #print("6666")
            seconds_since_update = (datetime.now() - acorn.time_stamp).total_seconds()

            if self.simulation:
                period = _SIMULATION_UPDATE_PERIOD
            else:
                period = _UPDATE_PERIOD

            if updated_object and seconds_since_update > period:
                acorn.time_stamp = datetime.now()
                #print(acorn.time_stamp)
                try:
                    self.server_comms_parent_conn.send([_CMD_UPDATE_ROBOT, acorn.key, pickle.dumps(acorn)])
                    acorn.energy_segment_list = []
                except zmq.error.Again as e:
                    self.logger.error("Remote server unreachable.")
                updated_object = False

            #print("$$$$$$")


            while self.server_comms_parent_conn.poll():
                #print("Got new data from server.")
                command, msg = self.server_comms_parent_conn.recv()
                #print('Client received command {} with message {}'.format(command, msg))
                acorn.last_server_communication_stamp = time.time()
                send_robot_object = True

                #print("7777")
                if command == _CMD_ROBOT_COMMAND:
                    robot_command = pickle.loads(msg)
                    #print("GOT COMMAND: {}".format(robot_command))
                    if robot_command.load_path != acorn.loaded_path_name and len(robot_command.load_path)>0:
                        self.logger.info("GETTING PATH DATA")
                        path = self.get_path(robot_command.load_path, acorn)
                    #    print("8888")
                        if path:
                            acorn.loaded_path_name = robot_command.load_path
                            acorn.loaded_path = path
                            self.logger.info(acorn.loaded_path_name)
                            updated_object = True
                            #print(path)

                    if robot_command.record_gps_path:
                        acorn.record_gps_command = robot_command.record_gps_path
                    acorn.activate_autonomy = robot_command.activate_autonomy
                    acorn.autonomy_velocity = robot_command.autonomy_velocity
                    acorn.clear_autonomy_hold = robot_command.clear_autonomy_hold
                    # if acorn.activate_autonomy == True:
                    #     acorn.request_autonomy_at_startup = False
                    self.logger.info("GPS Path: {}, Autonomy Hold: {}, Activate Autonomy: {}, Autonomy Velocity: {}, Clear Autonomy Hold: {}".format(robot_command.record_gps_path, acorn.autonomy_hold, robot_command.activate_autonomy, robot_command.autonomy_velocity, acorn.clear_autonomy_hold ))

            #print(time.time())

        #    print("((((()))))")
            # if time.time() - acorn.last_server_communication_stamp > _MAX_ALLOWED_SERVER_COMMS_OUTAGE_SEC:
            #     print("RESET SERVER CONNECTION")
                # acorn.last_server_communication_stamp = time.time()
        #    print("((8888))")


    def get_path(self, pathkey, robot):
        #TODO: Boy this function sure got complicated. Is there a better way?
        self.logger.info("SEND REQUEST FOR PATH DATA")
        while True:
            attempts = 0
            self.server_comms_parent_conn.send([_CMD_READ_PATH_KEY, bytes(pathkey, encoding='ascii'), robot.key])
            time.sleep(0.5)
            while attempts < 5:
                if self.simulation:
                    timeout = _SIMULATION_SERVER_REPLY_TIMEOUT_MILLISECONDS
                else:
                    timeout = _SERVER_REPLY_TIMEOUT_MILLISECONDS
                if self.server_comms_parent_conn.poll(timeout=timeout/1000.0):
                    self.logger.info("READING PATH DATA")
                    command, msg = self.server_comms_parent_conn.recv()
                    if command == _CMD_READ_KEY_REPLY:
                        msg = pickle.loads(msg)
                        if len(msg) == 2:
                            key = msg[0]
                            if key == bytes(pathkey, encoding='ascii'):
                                return pickle.loads(msg[1])
                            else:
                                self.logger.error("{} and {} dont match".format(key, pathkey))
                        else:
                            self.logger.info(msg)
                attempts+=1


def run_main(simulation):
    kill_main_procs()
    #sys.exit()
    main_process = MainProcess(simulation)
    main_process.run()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run the Acorn vehicle coordinator process.')
    parser.add_argument('--sim', dest='simulation', default=False, action='store_true')
    args = parser.parse_args()
    run_main(args.simulation)
