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

from datetime import datetime
import argparse
import logging
import multiprocessing as mp
import os
import pickle
import subprocess
import time
import yaml
import wifi
import zmq

import nvidia_power_process
from remote_control_process import run_control
import model
import server_comms
import voltage_monitor_process
from utils import AppendFIFO, config_logging


_YAML_NAME_SIMULATION = "vehicle/server_config_sim.yaml"
_YAML_NAME_RASPBERRY = "/home/pi/vehicle/server_config.yaml"
_YAML_NAME_DOCKER = "/acorn/vehicle/server_config.yaml"

_CMD_WRITE_KEY = bytes('w', encoding='ascii')
_CMD_READ_KEY = bytes('readkey', encoding='ascii')
_CMD_READ_PATH_KEY = bytes('readpathkey', encoding='ascii')
_CMD_READ_KEY_REPLY = bytes('readkeyreply', encoding='ascii')
_CMD_UPDATE_ROBOT = bytes('ur', encoding='ascii')
_CMD_ROBOT_COMMAND = bytes('rc', encoding='ascii')
_CMD_ACK = bytes('a', encoding='ascii')

_MAX_GPS_DISTANCES = 1000

_SIMULATION_UPDATE_PERIOD = 0.5
_SIMULATION_SERVER_REPLY_TIMEOUT_SECONDS = 0.3
_UPDATE_PERIOD = 2.0
_SERVER_REPLY_TIMEOUT_SECONDS = 3

_MAX_ALLOWED_SERVER_COMMS_OUTAGE_SEC = 60

_SERVER_CONNECT_TIME_LIMIT_MINUTES = 10

_SEC_IN_ONE_MINUTE = 60

_SERVER_PING_DELAY_SEC = 2


def load_yaml_config(yaml_path):
    with open(yaml_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise RuntimeError("Problem Loading Yaml File") from exc


# Have to keep the two aliases to be able to pickle old objects stored in Redis.
Robot = model.Robot
RobotCommand = model.RobotCommand


class MainProcess():
    def __init__(self, simulation, debug):
        self.simulation = simulation
        self.debug = debug
        self.logger = logging.getLogger('main')
        config_logging(self.logger, self.debug)

    def setup(self, name, server, site):
        # Initialize robot object.
        self.logger.info("Using server {}".format(server))
        self.acorn = model.Robot(self.simulation, self.logger)
        self.acorn.setup(name, server, site)

    def run(self, stop_signal):
        # This module throws out debug messages so we change its logger level.
        i2c_logger = logging.getLogger('Adafruit_I2C')
        i2c_logger.setLevel(logging.CRITICAL)

        self.print_banner()

        # Setup and start wifi config and monitor process.
        wifi_parent_conn, wifi_child_conn = mp.Pipe()
        wifi_proc = mp.Process(target=wifi.wifi_process,
                               args=(stop_signal, wifi_child_conn, logging, self.debug,))
        wifi_proc.start()

        # Setup and start vision config and monitor process.
        vision_parent_conn, vision_child_conn = mp.Pipe()
        vision_proc = mp.Process(target=nvidia_power_process.nvidia_power_loop,
                                 args=(stop_signal, vision_child_conn, self.simulation,))
        vision_proc.start()

        # Setup and start server communications process.
        self.server_comms_parent_conn, server_comms_child_conn = mp.Pipe()
        self.logger.warning(
            "Using custom logging level for server comms process.")
        server_comms_proc = mp.Process(
            target=server_comms.AcornServerComms,
            name="server_comms_proc",
            args=(
                stop_signal,
                server_comms_child_conn,
                'tcp://{}'.format(self.acorn.server),
                logging,
            ))
        server_comms_proc.start()

        self.acorn.last_server_communication_stamp = time.time()

        remote_control_manager = mp.Manager()
        remote_to_main_lock = remote_control_manager.Lock()
        main_to_remote_lock = remote_control_manager.Lock()

        remote_to_main_string = remote_control_manager.dict()
        main_to_remote_string = remote_control_manager.dict()

        main_to_remote_string["value"] = pickle.dumps(self.acorn)

        remote_control_proc = mp.Process(
            target=run_control,
            args=(stop_signal,
                  remote_to_main_lock, main_to_remote_lock,
                  remote_to_main_string, main_to_remote_string, logging,
                  self.debug, self.simulation))
        remote_control_proc.start()

        voltage_monitor_parent_conn, voltage_monitor_child_conn = mp.Pipe()
        voltage_proc = mp.Process(target=voltage_monitor_process.sampler_loop,
                                  args=(stop_signal, voltage_monitor_child_conn, self.simulation,))
        voltage_proc.start()

        self.ping_until_reachable(self.acorn.server.split(':')[0])

        self.message_tracker = []

        # reqs = 0
        # robot_id = bytes(self.acorn.name, encoding='ascii')
        updated_object = False
        self.gps_count = 0
        # send_robot_object = False

        while not stop_signal.is_set():

            # print("3333")

            # if send_robot_object:
            #     remote_control_parent_conn.send_pyobj(pickle.dumps(acorn))
            #     send_robot_object = False
            with main_to_remote_lock:
                main_to_remote_string["value"] = pickle.dumps(self.acorn)

            # print("4444")

            if voltage_monitor_parent_conn.poll():
                self.acorn.cell1, self.acorn.cell2, self.acorn.cell3, total = voltage_monitor_parent_conn.recv()
                # self.acorn.voltage = total
                updated_object = True

            while wifi_parent_conn.poll():
                self.acorn.wifi_strength, self.acorn.wifi_ap_name, self.acorn.cpu_temperature_c = wifi_parent_conn.recv()

            # print("5555")

            updated_object |= self.update_from_remote_control(remote_to_main_lock, remote_to_main_string)
            # print("6666")
            seconds_since_update = (datetime.utcnow() - self.acorn.time_stamp).total_seconds()

            if self.simulation:
                period = _SIMULATION_UPDATE_PERIOD
            else:
                period = _UPDATE_PERIOD

            if updated_object and seconds_since_update > period:
                self.acorn.time_stamp = datetime.utcnow()
                try:
                    self.server_comms_parent_conn.send([_CMD_UPDATE_ROBOT, self.acorn.key, pickle.dumps(self.acorn)])
                    self.acorn.energy_segment_list = []
                except zmq.error.Again:
                    self.logger.error("Remote server unreachable.")
                updated_object = False

            # print("$$$$$$")
            updated_object |= self.update_from_server()
            # print(time.time())

        #    print("((((()))))")
        # if time.time() - self.acorn.last_server_communication_stamp > _MAX_ALLOWED_SERVER_COMMS_OUTAGE_SEC:
        #     print("RESET SERVER CONNECTION")
        # self.acorn.last_server_communication_stamp = time.time()
        #    print("((8888))")

    def print_banner(self):
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

    def ping_until_reachable(self, server):
        while True:
            # Check to make sure we can at least reach the server.
            self.logger.info("trying to ping server...")
            ping = subprocess.run("ping -c 1 " + server,
                                  shell=True,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT)
            for line in ping.stdout.split(b'\n'):
                self.logger.info(line.decode("utf-8"))
            if ping.returncode == 0:
                self.logger.info("Ping Successful")
                break
            self.logger.error("Ping failed. Will wait and retry.")
            time.sleep(_SERVER_PING_DELAY_SEC)

    def get_path(self, pathkey, robot):
        # TODO: Boy this function sure got complicated. Is there a better way?
        self.logger.info("SEND REQUEST FOR PATH DATA")
        while True:
            attempts = 0
            self.server_comms_parent_conn.send([
                _CMD_READ_PATH_KEY,
                bytes(pathkey, encoding='ascii'), robot.key
            ])
            time.sleep(0.5)
            while attempts < 5:
                if self.simulation:
                    timeout = _SIMULATION_SERVER_REPLY_TIMEOUT_SECONDS
                else:
                    timeout = _SERVER_REPLY_TIMEOUT_SECONDS
                if self.server_comms_parent_conn.poll(timeout=timeout):
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
                attempts += 1

    def update_from_remote_control(self, remote_to_main_lock, remote_to_main_string):
        updated_object = False
        read_okay = False
        try:
            with remote_to_main_lock:
                # (acorn_location, self.acorn.live_path_data, self.acorn.turn_intent_degrees, self.acorn.debug_points,
                # self.acorn.control_state, self.acorn.motor_state, self.acorn.autonomy_hold, gps_distance, gps_angle,
                # gps_lateral_rate, gps_angular_rate, strafeP, steerP, strafeD, steerD,
                # autonomy_steer_cmd, autonomy_strafe_cmd, gps_fix, self.acorn.voltage, energy_segment,
                # self.acorn.motor_temperatures) = pickle.loads(remote_control_parent_conn.recv_pyobj(flags=zmq.NOBLOCK))
                (acorn_location, self.acorn.live_path_data, self.acorn.turn_intent_degrees, self.acorn.debug_points,
                 self.acorn.control_state, self.acorn.motor_state, self.acorn.autonomy_hold, gps_distance, gps_angle,
                 gps_lateral_rate, gps_angular_rate, strafeP, steerP, strafeD, steerD,
                 autonomy_steer_cmd, autonomy_strafe_cmd, gps_fix, self.acorn.voltage,
                 energy_segment, self.acorn.motor_temperatures) = pickle.loads(remote_to_main_string["value"])
            read_okay = True
            # send_robot_object = True
            if acorn_location is not None:
                self.acorn.location = acorn_location
            # print("44444")
        except Exception:
            pass
            # time.sleep(2)
            # print(e)
        if read_okay:
            if gps_fix:
                # print("GPS_FIX")
                self.acorn.autonomy_steer_cmd = AppendFIFO(
                    self.acorn.autonomy_steer_cmd, autonomy_steer_cmd,
                    _MAX_GPS_DISTANCES)
                self.acorn.autonomy_strafe_cmd = AppendFIFO(
                    self.acorn.autonomy_strafe_cmd, autonomy_strafe_cmd,
                    _MAX_GPS_DISTANCES)
                self.acorn.gps_distances = AppendFIFO(self.acorn.gps_distances,
                                                      gps_distance,
                                                      _MAX_GPS_DISTANCES)
                self.acorn.gps_angles = AppendFIFO(self.acorn.gps_angles, gps_angle,
                                                   _MAX_GPS_DISTANCES)
                self.acorn.gps_path_lateral_error_rates = AppendFIFO(
                    self.acorn.gps_path_lateral_error_rates, gps_lateral_rate,
                    _MAX_GPS_DISTANCES)
                self.acorn.gps_path_angular_error_rates = AppendFIFO(
                    self.acorn.gps_path_angular_error_rates, gps_angular_rate,
                    _MAX_GPS_DISTANCES)
                self.acorn.strafeP = AppendFIFO(self.acorn.strafeP, strafeP, _MAX_GPS_DISTANCES)
                self.acorn.steerP = AppendFIFO(self.acorn.steerP, steerP, _MAX_GPS_DISTANCES)
                self.acorn.strafeD = AppendFIFO(self.acorn.strafeD, strafeD, _MAX_GPS_DISTANCES)
                self.acorn.steerD = AppendFIFO(self.acorn.steerD, steerD, _MAX_GPS_DISTANCES)

            if energy_segment is not None:
                self.acorn.energy_segment_list.append(energy_segment)

            updated_object = True
            self.gps_count += 1
            if self.gps_count % 10 == 0:
                if self.acorn.record_gps_command == model.GPS_RECORDING_ACTIVATE:
                    self.acorn.gps_path_data.append(self.acorn.location)
                    self.logger.info("APPEND GPS. TEMP PATH LENGTH {}".format(len(self.acorn.gps_path_data)))
            if self.acorn.record_gps_command == model.GPS_RECORDING_PAUSE:
                pass
            if self.acorn.record_gps_command == model.GPS_RECORDING_CLEAR:
                self.acorn.gps_path_data = []

        return updated_object

    def update_from_server(self):
        updated_object = False
        while self.server_comms_parent_conn.poll():
            # print("Got new data from server.")
            command, msg = self.server_comms_parent_conn.recv()
            # print('Client received command {} with message {}'.format(command, msg))
            self.acorn.last_server_communication_stamp = time.time()
            # send_robot_object = True

            # print("7777")
            if command == _CMD_ROBOT_COMMAND:
                robot_command = pickle.loads(msg)
                # print("GOT COMMAND: {}".format(robot_command))
                if robot_command.load_path != self.acorn.loaded_path_name and len(robot_command.load_path) > 0:
                    self.logger.info("GETTING PATH DATA")
                    path = self.get_path(robot_command.load_path, self.acorn)
                    #    print("8888")
                    if path:
                        self.acorn.loaded_path_name = robot_command.load_path
                        self.acorn.loaded_path = path
                        self.logger.info(self.acorn.loaded_path_name)
                        updated_object = True
                        # print(path)

                if robot_command.record_gps_path:
                    self.acorn.record_gps_command = robot_command.record_gps_path
                self.acorn.activate_autonomy = robot_command.activate_autonomy
                self.acorn.autonomy_velocity = robot_command.autonomy_velocity
                self.acorn.clear_autonomy_hold = robot_command.clear_autonomy_hold
                # if self.acorn.activate_autonomy == True:
                #     self.acorn.request_autonomy_at_startup = False
                self.logger.info(f"GPS Path: {robot_command.record_gps_path}, "
                                 f"Autonomy Hold: {self.acorn.autonomy_hold}, "
                                 f"Activate Autonomy: {robot_command.activate_autonomy}, "
                                 f"Autonomy Velocity: {robot_command.autonomy_velocity}, "
                                 f"Clear Autonomy Hold: {self.acorn.clear_autonomy_hold}")
        return updated_object


def run_main(simulation, debug):
    main_process = MainProcess(simulation, debug)
    if simulation:
        yaml_path = _YAML_NAME_SIMULATION
    else:
        if os.path.isfile(_YAML_NAME_DOCKER):
            yaml_path = _YAML_NAME_DOCKER
        else:
            yaml_path = _YAML_NAME_RASPBERRY
    config = load_yaml_config(yaml_path)
    name = os.getenv('VEHICLE_NAME', config.get('vehicle_name', 'noname'))
    server = os.getenv('SERVER_IP', config.get('server', '127.0.0.1'))

    main_process.setup(name, "{}:5570".format(server), config.get('site', 'nosite'))
    stop_signal = mp.Event()
    try:
        main_process.run(stop_signal)
    except Exception:
        stop_signal.set()  # allow sub-processes to terminate gracefully
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Run the Acorn vehicle coordinator process.')
    parser.add_argument('--sim',
                        dest='simulation',
                        default=False,
                        action='store_true')
    parser.add_argument('--debug',
                        dest='debug',
                        default=False,
                        action='store_true')
    args = parser.parse_args()
    run_main(args.simulation, args.debug)
