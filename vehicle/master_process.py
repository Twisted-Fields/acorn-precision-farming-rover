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
import socketio

import nvidia_power_process
from remote_control_process import run_control
import model
import voltage_monitor
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
_UPDATE_PERIOD = 2.0

_SERVER_PING_DELAY_SEC = 2
_SERVER_CONNECT_DELAY_SEC = 2
_SERVER_DELAY_RECONNECT_WIFI_SECONDS = 120


# Have to keep the two aliases to be able to pickle old objects stored in Redis.
Robot = model.Robot
RobotCommand = model.RobotCommand


class MainProcess():
    def __init__(self, simulation, debug):
        self.simulation = simulation
        self.debug = debug
        self.last_wifi_restart_time = time.time()
        self.logger = logging.getLogger('main')
        config_logging(self.logger, self.debug)
        self.wifi = wifi.Wifi(self.debug)
        self.voltage_monitor = voltage_monitor.VoltageSampler(simulation)

    def setup(self, name, server, site):
        # Initialize robot object.
        self.logger.info("Using server {}".format(server))
        self.acorn = model.Robot(self.simulation, self.logger)
        self.acorn.setup(name, server, site)
        self.setup_server_communication()
        self.wifi.setup()
        # This module throws out debug messages so we change its logger level.
        i2c_logger = logging.getLogger('Adafruit_I2C')
        i2c_logger.setLevel(logging.CRITICAL)

    def run(self, stop_signal, fps=10):
        self.print_banner()

        # Setup and start vision config and monitor process.
        vision_parent_conn, vision_child_conn = mp.Pipe()
        vision_proc = mp.Process(target=nvidia_power_process.nvidia_power_loop,
                                 args=(stop_signal, vision_child_conn, self.simulation,))
        vision_proc.start()

        remote_control_manager = mp.Manager()
        remote_to_main_lock = remote_control_manager.Lock()
        main_to_remote_lock = remote_control_manager.Lock()
        remote_to_main_string = remote_control_manager.dict()
        main_to_remote_string = remote_control_manager.dict()
        main_to_remote_string["value"] = pickle.dumps(self.acorn)
        remote_control_proc = mp.Process(target=run_control,
                                         args=(stop_signal,
                                               remote_to_main_lock, main_to_remote_lock,
                                               remote_to_main_string, main_to_remote_string, logging,
                                               self.debug, self.simulation))
        remote_control_proc.start()

        self.ping_until_reachable(self.acorn.server.split(':')[0])
        self.wait_connect_server()

        # reqs = 0
        # robot_id = bytes(self.acorn.name, encoding='ascii')
        updated_object = False
        gps_count = 0
        # send_robot_object = False

        interval = 1.0 / fps
        while not stop_signal.is_set():

            start = time.time()
            # print("3333")

            # if send_robot_object:
            #     remote_control_parent_conn.send_pyobj(pickle.dumps(acorn))
            #     send_robot_object = False
            with main_to_remote_lock:
                main_to_remote_string["value"] = pickle.dumps(self.acorn)

            self.acorn.cell1, self.acorn.cell2, self.acorn.cell3, total = self.voltage_monitor.read()
            updated_object = True

            self.acorn.wifi_strength, self.acorn.wifi_ap_name, self.acorn.cpu_temperature_c = self.wifi.collect()

            updated_object |= self.update_from_remote_control(gps_count, remote_to_main_lock, remote_to_main_string)

            seconds_since_update = (datetime.utcnow() - self.acorn.time_stamp).total_seconds()
            if self.simulation:
                period = _SIMULATION_UPDATE_PERIOD
            else:
                period = _UPDATE_PERIOD
            if updated_object and seconds_since_update > period:
                self.acorn.time_stamp = datetime.utcnow()
                try:
                    self.sio.emit(_CMD_UPDATE_ROBOT, [self.acorn.key, pickle.dumps(self.acorn)])
                    self.acorn.energy_segment_list = []
                except socketio.exceptions.BadNamespaceError:
                    self.logger.error("Remote server unreachable.")

                updated_object = False

            self.maybe_restart_wifi()
            time_left = interval - (time.time() - start)
            if time_left > 0:
                time.sleep(time_left)

    def setup_server_communication(self):
        self.sio = socketio.Client()

        @self.sio.on(_CMD_ROBOT_COMMAND)
        def on_robot_command(msg):
            robot_command = pickle.loads(msg)
            if robot_command.load_path != self.acorn.loaded_path_name and len(robot_command.load_path) > 0:
                self.logger.info(f"requesting path for {robot_command.load_path}")
                self.sio.emit(_CMD_READ_PATH_KEY, robot_command.load_path)

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

        @self.sio.on(_CMD_READ_KEY_REPLY)
        def on_path(msg):
            path_name, path = msg
            self.logger.info(f"got path for {path_name}")
            self.acorn.loaded_path_name = path_name
            self.acorn.loaded_path = pickle.loads(path)

        @self.sio.event
        def connect_error(data):
            self.logger.warning("fail to connect server. will retry.")
            self.acorn.server_disconnected_at = time.time()

        @self.sio.event
        def connect():
            self.logger.debug("connected to server.")
            self.acorn.server_disconnected_at = None

        @self.sio.event
        def disconnect():
            self.logger.debug("server disconnected.")
            self.acorn.server_disconnected_at = time.time()

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

    def wait_connect_server(self):
        while True:
            try:
                self.sio.connect('http://{}'.format(self.acorn.server))
                return
            except socketio.exceptions.ConnectionError:
                time.sleep(_SERVER_CONNECT_DELAY_SEC)

    def update_from_remote_control(self, gps_count, remote_to_main_lock, remote_to_main_string):
        try:
            with remote_to_main_lock:
                (acorn_location, self.acorn.live_path_data, self.acorn.turn_intent_degrees, self.acorn.debug_points,
                 self.acorn.control_state, self.acorn.motor_state, self.acorn.autonomy_hold, gps_distance, gps_angle,
                 gps_lateral_rate, gps_angular_rate, strafeP, steerP, strafeD, steerD,
                 autonomy_steer_cmd, autonomy_strafe_cmd, gps_fix, self.acorn.voltage,
                 energy_segment, self.acorn.motor_temperatures) = pickle.loads(remote_to_main_string["value"])
            # send_robot_object = True
            if acorn_location is not None:
                self.acorn.location = acorn_location
        except Exception as e:
            print(e)
            return False

        updated_object = False
        if gps_fix:
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
        gps_count += 1
        if gps_count % 10 == 0:
            if self.acorn.record_gps_command == model.GPS_RECORDING_ACTIVATE:
                self.acorn.gps_path_data.append(self.acorn.location)
                self.logger.info("APPEND GPS. TEMP PATH LENGTH {}".format(len(self.acorn.gps_path_data)))
        if self.acorn.record_gps_command == model.GPS_RECORDING_PAUSE:
            pass  # TODO: anything to do here?
        if self.acorn.record_gps_command == model.GPS_RECORDING_CLEAR:
            self.acorn.gps_path_data = []

        return updated_object

    def maybe_restart_wifi(self):
        if (time.time() > self.last_wifi_restart_time + 500 and
                self.acorn.server_disconnected_at and
                time.time() - self.acorn.server_disconnected_at > _SERVER_DELAY_RECONNECT_WIFI_SECONDS):
            self.logger.error(
                "Last Wifi signal strength: {} dbm\r\n".format(
                    self.acorn.wifi_strength))
            self.logger.error("Last Wifi AP associated: {}\r\n".format(
                self.acorn.wifi_ap_name))
            self.logger.error("Restarting wlan1...")
            try:
                subprocess.check_call("ifconfig wlan1 down",
                                      shell=True)
                subprocess.check_call("ifconfig wlan1 up", shell=True)
            except BaseException:
                pass
            self.last_wifi_restart_time = time.time()
            self.logger.error("Restarted wlan1.")


def load_yaml_config(yaml_path):
    with open(yaml_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise RuntimeError("Problem Loading Yaml File") from exc


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

    main_process.setup(name, "{}:80".format(server), config.get('site', 'nosite'))
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
