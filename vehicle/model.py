import json
import time
from datetime import datetime
from enum import Enum

import gps_tools


MOTORS_TO_REMOTE_CONTROL_IPC = "ipc:///tmp/motors_to_control.sock"
REMOTE_CONTROL_MOTORS_TO_IPC = "ipc:///tmp/control_to_motors.sock"


class PubsubTopic(bytes, Enum):
    MOTORS_TO_REMOTE_CONTROL = b'motors_to_control'
    REMOTE_CONTROL_TO_MOTORS = b'control_to_motors'


class Control(str, Enum):
    STARTUP = "Initializing..."
    GPS_STARTUP = "Waiting for GPS fix."
    ONLINE = "Online and awaiting commands."
    AUTONOMY = "Autonomy operating."
    AUTONOMY_PAUSE = "Autonomy paused with temporary error."
    LOW_VOLTAGE = "Low voltage Pause."
    AUTONOMY_ERROR_DISTANCE = "Autonomy failed - too far from path."
    AUTONOMY_ERROR_ANGLE = "Autonomy failed - path angle too great."
    AUTONOMY_ERROR_RTK_AGE = "Autonomy failed - rtk base data too old."
    AUTONOMY_ERROR_SOLUTION_AGE = "Autonomy failed - gps solution too old."
    OVERRIDE = "Remote control override."
    SERVER_ERROR = "Server communication error."
    MOTOR_ERROR = "Motor error detected."
    NO_STEERING_SOLUTION = "No steering solution possible."


class MotorStatus(str, Enum):
    DISCONNECTED = "Not connected."
    DISABLED = "Motor error."
    ENABLED = "Motors enabled."


class GPSRecording(Enum):
    ACTIVATE = "Record"
    PAUSE = "Pause"
    CLEAR = "Clear"


class Cmd(bytes, Enum):
    WRITE_KEY = bytes('w', encoding='ascii')
    READ_KEY = bytes('readkey', encoding='ascii')
    READ_PATH_KEY = bytes('readpathkey', encoding='ascii')
    READ_KEY_REPLY = bytes('readkeyreply', encoding='ascii')
    UPDATE_ROBOT = bytes('ur', encoding='ascii')
    ROBOT_COMMAND = bytes('rc', encoding='ascii')
    ACK = bytes('a', encoding='ascii')


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
        self.control_state = Control.STARTUP
        self.motor_state = MotorStatus.DISCONNECTED
        self.loaded_path_name = ""
        self.loaded_path = []
        self.live_path_data = []
        self.gps_path_data = []
        self.record_gps_command = GPSRecording.CLEAR
        self.activate_autonomy = False
        self.autonomy_velocity = 0
        self.time_stamp = datetime.utcnow()
        self.debug_points = None
        self.wifi_strength = None
        self.wifi_ap_name = None
        self.server_disconnected_at = None
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
        return json.dumps(self.__dict__)

    def setup(self, name, server, site):
        self.name, self.server, self.site = name, server, site
        self.key = bytes("{}:robot:{}:key".format(self.site, self.name),
                         encoding='ascii')
        self.voltage = 0.0


class RobotCommand:
    def __init__(self):
        self.key = ""
        self.load_path = ""
        self.activate_autonomy = False
        self.clear_autonomy_hold = False
        self.autonomy_velocity = 0
        self.record_gps_path = GPSRecording.CLEAR

    def __repr__(self):
        return json.dumps(self.__dict__)
