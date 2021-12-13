import time
from datetime import datetime

import gps_tools


CONTROL_STARTUP = "Initializing..."
CONTROLGPS_STARTUP = "Waiting for GPS fix."
CONTROL_ONLINE = "Online and awaiting commands."
CONTROL_AUTONOMY = "Autonomy operating."
CONTROL_AUTONOMY_PAUSE = "Autonomy paused with temporary error."
CONTROL_LOW_VOLTAGE = "Low voltage Pause."
CONTROL_AUTONOMY_ERROR_DISTANCE = "Autonomy failed - too far from path."
CONTROL_AUTONOMY_ERROR_ANGLE = "Autonomy failed - path angle too great."
CONTROL_AUTONOMY_ERROR_RTK_AGE = "Autonomy failed - rtk base data too old."
CONTROL_AUTONOMY_ERROR_SOLUTION_AGE = "Autonomy failed - gps solution too old."
CONTROL_OVERRIDE = "Remote control override."
CONTROL_SERVER_ERROR = "Server communication error."
CONTROL_MOTOR_ERROR = "Motor error detected."
CONTROL_NO_STEERING_SOLUTION = "No steering solution possible."

MOTOR_DISCONNECTED = "Not connected."
MOTOR_DISABLED = "Motor error."
MOTOR_ENABLED = "Motors enabled."

GPS_RECORDING_ACTIVATE = "Record"
GPS_RECORDING_PAUSE = "Pause"
GPS_RECORDING_CLEAR = "Clear"


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
        self.control_state = CONTROL_STARTUP
        self.motor_state = MOTOR_DISCONNECTED
        self.loaded_path_name = ""
        self.loaded_path = []
        self.live_path_data = []
        self.gps_path_data = []
        self.record_gps_command = GPS_RECORDING_CLEAR
        self.activate_autonomy = False
        self.autonomy_velocity = 0
        self.time_stamp = datetime.utcnow()
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
        self.record_gps_path = GPS_RECORDING_CLEAR
