import time
from datetime import datetime
import numpy as np
import gps_tools


CONTROL_STARTUP = "Initializing..."
CONTROL_GPS_STARTUP = "Waiting for GPS fix."
CONTROL_ONLINE = "Online and awaiting commands."
CONTROL_AUTONOMY = "Autonomy operating."
CONTROL_AUTONOMY_PAUSE = "Temporary autonomy pause."
CONTROL_LOW_VOLTAGE = "Low voltage Pause."
CONTROL_AUTONOMY_ERROR_DISTANCE = "Autonomy failed - too far from path."
CONTROL_AUTONOMY_ERROR_ANGLE = "Autonomy failed - path angle too great."
CONTROL_AUTONOMY_ERROR_RTK_AGE = "Autonomy failed - rtk base data too old."
CONTROL_AUTONOMY_ERROR_SOLUTION_AGE = "Autonomy failed - gps solution too old."
CONTROL_OVERRIDE = "Remote control override."
CONTROL_SERVER_ERROR = "Server communication error."
CONTROL_MOTOR_ERROR = "Motor error detected."
CONTROL_NO_STEERING_SOLUTION = "No steering solution possible."

MOTOR_DISCONNECTED = 0
MOTOR_DISABLED = 1
MOTOR_ENABLED = 2

MOTOR_STATE_STRINGS = ["Not connected.",
"Motor error.",
"Motors enabled."]

# Due to shared memory errors, these must be rectangular in shape.
# Shared memory threw segfaults when these were stepped in shape (dtype object)
# Output is: [state, control flow, 0,0], [voltage x4],[ibus x4],[temperature x4],
# [encoder_estimates 0], [encoder_estimates 1], [encoder_estimates 2], [encoder_estimates 3]
MOTOR_SAMPLE_OUTPUT = np.array([[0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0],
                                [0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0]])
MOTOR_SAMPLE_INPUT = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0], [0,0]])

# Flow control flags for motor shared memory link.
MOTOR_READING = 1
CLEAR_TO_WRITE = 0
# Message status flags for motor shared memory link.
FRESH_MESSAGE = 1
STALE_MESSAGE = 0

MOTOR_READ_DELAY_MILLISECONDS = 0.5
MOTOR_READ_DELAY_SECONDS = MOTOR_READ_DELAY_MILLISECONDS / 1000.0

CORNER_NAMES={'front_right' : 0,
              'front_left': 1,
              'rear_right' : 2,
              'rear_left': 3}


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
        self.steering_debug = ()

    def __repr__(self):
        return 'Robot'

    def setup(self, name, server, site):
        self.name, self.server, self.site = name, server, site
        self.key = bytes("{}:robot:{}:key".format(self.site, self.name),
                         encoding='ascii')
        self.voltage = 0.0

class RobotSubset:
    def __init__(self, robot_object):
        self.last_server_communication_stamp = robot_object.last_server_communication_stamp
        self.wifi_ap_name = robot_object.wifi_ap_name
        self.wifi_strength = robot_object.wifi_strength
        self.cpu_temperature_c = robot_object.cpu_temperature_c
        self.loaded_path_name = robot_object.loaded_path_name
        self.loaded_path = robot_object.loaded_path
        self.activate_autonomy = robot_object.activate_autonomy
        self.clear_autonomy_hold = robot_object.clear_autonomy_hold

class RobotCommand:
    def __init__(self):
        self.key = ""
        self.load_path = ""
        self.activate_autonomy = False
        self.clear_autonomy_hold = False
        self.autonomy_velocity = 0
        self.record_gps_path = GPS_RECORDING_CLEAR
