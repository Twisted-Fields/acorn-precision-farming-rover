import zmq
import sys
import threading
import time
from datetime import datetime
from random import randint, random
import pickle
import redis
import psutil
import redis_utils
from zmq_server_pirate import REDIS_PORT
from server import active_site
# Necessary so pickle can access class definitions from vehicle.
sys.path.append('../vehicle')

from master_process import Robot, RobotCommand, _CMD_WRITE_KEY, _CMD_READ_KEY, _CMD_UPDATE_ROBOT, _CMD_ROBOT_COMMAND, _CMD_ACK, _CMD_READ_KEY_REPLY,_CMD_READ_PATH_KEY
from remote_control_process import CONTROL_ONLINE, CONTROL_AUTONOMY


AUTONOMY_AT_STARTUP = True
AUTONOMY_SPEED = 0.2

_ONLINE_DATA_AGE_MAXIMUM_SEC = 5
_OFFLINE_DATA_AGE_MINIMUM_SEC = 3600

_ONLINE_SETTLING_TIME_SEC = 1 * 60

_MAXIMUM_TIME_TO_ATTEMPT_AUTONOMY_SEC = 20 * 60

_CLEAR_AUTONOMY_HOLD_COMMAND_OBJECT_TIME_SEC = 5

_LONG_PAUSE_SEC = 10

_LOOP_DELAY_SEC = 2

def main():
    r = redis.Redis(
        host='localhost',
        port=REDIS_PORT)
    acorn_present = False
    acorn_activated = False
    acorn_activated_time = 0
    command_object_autonomy_hold_time = 0
    while True:
        keys = redis_utils.get_robot_keys(r)
        for key in keys:
            robot = pickle.loads(r.get(key))
            command_object = redis_utils.get_command_object_from_robot_key(r, key)

            if command_object.clear_autonomy_hold == True:
                if command_object_autonomy_hold_time == 0:
                    command_object_autonomy_hold_time = time.time()
                elif time.time() - command_object_autonomy_hold_time > _CLEAR_AUTONOMY_HOLD_COMMAND_OBJECT_TIME_SEC:
                    command_object.clear_autonomy_hold = False
                    redis_utils.save_command_object_from_robot_key(r, key, command_object)
                    command_object_autonomy_hold_time = 0
                    continue
                print("Clear autonomy hold set. Will clear command value in {} seconds.".format(_CLEAR_AUTONOMY_HOLD_COMMAND_OBJECT_TIME_SEC - (time.time() - command_object_autonomy_hold_time)))


            data_age = (datetime.now() - robot.time_stamp).total_seconds()
            print("Data age: {} , Control State: {}, Autonomy Hold {}, Activate Autonomy {}".format(data_age, robot.control_state, robot.autonomy_hold, robot.activate_autonomy))
            if data_age < _ONLINE_DATA_AGE_MAXIMUM_SEC:
                if acorn_present == False:
                    print("Acorn present now")
                    acorn_activated_time = time.time()
                    acorn_present = True
                time_since_activation = time.time() - acorn_activated_time

                if time_since_activation > _ONLINE_SETTLING_TIME_SEC:
                    if robot.autonomy_hold == True:
                        if time.time() - acorn_activated_time > _MAXIMUM_TIME_TO_ATTEMPT_AUTONOMY_SEC:
                            print("Exceeded time to attempt autonomy but autonomy hold still active.")
                            time.sleep(_LONG_PAUSE_SEC)
                            continue
                        print("Clearing Autonomy Hold")
                        redis_utils.clear_autonomy_hold(redis_client=r, vehicle_name=robot.name, value=True, active_site=active_site)
                        time.sleep(_LONG_PAUSE_SEC)
                        continue
                    if AUTONOMY_AT_STARTUP:
                        if robot.control_state == CONTROL_ONLINE:
                            print("Activating Autonomy")
                            redis_utils.set_vehicle_autonomy(redis_client=r, vehicle_name=robot.name, speed=float(AUTONOMY_SPEED), enable=True, active_site=active_site)
                        else:
                            print("Could activate autonomy but state is \"{}\", not \"{}\".".format(robot.control_state, CONTROL_ONLINE))
                else:
                    print("Acorn is active. Waiting {} seconds for it to settle.".format(_ONLINE_SETTLING_TIME_SEC - time_since_activation))

            if data_age > _OFFLINE_DATA_AGE_MINIMUM_SEC:
                print("Acorn offline.")
                acorn_present = False

        time.sleep(_LOOP_DELAY_SEC)

if __name__ == "__main__":
    main()








# time_stamp = json.dumps(robot.time_stamp, default=date_handler)
# live_path_data = [point._asdict() for point in robot.live_path_data]
# gps_path_data = [point._asdict() for point in robot.gps_path_data]
# try:
#     debug_points = [point._asdict() for point in robot.debug_points]
# except:
#     debug_points = []
# #print(json.dumps(robot.gps_path_data[0]._asdict()))
# robot_entry = { 'name': robot.name, 'lat': robot.location.lat, 'lon':
# robot.location.lon, 'heading': robot.location.azimuth_degrees,
# 'speed': robot.speed, 'turn_intent_degrees': robot.turn_intent_degrees,
# 'voltage': robot.voltage, 'control_state': robot.control_state,
# 'motor_state': robot.motor_state, 'time_stamp': time_stamp,
# 'loaded_path_name': "" if not robot.loaded_path_name else robot.loaded_path_name.split('gpspath:')[1].split(':key')[0],
# 'live_path_data' : live_path_data,
# 'gps_path_data' : gps_path_data,
# 'debug_points' : debug_points,
# 'autonomy_hold' : robot.autonomy_hold,
# 'activate_autonomy' : robot.activate_autonomy,
# 'access_point_name' : robot.wifi_ap_name,
# 'wifi_signal' : robot.wifi_strength,
# 'gps_distances' : robot.gps_distances,
# 'gps_angles' : robot.gps_angles,
# 'gps_distance_rates' : robot.gps_path_lateral_error_rates,
# 'gps_angle_rates' : robot.gps_path_angular_error_rates,
# 'strafe' : robot.strafe,
# 'rotation' : robot.rotation,
# 'strafeD' : robot.strafeD,
# 'steerD' : robot.steerD
