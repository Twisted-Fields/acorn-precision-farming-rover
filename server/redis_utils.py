

import sys
sys.path.append('../vehicle')
import pickle

from master_process import Robot, RobotCommand, _CMD_WRITE_KEY, _CMD_READ_KEY, _CMD_UPDATE_ROBOT, _CMD_ROBOT_COMMAND, _CMD_ACK, _CMD_READ_KEY_REPLY,_CMD_READ_PATH_KEY

def get_energy_segment_key(robot_key):
    return bytes(str(robot_key)[2:-1].replace(":key", ":energy_segment:key"), encoding='ascii')

def get_robot_command_key(robot_key):
    return bytes(str(robot_key)[2:-1].replace(":key", ":command:key"), encoding='ascii')

def get_robot_keys(redis_client):
    robot_keys = []
    for key in redis_client.scan_iter():
        if ':robot:' in str(key):
            if ':command:' not in str(key) and ':energy_segment' not in str(key):
                robot_keys.append(key)
    #print(robot_keys)
    return robot_keys

def get_command_object_from_robot_key(redis_client, robot_key):
    command_key = get_robot_command_key(robot_key)
    return get_valid_command_object(redis_client, command_key)

def save_command_object_from_robot_key(redis_client, robot_key, command_object):
    command_key = get_robot_command_key(robot_key)
    redis_client.set(command_key, pickle.dumps(command_object))

def get_valid_command_object(redis_client, command_key):
    if redis_client.exists(command_key):
        robot_command = pickle.loads(redis_client.get(command_key))
        # Create a new command object if the definition has changed.
        if len(dir(RobotCommand()))!=len(dir(robot_command)):
            robot_command = RobotCommand()
    else:
        robot_command = RobotCommand()
    return robot_command

def clear_autonomy_hold(redis_client=None, vehicle_name=None, value=None, active_site=None):
    if not all((vehicle_name, value, redis_client)):
        return "Missing info for clear_autonomy_hold. No changes made."
    if len(active_site) == 0:
        return "Active site not set. Please load a path."
    vehicle_command_key = "{}:robot:{}:command:key".format(active_site, vehicle_name)  # TODO: We have two different versions of getting a command key string now.
    robot_command = get_valid_command_object(redis_client, vehicle_command_key)
    robot_command.clear_autonomy_hold = value
    redis_client.set(vehicle_command_key, pickle.dumps(robot_command))
    print("Clear hold vehicle {}".format(vehicle_command_key))
    return "Clear hold vehicle {}".format(vehicle_command_key)

def set_vehicle_autonomy(redis_client=None, vehicle_name=None, speed=None, enable=None, active_site=None):
    if not all((vehicle_name, speed, enable, redis_client)):
        return "Missing something. No vehicle autonomy set."
    if len(active_site) == 0:
        return "Active site not set. Please load a path."
    vehicle_command_key = "{}:robot:{}:command:key".format(active_site, vehicle_name)  # TODO: We have two different versions of getting a command key string now.
    robot_command = get_valid_command_object(redis_client, vehicle_command_key)
    robot_command.activate_autonomy = enable
    robot_command.autonomy_velocity = float(speed)
    redis_client.set(vehicle_command_key, pickle.dumps(robot_command))
    print("Set vehicle {} autonomy to {}".format(vehicle_command_key, (speed, enable)))
    return "Set vehicle {} autonomy to {}".format(vehicle_command_key, (speed, enable))
