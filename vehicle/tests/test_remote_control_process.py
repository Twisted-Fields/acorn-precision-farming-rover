import pickle
import logging
import multiprocessing as mp
import random
import time

from model import Robot, MotorStatus, PubsubTopic
from remote_control_process import RemoteControl
import ipc


def test_tick(fixture_path, mocker):
    ps = ipc.MemPubSub()
    mocker.patch('ipc.ZMQPub', lambda _: ps)
    mocker.patch('ipc.LeakyZMQSub', lambda: ps)

    downward, upward = ipc.SharedObject().pair()
    robot_object = Robot(simulated_data=True, logger=logging.getLogger('test_tick'))
    downward.write(robot_object)
    rc = RemoteControl(mp.Event(), downward, upward, logging,
                       debug=True, simulated_hardware=True)
    rc.run_setup()
    rc.setup_shared_memory()

    # dry run without a valid path to follow
    rc.tick()
    data = upward.read()
    assert data is not None, "should have reported something back to the main process"

    # now test if it can send commands to motors after loading a valid path
    sub = ps.sub(None, PubsubTopic.REMOTE_CONTROL_TO_MOTORS)
    robot_object.loaded_path_name = "test"
    robot_object.loaded_path = pickle.loads(fixture_path)
    downward.write(robot_object)
    rc.tick()
    time.sleep(1)
    obj = sub()
    assert obj is not None, "should have sent the commands to motor"
    for corner in ["front_left", "front_right", "rear_left", "rear_right"]:
        assert obj[corner] is not None

    ps.pub(PubsubTopic.MOTORS_TO_REMOTE_CONTROL, (MotorStatus.DISABLED, None, None, None))
    rc.tick()
    assert rc.motor_state == MotorStatus.DISABLED
    assert rc.total_watts == 0

    def rand_4():
        return [random.uniform(0, 100) for i in range(4)]
    ps.pub(PubsubTopic.MOTORS_TO_REMOTE_CONTROL, (MotorStatus.ENABLED, rand_4(), rand_4(), rand_4()))
    rc.tick()
    assert rc.motor_state == MotorStatus.ENABLED
    assert rc.total_watts > 0
