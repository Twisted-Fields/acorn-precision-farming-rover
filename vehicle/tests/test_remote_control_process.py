import zmq
import pickle
import logging
import multiprocessing as mp
from model import Robot
from remote_control_process import RemoteControl


def test_run_once(fixture_path):
    remote_control_manager = mp.Manager()
    lk_r2m, lk_m2r = remote_control_manager.Lock(), remote_control_manager.Lock()
    r2m, m2r = remote_control_manager.dict(), remote_control_manager.dict()
    robot_object = Robot(simulated_data=True, logger=logging.getLogger('test_run_once'))
    m2r["value"] = pickle.dumps(robot_object)
    rc = RemoteControl(mp.Event(), lk_r2m, lk_m2r, r2m, m2r, logging,
                       debug=True, simulated_hardware=True)
    rc.run_setup()
    rc.setup_shared_memory()

    # dry run without a valid path to follow
    rc.run_once()
    data = pickle.loads(r2m["value"])
    assert data is not None, "should have reported something back to the main process"

    # now test if it can send commands to motors after loading a valid path
    def zmq_socket():
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        port = socket.bind_to_random_port("tcp://127.0.0.1")
        return socket, port
    socket, port = zmq_socket()
    robot_object.loaded_path_name = "test"
    robot_object.loaded_path = pickle.loads(fixture_path)
    m2r["value"] = pickle.dumps(robot_object)
    rc.connect_to_motors(port)
    rc.run_once()
    obj = pickle.loads(socket.recv_pyobj())
    assert obj is not None, "should have sent the commands to motor"
    for corner in ["front_left", "front_right", "rear_left", "rear_right"]:
        assert obj[corner] is not None
