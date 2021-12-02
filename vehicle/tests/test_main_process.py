import os
import psutil
import pickle
import time
import zmq
import yaml
import multiprocessing as mp
import threading
from master_process import MainProcess
from master_process import _CMD_READ_PATH_KEY, _CMD_READ_KEY_REPLY
from master_process import _CMD_UPDATE_ROBOT, _CMD_ROBOT_COMMAND


def zmq_socket(port=None):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    if port is None:
        port = socket.bind_to_random_port("tcp://127.0.0.1")
    else:
        socket.bind("tcp://127.0.0.1:{}".format(port))
    return socket, port


def test_main(fixture_path, fixture_robot_command):
    server_socket, server_port = zmq_socket()
    # have to use fixed port to be consistent with the remote control process
    motors_socket, _ = zmq_socket(5590)

    def mock_server():
        while True:
            if server_socket.poll(1):
                ident, command, key, msg = server_socket.recv_multipart()
                if command == _CMD_READ_PATH_KEY:
                    server_socket.send_multipart([ident, _CMD_READ_KEY_REPLY, pickle.dumps((key, fixture_path))])
                elif command == _CMD_UPDATE_ROBOT:
                    server_socket.send_multipart([ident, _CMD_ROBOT_COMMAND, fixture_robot_command])

    def mock_motors():
        while True:
            if motors_socket.poll(1):
                obj = pickle.loads(motors_socket.recv_pyobj())
                assert obj is not None, "should have sent the commands to motor"

    cfg = """
    vehicle_name: acorn1
    server: 127.0.0.1:{}
    site: twistedfields
    """.format(server_port)
    server_thread = threading.Thread(target=mock_server, daemon=True)
    server_thread.start()

    main_process = MainProcess(simulation=True, debug=True)
    main_process.setup(yaml.safe_load(cfg))
    stop_signal = mp.Event()
    main_thread = threading.Thread(target=lambda: main_process.run(stop_signal), daemon=True)
    main_thread.start()
    time.sleep(5)
    stop_signal.set()
    time.sleep(1)
    for proc in psutil.process_iter():
        assert proc.ppid != os.getpid() or proc.pid == os.getpid(), "all sub-processes should have been terminated"
