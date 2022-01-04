import logging
import multiprocessing as mp
import os
import pickle
import psutil
import threading
import time
import zmq

import model
from model import Cmd
import master_process

logger = logging.getLogger(__name__)


def zmq_socket(port=None):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    if port is None:
        port = socket.bind_to_random_port("tcp://127.0.0.1")
    else:
        socket.bind("tcp://127.0.0.1:{}".format(port))
    return socket, port


class MockSocketIOClient():
    def __init__(self):
        self.handlers = dict()

    def connect(self, _):
        logger.info("calling connect")

    def emit(self, *args):
        logger.info(f"calling emit: {args}")

    def event(self, f):
        logger.info(f"register event({f.__name__})")
        self.handlers[f.__name__] = f

    def on(self, name):
        def register(f):
            logger.info(f"register on({name}): {f}")
            self.handlers[name] = f
        return register


def test_main(fixture_path, fixture_robot_command, mocker):
    mocked = MockSocketIOClient()
    mocker.patch('socketio.Client', lambda: mocked)

    def mock_motors():
        # have to use fixed port to be consistent with the remote control process
        motors_socket, _ = zmq_socket(5590)
        while True:
            if motors_socket.poll(1):
                obj = pickle.loads(motors_socket.recv_pyobj())
                assert obj is not None, "should have sent the commands to motor"

    main_process = master_process.MainProcess(simulation=True, debug=True)
    main_process.setup("test", "127.0.0.1:whatever", "test-site")

    # make sure all socket IO handlers are properly registered
    expected_handlers = set([Cmd.ROBOT_COMMAND, Cmd.READ_KEY_REPLY,
                             'connect_error', 'connect', 'disconnect'])
    assert set(mocked.handlers.keys()) ^ expected_handlers == set()

    # should load the path when received a robot command with a different path name.
    cmd = model.RobotCommand()
    cmd.load_path = 'abc'
    mocked.emit = mocker.MagicMock()
    mocked.handlers[Cmd.ROBOT_COMMAND](pickle.dumps(cmd))
    mocked.emit.assert_called_once_with(Cmd.READ_PATH_KEY, 'abc')

    # should be able to load path.
    mocked.handlers[Cmd.READ_KEY_REPLY](['path_name', pickle.dumps([])])
    assert main_process.acorn.loaded_path_name == 'path_name'
    assert main_process.acorn.loaded_path == []

    mocked.emit = mocker.MagicMock()
    wifi = mocker.patch('wifi.Wifi', **{'return_value.collect.return_value': (1, '', 1)})
    voltage = mocker.patch('voltage_monitor.VoltageSampler', **{'return_value.read.return_value': (1, 2, 3, 4)})
    stop_signal = mp.Event()

    main_thread = threading.Thread(target=lambda: main_process.run(stop_signal, frequency=10), daemon=True)
    main_thread.start()
    time.sleep(1)
    stop_signal.set()
    main_thread.join()
    wifi.assert_called()
    voltage.assert_called()
    # make sure some update calls are made
    mocked.emit.assert_called_with(Cmd.UPDATE_ROBOT, [main_process.acorn.key, mocker.ANY])

    for proc in psutil.process_iter():
        assert proc.ppid != os.getpid() or proc.pid == os.getpid(), "all sub-processes should have been terminated"
