import zmq
import pickle
import logging
import multiprocessing as mp
from model import Robot, RobotSubset
from remote_control_process import RemoteControl, _VOLTAGE_CUTOFF, _PATH_END_PAUSE_SEC, _BEGIN_AUTONOMY_SPEED_RAMP_SEC
import time
import gps_tools
import model
from motors_can import AcornMotorInterface

GPS_POINT = gps_tools.GpsPoint(37.353766071, -122.332961493)


def test_run_once(fixture_path):

    acorn_motor_interface = AcornMotorInterface(manual_control=False, simulated_hardware=True, debug=False)
    # acorn_motor_interface.setup_shared_memory()
    p = mp.Process(target=acorn_motor_interface.run_main, args=())
    p.start()
    remote_control_manager = mp.Manager()
    lk_r2m, lk_m2r = remote_control_manager.Lock(), remote_control_manager.Lock()
    r2m, m2r = remote_control_manager.dict(), remote_control_manager.dict()
    robot_object = Robot(simulated_data=True, logger=logging.getLogger('test_run_once'))
    robot_object = RobotSubset(robot_object)
    m2r["value"] = pickle.dumps(robot_object)
    rc = RemoteControl(mp.Event(), lk_r2m, lk_m2r, r2m, m2r, logging,
                       debug=True, simulated_hardware=True)
    rc.run_setup()

    # dry run without a valid path to follow
    rc.run_single_loop()
    data = pickle.loads(r2m["value"])
    assert data is not None, "should have reported something back to the main process"


    # robot_object.loaded_path_name = "test"
    # robot_object.loaded_path = pickle.loads(fixture_path)
    # m2r["value"] = pickle.dumps(robot_object)
    # rc.run_single_loop()
    # obj = pickle.loads(socket.recv_pyobj())
    # assert obj is not None, "should have sent the commands to motor"
    # for corner in ["front_left", "front_right", "rear_left", "rear_right"]:
    #     assert obj[corner] is not None

    rc.load_path_time = time.time() - _PATH_END_PAUSE_SEC - _BEGIN_AUTONOMY_SPEED_RAMP_SEC
    rc.activate_autonomy = True
    rc.nav_path.points = [0, 0, 0, 0] # So that len > 2
    rc.voltage_average = _VOLTAGE_CUTOFF + 1
    rc.joy.throttle = 0.4
    rc.joy.steer = 0.0
    rc.joy.strafe = 0.0
    autonomy_vel_cmd = 0.5
    autonomy_steer_cmd = 0.0
    autonomy_strafe_cmd = 0.0
    zero_output = True
    # if math.fabs(self.joy.strafe) < 0.1:
    vel_cmd, _, _ = rc.calc_drive_commands(autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output)
    assert vel_cmd == 0.0
    zero_output = False
    vel_cmd, _, _ = rc.calc_drive_commands(autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output)
    assert vel_cmd == autonomy_vel_cmd
    rc.load_path_time = time.time() - _PATH_END_PAUSE_SEC - _BEGIN_AUTONOMY_SPEED_RAMP_SEC * 0.25
    vel_cmd, _, _ = rc.calc_drive_commands(autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output)
    assert vel_cmd > autonomy_vel_cmd * 0.2 and vel_cmd < autonomy_vel_cmd * 0.3
    rc.activate_autonomy = False
    vel_cmd, _, _ = rc.calc_drive_commands(autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output)
    assert vel_cmd == rc.joy.throttle
    rc.voltage_average = _VOLTAGE_CUTOFF - 1
    vel_cmd, _, _ = rc.calc_drive_commands(autonomy_vel_cmd, autonomy_steer_cmd, autonomy_strafe_cmd, zero_output)
    assert vel_cmd == 0.0


    # Reset some values for the following autonomy tests
    rc.activate_autonomy = True
    rc.voltage_average = _VOLTAGE_CUTOFF + 1
    rc.joy.throttle = 0.0

    # A straight line of points 15 meters long
    pt2 = gps_tools.project_point(GPS_POINT, 90, 3.0)
    pt3 = gps_tools.project_point(pt2, 90, 3.0)
    pt4 = gps_tools.project_point(pt3, 90, 3.0)
    pt5 = gps_tools.project_point(pt4, 90, 3.0)
    sample_path = [GPS_POINT, pt2, pt3, pt4, pt5]

    # Set up simulated navigation along the path
    robot_object.activate_autonomy = True
    robot_object.clear_autonomy_hold = True
    robot_object.last_server_communication_stamp = time.time()
    m2r["value"] = pickle.dumps(robot_object)
    rc.simulated_hardware = True
    rc.load_path(sample_path, simulation_teleport=True)
    rc.load_path_time = time.time() - _PATH_END_PAUSE_SEC - _BEGIN_AUTONOMY_SPEED_RAMP_SEC
    rc.loop_count = 0
    rc.motor_state = model.MOTOR_ENABLED
    HIT_END_POINT_1 = False
    HIT_END_POINT_2 = False


    # Run simulated autonomy
    while True:
        robot_object.last_server_communication_stamp = time.time()
        m2r["value"] = pickle.dumps(robot_object)
        rc.run_single_loop()
        rc.loop_count += 1
        # confirm the simulated robot makes it to both ends of the path
        # (it starts in the middle of the path, goes one way, then reverses)
        dist = gps_tools.get_distance(rc.gps.last_sample(), pt5)
        if dist < 2:
            HIT_END_POINT_1 = True
        dist = gps_tools.get_distance(rc.gps.last_sample(), GPS_POINT)
        if dist < 2:
            HIT_END_POINT_2 = True
        print(f"HIT_END_POINT_1: {HIT_END_POINT_1} HIT_END_POINT_2: {HIT_END_POINT_2}, loop_count {rc.loop_count}")
        if HIT_END_POINT_1 and HIT_END_POINT_2:
            break
        assert rc.loop_count < 1000
    p.terminate()
    p.join()
    assert HIT_END_POINT_1
    assert HIT_END_POINT_2
