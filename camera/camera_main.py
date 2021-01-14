import multiprocessing as mp
import subprocess as sp
import frame_control
import hid_trigger
import time
import signal

_VIDEO_COMMAND = "bash video_command4k.sh"


def run_camera_process():

    frame_child_conn, frame_parent_conn = mp.Pipe()
    frame_proc = mp.Process(target=frame_control.run_frame_control, args=(frame_child_conn,))
    frame_proc.start()

    hid_proc = mp.Process(target=hid_trigger.run_hid_process, args=())
    hid_proc.start()

    shutter_period = int(1000/9 + 1)
    # frame_parent_conn.send(shutter_period)

    time.sleep(10)

    frame_parent_conn.send(2000)

    proc = sp.Popen(_VIDEO_COMMAND, shell=True)
    time.sleep(5)
    frame_parent_conn.send(200)
    time.sleep(2)
    frame_parent_conn.send(shutter_period)
    time.sleep(30)
    proc.send_signal(signal.SIGINT) ## Send interrupt signal


if __name__ == '__main__':
    run_camera_process()
