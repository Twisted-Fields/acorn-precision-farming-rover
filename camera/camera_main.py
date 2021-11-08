"""
*********************************************************************
                     This file is part of:
                       The Acorn Project
             https://wwww.twistedfields.com/research
*********************************************************************
Copyright (c) 2019-2021 Taylor Alexander, Twisted Fields LLC
Copyright (c) 2021 The Acorn Project contributors (cf. AUTHORS.md).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*********************************************************************
"""
import multiprocessing as mp
import subprocess as sp
import frame_control
import hid_trigger
import time
import signal

_VIDEO_COMMAND = "bash video_command4k.sh"
_FRAME_RATE = 14

# _VIDEO_COMMAND = "bash video_command_13mp.sh"
# _FRAME_RATE = 9


def run_camera_process():

    frame_child_conn, frame_parent_conn = mp.Pipe()
    frame_proc = mp.Process(
        target=frame_control.run_frame_control, args=(frame_child_conn,))
    frame_proc.start()

    hid_proc = mp.Process(target=hid_trigger.run_hid_process, args=())
    hid_proc.start()

    shutter_period = int(1000/_FRAME_RATE + 1)
    # frame_parent_conn.send(shutter_period)

    time.sleep(10)

    frame_parent_conn.send(2000)

    proc = sp.Popen(_VIDEO_COMMAND, shell=True)
    time.sleep(5)
    frame_parent_conn.send(200)
    time.sleep(2)
    frame_parent_conn.send(shutter_period)
    # time.sleep(30)
    # proc.send_signal(signal.SIGINT) ## Send interrupt signal


if __name__ == '__main__':
    run_camera_process()
