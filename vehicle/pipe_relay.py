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

import socket
import subprocess
import time
import os
import math

_FAST_POLLING_DELAY_S = 0.05


"""
Buffer motor controller pipe data.

"""


def relay_data(master_conn, motor_conn):

    motor_data = []
    while True:
        while motor_conn.poll():
            motor_data.append(motor_conn.recv())

        if len(motor_data) > 0:
            master_conn.send(motor_data.pop(0))

        while master_conn.poll():
            motor_conn.send(master_conn.recv())

        time.sleep(0.001)


if __name__ == "__main__":
    pass
