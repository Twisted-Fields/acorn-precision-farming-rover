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

if __name__=="__main__":
    pass
