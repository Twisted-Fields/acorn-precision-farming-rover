#!/usr/bin/env python

import signal
import os
import fcntl
import sys

F_SETPIPE_SZ = 1031  # Linux 2.6.35+
F_GETPIPE_SZ = 1032  # Linux 2.6.35+

def open_fifo():
    fifo_fd, w = os.pipe()
    try:
        print("Checking fifo file ...")
        #fifo_fd = open(fifo, "rb+")
        print("Pipe size            : "+str(fcntl.fcntl(fifo_fd, F_GETPIPE_SZ)))
        fcntl.fcntl(fifo_fd, F_SETPIPE_SZ, 1000000)
        print("Pipe (modified) size : "+str(fcntl.fcntl(fifo_fd, F_GETPIPE_SZ)))
        return fifo_fd
    except Exception as e:
        print("Unable to create fifo, error: "+str(e))

fifo_fd = open_fifo()
