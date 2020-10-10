# import daemon
#
# from zmq_server import main


import subprocess
import os
import sys
import psutil
import time


ZMQ_CMD = ['/usr/bin/python3','zmq_server.py']


def kill_zmq_procs():
    for proc in psutil.process_iter():
    # check whether the process name matches
        if 'zmq_server.py' in proc.cmdline():
            print(proc.cmdline())
            proc.kill()
        if 'zmq_daemon.py' in proc.cmdline() and proc.pid != os.getpid():
            print(proc.cmdline())
            proc.kill()

def launch_zmq_server():

    # Get working directory before we start detaching things.
    cwd = os.getcwd()

    kill_zmq_procs()

    # This comes from:
    # https://www.oreilly.com/library/view/python-cookbook/0596001673/ch06s08.html
    try:
        pid = os.fork()
        if pid > 0:
            # Exit first parent
            #os.waitid(os.P_PID, pid, os.WEXITED)
            return
            #sys.exit(0)
    except OSError as e:
        sys.exit(1)

    # Decouple from parent environment
    os.chdir("/")
    os.setsid()
    os.umask(0)

    # Do second fork
    try:
        pid = os.fork()
        if pid > 0:
            sys.exit(0)
    except OSError as e:
        sys.exit(1)

    print(cwd)
    proc = subprocess.Popen(ZMQ_CMD, cwd=cwd, stdin=None, stdout=None, stderr=None, close_fds=True, shell=False)

    while True:
        time.sleep(0.001)



kill_zmq_procs()
while True:
    try:
        zmq_server_running = False
        for proc in psutil.process_iter():
            # check whether the process name matches
                if 'zmq_server.py' in proc.cmdline():
                    #print(proc.cmdline())
                    zmq_server_running = True
        if not zmq_server_running:
            print("=======================")
            print("=                     =")
            print("= RUNNING ZMQ PROCESS =")
            print("=                     =")
            print("=======================")
            launch_zmq_server()
            #pass
        time.sleep(10)
    except KeyboardInterrupt:
        kill_zmq_procs()

# with daemon.DaemonContext():
#     print("======================")
#     print("=                    =")
#     print("= RUNNING ZMQ DAEMON =")
#     print("=                    =")
#     print("======================")
#
#     main() # run zmq main
#
#     print("======================")
#     print("=                    =")
#     print("= ZMQ DAEMON CRASHED =")
#     print("=                    =")
#     print("======================")
#     time.sleep(1)
