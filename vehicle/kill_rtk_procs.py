
import psutil
import time

for proc in psutil.process_iter():
# check whether the process name matches
    if proc.name() == 'rtkrcv':
        print(proc.cmdline())
        proc.kill()
    else:
        print(proc.name())

    time.sleep(0.001)
