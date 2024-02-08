#!/bin/bash
echo $PWD
echo Sleep 5...
sleep 5
echo Start Sessions...
tmux new-session -d -s "main" bash /home/acorn/vehicle/start_main.sh&
tmux new-session -d -s "motors" bash /home/acorn/vehicle/start_motors.sh&
tmux new-session -d -s "leds" python3 /home/acorn/vehicle/led_control.py&
tmux new-session -d -s "rtcm" bash /home/acorn/bringup/serial/rtcm_bash.sh&
sleep 60
sudo ntpdate -s time.nist.gov
echo Started. Begin infinite loop.
trap : TERM INT; (while true; do sleep 1000; done) & wait
