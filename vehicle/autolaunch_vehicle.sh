#!/bin/bash
echo $PWD
echo Sleep 5...
sleep 3
echo Start Sessions...
python3 /home/acorn/bringup/serial/set_baud.py
sleep 1
python3 /home/acorn/bringup/serial/set_baud.py
sleep 1
tmux new-session -d -s "main" bash /home/acorn/vehicle/start_main.sh&
tmux new-session -d -s "motors" bash /home/acorn/vehicle/start_motors.sh&
tmux new-session -d -s "leds" python3 /home/acorn/vehicle/led_control.py&
tmux new-session -d -s "rtcm" python3 /home/acorn/bringup/serial/rtcm_relay.py&
echo Started. Begin infinite loop.
trap : TERM INT; (while true; do sleep 1000; done) & wait
