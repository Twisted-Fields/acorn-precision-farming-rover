#!/bin/bash
echo $PWD
echo Sleep 5...
sleep 5
echo Start Sessions...
tmux new-session -d -s "main" bash /home/pi/vehicle/start_main.sh&
tmux new-session -d -s "motors" bash /home/pi/vehicle/start_motors.sh&
tmux new-session -d -s "leds" python3 /home/pi/vehicle/led_control.py&
tmux new-session -d -s "rtcm" python3 /home/pi/bringup/serial/rtcm_relay.py&
echo Started. Begin infinite loop.
trap : TERM INT; (while true; do sleep 1000; done) & wait
