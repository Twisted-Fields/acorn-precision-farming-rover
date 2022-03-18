#!/bin/sh

echo $PWD
echo Sleep 5...
sleep 5
echo Start Sessions...
tmux new-session -d -s "main" sh /home/pi/vehicle/start_main.sh --sim &
tmux new-session -d -s "motors" sh /home/pi/vehicle/start_motors.sh --simulated_hardware &
echo Started. Begin infinite loop.
trap : TERM INT; (while true; do sleep 1000; done) & wait
