#!/bin/bash
echo $PWD
echo Sleep 3...
sleep 3
echo Start Sessions...
# python3 /home/acorn/bringup/serial/set_baud.py
# sleep 1
# python3 /home/acorn/bringup/serial/set_baud.py
# sleep 1

tmux new-session -d -s "gps_grid" python3 /home/acorn/farm_extras/rtk_gps_grid.py&
tmux new-session -d -s "rtcm" python3 /home/acorn/bringup/serial/rtcm_relay_gps_grid.py&
echo Started. Begin infinite loop.
trap : TERM INT; (while true; do sleep 1000; done) & wait
