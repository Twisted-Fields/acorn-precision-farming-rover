#!/bin/bash

while true; do
  python3 /home/acorn/bringup/serial/rtcm_relay.py &
  wait $!
done
