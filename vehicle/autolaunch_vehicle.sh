echo $PWD
tmux new-session -d -s "main" sh /home/pi/vehicle/start_main.sh&
tmux new-session -d -s "vehicle" sh /home/pi/vehicle/start_motors.sh&
trap : TERM INT; (while true; do sleep 1000; done) & wait
