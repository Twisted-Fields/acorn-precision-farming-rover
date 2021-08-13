echo $PWD
echo Sleep 5...
sleep 5
tar -xvf /home/pi/bringup/rtk_bin_docker.tar -C /
echo Start Sessions...
tmux new-session -d -s "main" sh /home/pi/vehicle/start_main.sh&
tmux new-session -d -s "motors" sh /home/pi/vehicle/start_motors.sh&
echo Started. Begin infinite loop.
trap : TERM INT; (while true; do sleep 1000; done) & wait
