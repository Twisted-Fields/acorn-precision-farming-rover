sleep 5
source /home/pi/.bashrc
PATH=/home/pi/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games
tmux  pipe-pane -o 'cat >>/home/pi/logs/tmux_output_$(date +"%Y_%m_%d_%I_%M_%p").#S:#I-#P'
sudo python3 /home/pi/vehicle/master_process.py
