source /home/acorn/.bashrc
PATH=/home/acorn/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games
tmux  pipe-pane -o 'cat >>/home/acorn/logs/tmux_output_$(date +"%Y_%m_%d_%I_%M_%p").#S:#I-#P'
python3 /home/acorn/vehicle/motors.py $@
