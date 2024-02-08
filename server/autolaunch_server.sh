echo $PWD
tmux new-session -d -s "redis" sh redis-command.sh&
tmux new-session -d -s "zmq" python3 zmq_server_pirate.py&
tmux new-session -d -s "zmq_ppq" python3 zmq_ppqueue.py&
# tmux new-session -d -s "web" gunicorn -c gunicorn_config.py gunicorn_main:app&
tmux new-session -d -s "web" bash flask_server_bash.sh&
sleep 5 # wait for Redis to start up
tmux new-session -d -s "system_manager" python3 system_manager.py&
trap : TERM INT; (while true; do sleep 1000; done) & wait
