echo $PWD
tmux new-session -d -s "redis" sh redis-command.sh&
tmux new-session -d -s "zmq" python3 zmq_daemon.py&
tmux new-session -d -s "web" python3 server.py&
trap : TERM INT; (while true; do sleep 1000; done) & wait
