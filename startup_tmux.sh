#/usr/bin/bash

session="aica_frontend"

tmux new-session -d -s $session

tmux rename-window -t 0 'backend'
sleep 1.0
tmux send-keys "FLASK_APP=src/run.py flask run" C-m

# tmux split-window -h -t $session
# tmux split-window -h -t $session:1 -n 'frontend'
tmux new-window -t $session:1 -n 'frontend'
tmux send-keys "cd frontend; npm run dev" C-m

# tmux split-window -h -t $session
tmux split-window -t $session:2 -n 'editor'
# tmux new-window -t $session:2 -n 'editor'
tmux send-keys "emacs run.py &" C-m

# TODO: in one window?
# tmux split-window -h
# tmux split-window -v

# Tile & activate
# tmux select-layout -t $session tiled
tmux -2 attach-session -d 
