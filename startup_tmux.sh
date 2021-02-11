#/usr/bin/bash

tmux new-session -s "Front End - AICA" -d

tmux rename-window -t 0 'backend'
tmux send-keys "cd frontend; npm run dev" C-m

# tmux split-window -h -t $session:1 -n 'front-end'
tmux new-window -t $session:1 -n 'frontend'
tmux send-keys "FLASK_APP=run.py flask run" C-m

tmux new-window -t $session:2 -n 'editor'
tmux send-keys "emacs run.py" C-m

# TODO: in one window?
# tmux split-window -h
# tmux split-window -v
tmux -2 attach-session -d 
