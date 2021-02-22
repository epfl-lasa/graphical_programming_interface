#/usr/bin/bash

## Development mode
# gnome-terminal -e 'sh -c "cd frontend; gedit && exec bash"'
# gnome-terminal -e 'sh -c "cd frontend; npm run dev; exec bash"'
gnome-terminal -e 'sh -c "cd frontend; npm run dev; exec bash"'
gnome-terminal -e 'sh -c "FLASK_APP=run.py flask run; exec bash"'

# Open code in terminal & keep terminal
gnome-terminal -e 'sh -c "emacs run.py; exec bash"'


