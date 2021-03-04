# AICA Flowchart Frontend & Bridge
<< Simplify the interaction with robots>>

## Technologies
* Python: 3.6.3
* Vue.js: 2.5.2
* vue-router: 3.0.1
* axios: 0.16.2

@thanksto:  
- oleg-agapov / flask-vue-spa 
- ghostiam / vue-blocks 

# TODO
> 

## Structure
### Frontend
Front end is structured as a vue-one-page application. This allows deployment through standard webbroswers.
``` bash
./frontend
```

### Backend
Main python script is 'run.py'
Backend includes user data, python-tools (except main script.
``` bash
./module_library
```

### Module Library
``` bash
./module_library
```
Back & Front end is split in libraries, which are further split in modules.
This allows 
    

## Build Setup
``` bash
# install front-end
cd frontend
npm install

# serve with hot reload at localhost:8080
npm run dev

# build for production/Flask with minification
npm run build


# install back-end
cd ../backend
virtualenv -p python3 venv
source venv/bin/activate
pip install -r requirements.txt
cd ..
```

## Run Setup
Turn on debug mode
``` bash
export FLASK_DEBUG=0
```
TODO: why does debugging not work?

Set execution file
``` bash
# serve back-end at localhost:5000
FLASK_APP=run.py flask run
```

Reminder: to run front-end with hot-reload:
``` bash
# serve with hot reload at localhost:8080
cd frontend
npm run dev
```

# IP / Localhost setup
To make sure that the app is accessible from an external device, the frontend matches the IP of your device.

``` bash
cd frontend/src/main.js
```
For local development only, you can set it to 'localhost'



## Run using Docker
Build the Dockerimage. In a second window, build and run the backend. In the first window, run the Dockerimage and run the following code
``` 
FLASK_APP=src/run.py flask run --host 0.0.0.0
```

## Run a new terminal in existing docker with ID
``` bash
docker exec -it ID bash
```

## Run 'simulation' controller
``` bash
python3 /home/ros2/src/backend/python/node_controller_simulator.py
```

## Replay rosbag
``` bash
ros2 bag play /home/ros2/userdata/ros_recording/ros2_bag/rosbag2_2021_02_25-10_54_58_0.db3
```

