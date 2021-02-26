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
> Update fields of block
> Synchronize back & front-end 
> Visualize force (?)
> Send parameters / commands to module
> Filehandling: Save to file (!), save as, create new,
> Code-Generation
> Record Data / Delete Data / Learn Motion (send to file)
> Get Reference Frames (?) HOW?
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


## Run using Docker
Build the Dockerimage. In a second window, build and run the backend. In the first window, run the Dockerimage and run the following code
``` 
FLASK_APP=src/run.py flask run --host 0.0.0.0
```

## Run a new docker with id
``` bash
docker exec -it ID_ bash
```


