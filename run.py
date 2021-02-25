'''
AICA user interface launcher

(c) AICA SarL
'''
__author__ = "Lukas, Gustav"
__email__ = "lukas@aica.tech"

import os
import sys
import warnings
import copy
import json # use json for data-exchange
import yaml # use yaml for configuration 
import time
import datetime
from random import *

# To import
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import requests

# Import custom libraries from path (TODO: install locally)
dir_path = os.path.dirname(os.path.realpath(__file__))
path_communication_handler = os.path.join(dir_path, "backend", "python")
if not path_communication_handler in sys.path:
    sys.path.append(path_communication_handler)

from data_handler import DataHandler
DataLocalHandler = DataHandler(dir_path)

from ros_handler import RosHandler
RosMainHandler = RosHandler()

app = Flask(__name__,
            static_folder="./dist/static",
            template_folder="./dist")

# Everything allowed for api (reduced security)
# cors = CORS(app, resources={r"/api/*": {"origins": "*"}})
# cors = CORS(app, resources={r"/": {"origins": "*"}})
cors = CORS(app)

data_directory = os.path.join('backend', 'userdata')

IMPORT_COMMUNACTION_CONTROLLER = False

# ----------------------------------------
#    Execute this at the startup
# ----------------------------------------
@app.route('/startup')
def startup():
    ''' Startup synchonization between back & front-end. '''
    print('Startup')
    return 0


# ----------------------------------------
#     Main Robot Handler Module-Parts
# ---------------------------------------
@app.route('/emergencystop')
def emergencystop():
    RosMainHandler.callback_emergency_stop()
    return '0'

@app.route('/stoprobot')
def stoprobot():
    RosMainHandler.callback_stop_robot()
    return '0'

@app.route('/movetomodulestart/<int:module_id>')
def movetomodulestart(module_id):
    status = RosMainHandler.move_to_module_start(module_id=module_id)
    return '0: Success'

@app.route('/movetoposition')
def movetoposition(*args, **kwargs):
    try:
        euler_pose = request.args.get('eulerPose')
    except:
        print('Could not retrieve data')
        return '202: Failed retrieving data.'
    # Start / end position
    status = RosMainHandler.move_to_position(euler_pose=euler_pose)

    return '0: Success'

@app.route('/executemodule/<int:module_id>')
def executemodule(module_id):
    # Start / end position
    status = RosMainHandler.execute_module(module_id=module_id)
    return '0: Success'

@app.route('/executesequence/<int:module_id>')
def executesequence(module_id):
    # Start / end position
    status = RosMainHandler.execute_sequence(module_id=module_id)
    return '0: Success'

@app.route('/getrobotposition')
def getcurrentrobotposition():
    pose_data = RosMainHandler.get_robot_position(pose_type='eulerpose')

    print('pose_data', pose_data)
    return pose_data


# ----------------------------------------
#   Record Data for a specific Module
# ----------------------------------------
@app.route('/recordmoduledatabase/<int:my_id>')
def recordmoduledatabase(my_id):
    # print('@run: Start recording')
    data_list = RosMainHandler.record_module_database(my_id, DataLocalHandler)
    return data_list

@app.route('/replaymoduledatabase/<int:my_id>/<string:my_filename>')
def replay(my_id, my_filename):
    RosMainHandler.replay_module_database(module_id=my_id, file_name=my_filename)
    return '0: replay succesfuly finished'

@app.route('/stoprecording')
def stoprecording():
    RosMainHandler.callback_stoprecording()
    return '0: Stopping executed without problems.'

@app.route('/deletemdouledatabase/<int:my_id>/<string:my_filename>')
def deletemdouledatabase(my_id, my_filename):
    data_list = DataLocalHandler.delete_module_database(module_id=my_id, file_name=my_filename)
    return data_list

@app.route('/getdataofmodule/<int:my_id>')
def getdataofmodule(my_id):
    data = DataLocalHandler.get_module_database_list(my_id)
    return data



# ---------------------------------------- 
#   Update Code Generation
# ----------------------------------------
@app.route('/updatebackend')
def updatebackend(*args, **kwargs):
    ''' Update ros backend'''
    print('Updating backend.')

    try:
        scene = request.args.get('scene')
        blockContent = request.args.get('blockContent')
        scene = json.loads(scene)
        
    except:
        print('Could not store data')
        # Error no succesffull transmission of data
        return '202: Could not transfer data' 

    statusMessage = RosMainHandler.update_scene(scene)
    
    return '0: Transfer successful'

@app.route('/updatemodule')
def saveproperty(*args, **kwargs):
    try:
        module_id = request.args.get('module_id')
        module_data = request.args.get('module_data')
        
    except:
        print('Could not store data')
        # Error no succesffull transmission of data
        return '202: Could not transfer data' 

    import pdb; pdb.set_trace()
    RosMainHandler.update_module(module_id, module_data)

    return '0: No error occured'


# ---------------------------------------- 
#   Save & Retrieve data from backend
# ----------------------------------------
@app.route('/savetofile/<string:my_filename>')
def savetofile(my_filename, *args, **kwargs):
    ''' Save data to file. '''
    print('Data for storage recieved.')
    
    try:
        scene = request.args.get('scene')
        blockContent = request.args.get('blockContent')
        # Transform to dict
        data = {}
        data['scene'] = json.loads(scene)
        data['blockContent'] = json.loads(blockContent)
        
    except:
        print('Could not store data')
        return '202: Could not store data.'

    DataLocalHandler.save_to_file(my_data=data, my_filename=my_filename)
    
    print('Successfully saved data to file.')
    
    return '0: saving succesffull.'

@app.route('/loadfromfile/<string:my_filename>')
def loadfromfile(my_filename):
    print('Backend recieved loading request.')
    
    data = DataLocalHandler.load_from_file(my_filename)
    
    # Transform to dict (?)
    if not 'scene' in data:
        data = {'scene': data}

    return data

@app.route('/getfilelist')
def getfilelist():
    ''' Get a list of filenames from a directory. '''
    data = DataLocalHandler.get_file_list()
    return data

@app.route('/getlibrariesandmodules')
def getlibrariesandmodules():
    warnings.warn('For demonstration purposes, only one library is shared')
    return DataLocalHandler.get_libraries_and_modules(library_name='polishing_machine')

@app.route('/home')
def home():
    return DataLocalHandler.get_scene()
    
@app.route('/', defaults={'path': ''})
@app.route('/<path:path>')
def catch_them_all(path):
    if app.debug:
        return requests.get('http://localhost:8080/{}'.format(path)).text
    return render_template("index.html")


if __name__ == "__main__":
    app.run(debug=True)

    print('Starting Flask')

