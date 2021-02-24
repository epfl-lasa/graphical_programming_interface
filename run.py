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

@app.route('/test')
def test():
    print('Test is done')
    return render_template("test.html")


# Update Environment
@app.route('/updateenvironment')
def update_environment():
    pass

# Update Environment
@app.route('/update_block/<int:id>')
def update_bock():
    pass


@app.route('/movetoposition')
def move_to_start(*args, **kwargs):
    # Start / end position
    pass


# Handle data here
@app.route('/record_data/<int:id>')
def record_data():
    datetime.datetime

@app.route('/moveto')
def move_to(*args, **kwargs):
    try:
        euler_pose = request.args.get('eulerPose')
        print('Send start command to robot')
        print(euler_pose)
    except:
        print('Could not retrieve data')
        return 'Fail'
    
    return 'Success'

@app.route('/emergencystop')
@app.route('/stopmotion')
def stopmotion(*args, **kwargs):
    RosMainHandler.update_scene(stopmotion)
    
    print('Send stop command to robot')
    return 'Success'

@app.route('/emergencystop')
def updatemodule(*args, **kwargs):
    pass


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

    DataLocalHandler.save_to_file(my_filename, data)
    
    print('Successfully saved data to file.')
    
    return '0: saving succesffull.'

@app.route('/loadfromfile/<string:my_filename>')
def loadfromfile(my_filename):
    # TODO: from json to yaml!
    
    print('Backend recieved loading request.')

    data = DataLocalHandler.load_from_file(my_filename)
    
    # Transform to dict (?)
    if not 'scene' in data:
        data = {'scene': data}

    return data

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

@app.route('/getfilelist')
def getfilelist():
    ''' Get a list of filenames from a directory. '''
    file_data = DataLocalHandler.get_file_list()

    return {'localfiles': file_data}


@app.route('/getlibrariesandmodules')
def getlibrariesandmodules():
    return DataLocalHandler.get_libraries_and_modules(library_name='polishing_machine')

# @app.route('/loadiconpathsanddescription/<string:my_library>')
# def loadiconpathsanddescription(my_library):
#     return DataLocalHandler.load_module_descriptions(my_library)

@app.route('/home')
def get_scene():
    return DataLocalHandler.get_scene()
    

@app.route('/', defaults={'path': ''})
@app.route('/<path:path>')
def catch_all(path):
    if app.debug:
        return requests.get('http://localhost:8080/{}'.format(path)).text
    return render_template("index.html")


if __name__ == "__main__":
    app.run(debug=True)

    print('start flask')

