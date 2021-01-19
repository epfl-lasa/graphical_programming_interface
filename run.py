'''
AICA user interface launcher

(c) AICA SarL
'''
__author__ = "Lukas, Gustav"
__email__ = "lukas@aica.tech"

import os
import warnings
import copy
import json # use yaml for data-exchange
import yaml # use yaml for configuration 
import time
import datetime
from random import *

# To import
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import requests

app = Flask(__name__,
            static_folder = "./dist/static",
            template_folder = "./dist")

# Everything allowed for api (reduced security)
# cors = CORS(app, resources={r"/api/*": {"origins": "*"}})
# cors = CORS(app, resources={r"/": {"origins": "*"}})
cors = CORS(app)

data_directory = os.path.join('backend', 'userdata')


@app.route('/api/random')
def random_number():
    # import pdb; pdb.set_trace()
    response = {
        'randomNumber': randint(1, 100)
    }
    print('Get random from python...')
    return jsonify(response)


@app.route('/test')
def test():
    # print('test succesfull --- Timestamp:{}'.format(time.time()))
    # import pdb; pdb.set_trace()
    # return 1
    # response = {'State' : 1}
    # return jsonify(response)
    # return
    return render_template("test.html")

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


@app.route('/stopmotion')
def stopmotion(*args, **kwargs):
    print('Send stop command to robot')
    return 'Success'


@app.route('/savetofile/<string:my_filename>')
def savetofile(my_filename, *args, **kwargs):
    print('Data for storage recieved.')
    
    try:
        scene = request.args.get('scene')
        blockContent = request.args.get('blockContent')
        # import pdb; pdb.set_trace()
        # Transform to dict
        data = {}
        data['scene'] = json.loads(scene)
        data['blockContent'] = json.loads(blockContent)
        
    except:
        print('Could not store data')
        return 'Fail.'

    my_data_file = get_relative_filename(my_filename)
    with open(my_data_file, 'w') as f:
        # f.write(scene)
        json.dump(data, f)

    print('Successfully saved data to file.')
    
    return 'Success.'

@app.route('/loadfromfile/<string:my_filename>')
def loadfromfile(my_filename):
    print('Backend recieved loading request.')

    my_data_file = get_relative_filename(my_filename)
    
    with open(my_data_file, 'r') as f:
        data = f.read()

    # Transform to dict (?)
    data = json.loads(data)
    if not 'scene' in data:
        data = {'scene': data}
    # import pdb; pdb.set_trace()
    
    return data


@app.route('/getfilelist')
def getfilelist():
    ''' Get a list of filenames from a directory. '''

    print('get file list')
    local_library_list = os.listdir(data_directory)

    file_data = []
    for filename in local_library_list:
        # TODO: check if works!
        file_data.append({'name': filename})

        # Get time and Date
        statbuf = os.stat( os.path.join(data_directory, filename) )
        datemodified = datetime.datetime.fromtimestamp(statbuf.st_mtime)
        file_data[-1]['datemodified'] = datemodified.strftime("%m/%d/%Y, %H:%M:%S")

        # Get filetypex
        if os.path.isdir(os.path.join(data_directory, filename)):
            file_data[-1]['type'] = 'dir'
        else:
            file_data[-1]['type'] = 'file'

    # print('Get data')
    # print(file_data)
    return {'localfiles': file_data}


def get_relative_filename(my_filename):
    ''' Check correct file ending and return realtive directory name. '''
    if not my_filename[-5:]=='.json':
        my_filename = my_filename + '.json'

    return os.path.join(data_directory,  my_filename)


@app.route('/getlibrariesandmodules')
def get_libraries_and_modules():
    module_libraries = {}
    
    # Import all js files
    local_library_list = os.listdir('module_library')
    print(local_library_list)

    module_content = []

    for library in local_library_list:
        if library != 'basic':
            warnings.warn('Only doing it for basic right now... Debugging')
            continue
        if library[0]=='_':
            continue
        lib_dir = os.path.join('module_library', library)
        if not os.path.isdir(lib_dir):
            continue
        local_module_list = os.listdir(lib_dir)

        module_libraries[library] = []
        
        for module in local_module_list:
            if module[0]=='_':
                continue
            module_description = os.path.join(lib_dir, module, 'description.json')
            if not os.path.isfile(module_description):
                continue
            module_libraries[library].append(module)

            with open(module_description, 'r') as f:
                data = f.read()
            data = json.loads(data)

            module_content.append(data)
    print('module_libraries', module_content)
    print('module_content', module_content)
    return {'moduleLibraries': module_libraries, 'blockContent': module_content}
    
# @app.route('/getlibrariesandmodules')

@app.route('/home')
def get_scene():
    print('going home in pythonx')
    libs_and_mods = {}
    
    # Import all js files
    local_library_list = os.listdir('module_library')
    print(local_library_list)

    for library in local_library_list:
        lib_dir = os.path.join('module_library', library)
        if not os.path.isdir(lib_dir) or library[0]=='_':
            continue
        local_module_list = os.listdir(lib_dir)

        libs_and_mods[library] = []
        
        for module in local_module_list:
            mod_dir = os.path.join(lib_dir, module)
            if not os.path.isdir(lib_dir) or library[0]=='_':
                continue
            libs_and_mods[library].append(module)

    # print('Libraries')
    # print(libs_and_mods)

    # Todo - maybe order ditcionary alphabetically
    # if os.path.isfile('main.js'):
        # pass
        
    return libs_and_mods


@app.route('/', defaults={'path': ''})
@app.route('/<path:path>')
def catch_all(path):
    if app.debug:
        return requests.get('http://localhost:8080/{}'.format(path)).text
    return render_template("index.html")


if __name__ == "__main__":
# if True:
    # TODO: remove for production
    app.run(debug=True)

    print('start flask')
    # Start ROS?

