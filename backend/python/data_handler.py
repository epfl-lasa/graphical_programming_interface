'''
AICA user data handler

(c) AICA SarL
'''

__author__ = "Lukas Huber"
__email__ = "lukas@aica.tech"

import os
import sys
import datetime

import warnings

import yaml
import json

 # Backend handler which interacts with file-system

class DataHandler():
    ''' The datahandler manages the file-management and storage. '''
    def __init__(self, local_path='.'):
            
        self.data_directory = os.path.join(local_path, 'backend', 'userdata', 'projects')
        
    def load_file(self):
        pass

    def get_relative_filename(self, my_filename):
        ''' Check correct file ending and return realtive directory name. '''
        if my_filename[-5:] == '.json':
            warnings.warn('Old naming convention')
            my_filename = my_filename[-5:]
            
        # if not my_filename[-5:] == '.json':
            # my_filename = my_filename + '.json'
            
        return os.path.join(self.data_directory,  my_filename, 'main.json')
    

    def save_to_file(self, my_filename, my_data):
        my_data_file = self.get_relative_filename(my_filename)

        with open(my_data_file, 'w') as ff:
            # f.write(scene)
            json.dump(my_data, ff)
            # yaml.dump(data, f)

        import pdb; pdb.set_trace()
        return 0

    
    def load_from_file(self, my_filename):
        print('@load_from_file', my_filename)
        
        my_data_file = self.get_relative_filename(my_filename)
    
        with open(my_data_file, 'r') as ff:
            my_data = ff.read()

        my_data = json.loads(my_data)

        return my_data
    
    
    def get_file_list(self):
        local_library_list = os.listdir(self.data_directory)

        file_data = []
        for filename in local_library_list:
            # TODO: check if works!
            file_data.append({'name': filename})

            # Get time and Date
            statbuf = os.stat(os.path.join(self.data_directory, filename))
            datemodified = datetime.datetime.fromtimestamp(statbuf.st_mtime)
            file_data[-1]['datemodified'] = datemodified.strftime("%m/%d/%Y, %H:%M:%S")

            # Get filetypex
            if os.path.isdir(os.path.join(self.data_directory, filename)):
                file_data[-1]['type'] = 'dir'
            else:
                file_data[-1]['type'] = 'file'
        return file_data

    def get_libraries_and_modules(self):
        module_libraries = {}

        # Import all js files
        local_library_list = os.listdir('module_library')
        print(local_library_list)

        module_content = []

        for library in local_library_list:
            if library != 'basic':
                warnings.warn('Only doing it for basic right now... Debugging')
                continue
            if library[0] == '_':
                continue
            lib_dir = os.path.join('module_library', library)
            if not os.path.isdir(lib_dir):
                continue
            local_module_list = os.listdir(lib_dir)

            module_libraries[library] = []

            for module in local_module_list:
                if module[0] == '_':
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

    def get_scene(self):
        print('going home in python')
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

        return libs_and_mods

        
