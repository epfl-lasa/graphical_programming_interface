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
        self.module_directory = os.path.join(local_path, 'module_library')

        print('@data_hanlder: Create DataHandler')
        # TODO: make sure this is always properly synchronized..
        self.project_name = 'default'
        
    def load_file(self, module_id, file_name):
        ''' Load data from  module. '''
        pass
    
    def store_module_database(self, module_id, my_data):
        ''' Store the data of a module. '''
        recording_path = self.get_recordings_path(module_id=module_id)

        date_str = datetime.datetime.now().strftime("%Y%d%m_%H%M%S")
        my_file_name = 'recording_' + date_str + '.json'
        file_path = os.path.join(recording_path, my_file_name)
        print(file_path)
        # import pdb; pdb.set_trace()

        with open(file_path, 'w') as ff:
            # f.write(scene)
            # TODO: maybe use numpy for this...
            json.dump(my_data, ff)

        return self.get_module_database_list(module_id)
        # Try to create file_path if it does not exist

    def delete_module_database(self, module_id, file_name):
        ''' Delete the stored file '''
        if file_name =='ALL':
            print("TODO: delete directory")
            return

        module_path = self.get_recordings_path(module_id, try_to_create_dir=False)
        # print(module_path)
        # import pdb; pdb.set_trace()
        os.remove(os.path.join(module_path, file_name))
        
        return self.get_module_database_list(module_id)
    
    def replay_module_database(self, module_id):
        # TODO: replay a specific module
        pass

    def get_module_database_list(self, module_id):
        ''' Get the database from a module.'''
        path = self.get_recordings_path(module_id, try_to_create_dir=True)
        
        local_library_list = os.listdir(path)

        file_data = []
        id_it = 1
        for filename in local_library_list:
            # TODO: check if works!

            # Get time and Date
            statbuf = os.stat(os.path.join(path, filename))
            datemodified = datetime.datetime.fromtimestamp(statbuf.st_mtime)
            # file_data[-1]['datemodified'] = datemodified.strftime("%m/%d/%Y, %H:%M:%S")
            file_data.append({
                'name': filename,
                # 'datemodified': datemodified.strftime("%m/%d"),
                'datemodified': datemodified.strftime("%H:%M:%S - %m/%d/%y"),
                'id': id_it,
            })
            id_it += 1
            
        return {'moduledatabase': file_data}

    def get_recordings_path(self, module_id, try_to_create_dir=True):
        ''' Get the path where all the recordings are stored '''
        if self.project_name is None:
            warnings.warn('Store file once before recording data')
            # TODO: directly create path at module-creation
            
        recordings_path = os.path.join(
            self.data_directory, self.project_name, 'recordings', str(module_id))

        if try_to_create_dir:
            try:
                os.makedirs(recordings_path)
            except FileExistsError:
                pass
                # print('Directory already exists')

        return recordings_path

    def get_relative_filename(self, my_filename, create_dir=False):
        ''' Check correct file ending and return realtive directory name. '''
        if my_filename[-5:] == '.json':
            warnings.warn('Old naming convention')
            my_filename = my_filename[-5:]
            
        # if not my_filename[-5:] == '.json':
            # my_filename = my_filename + '.json'
        file_path = os.path.join(self.data_directory,  my_filename)
        if create_dir:
            try:
                os.makedirs(file_path)
            except FileExistsError:
                print('Directory already exists')
            
        return os.path.join(file_path, 'main.json')

    def save_to_file(self, my_data, my_filename=None):
        ''' Save project to file. ''' 
        if my_filename is not None:
            self.project_name = my_filename
            
        my_data_file = self.get_relative_filename(self.project_name, create_dir=True)

        with open(my_data_file, 'w') as ff:
            json.dump(my_data, ff)
            # yaml.dump(my_data, ff)
        return 0
    
    def load_from_file(self, my_filename=None):
        ''' Load new Project. '''
        if my_filename is not None:
            self.project_name = my_filename
            
        print('@load_from_file', self.project_name)
        
        my_data_file = self.get_relative_filename(self.project_name)
    
        with open(my_data_file, 'r') as ff:
            my_data = ff.read()
            my_data = json.loads(my_data)

            # my_data = yaml.load(ff, Loader=yaml.FullLoader)

        return my_data

    def get_file_list(self):
        ''' Get the list of projects which are stored in the userdata-directory.'''
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
        return {'localfiles': file_data}


    def get_libraries_and_modules(self, library_name=None):
        ''' Import libraries and modules from directory file. '''
        module_libraries = {}

        # Import all js files
        local_library_list = os.listdir('module_library')
        print(local_library_list)

        print('@data_handler -- project list')
        module_content = []

        for library in local_library_list:
            if (library_name is not None) and not (library == library_name):
                # TODO: remove after debugging
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
                module_description = os.path.join(lib_dir, module, 'description.yaml')
                if not os.path.isfile(module_description):
                    continue
                module_libraries[library].append(module)

                with open(module_description, 'r') as ff:
                    data = yaml.load(ff, Loader=yaml.FullLoader)

                data['type'] = module
                data['library'] = library

                module_content.append(data)
        # print('module_libraries', module_libraries)
        # print('module_content', module_content)
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
