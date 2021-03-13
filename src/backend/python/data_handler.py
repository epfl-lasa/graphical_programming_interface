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

import numpy as np

class DataHandler():
    ''' The datahandler manages the file-management and storage. 
    Backend handler which interacts with file-system '''
    def __init__(self, local_path='.'):
        self.data_directory = os.path.join(local_path, '..', 'userdata', 'projects')
        self.module_directory = os.path.join(local_path, 'module_library')

        # TODO: make sure this is always properly synchronized..
        self.project_name = 'default'

    #############################
    #    Module data handling
    ##############################
    def save_to_module_database(self, module_id, my_data):
        ''' Save the data of a module. '''
        recordings_path = self.get_recordings_path(module_id=module_id)

        str_header = 'pos[x], pos[y], pos[z], quat[w], quat[x], quat[y], quat[z]'
        
        np_data = np.zeros((7, len(my_data)))
        # Transfer from Pose to 'json'
        for ii in range(len(my_data)):
            np_data[:, ii] = [
                my_data[ii].position.x,
                my_data[ii].position.y,
                my_data[ii].position.z,
                my_data[ii].orientation.w,
                my_data[ii].orientation.x,
                my_data[ii].orientation.y,
                my_data[ii].orientation.z
            ]
        
        date_str = datetime.datetime.now().strftime("%Y%d%m_%H%M%S")
        my_file_name = 'recording_' + date_str + '.csv'
        file_path = os.path.join(recordings_path, my_file_name)

        np.savetxt(file_path, np_data.T, header=str_header, delimiter=',')
            
        return self.get_module_database_list(module_id)
        # Try to create file_path if it does not exist

    def load_from_module_database(self, module_id, file_name=None, data_it=None):
        ''' Load from data of a recording of one specific module. '''
        module_path = self.get_recordings_path(module_id, try_to_create_dir=False)
        
        if file_name is None:
            # By default take the last (recorded) element
            local_library_list = os.listdir(module_path)
            file_name = local_library_list[-1]

        file_path = os.path.join(module_path, file_name)
        with open(file_path, "r") as file:
            first_line = file.readline()

        # Remove comment from string
        while first_line[0] == '#' or first_line[0] == ' ':
            first_line = first_line[1:]
        if first_line[-1:] == '\n':
            first_line = first_line[:-1]

        # Skip header row
        file_data = np.loadtxt(file_path, skiprows=1, delimiter=',')

        if data_it is not None:
            file_data = file_data[data_it, :]
             # Recreate 2D array
            file_data = np.reshape(file_data, (1, -1))

        return first_line, file_data

    def delete_module_database(self, module_id, file_name):
        ''' Delete the stored file '''
        if file_name == 'ALL':
            print("TODO: delete directory / ")
            return

        module_path = self.get_recordings_path(module_id, try_to_create_dir=False)
        # print(module_path)
        # import pdb; pdb.set_trace()
        os.remove(os.path.join(module_path, file_name))
        
        return self.get_module_database_list(module_id)

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

    #############################################
    #    Saving, loading & listing of projects
    #############################################
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

    ##############################################
    #   Initial set up & load environment
    ##############################################
    def get_libraries_and_modules(self, library_name=None):
        ''' Import libraries and modules from directory file. '''
        module_libraries = {}
        # Import all js files
        local_library_list = os.listdir(self.module_directory)
        # print('@data_handler -- project list')
        # print(local_library_list)
        module_content = []

        for library in local_library_list:
            if (library_name is not None) and not (library == library_name):
                warnings.warn('Only doing it for {} right now... Debugging'.format(library_name))
                continue

            if library[0] == '_':
                continue
            
            lib_dir = os.path.join(self.module_directory, library)
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
        print('What is the difference between this & load from file')
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
