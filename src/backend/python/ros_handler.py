'''
AICA user data handler

(c) AICA SarL
'''

__author__ = "Lukas Huber"
__email__ = "lukas@aica.tech"

import os
import sys
import copy
import warnings
import time

import yaml

# import ros?
# Backend handler which interacts with file-system

class RosHandler():
    ''' The datahandler manages the ROS and sending it to the frontend. '''
    def __init__(self):
        # Do stuff
        self.emergency_stop_activated = False
        self.data_recording = True
        pass

    @property
    def emergency_stop_activated(self):
        return self._emergency_stop_activated
    
    @emergency_stop_activated.setter
    def emergency_stop_activated(self, value):
        self._emergency_stop_activated = value

    @staticmethod
    def transform_pose_to_euler(pose):
        pass

    @staticmethod
    def transform_euler_to_pose(euler):
        pass


    def relase_emergency_stop(self):
        ''' Relase emergency stop. '''
        self.emergency_stop_activated = False

    def move_to_module(self, module_id):
        ''' Move to reference point of the module (i.e. the end-point). '''
        print('TODO: move to reference point of module')

    def execute_module(self, module_id):
        ''' Run the module '''
        print('TODO: execute sequence')
        # TODO

        # TODO:
        # move to starting point
        ##  previous end point for motion
        ## same starting point for idle
        # move
        # end at end point (current end point)

    def execute_sequence(self, module_id=None):
        ''' Run sequence starting at module_id. If module_id is none.'''
        print('TODO: execute sequence')
        print('Per default start at one')
    
    def get_current_robot_joint_position(self):
        pass

    def move_to_position(self, euler_pose=None):
        print('@ros_handler: TODO --- move to position')
    
    def get_robot_position(self, pose_type=None):
        ''' Get current position of the robot. '''
        print('TODO: return current position as list.')

        self.pose_topic = ''
        self.joint_pose_topic = ''

        # DEBUG (!)
        warnings.warn('@ros_handler: DEBUG POSE')
        pose_data = {
            'type': 'eulerPose',
            'frameId': 'base_link',
            'position': {'x': 0, 'y': 1, 'z': 2},
            'orientation': {'x': 0, 'y': 1.2134, 'z': 2.234},
        }
        
        return {'pose': pose_data}
    
    def record_module_database(self, module_id, DataHandler, max_recording=1000):
        print('@Ros_handler: recording start')
        self.data_recording = True

        recorded_data = []
        it_recording = 0
        while(self.data_recording):
            it_recording += 1
            if it_recording > max_recording:
                print('Maximum iterations of recording reached')
                break

            time.sleep(0.1)
            recorded_data.append({'x': 0, 'y': 1, 'z': 2})
        self.data_recording = False
            
        print('@Ros_handler: recording stop')
        data_list = DataHandler.store_module_database(module_id, my_data=recorded_data)
        return data_list

    
    def stop_robot(self):
        ''' Emergency Stop of the Robot. '''
        self.robot_is_moving = False
        # TODO: stop the robot!

    def update_scene(self, scene):
        ''' Send data to ROS and initiate code-generation. '''
        loop_is_closed, ordered_block_list = RosHandler.get_ordered_list(scene)

        print('Successfully ordered the blocks')
        # TODO: Send to ROS & create code-generation
        if loop_is_closed:
            print('Do something')
        else:
            print('Loop not closed. No updating')
            
    @staticmethod
    def get_ordered_list(scene):
        # TODO: do this in module itself - and only send scene when succesfully closed
        ''' Simplify a scene of links & blocks to an ordered_block_list. ''' 
        link_list = copy.deepcopy(scene['links'])
        block_list = copy.deepcopy(scene['blocks'])
        
        ordered_block_list = []
        # Find start bock
        it_block = 0
        while(it_block < len(block_list)):
            block_type = block_list[it_block]['name']
            if block_type == 'idle':
                ordered_block_list.append(block_list[it_block])

                del block_list[it_block]
                break
            it_block += 1
            
        it_link = 0
        while it_link < len(link_list) and len(block_list): # Nonzero
            if link_list[it_link]['originID'] == ordered_block_list[-1]['id']:
                it_block = 0
                while it_block < len(block_list):
                    if block_list[it_block]['id'] == link_list[it_link]['targetID']:
                        ordered_block_list.append(block_list[it_block])
                        
                        del block_list[it_block]
                        del link_list[it_link]
                        break
                    else:
                        it_block += 1
            else:
                it_link += 1
            
        # Check
        loop_is_closed = True
        if len(link_list) > 1:
            warnings.warn('Not connected loops detected.')
            loop_is_closed = False
            
        elif len(link_list) == 0:
            warnings.warn('Loop is not closed.')
            loop_is_closed = False
            
        elif link_list[-1]['targetID'] != ordered_block_list[0]['id']:
            warnings.warn('Tail-loop detected.')
            loop_is_closed = False

        return loop_is_closed, ordered_block_list
    
    def update_module(self, module_id, module_data):
        # Module ID and data
        print('TODO: @Gustav --- update_module')

    def callback_stoprecording(self):
        # print('Stop recording')
        self.data_recording = False

    def callback_emergency_stop(self):
        print('EMERGENCY STOP WAS CALLED')
        self.robot_is_moving = False

    def callback_stop_robot(self):
        self.robot_is_moving = False




if (__name__)=="__main__":
    print('Do a test')
