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

import yaml

# import ros?

# Backend handler which interacts with file-system

class RosHandler():
    ''' The datahandler manages the ROS and sending it to the frontend. '''
    def __init__(self):
        # Do stuff
        pass

    def update_scene(self, scene):
        print('Update scene')
        loop_is_closed, ordered_block_list = RosHandler.get_ordered_list(scene)

        print('Successfully ordered the blocks')
        # TODO: Send to ROS & create code-generation
        if loop_is_closed:
            print('Do something')
        else:
            print('Loop not closed. No updating')

    @staticmethod
    def get_ordered_list(scene):
        ''' Simplify a scene of links & blocks to an ordered_block_list. ''' 
        print('Start ordering')
        
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
            print('Loop')

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

        print('End ordering')
        return loop_is_closed, ordered_block_list
            

    def update_module(self, moulde_id, data):
        print('TODO: update_module')
        print('module_id')
        print('data')
        pass


if (__name__)=="__main__":
    print('Do a test')
