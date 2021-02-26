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

# Math / orientation tools
from scipy.spatial.transform import Rotation

# ROS2 utils
import rclpy
from rclpy.node import Node

# ROS messages
from std_msgs.msg import Bool
from nav_msgs_msgs.msg import Path
from geometry_msgs.msg import WrenchStamped, Pose

from rclpy.parameter import Parameter

# Personal library
from sequence_handler import SequenceHandler
from rclpy.parameter import Parameter

   
class RosHandler(Node):
    ''' The datahandler manages the ROS and sending it to the frontend. '''
    topic_names = {
        'controller_stop': '/linear_controller/stop',
        'controller_command': 'linear_controller/command',
        'ft_sensor': '/ft_sensor/netft_data',
        'robot_pose': '/iiwa/CartesianPosition',
    }
    
    parameter_name = {
        'controller_force': '/linear_controller/force',
    }

        
    def __init__(self, controller_topic='controller'):
        super().__init__('ros_main_handler')
        
        self.controller_topic = controller_topic

        # Initialize the sequence handler
        # self.SequenceHandler = SequenceHandler(MainRosHandler=self, controller_topic=self.controller_topic)
        
        # Publisher
        self.publisher_trajcetory = self.create_publisher(
            Path, self.topic_names['controller_command'], 2)
        self.publisher_stop = self.create_publisher(
            Bool, self.topic_names['controller_stop'], 2)

        # Declare callback messages
        self.msg_robot_pose = None
        self.msg_ft_sensor = None
        
        # Subcription (potentially: only activate this when actually needed...)
        self.subscription_robot_robot = self.create_subscription(
            self.topic_names['robot_pose'], self.callback_robot_pose, 5)

        self.subscription_ft_sensor = self.create_subscription(
            self.topic_names['ft_sensor'], self.callback_ft_sensor, 5)
            
            
        # self.publisher_controller = self.create_publisher(String, self.controller_topic, 10)
        # self.publisher_trajcetory = self.create_publisher(String, self.controller_topic, 10)
        
        # Do stuff
        self.emergency_stop_activated = False
        self.data_recording = True
        
    # @property
    # def emergency_stop_activated(self):
        # return self._emergency_stop_activated
    
    # @emergency_stop_activated.setter
    # def emergency_stop_activated(self, value):
        # self._emergency_stop_activated = value

    @property
    def robot_is_moving(self):
        # TODO: maybe make this a global parameter?
        self.publisher_stop.publish(True)
        return self._robot_is_moving

    @robot_is_moving.setter
    def robot_is_moving(self, value):
        self._robot_is_moving = value

    @staticmethod
    def transform_pose_to_euler(pose):
        pass

    @staticmethod
    def transform_euler_to_pose(euler):
        pass

    # def relase_emergency_stop(self):
        # ''' Relase emergency stop. '''
        # self.emergency_stop_activated = False

    def stop_robot(self):
        self.robot_is_moving = False

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
        ''' Move to specific euler pose.'''
        self.set_parameters([
            Parameter(parameter_name['controller_force'], Parameter.Type.FLOAT64, 0),
            ])
        
        # print('@ros_handler: TODO --- move to position')

    def get_robot_position(self, pose_type=None, time_max_wait=5, time_sleep=0.1):
        ''' Get current position of the robot. '''
        print('TODO: return current position as list.')

        num_try_max = time_max_wait/time_sleep
        rate_sleep = node.create_rate(time_sleep)
        it = 0
        while self.msg_robot_pose is None:
            if it > num_try_max:
                warnings.warn('Maximum iterations reached wihout position')
                return
            it += 1
            rate_sleep.sleep()
            
        import pdb; pbd.set_trace()

        orientation = Rotation.from_quat([
            self.msg_robot_pose.x,
            self.msg_robot_pose.y,
            self.msg_robot_pose.z,
            self.msg_robot_pose.w
        ])
        
        orientation_euler = orientation.as_quat('zyx', degrees=True)

        # DEBUG (!)
        warnings.warn('@ros_handler: DEBUG POSE')
        pose_data = {
            'type': 'eulerPose',
            'frameId': 'base_link',
            'position': {
                'x': self.msg_robot_pose.position.x,
                'y': self.msg_robot_pose.position.y,
                'z': self.msg_robot_pose.position.z
            },
            'orientation': {
                'x': orientation_euler.x,
                'y': orientation_euler.y,
                'z': orientation_euler.z
            },
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

    def callback_robot_pose(self, msg):
        '''ROS callback '''
        self.msg_robot_pose = msg

    def callback_ft_sensor(self, msg):
        '''ROS callback '''
        self.msg_ft_sensor = msg
        


class ListenerFt(Node):
    def __init__(self, topic="/ft_sensor/netft_data", ):
        ''' ''' 
        self.topic = topic

    def callback_msg(self, msg):
        ''' Get message '''
        


if (__name__)=="__main__":
    print('Do a test')
