'''
AICA user data handler

(c) AICA SarL
'''
__author__ = "Lukas Huber"
__email__ = "lukas@aica.tech"

import time

import numpy as np

# ROS2 utils
import rclpy
from rclpy.node import Node

# ROS messages
from geometry_msgs.msg import WrenchStamped


class ForceListener(Node):
    def __init__(self, start_time=0, delta_time_max=5, _RosHandler=None):
        ''' Force recording'''
        super().__init__('ros_force_listener')
        
        self.topic = _RosHandler.topic_names['ft_sensor']

        self.time_data = []
        self.force_data = []

        self.delta_time_max = delta_time_max
        self.is_active = False

        self.type_magnitude = True

        self.start_time = start_time

        self.subscription_robot_robot = self.create_subscription(
            WrenchStamped, self.topic, self.callback_msg, 5)

        self.last_update = None

        self.new_force = None

    def reset_data(self):
        ''' Set graph data to zero. '''
        self.time_data = []
        self.force_data = []

    def activate(self, time_sleep=0.1, timeout_time=5):
        ''' Activate & loop to collect data.'''
        self.is_active = True

        self.reset_data()

        self.last_update_request = time.time()
        
        while self.is_active:
            rclpy.spin_once(node=self, timeout_sec=time_sleep)

            # Timeout (!)
            if time.time() - self.last_update_request > timeout_time:
                print('Node is stopped, since no request recieved.')
                self.deactivate()
            
    def deactivate(self):
        ''' Deactivate this node. '''
        self.is_active = False

    def get_updated_data(self):
        ''' Send data back. '''
        self.last_update_request = time.time()

        data_dict = {}
        if self.new_force is not None:
            data_dict['force'] = self.new_force
            self.new_force = None
        
        if len(self.time_data) == 0:
            data_dict['time'] = []
            data_dict['data'] = []
            return data_dict
        
        while self.time_data[0] < self.time_data[-1] - self.delta_time_max:
            del self.time_data[0]
            del self.force_data[0]
            
        # Project time to [-delta_time_max, 0]
        time_data = [(tt - self.time_data[-1]) for tt in self.time_data]
        
        data_dict['time'] = time_data
        data_dict['data'] = self.force_data
        return data_dict

    def callback_msg(self, msg, min_delta_time=0.01):
        ''' Get message '''
        force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        time_in_sec = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        
        self.time_data.append(time_in_sec)
        self.force_data.append(np.linalg.norm(force))

    # TODO: property-listener for force
