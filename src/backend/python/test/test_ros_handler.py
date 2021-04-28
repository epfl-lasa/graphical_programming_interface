'''
test script for the ros handler
'''

__author__ = "Lukas Huber"
__email__ = "lukas@aica.tech"

import os
import sys
import copy
import warnings
import threading
import time

import yaml

import rclpy

# from data_handler import DataHandler
# MyDataHandler = DataHandler(dir_path)
dir_path = os.path.dirname(os.path.realpath(__file__))

print('dir_path')
print(dir_path)

path_communication_handler = os.path.join(dir_path, "..")
if not path_communication_handler in sys.path:
    sys.path.append(path_communication_handler)

from ros_handler import RosHandler
RosHandler.initialize_ros()
MyRosHandler = RosHandler(set_parameters=False)

def test_euler_pose_converter_with_ros():
    ''' Testing euler pose convertion. '''

    max_test_it = 100
    it_count = 0
    while(MyRosHandler.msg_robot_pose is None):
        rclpy.spin_once(node=MyRosHandler, timeout_sec=0.1)
        
        print('Waiting again')
        it_count += 1
        if (it_count > max_test_it):
            break

    initial_pose = copy.deepcopy(MyRosHandler.msg_robot_pose)

    euler_pose = MyRosHandler.transform_poseROS_to_eulerPose(initial_pose)

    reset_pose = MyRosHandler.transform_eulerPose_to_poseROS(euler_pose)

    print('Initial pose')
    print(initial_pose)
    
    print('Reset pose')
    print(reset_pose)


def test_euler_pose_and_back():
    ''' Testing euler pose convertion. '''
    initial_euler_pose = {
        'position': {
            'x': 234,
            'y': 123,
            'z': -234,
            },
        'orientation': {
            'x': 45,
            'y': 45,
            'z': -235,
        },
    }
    
    print('Initial pose')
    print(initial_euler_pose)

    ros_pose = MyRosHandler.transform_eulerPose_to_poseROS(initial_euler_pose)

    print('Ros pose')
    print(ros_pose)
    
    # And go back
    final_euler_pose = MyRosHandler.transform_poseROS_to_eulerPose(ros_pose)

    
    print('Reset pose')
    print(final_euler_pose)


if __name__=="__main__":
    # test_euler_pose_converter_with_ros()
    
    test_euler_pose_and_back()
