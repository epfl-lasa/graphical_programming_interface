'''
AICA user data handler

(c) AICA SarL
'''
__author__ = "Lukas Huber"
__email__ = "lukas@aica.tech"

import warnings

# ROS2 utils
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from modulo_msgs.action import FollowPath

class SequenceHandler(Node):
    def __init__(self, MainRosHandler, sequence=[],
                 max_sequence_loops=-1):
        super().__init__('sequence_handler')
        
        self._MainRosHandler = MainRosHandler

        # Sequence Rememberer & Current State in the sequence
        self.sequence = sequence
        self._sequence_it = 0

        self.max_sequence_loops = max_sequence_loops

        # self.subscription_successfull_execution = self.create_subscription(
            # Bool, self._MainRosHandler.topic_names['controller_success'],
            # self.callback_loop_succesfull, 2)

        # TIMERS / Create here or in script(?)
        # self.create_timer(timer_period, self.timer_callback)

    @property
    def robot_is_moving(self):
        return self._MainRosHandler.robot_is_moving

    @robot_is_moving.setter
    def robot_is_moving(self, value):
        self.sequencing = False
        self.executing_one_module = False
        self._MainRosHandler.robot_is_moving = value

    @property
    def movement_execution_in_progress(self):
        # if not self.robot_is_moving:
            # return False
        return self._MainRosHandler.movement_execution_in_progress
        
    # @movement_execution_in_progress.setter
    # def movement_execution_in_progress(self, value):
        # self._movement_execution_in_progress = value
        
    @property
    def sequence(self):
        # Sequence of blocks
        return self._sequence

    @sequence.setter
    def sequence(self, value):
        print('Sequence updated', value)
        self._sequence = value
    
    @property
    def sequence_it(self):
        return self._sequence_it

    @sequence_it.setter
    def sequence_it(self, value):
        ''' Cap at maximum iteration. Assumption of only one increment at a time.'''
        self._sequence_it = value
        if self._sequence_it >= len(self.sequence):
            self._sequence_it = self._sequence_it - len(self.sequence)
    
    @staticmethod
    def get_euler_pose(seq):
        return seq['values']['property']['reference']['value']

    def publish_trajectory(self, msg, force=0, time_sleep=0.02):
        while self.movement_execution_in_progress:
            # rclpy.spin_once(node=self, timeout_sec=time_sleep)
            rclpy.spin_once(node=self._MainRosHandler, timeout_sec=time_sleep)

        if force is not None:
            self._MainRosHandler.set_force_parameter(force)
            
        # self.movement_execution_in_progress = True
        # self._MainRosHandler.publisher_trajectory.publish(msg)
        self._MainRosHandler.send_action_path(msg)

    # @staticmethod
    # dep get_module_reference(self, self.sequence_it):

    def get_current_module_id(self):
        return self.sequence[self.sequence_it]['id']
        
    def get_position_of_module_id(self, module_id):
        ''' Find position of module with the specific id.'''
        for seq_it in range(len(self.sequence)):
            if self.sequence[seq_it]['id'] == module_id:
                return seq_it
        # Not found
    
    def execute_module(self, sequence_it, time_sleep=0.1):
        ''' Exectue a module without resetting. '''
        if len(self.sequence) == 0:
            print('Zero sequence -- shutting down.')
            self.robot_is_moving = False
            return

        print('seq it', sequence_it)
        if self.sequence[sequence_it]['type'] == 'idle':
            print('Module: Idle / home')
            pose = PoseStamped()
            pose.header.frame_id = 'none'
            # pose.header.stamp = (?)
            pose.pose = self._MainRosHandler.transform_eulerPose_to_poseROS(
                self.get_euler_pose(self.sequence[self.sequence_it]))

            msg_path = Path()
            msg_path.poses.append(pose)

        elif self.sequence[sequence_it]['type'] == 'gripper':
            print('Moduel: Gripper')
            # rclpy.spin_once(node=self, timeout_sec=1.0)
            # print('Gripper: {}'.format(self.sequence[sequence_it]['values']['property']['gripper'])
            # Publish empty path
            msg_path = Path() 
        
        elif (self.sequence[sequence_it]['type'] == 'linear'
              or self.sequence[sequence_it]['type'] == 'constant_force'):
            print('Module: Linear with/without force')
            pose = PoseStamped()
            pose.header.frame_id = 'none'
            # pose.header.stamp = (?)
            pose.pose = self._MainRosHandler.transform_eulerPose_to_poseROS(
                self.get_euler_pose(self.sequence[self.sequence_it]))

            msg_path = Path()
            msg_path.poses.append(pose)

        elif (self.sequence[sequence_it]['type'] == 'learn_motion'):
            print('Module: Learned motion')
            msg_path = self._MainRosHandler.get_module_database_as_path(self.sequence[sequence_it]['id'])
            
        else:
            warnings.warn("Unknown module of type <{}>.".format(self.sequence[sequence_it]['type']))
            return

        desired_force = 0
        if 'force' in self.sequence[sequence_it]['values']['property']:
            desired_force = self.sequence[sequence_it]['values']['property']['force']['value']
            
        # Publish / send data
        self.publish_trajectory(msg_path, force=desired_force)

        return 0
        
    def execute_module_including_reseting(self, module_id=None):
        ''' Exectue a module without resetting '''
        if len(self.sequence) == 0:
            return
        
        if module_id is not None:
            print('No module sequence found')
            warnings.warn('No module sequence found')
            self.sequence_it = self.get_position_of_module_id(module_id)

        # Move to previous reference position
        pose = PoseStamped()
        pose.header.frame_id = 'none'
        # pose.header.stamp = (?)
        pose.pose = self._MainRosHandler.transform_eulerPose_to_poseROS(
            self.get_euler_pose(self.sequence[self.sequence_it-1]))
        msg_path = Path()
        msg_path.poses.append(pose)
        print('Doing traj now')
        self.publish_trajectory(msg_path, force=0)
        print('Trajectory is done.')
        
        # Execute module
        self.execute_module(self.sequence_it)

        print('Succesfull execution')
        return 0

    def execute_sequence(self, sequence_it=None, module_id=None):
        ''' Execute the whole sequence (not only the points). '''
        print('@seq hanlder: Start seq')
        
        if module_id is not None:
            self.sequence_it = self.get_position_of_module_id(module_id)
        elif sequence_it is not None:
            self.sequence_it = sequence_it

        sequence_loop_it = 0
        while self.robot_is_moving:
            # The module is only executed when previous is completed
            self.execute_module(self.sequence_it)
            self.sequence_it += 1

            if self.sequence_it == 0:
                sequence_loop_it += 1

            if (self.max_sequence_loops > 0     # otherwise infinite loops
                and self.sequence_it > self.max_sequence_loops):
                self.robot_is_moving = False
                print('Max iteration reached')
                break

        print('Done looping')
        return 0

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = FollowPath.Result()
        return result

    # def callback_loop_succesfull(self, msg):
        # ''' Returns true if module is exectued. '''
        # self.sequence_it += 1

        # self.movement_execution_in_progress = False

        # if msg.data is False:
            # warnings.warn('Problem during movement execution.')
        # self.sub_module_completed = True
