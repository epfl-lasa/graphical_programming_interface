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
import threading
import time

import yaml

# Math / orientation tools
# import numpy as np
from scipy.spatial.transform import Rotation

# ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ROS messages
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import WrenchStamped, Pose, PoseStamped, Quaternion

# from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterDescriptor, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

# Personal library
from sequence_handler import SequenceHandler
from force_listener import ForceListener

# Custom action & messages
from modulo_msgs.action import FollowPath

default_frame = 'iiwa_link_0'

class RosHandler(Node):
    ''' The datahandler manages the ROS and sending it to the frontend. '''
    topic_names = {
        'controller_stop': '/linear_controller/stop',
        'controller_command': '/linear_controller/command',
        'controller_success': '/linear_controller/path_executed',
        'ft_sensor': '/ft_sensor/netft_data',
        'robot_pose': '/iiwa/CartesianPosition',
    }
    
    parameter_name = {
        'controller_force': '/linear_controller/force',
    }

    # action_name = {
        # 'action_path': '/motion_generator/follow_path',
    # }

    controller_node_name = '/motion_generator'

    compliant_node_name = '/robot_interface'
    
    # transform from mm to m
    factor_dist_front_backend_dist = 1000

    default_frame = 'iiwa_link_0'

    @staticmethod
    def initialize_ros():
        # Initialize(?)
        rclpy.init()

    def __init__(self, DataHandler=None, DEBUG_FLAG=False):
        super().__init__('ros_main_handler')
        # Spin in a separate thread
        # self.thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        # self.thread.start()
        
        # Publisher
        self.publisher_trajectory = self.create_publisher(
            Path, self.topic_names['controller_command'], 2)

        print('got publisher')
        self.publisher_stop = self.create_publisher(
            Bool, self.topic_names['controller_stop'], 2)

        # Declare callback messages
        self.msg_robot_pose = None
        self.msg_ft_sensor = None

        # The handler of goals for the action-server
        self._goal_handle = None
        
        # Subcription (potentially: only activate this when actually needed...)
        self.subscription_robot_robot = self.create_subscription(
            Pose, self.topic_names['robot_pose'], self.callback_robot_pose, 5)

        self.subscription_ft_sensor = self.create_subscription(
            WrenchStamped, self.topic_names['ft_sensor'], self.callback_ft_sensor, 5)
        
        # self._action_client_path = ActionClient(self, FollowPath, RosHandler.action_name[0])
        self._action_client_path = ActionClient(
            self, FollowPath, self.controller_node_name + '/follow_path')

        # Initialize the sequence handler
        self.SequenceHandler = SequenceHandler(MainRosHandler=self)

        if not DEBUG_FLAG:
            self.client_setparams_controller = self.create_client(
                SetParameters, self.controller_node_name+'/set_parameters')

            self.client_setparams = self.create_client(
                SetParameters, self.compliant_node_name+'/set_parameters')

            while (not self.client_setparams.wait_for_service(timeout_sec=1.0)
                   or
                   not self.client_setparams_controller.wait_for_service(timeout_sec=1.0)):
                self.get_logger().info("service not available, waiting again...") #'" 
            self.req_params = SetParameters.Request() 

        # self.publisher_controller = self.create_publisher(String, self.controller_topic, 10)
        # self.publisher_trajectory = self.create_publisher(String, self.controller_topic, 10)

        self.movement_execution_in_progress = False

        # Force Listener Noe [initialize only later]
        self.force_listener = None
        
        # Do stuff
        self.emergency_stop_activated = False
        self.data_recording = True

        self.start_time = self.get_clock().now().to_msg()
        
        self.DataHandler = DataHandler
        
        print('Finished init.')
        warnings.warn('Finished init.')

    def reset(self):
        # How to differentiate from del (?!)
        pass

    def __del__(self):
        # Sure about shutdown of rclpy(?)
        self.robot_is_moving = False # Turn robot of
        self.reset_attractor_to_current_pose() # TODO: integrate this in normal (?)
        rclpy.shutdown()
        # self.thread.join()
        
    # @property
    # def emergency_stop_activated(self):
        # return self._emergency_stop_activated
    
    # @emergency_stop_activated.setter
    # def emergency_stop_activated(self, value):
        # self._emergency_stop_activated = value

    @property
    def robot_is_moving(self):
        # TODO: maybe (?!) make this a global parameter?
        # import pdb; pdb.set_trace()
        # self.publisher_stop.publish(Bool(data=True))
        return self._robot_is_moving

    @robot_is_moving.setter
    def robot_is_moving(self, value):
        if value:
            self._goal_handle = None
            print('Doing nothing -- robot is moving.')
            # self.publisher_stop.publish(Bool(data=False))
        else:
            # Cancel_goal_async
            if self._goal_handle is None:
                warnings.warn('No goal handle detected..')
            else:
                self._goal_handle.cancel_goal_async()
            self.publisher_stop.publish(Bool(data=True))
        self._robot_is_moving = value

    
    @staticmethod
    def transform_poseROS_to_eulerPose(pose, frameId=None,
                                       round_pose=True, euler_pose_in_degrees=True):
        ''' Torm to front-end readable & human-understandable degrees json.'''
        # Default frame Id
        pose = copy.deepcopy(pose)
        
        if frameId is None:
            frameId = RosHandler.default_frame
            
        orientation = Rotation.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        
        orientation_euler = orientation.as_euler(seq='zyx', degrees=euler_pose_in_degrees)
        if euler_pose_in_degrees:
            orientation_euler = orientation_euler 
            
        euler_pose = {
            'type': 'eulerPose',
            'frameId': frameId,
            'position': {
                'x': pose.position.x * RosHandler.factor_dist_front_backend_dist,
                'y': pose.position.y * RosHandler.factor_dist_front_backend_dist,
                'z': pose.position.z * RosHandler.factor_dist_front_backend_dist
            },
            'orientation': {
                'x': orientation_euler[0],
                'y': orientation_euler[1],
                'z': orientation_euler[2],
            },
        }

        if round_pose:
            for key in euler_pose['position']:
                euler_pose['position'][key] = round(euler_pose['position'][key], 1)
                
            ang_round_ind = 1 if euler_pose_in_degrees else 3    
            for key in euler_pose['orientation']:
                euler_pose['orientation'][key] = round(euler_pose['orientation'][key],
                                                       ang_round_ind)
        
        return euler_pose

    @staticmethod
    def transform_eulerPose_to_poseROS(euler_pose, euler_pose_in_degrees=True):
        ''' Transfrom front-end data to ROS2 pose. '''
        # deg to rad
        # import pdb; pdb.set_trace()
        euler_pose = copy.deepcopy(euler_pose)
        
        if euler_pose_in_degrees:
            euler_pose['orientation']['x'] = float(euler_pose['orientation']['x'])
            euler_pose['orientation']['y'] = float(euler_pose['orientation']['y'])
            euler_pose['orientation']['z'] = float(euler_pose['orientation']['z'])
        
        orientation = Rotation.from_euler(
            seq='zyx', degrees=euler_pose_in_degrees,
            angles=[
                float(euler_pose['orientation']['x']),
                float(euler_pose['orientation']['y']),
                float(euler_pose['orientation']['z']),
            ])
        qq = orientation.as_quat()

        pose = Pose()
        pose.position.x = float(euler_pose['position']['x']) / RosHandler.factor_dist_front_backend_dist
        pose.position.y = float(euler_pose['position']['y']) / RosHandler.factor_dist_front_backend_dist
        pose.position.z = float(euler_pose['position']['z']) / RosHandler.factor_dist_front_backend_dist
        
        pose.orientation.x = qq[0]
        pose.orientation.y = qq[1]
        pose.orientation.z = qq[2]
        pose.orientation.w = qq[3]

        return pose

    def set_compliant_mode(self, value):
        ''' Set Compliant mode. '''
        # ros2 service type /robot_interface/set_parameters
        # rcl_interfaces/srv/SetParameters
        # compliant_mode
        # type_bool = 1

        new_param_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
        new_param = Parameter(name='compliant_mode',  value=new_param_value)
        self.req_params.parameters = [new_param]

        self.future = self.client_setparams.call_async(self.req_params)

        if not value: # is False
            if self.msg_robot_pose is None:
                warnings.warn('No robot state ever recieved. Not resetting attractor.')
                return
            
            self.reset_attractor_to_current_pose()
            
    def reset_attractor_to_current_pose(self):
        ''' Put robot in default position.'''
        rclpy.spin_once(node=self, timeout_sec=0.02)
        param_value = [self.msg_robot_pose.position.x,
                       self.msg_robot_pose.position.y,
                       self.msg_robot_pose.position.z,
                       self.msg_robot_pose.orientation.w,
                       self.msg_robot_pose.orientation.x,
                       self.msg_robot_pose.orientation.y,
                       self.msg_robot_pose.orientation.z
        ]

        # param_value = [6,6,6, 6,6,6,6]
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=param_value)
        new_param = Parameter(name='attractor_pose',  value=new_param_value)
        self.req_params.parameters = [new_param]
        # self.future = self.client_setparams.call_async(self.req_params)
        self.future_controller = self.client_setparams_controller.call_async(self.req_params)

        

    def set_force_parameter(self, desired_force=0):
        debugging = True
        if debugging:
            return 
        Parameter(self._MainRosHandler.parameter_name['controller_force'],
                  Parameter.Type.FLOAT64, desired_force),

    def stop_robot(self):
        ''' Emergency Stop of the Robot. '''

        self.robot_is_moving = False

    def startup_robot(self):
        ''' Startup robot. '''
        self.robot_is_moving = True

    # def relase_emergency_stop(self):
        # ''' Relase emergency stop. '''
        # self.emergency_stop_activated = False

    ################################################
    ### Active moving of the robots
    ################################################
    def move_to_position(self, euler_pose=None, msg_path=None):
        ''' Move to specific euler pose.'''
        self.set_force_parameter(0)
        print('Start moving')

        if msg_path is None:
            pose = PoseStamped()
            pose.header.frame_id = RosHandler.default_frame
        
            pose.pose = self.transform_eulerPose_to_poseROS(euler_pose)
            # Create Path
            msg_path = Path()
            msg_path.poses.append(pose)

        self.startup_robot()
        # self.publisher_trajectory.publish(msg_path)
        print('moving to position')
        self.send_action_path(msg_path)

        timeout_sec = 0.1
        while(self.movement_execution_in_progress):
            # print('Doing execution.')
            # print(self._goal_handle)
            print('Movement in progress')
            rclpy.spin_once(node=self, timeout_sec=timeout_sec)
            # time.sleep(timeout_sec)
            
        print('Finished movmeemet')
        return 0

    def send_action_path(self, msg_path):
        ''' Send path to the action.'''
        print('308 ---------------- Send action path ')
        self.movement_execution_in_progress = True
        msg_goal = FollowPath.Goal()
        msg_goal.path = msg_path
        
        self._action_client_path.wait_for_server()
        self._send_goal_future = self._action_client_path.send_goal_async(
            msg_goal, feedback_callback=self.callback_path_feedback)
        self._send_goal_future.add_done_callback(self.callback_goal_response)

    def callback_goal_response(self, future):
        '''ROS action-callback '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        
        self.get_logger().info('Goal accepted.')

        print('finished', goal_handle.status)
        self._get_result_future = goal_handle.get_result_async()
        self._goal_handle = goal_handle
        self._get_result_future.add_done_callback(self.callback_goal_result)
    
    def callback_goal_result(self, future):
        '''ROS action-callback '''
        self.get_logger().info('Goal is reached successuflly.')
        self.movement_execution_in_progress = False
        print('Movement execution')
        # rclpy.shutdown()

    def callback_path_feedback(self, feedback_msg):
        '''ROS action-callback '''
        # No result expected
        feedback = feedback_msg.feedback
        self.get_logger().info('Path execution finished: {0}%'.format(
            round(feedback.percentage_of_completion*100.0, 0)))

    def execute_module(self, module_id):
        ''' Run the module '''
        print('TODO: execute sequence')
        self.startup_robot()
        # status_message = self.SequenceHandler.execute_module_including_reseting(module_id)
        status_message = self.SequenceHandler.execute_module(module_id)

        timeout_sec=0.1
        while(self.movement_execution_in_progress):
            rclpy.spin_once(node=self, timeout_sec=timeout_sec)
            # time.sleep(timeout_sec)

        return status_message

    def execute_sequence(self, module_id=None):
        ''' Run sequence starting at module_id. If module_id is none.'''
        self.startup_robot()
        status_message = self.SequenceHandler.execute_sequence(module_id)

        timeout_sec = 0.1
        while(self.movement_execution_in_progress):
            rclpy.spin_once(node=self, timeout_sec=timeout_sec)
            # time.sleep(timeout_sec)
        
        return status_message

    def get_active_module_id(self):
        return self.SequenceHandler.get_current_module_id()

    ################################################
    ### Force Recording
    ################################################
    def start_force_recording(self):
        ''' Activate or initialize force listener. '''
        if self.force_listener is None:
            self.force_listener = ForceListener(start_time=self.start_time)

        # self.set_compliant_mode(True)
        self.force_listener.activate()
        return 0

    def stop_force_recording(self):
        ''' Deactivate force recording, but dont delete node yet.'''
        self.force_listener.deactivate()
        return 0

    def update_force_data(self):
        ''' Send data back'''
        if self.force_listener is None:
            self.start_force_recording()
            
        return self.force_listener.get_updated_data()

    ################################################
    ### Get position and states
    ################################################
    def get_default_position_from_file(self, file_name='default_pos.txt'):
        ''' Get current position of the robot. '''
        # TODO: don't do data importing here...
        default_position_file = os.path.join('/home/ros2/src/backend/python', file_name)
        with open(default_position_file, 'r') as ff:
            default_data = yaml.load(ff, Loader=yaml.FullLoader)

        default_pose = Pose()
        default_pose.position = Position(default_data[0], default_data[1], default_data[2])
        default_pose.orientation = Quaternion(default_pose[0], default_pose[1], default_pose[2])

        default_pose = self.RosMainHandler.transform_poseROS_to_eulerPose()
        return default_pose

    def set_default_position(self, file_name='default_pos.txt'):
        ''' Get current position of the robot. '''
        pose_data = self.RosMainHandler.get_default_pose(pose_type='eulerpose')
        return pose_data

    def get_current_robot_joint_position(self):
        # TODO - for future...
        raise NotImplementedError('Not implented')

    def get_robot_position(self, pose_type=None, time_max_wait=5, time_sleep=0.1):
        ''' Get current position of the robot. '''
        num_try_max = time_max_wait/time_sleep
        
        it = 0
        while self.msg_robot_pose is None:
            if it > num_try_max:
                warnings.warn('Maximum iterations reached wihout position')
                return ' '
            it += 1
            rclpy.spin_once(node=self, timeout_sec=time_sleep)
            # time.sleep(time_sleep)

        euler_data = self.transform_poseROS_to_eulerPose(self.msg_robot_pose)
        return {'pose': euler_data}
    
    def record_module_database(self, module_id, DataHandler, max_recording=1000,
                               recording_delta_time=0.5):
        ''' Record Data and Send to Database. '''
        self.set_compliant_mode(True)
        self.data_recording = True

        self.get_logger().info("Start recording with delta_time={}s".format(recording_delta_time))
        recorded_data = []
        it_recording = 0
        while(self.data_recording):
            it_recording += 1
            if it_recording > max_recording:
                print('Maximum iterations of recording reached')
                break

            if self.msg_robot_pose is not None:
                recorded_data.append(copy.deepcopy(self.msg_robot_pose))
            else:
                warnings.warn('@ros_handler: recieved none robot position')

            # This decides about the recording frequency
            rclpy.spin_once(node=self, timeout_sec=recording_delta_time)
            # time.sleep(time_sleep)
            
        self.data_recording = False
        # self.set_attractor_as_current(Pose)
        self.set_compliant_mode(False)
            
        data_list = self.DataHandler.save_to_module_database(module_id, my_data=recorded_data)
        return data_list

    def get_module_database_as_path(self, module_id, data_it=None, file_name=None):
        ''' Replay a module based on points in the database. '''
        # data = 
        # import pdb; pdb.set_trace()
        header, module_data = self.DataHandler.load_from_module_database(
            module_id, file_name=file_name, data_it=data_it)

        print('Loaded data')
        print(module_data)
        
        msg_path = Path()
        if header == 'pos[x], pos[y], pos[z], quat[w], quat[x], quat[y], quat[z]':
            # TODO: include time-stamp and
            pose = PoseStamped()
            pose.header.frame_id = RosHandler.default_frame
            # pose.header.stamp = (?)

            module_data = module_data.T
            
            for ii in range(module_data.shape[1]):
                pose.pose.position.x = module_data[0, ii]
                pose.pose.position.y = module_data[1, ii]
                pose.pose.position.z = module_data[2, ii]

                pose.pose.orientation.w = module_data[3, ii]
                pose.pose.orientation.x = module_data[4, ii]
                pose.pose.orientation.y = module_data[5, ii]
                pose.pose.orientation.z = module_data[6, ii]
                
                msg_path.poses.append(copy.deepcopy(pose))
        else:
            warnings.warn("Unexpected file format.")

        return msg_path
    
    ##############################################
    ###   Scene & Module updating
    ##############################################
    def update_module(self, module_id, module_data):
        # Module ID and data
        self.SequenceHandler.update_module(module_id, module_data)

    def update_scene(self, scene):
        ''' Send data to ROS and initiate code-generation. '''
        self.SequenceHandler.sequence = scene['blocks']
            
    @staticmethod
    def get_ordered_list(scene):
        # DEPRECIATED: now integrated in front end, not used anymore...x
        # TODO: DELTE
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
        

    ##############################################
    ###  Callbacks
    ##############################################
    def callback_stoprecording(self):
        # print('Stop recording')
        self.data_recording = False

    def callback_emergency_stop(self):
        warnings.warn('EMERGENCY STOP WAS CALLED')
        self.robot_is_moving = False

    def callback_stop_robot(self):
        ''' Shutdown all processes. '''
        print('Stopping during the movement. Resetting attractor.')
        self.reset_attractor_to_current_pose()
        
        self.movement_execution_in_progress = False
        self.robot_is_moving = False

    def callback_robot_pose(self, msg):
        '''ROS callback '''
        self.msg_robot_pose = msg

    def callback_ft_sensor(self, msg):
        '''ROS callback '''
        self.msg_ft_sensor = msg


if (__name__)=="__main__":
    print('Do a test')
