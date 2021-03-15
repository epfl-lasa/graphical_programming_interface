import time
import datetime 

# ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# ROS messages
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import WrenchStamped, Pose, PoseStamped

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParameters

# Personal library
from sequence_handler import SequenceHandler

# Custom action & messages
from modulo_msgs.action import FollowPath


class ControllerNodeSimulator(Node):
    topic_names = {
        'controller_stop': '/linear_controller/stop',
        'controller_command': '/linear_controller/command',
        'controller_success': '/linear_controller/path_executed',
    }

    my_node_name = 'motion_generator'
    def __init__(self):
        
        super().__init__(self.my_node_name)

        self.subscription_trajectory = self.create_subscription(
            Path, self.topic_names['controller_command'], self.callback_trajectory, 2)
        self.subscription_stop = self.create_subscription(
            Bool, self.topic_names['controller_stop'], self.callback_stop, 2)

        self.publisher_success = self.create_publisher(
            Bool, self.topic_names['controller_success'], 2)

        self._action_server = ActionServer(self,
                                           FollowPath,
                                           '/' + self.my_node_name + '/' + 'follow_path',
                                           self.callback_execute_action)

        self.robot_is_moving = False

        # my_parameter_descriptor = ParameterDescriptor(type=ParameterType.BOOl)
        self.declare_parameter('compliant_mode')
        # self.param_compliant = Parameter('compliant_mode', Parameter.Type.BOOL, False)
        self.param_compliant = Parameter(
            name='compliant_mode', value=False, type_=Parameter.Type.BOOL)
        
        # self.param_compliant = Parameter(name='compliant_mode', value=ParameterValue(integer_value=3, type=ParameterType.PARAMETER_INTEGER))
        self.set_parameters([self.param_compliant])

        # self.create_service(SetParameters, '/motion_generator2/compliant_mode', self.param_callback)
        print('Start spinning')
        # rclpy.spin(node=self)
        while True:
            rclpy.spin_once(node=self, timeout_sec=0.5)
            print('Parameter value: {}', self.param_compliant.value)

    def param_callback(self):
        print('This is workign')
        
    def __del__(self):
        rclpy.shutdown()
        print('\n\n Stop spinning\n\n')

    @staticmethod
    def get_date_str():
        return datetime.datetime.now().strftime("%H:%M:%S")

    @property
    def robot_is_moving(self):
        # TODO: maybe (?!) make this a global parameter?
        return self._robot_is_moving

    @robot_is_moving.setter
    def robot_is_moving(self, value):
        self._robot_is_moving = value

    def callback_stop(self, msg):
        print('{}: @Controller - recieved stopping message: {}'.format(self.get_date_str(), msg.data))
        if msg.data is True:
            self.robot_is_moving = False
        # else:
            # print('

    ##########################################
    ### Callbacks
    ##########################################
    def callback_trajectory(self, msg):
        print('@ {}'.format(self.get_date_str()))
        print('@Controller Recieved msg_type <{}> of length {}'.format(type(msg), len(msg.poses)))

        self.robot_is_moving = True
        time_sleep = 1.0
        # rclpy.spin_once(node=self, timeout_sec=time_sleep)
        time.sleep(time_sleep)
        
        if self.robot_is_moving:
            print('@Controller: sending finished succesfully')
            self.publisher_success.publish(Bool(data=True))
        else:
            print('@Controller: sending stopped inbetween')
            self.publisher_success.publish(Bool(data=False))


    def callback_execute_action(self, goal_handle, max_it=3):
        ''' No result being returned. '''
        path = goal_handle.request.path
        print('Requested path execution: path')
        self.get_logger().info('Exectuing goal...')
        feedback_msg = FollowPath.Feedback()
        # feedback_msg.percentage_of_completion = Float32(data=0.0)
        feedback_msg.percentage_of_completion = float(0.0)
        
        for ii in range(max_it):
            print('Loop loop at {}%'.format(round(100.0*ii/max_it, 0)))
            time.sleep(0.5)
            feedback_msg.percentage_of_completion = float(1.0*(ii+1)/max_it)

            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        print('About to send result')
        # Return empty result
        result = FollowPath.Result()
        return result

if (__name__) == "__main__":
    # Initialize(?)
    rclpy.init()
    node = ControllerNodeSimulator()
