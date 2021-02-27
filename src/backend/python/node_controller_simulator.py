import time
import datetime 

# ROS2 utils
import rclpy
from rclpy.node import Node

# ROS messages
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import WrenchStamped, Pose, PoseStamped

from rclpy.parameter import Parameter

# Personal library
from sequence_handler import SequenceHandler

# Initialize(?)
rclpy.init()


class ControllerNodeSimulator(Node):
    topic_names = {
        'controller_stop': '/linear_controller/stop',
        'controller_command': '/linear_controller/command',
        'controller_success': '/linear_controller/path_executed',
    }
    
    def __init__(self):
        super().__init__('ros_main_handler')

        self.subscription_trajectory = self.create_subscription(
            Path, self.topic_names['controller_command'], self.callback_trajectory, 2)
        self.subscription_stop = self.create_subscription(
            Bool, self.topic_names['controller_stop'], self.callback_stop, 2)

        self.publisher_success = self.create_publisher(
            Bool, self.topic_names['controller_success'], 2)

        self.robot_is_moving = False

        print('Start spinning')
        rclpy.spin(node=self)
        
    def __del__(self):
        rclpy.shutdown()

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

        

if (__name__) == "__main__":
    node = ControllerNodeSimulator()
