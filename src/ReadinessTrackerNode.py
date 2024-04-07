
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


import time
import sys
sys.path.append("../dependencies/")

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from pynput import keyboard


import my_interfaces
from my_interfaces.msg import ReadyStatus  # CHANGE
from my_interfaces.srv import CheckReadiness


class ReadinessTrackerNode(Node):
    def __init__(self):
        super().__init__('readiness_tracker_node')
        self.service = self.create_service(CheckReadiness, 'check_readiness', self.check_readiness_callback)
        self.robot_order = ['roomba_base2', 'beaker', 'beaker_conv', 'bunsen_conv', 'bunsen', 'roomba_base3' ]

    def read_robot_statuses(self):
        with open('robot_status.txt', 'r') as file:
            statuses = [status == 'True' for status in file.read().strip().split(',')]
            
            if len(statuses) == len(self.robot_order):
                return dict(zip(self.robot_order, statuses))
            else:
                self.get_logger().error('Mismatch in the number of statuses and robot identifiers.')
                return {}

    def check_readiness_callback(self, request, response):
        statuses = self.read_robot_statuses()
        # Directly return the status of robot1 (or robot2; they should be the same in this new approach)
        response.ready = statuses.get(request.robot1.lower(), False)
        return response
