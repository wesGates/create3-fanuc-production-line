
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

    def read_robot_statuses(self):
        # Assuming the file has a simple comma-separated format like "roomba,True,beaker,False"
        with open('robot_status.txt', 'r') as file:
            content = file.read().strip().split(',')
            statuses = dict(zip(content[::2], content[1::2]))  # Create a dict {'roomba': 'True', 'beaker': 'False'}
            return {robot: status == 'True' for robot, status in statuses.items()}  # Convert string 'True'/'False' to boolean

    def check_readiness_callback(self, request, response):
        statuses = self.read_robot_statuses()
        response.ready = statuses.get(request.robot1.lower(), False) and statuses.get(request.robot2.lower(), False)
        return response

