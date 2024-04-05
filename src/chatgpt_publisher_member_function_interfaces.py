import sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node
from pynput.keyboard import KeyCode
import my_interfaces
from my_interfaces.msg import ReadyStatus

class ReadyStatusPublisherNode(Node):
    def __init__(self):
        super().__init__('ready_status_publisher')
        self.ready_status_publisher_ = self.create_publisher(ReadyStatus, 'robot_ready_status', 10)
        
        # Initialize variables
        self.status_file_path = 'robot_status.txt'
        self.initialize_status_file()

    def initialize_status_file(self):
        """
        Initialize the status file with default values for each robot.
        """
        with open(self.status_file_path, 'w') as file:
            file.write('Roomba:False\nBeaker:False\nBunsen:False\n')

    def set_ready_status(self, roomba_status=None, beaker_status=None, bunsen_status=None):
        print("Updating robot statuses...")

        # Reading current statuses from the file
        with open(self.status_file_path, 'r') as file:
            lines = file.readlines()
            current_statuses = {}
            for line in lines:
                key, value = line.strip().split(':')
                current_statuses[key] = value == 'True'

        # Updating the statuses with provided values
        if roomba_status is not None:
            current_statuses['Roomba'] = roomba_status
        if beaker_status is not None:
            current_statuses['Beaker'] = beaker_status
        if bunsen_status is not None:
            current_statuses['Bunsen'] = bunsen_status

        # Writing updated statuses back to the file
        with open(self.status_file_path, 'w') as file:
            for robot, status in current_statuses.items():
                file.write(f'{robot}:{status}\n')

        print("Statuses updated.")

    def publish_ready_status(self):
        print("\nPublishing ready statuses...")

        # Reading statuses from the file
        with open(self.status_file_path, 'r') as file:
            lines = file.readlines()
            current_statuses = {}
            for line in lines:
                key, value = line.strip().split(':')
                current_statuses[key] = value == 'True'

        # Creating the message to publish
        msg = ReadyStatus()
        msg.roomba = current_statuses['Roomba']
        msg.beaker = current_statuses['Beaker']
        msg.bunsen = current_statuses['Bunsen']

        # Publishing the message
        self.ready_status_publisher_.publish(msg)
        print("Ready statuses published.")

