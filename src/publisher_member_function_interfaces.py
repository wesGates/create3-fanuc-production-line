""" 
This is the big publisher that publishes all ready statuses.

Import this node, and use the publish_ready_status method to view the status of all robots.

Use the set_ready_status method to update the status of your robot only.
"""

import sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

import my_interfaces
from my_interfaces.msg import Num, ReadyStatus, Base3status, Base2status   # CHANGE depending on messages to include

# class Base2Publisher(Node):
class ReadyStatusPublisherNode(Node):

    # def __init__(self, namespace):
    def __init__(self):
        
        super().__init__('ready_status_publisher')
        self.ready_status_publisher_ = self.create_publisher(ReadyStatus, 'robot_ready_status', 10)     # CHANGE
        
        # Initialize variables
        self.roomba = False
        self.beaker = False
        self.bunsen = False


    def set_ready_status(self, roomba_status=None, beaker_status=None, bunsen_status=None):
        """
        Update the internal status of roomba, beaker, and bunsen. Only updates values that are provided.

        
        ex) Set the ready status of beaker to true
        self.ReadyStatusPublisherNode_instance.set_ready_status(beaker=True)

        ex) View the ready status of all robots and store 

        """
        print(f"Statuses before set_ready_status () update - Roomba: {self.roomba}, Beaker: {self.beaker}, Bunsen: {self.bunsen}")

        if roomba_status is not None:
            self.roomba = roomba_status
        if beaker_status is not None:
            self.beaker = beaker_status
        if bunsen_status is not None:
            self.bunsen = bunsen_status

        print(f"Updated statuses - Roomba: {self.roomba}, Beaker: {self.beaker}, Bunsen: {self.bunsen}")




    def publish_ready_status(self):
        """
        Publish the status of robots.

        :param ready: A boolean value indicating if Roomba is ready/has reached the position.
        """
        print("\nStart of the publish_ready_status() function within the publisher node")
        # msg = Bool()
        msg = ReadyStatus()
        msg.roomba = self.roomba
        msg.beaker = self.beaker
        msg.bunsen = self.bunsen

        self.ready_status_publisher_.publish(msg)
        print("Ready status published using publish_ready_status() ", msg)
        print("End of the publish_ready_status() function within the publisher node")
