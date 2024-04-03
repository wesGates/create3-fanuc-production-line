""" 
This is the big publisher that publishes all ready statuses
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
        
        # timer_period = 1.0
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize variables
        self.roomba = False
        self.beaker = False
        self.bunsen = False


    # def timer_callback(self):

    #     print("Start of the publisher's timer_callback")
    #     # Continuously read the ready states
    #     msg = ReadyStatus() # msg is set to the ReadyStatus msg

    #     # Ex) how to access the ready state for each value
    #     msg.roomba = self.roomba
    #     msg.beaker = self.beaker
    #     msg.bunsen = self.bunsen

    #     # # DEBUGGING
    #     print("roomba status (within the publisher): ", msg.roomba)
    #     print("beaker status (within the publisher): ", msg.beaker)
    #     print("beaker status (within the publisher): ", msg.bunsen)

    #     # ## TESTING: Change the message to be published
    #     # msg.roomba = True
    #     # print("Roomba message after changing to true, within publisher:", msg.roomba)

    #     # Publish the message
    #     print("Full message published by the publisher:", msg)
    #     self.ready_status_publisher_.publish(msg)

    #     # self.get_logger().info('Publishing roomba status: "%d"' % msg.roomba)  # CHANGE
    #     # self.get_logger().info('Publishing beaker status: "%d"' % msg.beaker)  # CHANGE
    #     # self.get_logger().info('Publishing beaker status: "%d"' % msg.bunsen)  # CHANGE
    #     print("End of the publisher's timer_callback")


    def set_ready_status(self, roomba_status=None, beaker_status=None, bunsen_status=None):
        """
        Update the internal status of roomba, beaker, and bunsen. Only updates values that are provided.

        :param roomba_status: A boolean value indicating Roomba's ready status or None if no update is needed.
        :param beaker_status: A boolean value indicating Beaker's ready status or None if no update is needed.
        :param bunsen_status: A boolean value indicating Bunsen's ready status or None if no update is needed.
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
        print("Start of the publish_ready_status() function within the publisher node")
        # msg = Bool()
        msg = ReadyStatus()
        msg.roomba = self.roomba
        msg.beaker = self.beaker
        msg.bunsen = self.bunsen

        self.ready_status_publisher_.publish(msg)
        print("Ready status published using publish_ready_status() ", msg)
        print("End of the publish_ready_status() function within the publisher node")

        # print(f"Updated Roomba status to: {self.roomba}")


# def main(args=None):
#     rclpy.init(args=args)

#     ready_status_publisher = ReadyStatusPublisher()
#     rclpy.spin(ready_status_publisher)

#     ready_status_publisher.roomba_status_updater(read=True)

#     ready_status_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()