import rclpy
from rclpy.node import Node

import sys
sys.path.append("../dependencies/")
# DEBUG: print("PRINT PATH: ", sys.path)

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

import my_interfaces
from my_interfaces.msg import Num, ReadyStatus, Base3status, Base2status   # CHANGE depending on messages to include

# This is the big publisher that publishes the status of all ready statuses

# class Base2Publisher(Node):
class ReadyStatusPublisher(Node):

    def __init__(self):
        super().__init__('ready_status_publisher')
        self.ready_status_publisher_ = self.create_publisher(ReadyStatus, 'robot_ready_status', 10)     # CHANGE
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize variables
        self.roomba = False
        self.beaker = False
        self.bunsen = False


    def timer_callback(self):
        # Continuously read the ready states
        msg = ReadyStatus() # CHANGE

        msg.roomba = self.roomba
        msg.beaker = self.beaker
        msg.bunsen = self.bunsen

        # # DEBUG
        print("roomba status: ", msg.roomba)
        print("beaker status: ", msg.beaker)
        print("beaker status: ", msg.bunsen)

        # Publish the message
        self.ready_status_publisher_.publish(msg)
        print("\n")

        # self.get_logger().info('Publishing roomba status: "%d"' % msg.roomba)  # CHANGE
        # self.get_logger().info('Publishing beaker status: "%d"' % msg.beaker)  # CHANGE
        # self.get_logger().info('Publishing beaker status: "%d"' % msg.bunsen)  # CHANGE


    def roomba_status_updater(self, ready):
        """
        Update the status of roomba.

        :param ready: A boolean value indicating if Roomba is ready/has reached the position.
        """
        self.roomba = ready
        print(f"Updated Roomba status to: {self.roomba}")


def main(args=None):
    rclpy.init(args=args)

    ready_status_publisher = ReadyStatusPublisher()

    rclpy.spin(ready_status_publisher)

    ready_status_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()