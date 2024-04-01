import rclpy
from rclpy.node import Node

import sys
sys.path.append("../dependencies/")
# print("PRINT PATH: ", sys.path)

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

import my_interfaces
from my_interfaces.msg import Num, Base2status, Base3status   # CHANGE


class Base2Publisher(Node):

    def __init__(self):
        super().__init__('base2_publisher')
        self.base2_publisher_ = self.create_publisher(Base2status, 'topic', 10)     # CHANGE
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize variables
        self.roomba = False
        self.beaker = False


    def timer_callback(self):
        msg = Base2status()                                           # CHANGE
        msg.roomba = self.roomba
        msg.beaker = self.beaker
        print("roomba status: ", msg.roomba)
        print("beaker status: ", msg.beaker)

        # Publish the message
        self.base2_publisher_.publish(msg)
        print("\n")

        # self.get_logger().info('Publishing roomba status: "%d"' % msg.roomba)  # CHANGE
        # self.get_logger().info('Publishing beaker status: "%d"' % msg.beaker)  # CHANGE


    # def status_updater(self):
    #     print(msg.roomba)


def main(args=None):
    rclpy.init(args=args)

    base2_publisher = Base2Publisher()

    rclpy.spin(base2_publisher)

    base2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()