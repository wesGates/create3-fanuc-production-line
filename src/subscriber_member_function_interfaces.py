import rclpy
from rclpy.node import Node

import sys
sys.path.append("../dependencies/")
# print("PRINT PATH: ", sys.path)

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

import my_interfaces
from my_interfaces.msg import Num, Base2status, Base3status   # CHANGE

# Subscriber is placed in the robot's main code.

class Base2Subscriber(Node):

    def __init__(self):
        super().__init__('base2_subscriber')
        self.base2_subscription_ = self.create_subscription(Base2status, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
            self.get_logger().info('I heard roomba status: "%d"' % msg.roomba) # CHANGE
            self.get_logger().info('I heard beaker status: "%d"' % msg.beaker) # CHANGE
            print("\n")


def main(args=None):
    rclpy.init(args=args)

    base2_subscriber = Base2Subscriber()

    rclpy.spin(base2_subscriber)

    base2_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()