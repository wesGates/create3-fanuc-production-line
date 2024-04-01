import rclpy
from rclpy.node import Node

import sys
sys.path.append("../dependencies/")
# print("PRINT PATH: ", sys.path)

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

import my_interfaces
from my_interfaces.msg import Num, Base2status, Base3status   # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Base2status,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard roomba status: "%d"' % msg.roomba) # CHANGE
            self.get_logger().info('I heard beaker status: "%d"' % msg.beaker) # CHANGE

    # def __init__(self):
    #     super().__init__('minimal_subscriber')
    #     self.subscription = self.create_subscription(
    #         Num,                                              # CHANGE
    #         'topic',
    #         self.listener_callback,
    #         10)
    #     self.subscription

    # def listener_callback(self, msg):
    #         self.get_logger().info('I heard roomba status: "%d"' % msg.Num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()