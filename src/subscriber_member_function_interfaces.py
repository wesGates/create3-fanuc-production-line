import rclpy
from rclpy.node import Node

import sys
sys.path.append("../dependencies/")
# print("PRINT PATH: ", sys.path)

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

import my_interfaces
from my_interfaces.msg import ReadyStatus, Num, Base2status, Base3status   # CHANGE


namespace = 'create3-05AE'

class ReadyStatusSubscriber(Node):

    def __init__(self, namespace):
        super().__init__('base2_subscriber')
        self.base2_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', self.listener_callback, 10)

    def listener_callback(self, msg):
            self.get_logger().info('I heard roomba status: "%d"' % msg.roomba) # CHANGE
            self.get_logger().info('I heard beaker status: "%d"' % msg.beaker) # CHANGE
            self.get_logger().info('I heard beaker status: "%d"' % msg.bunsen) # CHANGE
            print("\n")


def main(args=None):
    rclpy.init(args=args)

    base2_subscriber = ReadyStatusSubscriber(namespace)

    rclpy.spin(base2_subscriber)

    base2_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()