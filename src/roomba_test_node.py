import rclpy
from rclpy.node import Node

import time
import sys
sys.path.append("../dependencies/")
# print("PRINT PATH: ", sys.path)

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

import my_interfaces
from my_interfaces.msg import ReadyStatus, Num, Base2status, Base3status   # CHANGE
from subscriber_member_function_interfaces import ReadyStatusSubscriber


# This code uses the subscriber_member_function_interfaces.py

rclpy.init()
namespace = 'create3-05AE'
# base2_status = Base2Subscriber(namespace)
ready_status = ReadyStatusSubscriber(namespace)

class Roomba(Node):

    def __init__(self, namespace):
        super().__init__('roomba_node')
        # self.base2_status = base2_status
        self.ready_status = ready_status
        self.robot_ready_subscription_ = self.create_subscription(Base2status, 'robot_ready_status', self.listener_callback, 10)

    def listener_callback(self, msg):
            self.get_logger().info('I heard roomba status: "%d"' % msg.roomba) # CHANGE
            self.get_logger().info('I heard beaker status: "%d"' % msg.beaker) # CHANGE
            print("\n")

    def update_roomba_status(self, ready=False):
        # Logic to determine if Roomba is ready (this is just a placeholder for actual logic)
        ready = not ready
        is_ready = ready  # or some condition that evaluates Roomba's readiness
        self.ready_status_publisher.roomba_status_updater(is_ready)


def main(args=None):
    # rclpy.init(args=args)

    roomba = Roomba(namespace)
    # exec.add_node(base2_status)

    rclpy.spin(ready_status)

    time.sleep(1.0)
    

    ready_status.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()