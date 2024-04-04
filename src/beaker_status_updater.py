import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
import sys
sys.path.append("../dependencies/")

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander


import my_interfaces
from my_interfaces.msg import ReadyStatus, Num, Base2status, Base3status   # CHANGE
from publisher_member_function_interfaces import ReadyStatusPublisherNode

import rclpy
from rclpy.node import Node
import time

from my_interfaces.msg import ReadyStatus  # Adjust the import path based on your package structure


class BeakerStatusUpdater(Node):
    def __init__(self, ready_status_publisher_node):
        super().__init__('beaker_status_updater')
        # Store the instance of ReadyStatusPublisherNode
        self.ready_status_publisher_node = ready_status_publisher_node

    def update_and_publish_beaker_status(self):
        # Update Beaker's status to ready
        self.ready_status_publisher_node.set_ready_status(beaker_status=True)
        self.get_logger().info('Updated Beaker status to ready')

        # Now, publish the updated status
        self.ready_status_publisher_node.publish_ready_status()
        self.get_logger().info('Published the updated ready status')

def main(args=None):
    rclpy.init(args=args)
    ready_status_publisher_node = ReadyStatusPublisherNode()
    beaker_status_updater = BeakerStatusUpdater(ready_status_publisher_node)

    # Update and publish Beaker's status
    beaker_status_updater.update_and_publish_beaker_status()

    # Keep the node alive and responsive
    rclpy.spin(beaker_status_updater)

    # Clean up before shutting down
    beaker_status_updater.destroy_node()
    ready_status_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()