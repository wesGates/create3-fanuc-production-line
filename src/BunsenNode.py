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
from ReadyStatusPublisherNode import ReadyStatusPublisherNode

import rclpy
from rclpy.node import Node
import time

from my_interfaces.msg import ReadyStatus  # Adjust the import path based on your package structure


# Initialize and connect to OTHER nodes
rclpy.init()
namespace = 'bunsen'
ready_status_publisher_node = ReadyStatusPublisherNode()

class BunsenNode(Node):
	""" Initialization and other methods remain unchanged from RoombaNode"""


	def __init__(self):
		super().__init__('bunsen_node')

		# Establish the node objects within the class for node operations
		self.ready_status_publisher_node = ready_status_publisher_node


	def update_robot_status(self):
		# Update bunsen's status to ready
		self.ready_status_publisher_node.set_ready_status(bunsen_status=True)
		self.get_logger().info('Updated bunsen status to ready')


	def publish_robot_status(self):
		# Now, publish the updated status
		self.ready_status_publisher_node.publish_ready_status()
		self.get_logger().info('Published the updated ready status')


	def display_robot_statuses(self):
		# Used in key commander to display the robot states on keypress in the terminal
		try: 
			self.ready_status_publisher_node.display_robot_statuses()
		except:
			self.get_logger().error(f"Error in display: {error}")

  
	def set_bunsen_true(self):
		try: 
			self.ready_status_publisher_node.set_ready_status(bunsen_status=True)
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_bunsen_false(self):
		try: 
			self.ready_status_publisher_node.set_ready_status(bunsen_status=False)
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


# The following Key Commander is used for manually checking outputs in the terminal
if __name__ == '__main__':
	# rclpy.init()

	bunsen = BunsenNode()
	exec = MultiThreadedExecutor(8)

	exec.add_node(bunsen)
	exec.add_node(ready_status_publisher_node)

	time.sleep(1.0)
	
	keycom = KeyCommander([
		(KeyCode(char='d'), bunsen.display_robot_statuses),
		(KeyCode(char='u'), bunsen.set_bunsen_true),
		(KeyCode(char='h'), bunsen.set_bunsen_false),

	])
	print(" Press 'd' to display all robot states in the text file")
	print(" Press 'u' to set bunsen's status as 'True'")
	print(" Press 'h' to set bunsen's status as 'False'")

	try:
		exec.spin()  # execute Roomba callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		bunsen.destroy_node()
		rclpy.try_shutdown()
