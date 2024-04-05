import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
import sys
sys.path.append("../dependencies/")

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from pynput import keyboard


import my_interfaces
from my_interfaces.msg import ReadyStatus  # CHANGE
from ReadyStatusPublisherNode import ReadyStatusPublisherNode

# Initialize and connect to OTHER nodes
rclpy.init()
namespace = 'create3_05AE'
ready_status_publisher_node = ReadyStatusPublisherNode()


class RoombaNode(Node):
	"""TODO: Determine if/how 'namespace' is important. """

	def __init__(self):
		super().__init__('roomba_node')

		# Establish the node objects within the class for node operations
		self.ready_status_publisher_node = ready_status_publisher_node

		self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
															 self.ready_status_callback, 10)

		# Initializing status variables to store the 'ready' data received from the publisher
		# These variables are updated in the callback functions.
		self.latest_ready_status = None

	def ready_status_callback(self, msg):
		"""
		The other nodes don't have this function, yet. 
		I think they will need it in order to do their own wait_for_all_ready operations.
		"""
		print("Start of RoombaNode callback...")
		
		# Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		self.latest_ready_status = msg
		# self.roomba_status = self.latest_ready_status.roomba

		# DEBUGGING
		print("msg.roomba (received from publisher): ", msg.roomba)
		print("msg.beaker (received from publisher): ", msg.beaker)
		print("msg.bunsen (received from publisher): ", msg.bunsen)

		# self.get_logger().info(f"Received /robot_ready_status: {self.latest_ready_status}")
		print("End of RoombaNode callback \n")


	def wait_for_base2_ready(self):
		"""
		This function waits until the status of both the roomba, and beaker is True.
		This function will be placed in both RoombaNode and BeakerNode.
		NOTE: The robot that calls this function must first set their status to True, or it will time out.
		"""
		
		print("Waiting for ready: roomba and beaker. ")
		timeout = 10  # seconds
		check_interval = 2  # seconds
		start_time = time.time()

		while time.time() - start_time < timeout:

			# Publish the status to trigger an update
			self.ready_status_publisher_node.publish_ready_status()
			print("Check latest ready statuses: ", self.latest_ready_status)

			# Wait a short time for the message to be published and processed
			rclpy.spin_once(self, timeout_sec=check_interval)
			time.sleep(1.0)

			# Check the latest status
			if self.latest_ready_status and self.latest_ready_status.roomba and self.latest_ready_status.beaker:
				print("!!! Both Roomba and Beaker are ready!!!!")
				# !!!! Perhaps reset the ready statuses of both robots to false?
				return
			else:
				print("Waiting for both Roomba and Beaker to be ready...")

		print("Timeout reached without both robots being ready.")

	def dice_block_handoff(self):
		""" Logic for docking the roomba, waiting for beaker to be ready, then doing the next action or something"""


	def display_robot_statuses(self):
		try: 
			self.ready_status_publisher_node.display_robot_statuses()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_roomba_true(self):
		try: 
			self.ready_status_publisher_node.set_ready_status(roomba_status=True)
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_roomba_false(self):
		try: 
			self.ready_status_publisher_node.set_ready_status(roomba_status=False)
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def main(self):
		print("Navigation code goes here. ")


if __name__ == '__main__':
	# rclpy.init()

	roomba = RoombaNode()
	exec = MultiThreadedExecutor(8)

	exec.add_node(roomba)
	exec.add_node(ready_status_publisher_node)

	time.sleep(1.0)
	
	keycom = KeyCommander([
		(KeyCode(char='u'), roomba.main),
		(KeyCode(char='d'), roomba.display_robot_statuses),
		(KeyCode(char='t'), roomba.set_roomba_true),
		(KeyCode(char='f'), roomba.set_roomba_false),
		(KeyCode(char='w'), roomba.wait_for_base2_ready),
	])
	print(" Press 'u' to intitiate main routine") # This will be the navigation code
	print(" Press 'd' to display all robot states in the text file")
	print(" Press 't' to set the roomba's status as 'True'")
	print(" Press 'f' to set the roomba's status as 'False'")
	print(" Press 'w' to wait for all statuses to be true before continuing'")

	try:
		exec.spin()  # execute Roomba callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		roomba.destroy_node()
		rclpy.try_shutdown()
