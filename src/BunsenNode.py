import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


import time
import sys
sys.path.append("../dependencies/")

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from pynput import keyboard


import my_interfaces
from my_interfaces.msg import ReadyStatus  # CHANGE
from my_interfaces.srv import CheckReadiness
from ReadyStatusPublisherNode import ReadyStatusPublisherNode
from ReadinessTrackerNode import ReadinessTrackerNode


# Initialize and connect to OTHER nodes
rclpy.init()
namespace = 'bunsen'
ready_status_publisher_node = ReadyStatusPublisherNode()
readiness_tracker_node = ReadinessTrackerNode()


class BunsenNode(Node):
	""" Initialization and other methods remain unchanged from RoombaNode"""


	def __init__(self):
		super().__init__('bunsen_node')


		# Initialize node objects within the class for node operations
		self.ready_status_publisher_node = ready_status_publisher_node
		self.readiness_tracker_node = readiness_tracker_node


		# Creating callback group for busy ReadyStatus message
		cb_ready_status = MutuallyExclusiveCallbackGroup()

		# Subscribe to the published topic to update latest_ready_status
		self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
															 self.ready_status_callback, 10, callback_group=cb_ready_status)
		
		# Service for checking the status of any robot
		self.client = self.create_client(CheckReadiness, 'check_readiness')

		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
		print("Readiness Tracker is Available. ")

		# Initialize the variable that stores the robot ready status information
		# Ready statuses are updated in the callback functions.
		self.latest_ready_status = None


	def ready_status_callback(self, msg):
		"""
		The other nodes don't have this function, yet. 
		I think they will need it in order to do their own wait_for_all_ready operations.
		"""
		# Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		self.latest_ready_status = msg
		# self.roomba_status = self.latest_ready_status.roomba

		# DEBUGGING
		print("DEBUG: Start of RoombaNode callback...")
		print("msg.roomba : 	", msg.roomba)
		print("msg.beaker : 	", msg.beaker)
		print("msg.beaker_conv:",msg.beaker_conv)
		print("msg.bunsen_conv:",msg.bunsen_conv)
		print("msg.bunsen : 	", msg.bunsen)
		print("End of RoombaNode callback \n")

		# self.get_logger().info(f"Received /robot_ready_status: {self.latest_ready_status}")


	def display_robot_statuses(self):
		# DUBUGGING: Used in key commander to display the robot states on keypress in the terminal
		try: 
			self.ready_status_publisher_node.display_robot_statuses()
		except:
			self.get_logger().error(f"Error in display: {error}")


	def publish_robot_status(self):
		self.ready_status_publisher_node.publish_ready_status()
		self.get_logger().info('Published the updated ready status')

  
	def set_bunsen_true(self):
		try: 
			print("Setting BUNSEN: True")
			self.ready_status_publisher_node.set_ready_status(bunsen_status=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_bunsen_false(self):
		try: 
			print("Setting BUNSEN: False")
			self.ready_status_publisher_node.set_ready_status(bunsen_status=False)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging




	def send_request(self, other_robot):
		request = CheckReadiness.Request()
		request.robot1 = other_robot
		future = self.client.call_async(request)
		return future


	def check_robot_status(self, other_robot, expected_status):
		self.get_logger().info(f"Checking if {other_robot} is {'ready' if expected_status else 'not ready'}.")

		while True:
			future = self.send_request(other_robot)
			rclpy.spin_until_future_complete(self, future)

			if future.result() is not None:
				actual_status = future.result().ready
				if actual_status == expected_status:
					self.get_logger().info(f'{other_robot} status matches the expected status: {expected_status}.')
					break  # Exit the loop if the status matches
				else:
					# This message will now reflect the actual status received and the expected status.
					self.get_logger().info(f'{other_robot} status does not match. Expected: {expected_status}. Actual: {actual_status}. Retrying...')
					time.sleep(1)  # Wait for a bit before retrying
			else:
				self.get_logger().error(f'Exception while calling service: {future.exception()}')
				time.sleep(1)  # Wait for a bit before retrying in case of an exception


	##############################################################################################
	""" Robot-specific code starts here. """			
	##############################################################################################




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
		(KeyCode(char='b'), bunsen.check_readiness),

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
