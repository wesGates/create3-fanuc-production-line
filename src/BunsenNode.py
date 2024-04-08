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

from ReadyStatusPublisherNode import ReadyStatusPublisherNode

import rclpy
from rclpy.node import Node
import time

from my_interfaces.msg import ReadyStatus  # Adjust the import path based on your package structure


# Initialize and connect to OTHER nodes
rclpy.init()
namespace = 'bunsen'
ready_status_publisher_node = ReadyStatusPublisherNode()
readiness_tracker_node = ReadinessTrackerNode()


class BunsenNode(Node):
	""" Initialization and other methods remain unchanged from RoombaNode"""


	def __init__(self):
		super().__init__('bunsen_node')
		self.status_file_path = 'robot_status.txt'

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
		print("msg.roomba_base2 : 	", msg.roomba)
		print("msg.beaker : 	", msg.beaker)
		print("msg.beaker_conv:",msg.beaker_conv)
		print("msg.bunsen_conv:",msg.bunsen_conv)
		print("msg.bunsen : 	", msg.bunsen)
		print("msg.roomba_base3 : 	", msg.roomba_base3)
		print("End of RoombaNode callback \n")

		statuses = [
            str(msg.roomba_base2),
            str(msg.beaker),
            str(msg.beaker_conv),
            str(msg.bunsen_conv),
            str(msg.bunsen),
            str(msg.roomba_base3),
        ]

		with open(self.status_file_path, 'w') as file:
			file.write(','.join(statuses) + '\n')

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

	def set_bunsen_conv_true(self):
		try: 
			print("Setting bunsen_CONV: True")
			self.ready_status_publisher_node.set_ready_status(bunsen_conv_status=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging

	def set_bunsen_conv_false(self):
		try: 
			print("Setting bunsen_CONV: False")
			self.ready_status_publisher_node.set_ready_status(bunsen_conv_status=False)
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
					print("SUCCESS")
					self.get_logger().info(f'{other_robot} status matches the expected status: {expected_status}.')
					break  # Exit the loop if the status matches
				else:
					# This message will now reflect the actual status received and the expected status.
					self.get_logger().info(f'{other_robot} status does not match. Expected: {expected_status}. Actual: {actual_status}. Retrying...')
					time.sleep(1)  # Wait for a bit before retrying
			else:
				self.get_logger().error(f'Exception while calling service: {future.exception()}')
				time.sleep(1)  # Wait for a bit before retrying in case of an exception

	def wait_for_ready(self, robot_status_expectations):
		"""
		Waits for the specified robots to reach the desired ready status.

		:param robot_status_expectations: A dictionary with robot names as keys and expected statuses (True/False) as values.
		
		Ex) This example function call waits for the 'roomba' status to be False, and 'beaker' to be True:
		node.wait_for_ready({'roomba': False, 'beaker': True})

		NOTE: Always set your robot's status to the status you are calling before calling this function.
		^ If you call 'beaker': True, be sure to set beaker to True before calling this function or you will get stuck
		NOTE: Be aware this function is GPT-generated.
		"""

		print("Waiting for specific ready statuses...")
		timeout = 100  # Timeout for the overall waiting
		check_interval = 1.0  # Time to wait in spin_once for new messages
		start_time = time.time()


		while time.time() - start_time < timeout:
			# Process incoming messages and wait up to 'check_interval' seconds for new ones
			rclpy.spin_once(self, timeout_sec=check_interval)
			
			if self.latest_ready_status:
				# Print current statuses for printing
				current_statuses = {robot: getattr(self.latest_ready_status, robot.lower(), None) 
									for robot in robot_status_expectations.keys()}
				print(f"\n Current statuses: {current_statuses}")

				# Check if the desired status is achieved
				status_check = all(getattr(self.latest_ready_status, robot.lower()) == status 
								for robot, status in robot_status_expectations.items())

				if status_check:
					print(f"\n !!! All specified robots have reached the expected ready statuses !!!")
					return
				else:
					statuses = ', '.join([f"{robot} is {'True' if status else 'False'}" for robot, status in robot_status_expectations.items()])
					print(f"       Waiting for: {statuses}...")
			else:
				print("\n Waiting for the first status update...")

		print("Timeout reached without all specified robots reaching the expected ready statuses.")


	##############################################################################################
	""" Robot-specific status checking starts here. """			
	##############################################################################################

	def check_roomba_base3(self):
		# NOTE: Remember to set beaker status to False after picking up dice block!
		print("Setting beaker status to True")
		self.set_beaker_true()
		print("Check if roomba is ready at base3")
		self.check_robot_status('roomba', True)

	def check_beaker_conv(self):
		print("Check if beaker conveyor is ready")
		self.check_robot_status('beaker_conv', True)



# The following Key Commander is used for manually checking outputs in the terminal
if __name__ == '__main__':
	# rclpy.init()

	bunsen = BunsenNode()
	exec = MultiThreadedExecutor(8)

	exec.add_node(bunsen)
	exec.add_node(ready_status_publisher_node)

	time.sleep(1.0)
	
	keycom = KeyCommander([
		(KeyCode(char='v'), bunsen.display_robot_statuses),
		(KeyCode(char='i'), bunsen.set_bunsen_conv_true),
		(KeyCode(char='j'), bunsen.set_bunsen_conv_false),
		(KeyCode(char='o'), bunsen.set_bunsen_true),
		(KeyCode(char='k'), bunsen.set_bunsen_false),
		(KeyCode(char='p'), bunsen.publish_robot_status),

		(KeyCode(char='4'), bunsen.check_roomba_base3),
		(KeyCode(char='5'), bunsen.check_beaker_conv),

	])
	print(" Press 'v' to display all robot states in the text file")
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
