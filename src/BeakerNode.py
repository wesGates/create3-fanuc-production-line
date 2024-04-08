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


import my_interfaces
from my_interfaces.msg import ReadyStatus
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
namespace = 'beaker'
ready_status_publisher_node = ReadyStatusPublisherNode()
readiness_tracker_node = ReadinessTrackerNode()

class BeakerNode(Node):


	def __init__(self):
		super().__init__('beaker_node')
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

	##############################################################################################
	""" General ready status services and publishing stuff starts here """			
	##############################################################################################
	def ready_status_callback(self, msg):
		"""
		The other nodes don't have this function, yet. 
		I think they will need it in order to do their own wait_for_all_ready operations.
		"""
		# Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		
		# self.roomba_status = self.latest_ready_status.roomba

		# DEBUGGING
		print("DEBUG: Start of RoombaNode callback...")
		print("msg.roomba_base2 : 	", msg.roomba_base2)
		print("msg.beaker : 	", msg.beaker)
		print("msg.beaker_conv:",msg.beaker_conv)
		print("msg.bunsen_conv:",msg.bunsen_conv)
		print("msg.bunsen : 	", msg.bunsen)
		print("msg.roomba_base3 : 	", msg.roomba_base3)
		print("End of RoombaNode callback \n")

		self.latest_ready_status = msg
		statuses = [None,None,None,None,None,None]
		statuses[0] = msg.roomba_base2
		statuses[1] = msg.beaker
		statuses[2] = msg.beaker_conv
		statuses[3] = msg.bunsen_conv
		statuses[4] = msg.bunsen
		statuses[5] = msg.roomba_base3

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


	def set_beaker_true(self):
		try: 
			print("Setting BEAKER: True")
			self.ready_status_publisher_node.set_ready_status(beaker_status=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_beaker_false(self):
		try: 
			print("Setting BEAKER: False")
			self.ready_status_publisher_node.set_ready_status(beaker_status=False)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging

	def set_beaker_conv_true(self):
		try: 
			print("Setting BEAKER_CONV: True")
			self.ready_status_publisher_node.set_ready_status(beaker_conv_status=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging

	def set_beaker_conv_false(self):
		try: 
			print("Setting BEAKER_CONV: False")
			self.ready_status_publisher_node.set_ready_status(beaker_conv_status=False)
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


	##############################################################################################
	""" Robot-specific status checking starts here. """			
	##############################################################################################

	# IMPORTANT! Always set crx10 statuses to False after picking up dice block

	def check_roomba_base2(self):
		# NOTE: Remember to set beaker status to False after picking up dice block!
		print("Setting beaker status to True")
		self.set_beaker_true()

		print("Check if roomba is ready at base2")
		self.check_robot_status('roomba_base2', True)

		print("!!! Both roomba_base2 and beaker are ready !!!")
		print("\n Remember to set beaker to False for the dice block handoff. ")

	def check_bunsen_conv(self):
		# NOTE: Remember to set beaker_conv to false after this function

		print("Setting beaker_conv to true")
		self.set_beaker_conv_true

		print("Check if bunsen is ready at conveyor")
		self.check_robot_status('bunsen_conv', True)

		print("!!! Both beaker_conv and bunsen_conv are ready !!!")
		print("\n Remember to set beaker_conv to False.")



# The following Key Commander is used for manually checking outputs in the terminal
if __name__ == '__main__':
	# rclpy.init()

	beaker = BeakerNode()
	exec = MultiThreadedExecutor(8)

	exec.add_node(beaker)
	exec.add_node(ready_status_publisher_node)

	time.sleep(1.0)
	
	keycom = KeyCommander([
		(KeyCode(char='v'), beaker.display_robot_statuses),
		(KeyCode(char='y'), beaker.set_beaker_true),
		(KeyCode(char='g'), beaker.set_beaker_false),
		(KeyCode(char='u'), beaker.set_beaker_conv_true),
		(KeyCode(char='h'), beaker.set_beaker_conv_false),
		(KeyCode(char='p'), beaker.publish_robot_status),

		(KeyCode(char='4'), beaker.check_roomba_base2),
		(KeyCode(char='5'), beaker.check_bunsen_conv),

	])
	print(" Press 'v' to display all robot states in the text file")
	print(" Press 'y' to set beaker's status as 'True'")
	print(" Press 'g' to set beaker's status as 'False'")
	print(" Press 'u' to set beaker_conv status as 'True'")
	print(" Press 'h' to set beaker_conv status as 'False'")
	print(" Press 'p' to manually publish the contents of the status text file")

	print(" Press '4' to call the service and check if roomba is ready at base2")
	print(" Press '5' to call the service and check if bunsen is ready at the conveyor")


	try:
		exec.spin()  # execute Roomba callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		beaker.destroy_node()
		rclpy.try_shutdown()
