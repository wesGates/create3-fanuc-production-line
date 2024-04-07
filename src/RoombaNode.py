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
namespace = 'create3_05AE'
ready_status_publisher_node = ReadyStatusPublisherNode()
readiness_tracker_node = ReadinessTrackerNode()


class RoombaNode(Node):


	def __init__(self):
		super().__init__('roomba_node')

		# Initialize node objects within the class for node operations
		self.ready_status_publisher_node = ready_status_publisher_node
		self.readiness_tracker_node = readiness_tracker_node

		# Creating callback groups for managing subscription callbacks
		cb_ready_status = MutuallyExclusiveCallbackGroup()


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
		print("msg.roomba_base2 : 	", msg.roomba_base2)
		print("msg.beaker : 		", msg.beaker)
		print("msg.beaker_conv :	",msg.beaker_conv)
		print("msg.bunsen_conv:  	",msg.bunsen_conv)
		print("msg.bunsen : 		", msg.bunsen)
		print("msg.roomba_base3 :	", msg.roomba_base3)
		print("End of RoombaNode callback \n")

		# self.get_logger().info(f"Received /robot_ready_status: {self.latest_ready_status}")


	def display_robot_statuses(self):
		# DUBUGGING: Used in key commander to display the robot states on keypress in the terminal
		try: 
			self.ready_status_publisher_node.display_robot_statuses()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging

	def publish_robot_status(self):
		self.ready_status_publisher_node.publish_ready_status()
		self.get_logger().info('Published the updated ready status')


	def set_roomba_base2_true(self):
		try: 
			print("Setting ROOMBA: True")
			self.ready_status_publisher_node.set_ready_status(roomba_status_base2=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging



	def set_roomba_base2_false(self):
		try: 
			print("Setting ROOMBA: False")
			self.ready_status_publisher_node.set_ready_status(roomba_status_base2=False)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_roomba_base3_true(self):
		try: 
			print("Setting ROOMBA at BASE3: True")
			self.ready_status_publisher_node.set_ready_status(roomba_status_base3=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging
	
	def set_roomba_base3_false(self):
		try: 
			print("Setting ROOMBA at BASE3: False")
			self.ready_status_publisher_node.set_ready_status(roomba_status_base3=False)
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
	""" Robot-specific status checking starts here. """			
	##############################################################################################

	# IMPORTANT! Always set your robot's status before checking other robots
	# NOTE: Always set crx10 statuses to False after picking up dice block

	def check_base2(self):
		self.set_roomba_true()
		print("Check if beaker is ready at base2")
		self.check_robot_status('beaker', True)

	def check_dice_block_handoff_base2(self):
		print("Checking whether beaker has the block before moving to base3")
		self.check_robot_status('beaker', False)

	def check_base3(self):
		print("Check if beaker is ready at base3")
		self.check_robot_status('bunsen', True)

	def check_dice_block_handoff_base3(self):
		# Check whether beaker has retrived the dice block
		print("Checking whether beaker has the block before moving to base1")
		self.check_robot_status('bunsen', False)



	def main(self):

		pass



# The following Key Commander is used for manually checking outputs in the terminal
if __name__ == '__main__':

	roomba = RoombaNode()
	exec = MultiThreadedExecutor(8)

	exec.add_node(roomba)
	exec.add_node(ready_status_publisher_node)
	exec.add_node(readiness_tracker_node)


	time.sleep(1.0)
	
	keycom = KeyCommander([
		(KeyCode(char='u'), roomba.main), 
		(KeyCode(char='v'), roomba.display_robot_statuses),
		(KeyCode(char='r'), roomba.set_roomba_base2_true),
		(KeyCode(char='d'), roomba.set_roomba_base2_false),
		(KeyCode(char='t'), roomba.set_roomba_base3_true),
		(KeyCode(char='f'), roomba.set_roomba_base3_false),

		(KeyCode(char='`'), roomba.check_base2),
		(KeyCode(char='1'), roomba.check_dice_block_handoff_base2),
		(KeyCode(char='2'), roomba.check_base3),
		(KeyCode(char='3'), roomba.check_dice_block_handoff_base3),


	])
	# print(" Press 'u' to intitiate main routine") # This will be the navigation code
	print(" Press 'u' to intitiate main routine") # TESTING W/ PASSING VARS (KEY COMM ISSUES)
	print(" Press 'v' to display all robot states in the text file")
	print(" Press 'r' to set the roomba's status as 'True'")
	print(" Press 'd' to set the roomba's status as 'False'")
	print(" Press 't' to set the roomba_base2 status as 'True'")
	print(" Press 'f' to set the roomba_base2 status as 'False'")
	print(" Press 'c' to check the readiness of beaker")


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
