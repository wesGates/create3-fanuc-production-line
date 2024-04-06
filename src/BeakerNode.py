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
from my_interfaces.msg import ReadyStatus, Num, Base2status, Base3status   # CHANGE
from ReadyStatusPublisherNode import ReadyStatusPublisherNode

import rclpy
from rclpy.node import Node
import time

from my_interfaces.msg import ReadyStatus  # Adjust the import path based on your package structure


# Initialize and connect to OTHER nodes
rclpy.init()
namespace = 'beaker'
ready_status_publisher_node = ReadyStatusPublisherNode()

class BeakerNode(Node):


	def __init__(self):
		super().__init__('beaker_node')

		# Establish the node objects within the class for node operations
		self.ready_status_publisher_node = ready_status_publisher_node

		# Creating callback groups for managing subscription callbacks
		cb_ready_status = MutuallyExclusiveCallbackGroup()

		# Subscribe to the published topic to update latest_ready_status
		self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
															 self.ready_status_callback, 10, callback_group=cb_ready_status)
		
		# Initialize the variable that stores the robot ready status information
		# Ready statuses are updated in the callback functions.
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


	def show_latest_ready(self):
		print("!!!!!!!!!", self.latest_ready_status)


	def publish_robot_status(self):
		# Now, publish the updated status
		self.ready_status_publisher_node.publish_ready_status()
		self.get_logger().info('Published the updated ready status')


	def display_robot_statuses(self):
		# DUBUGGING: Used in key commander to display the robot states on keypress in the terminal
		try: 
			self.ready_status_publisher_node.display_robot_statuses()
		except:
			self.get_logger().error(f"Error in display: {error}")


	def set_beaker_true(self):
		try: 
			print("Setting BEAKER to True")
			self.ready_status_publisher_node.set_ready_status(beaker_status=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_beaker_false(self):
		try: 
			print("Resetting BEAKER to False")
			self.ready_status_publisher_node.set_ready_status(beaker_status=False)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging





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
	""" Robot-specific code starts here. """			
	##############################################################################################


	def main_wait_for_base2_ready(self):
		self.set_beaker_true() # NOTE: Always set robot status before calling wait_for_ready()
		# self.ready_status_publisher_node.publish_ready_status() # Publish the changed state
		print("Waiting for roomba and beaker robots ready at base 2")
		self.wait_for_ready({'roomba': True, 'beaker': True})
		print("Robots are ready at base 2")


	def dice_block_handoff_base2(self):
		""" 
		Dice block handoff routine for beaker. Differs from the same function in RoombaNode. 
		"""
		# After a succesful dock, set beaker to true
		self.set_beaker_true()

		# Wait until roomba has its status set to true
		self.wait_for_base2_ready()

		# After both robots are ready, wait for beaker to pick up the dice block
		print("Code for having Beaker pick up the dice block")
		
		# NOTE: beaker should set its status to false after it has picked up the dice block
		self.set_beaker_false()

		# After beaker is false, reset roomba status and 
		print("Beaker will continue with the dice flip...")


# The following Key Commander is used for manually checking outputs in the terminal
if __name__ == '__main__':
	# rclpy.init()

	beaker = BeakerNode()
	exec = MultiThreadedExecutor(8)

	exec.add_node(beaker)
	exec.add_node(ready_status_publisher_node)

	time.sleep(1.0)
	
	keycom = KeyCommander([
		(KeyCode(char='d'), beaker.display_robot_statuses),
		(KeyCode(char='y'), beaker.set_beaker_true),
		(KeyCode(char='g'), beaker.set_beaker_false),
		(KeyCode(char='e'), beaker.main_wait_for_base2_ready),
		(KeyCode(char='4'), beaker.dice_block_handoff_base2),
		(KeyCode(char='p'), beaker.publish_robot_status),
		(KeyCode(char='v'), beaker.show_latest_ready),
		# (KeyCode(char='5'), beaker.dice_block_handoff_conveyor), # TODO

	])
	print(" Press 'd' to display all robot states in the text file")
	print(" Press 'y' to set beaker's status as 'True'")
	print(" Press 'g' to set beaker's status as 'False'")
	print(" Press 'e' to initiate wait_for_base2_ready routine")
	print(" Press '4' to initiate dice block handoff routine at base2")
	print(" Press 'p' to manually publish the contents of the status text file")
	print(" Press 'v' to show contents of the local latest ready status from the callback")
	# print(" Press '5' to initiate dice block handoff routine at conveyor") # TODO


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
