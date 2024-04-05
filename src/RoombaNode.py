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


	def __init__(self):
		super().__init__('roomba_node')

		# Establish the node objects within the class for node operations
		self.ready_status_publisher_node = ready_status_publisher_node

		self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
															 self.ready_status_callback, 10)

		# Initialize the variable that stores the robot ready status information
		# Ready statuses are updated in the callback functions.
		self.latest_ready_status = None


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
			print("Resetting ROOMBA to False")
			self.ready_status_publisher_node.set_ready_status(roomba_status=False)
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging
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
			This function...
			1. Continuously asks the publisher to publish the current robot_status.txt contents
			2. Halts further actions until the statuses of both the roomba and beaker are True.
			
			NOTE: Variations of this function will be placed in RoombaNode, BeakerNode, and BunsenNode.
			NOTE: To avoid timeout, it is important that the robot calling this function has set its ready state to True beforehand.
			"""
			
			self.set_roomba_true()

			print("Waiting for ready: roomba and beaker. ")
			timeout = 100  # seconds
			check_interval = 2  # seconds
			start_time = time.time()

			while time.time() - start_time < timeout:

				# Publish the status to trigger an update
				# This will update all robot's self.latest_ready_status varriables
				self.ready_status_publisher_node.publish_ready_status()

				# Wait a short time for the message to be published and processed
				rclpy.spin_once(self, timeout_sec=check_interval)
				time.sleep(1.0)

				# Check the latest status
				if self.latest_ready_status and self.latest_ready_status.roomba and self.latest_ready_status.beaker:
					print("\n !!! Both Roomba and Beaker are ready for dice block handoff at base2 !!!!")
					return
				else:
					print("\n Waiting for both Roomba and Beaker to be ready at base2...")

			print("Timeout reached without both robots being ready for dice block handoff.")


	def wait_for_base2_block_pickup(self):
		"""
		After the robots are ready, the roomba will wait until beaker is set it false (after the dice block is retrieved)
		"""
		# # ! Temporarily placing this here for testing with wait_for_base2_ready from the beaker node
		# self.set_roomba_true()

		print("\nWaiting for beaker to pick up dice block.")
		timeout = 100  # seconds
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
			if self.latest_ready_status and not self.latest_ready_status.beaker:
				print("!!! Beaker has retrieved the dice block !!!!")
				# !!!! Perhaps reset the ready statuses of both robots to false?
				return
			else:
				print("Waiting for Beaker to set status to false...")

		print("Timeout reached without communicating dice block pickup.")


	def dice_block_handoff_base2(self):
		""" 
		Post-docking logic for the roomba. 
		"""

		# After a succesful dock, set roomba to true
		self.set_roomba_true()

		# Wait until beaker has its status set to true
		self.wait_for_base2_ready()

		# After both robots are ready, wait for beaker to pick up the dice block
		# NOTE: beaker should set its status to false after it has picked up the dice block
		print("Wait for beaker to set its status to false before moving to base3. ")
		self.wait_for_base2_block_pickup()

		# After beaker is false, reset roomba status and contine the roomba's navigation
		self.set_roomba_false()
		print("Specific driving actions occurr here")


	def wait_for_base3_ready(self):
			"""
			This function shares the same purpose as wait_for_base2_ready()this function has set its ready state to True beforehand.
			"""
						
			print("Waiting for ready: roomba and bunsen. ")
			timeout = 100  # seconds
			check_interval = 2  # seconds
			start_time = time.time()

			while time.time() - start_time < timeout:

				# Publish the status to trigger an update
				self.ready_status_publisher_node.publish_ready_status()
				# print("Show latest ready statuses within wait_for_base_ready: ", self.latest_ready_status)

				# Wait a short time for the message to be published and processed
				rclpy.spin_once(self, timeout_sec=check_interval)
				time.sleep(1.0)

				# Check the latest status
				if self.latest_ready_status and self.latest_ready_status.roomba and self.latest_ready_status.bunsen:
					print("\n !!! Both Roomba and bunsen are ready for dice block handoff at base3 !!!!")
					return
				else:
					print("Waiting for both Roomba and bunsen to be ready at base3...")

			print("Timeout reached without both robots being ready for dice block handoff.")


	def wait_for_base3_block_pickup(self):
		"""
		After the robots are ready, the roomba will wait until bunsen is set it false (after the dice block is retrieved)
		"""
		print("\nWaiting for bunsen to pick up dice block.")
		timeout = 100  # seconds
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
			if self.latest_ready_status and not self.latest_ready_status.bunsen:
				print("!!! bunsen has retrieved the dice block !!!!")
				# !!!! Perhaps reset the ready statuses of both robots to false?
				return
			else:
				print("Waiting for bunsen to set status to false...")

		print("Timeout reached without communicating dice block pickup.")


	def dice_block_handoff_base3(self):
		""" 
		Post-docking logic for the roomba. 
		"""

		# After a succesful dock, set roomba to true
		self.set_roomba_true()

		# Wait until bunsen has its status set to true
		self.wait_for_base3_ready()

		# After both robots are ready, wait for bunsen to pick up the dice block
		# NOTE: bunsen should set its status to false after it has picked up the dice block
		print("Wait for bunsen to set its status to false before moving to base3. ")
		self.wait_for_base3_block_pickup()

		# After bunsen is false, reset roomba status and contine the roomba's navigation
		self.set_roomba_false()
		print("Moving to base 3")
		print("Specific driving actions go here")


	def main(self):
		print("Navigation code goes here. ")


# The following Key Commander is used for manually checking outputs in the terminal
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
		(KeyCode(char='2'), roomba.dice_block_handoff_base2),
		(KeyCode(char='3'), roomba.dice_block_handoff_base3),
	])
	print(" Press 'u' to intitiate main routine") # This will be the navigation code
	print(" Press 'd' to display all robot states in the text file")
	print(" Press 't' to set the roomba's status as 'True'")
	print(" Press 'f' to set the roomba's status as 'False'")
	print(" Press 'w' to wait for select statuses to be true before continuing'")
	print(" Press '2' to initiate dice block handoff routine at base2")
	print(" Press '3' to initiate dice block handoff routine at base3")

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
