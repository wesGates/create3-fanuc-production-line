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
namespace = 'beaker'
ready_status_publisher_node = ReadyStatusPublisherNode()

class BeakerNode(Node):


	def __init__(self):
		super().__init__('beaker_node')

		# Establish the node objects within the class for node operations
		self.ready_status_publisher_node = ready_status_publisher_node
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

	def update_robot_status(self):
		# Update Beaker's status to ready
		self.ready_status_publisher_node.set_ready_status(beaker_status=True)
		self.get_logger().info('Updated Beaker status to ready')


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


	def set_beaker_true(self):
		try: 
			self.ready_status_publisher_node.set_ready_status(beaker_status=True)
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_beaker_false(self):
		try: 
			print("Resetting BEAKER to False")
			self.ready_status_publisher_node.set_ready_status(beaker_status=False)
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def wait_for_base2_ready(self):
			"""
			This function waits until the status of both the roomba, and beaker is True.
			Variations of this function will be placed in both RoombaNode and BeakerNode.
			NOTE: The robot that calls this function must first set their status to True, or it will time out.
			"""
			
			# ! Temporarily placing this here for testing with wait_for_base2_ready from the beaker node
			self.set_beaker_true()

			print("Waiting for ready: roomba and beaker. ")
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
				if self.latest_ready_status and self.latest_ready_status.roomba and self.latest_ready_status.beaker:
					print("\n !!! Both Roomba and Beaker are ready for dice block handoff at base2 !!!!")
					return
				else:
					print("\nWaiting for both Roomba and Beaker to be ready at base2...")

			print("\n *** Timeout reached without both robots being ready for dice block handoff ***")


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
		(KeyCode(char='1'), beaker.wait_for_base2_ready),
		(KeyCode(char='4'), beaker.dice_block_handoff_base2),

	])
	print(" Press 'd' to display all robot states in the text file")
	print(" Press 'y' to set beaker's status as 'True'")
	print(" Press 'g' to set beaker's status as 'False'")
	print(" Press '4' to initiate dice block handoff routine at base2")



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
