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

# Initialize and connect to OTHER nodes
rclpy.init()
namespace = 'create3_05AE'
ready_status_publisher_node = ReadyStatusPublisherNode()

class RoombaNode(Node):

	# def __init__(self, namespace):
	def __init__(self):
		# super().__init__('ready_status_subscriber')
		super().__init__('roomba_node')


		# Establish the node objects within the class for node operations
		self.ready_status = ready_status_publisher_node

		self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
															 self.ready_status_callback, 10)

		# Initializing variables to store data received from other nodes
        # These variables are updated in the callback functions.
		self.latest_ready_status = None

	def ready_status_callback(self, msg):
		# Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		print("I heard...")
		self.latest_ready_status = msg
		self.roomba_status = self.latest_ready_status.roomba

		# DEBUGGING
		print("msg.roomba (received from publisher): ", msg.roomba)
		print("msg.beaker (received from publisher): ", msg.beaker)
		print("msg.bunsen (received from publisher): ", msg.bunsen)

		# self.get_logger().info(f"Received /robot_ready_status: {self.latest_ready_status}")
		print("End of RoombaNode callback \n")


	def listener_callback(self, msg):
			self.get_logger().info('I heard roomba status: "%d"' % msg.roomba) # CHANGE
			self.get_logger().info('I heard beaker status: "%d"' % msg.beaker) # CHANGE
			self.get_logger().info('I heard beaker status: "%d"' % msg.bunsen) # CHANGE
			print("\n")


	def main(self):
		try:
			print("\nStep 1: Get the publisher to publish on command and access the information. ")
			self.ready_status.publish_ready_status()
			print("\n")
			time.sleep(1.0)


			print("Step 2: Change the status of a variable")
			self.ready_status.set_ready_status(roomba_status=True)
			print("\n")
			time.sleep(1.0)
			
			print("Step 3: Re-publish the hopefully altered statuses")
			self.ready_status.publish_ready_status()
			print("\n")
		except Exception as error:
			self.get_logger().error(f"Error in main: {error}") # Error logging



if __name__ == '__main__':
	# rclpy.init()

	roomba = RoombaNode()
	exec = MultiThreadedExecutor(8)

	exec.add_node(roomba)
	exec.add_node(ready_status_publisher_node)

	time.sleep(1.0)
	
	keycom = KeyCommander([
		(KeyCode(char='u'), roomba.main),
	])
	print(" Press 'U' to intitiate routine")

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
