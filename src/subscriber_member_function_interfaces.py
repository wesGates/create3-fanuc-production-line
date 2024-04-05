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
from publisher_member_function_interfaces import ReadyStatusPublisher

# Initialize and connect to OTHER nodes
rclpy.init()
namespace = 'create3_05AE'
ready_status_publisher = ReadyStatusPublisher()
# ready_status_subscriber = ReadyStatusSubscriber()

class ReadyStatusSubscriber(Node):

	# def __init__(self, namespace):
	def __init__(self):
		super().__init__('ready_status_subscriber')

		# Establish the node objects within the class for node operations
		self.ready_status_publisher = ready_status_publisher

		self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
															 self.ready_status_callback, 10)

		# self.ready_status_subscription_ = self.create_subscription(ReadyStatus, 'robot_ready_status', 
		# 													 self.listener_callback, 10)
		
		# Initializing variables to store data received from other nodes
        # These variables are updated in the callback functions.
		self.latest_ready_status = None

	def ready_status_callback(self, msg):
		# Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		self.latest_ready_status = msg
		self.roomba_status = self.latest_ready_status.roomba

		# DEBUGGING
		print("Printing msg.roomba: ", msg.roomba)
		print("Printing msg.beaker: ", msg.beaker)
		print("Printing msg.bunsen: ", msg.bunsen)


		self.get_logger().info(f"Received /robot_ready_status: {self.latest_ready_status}")


	def listener_callback(self, msg):
			self.get_logger().info('I heard roomba status: "%d"' % msg.roomba) # CHANGE
			self.get_logger().info('I heard beaker status: "%d"' % msg.beaker) # CHANGE
			self.get_logger().info('I heard beaker status: "%d"' % msg.bunsen) # CHANGE
			print("\n")

	def change_roomba_status(self, ready=False):
		# Logic to determine if Roomba is ready (this is just a placeholder for actual logic)
		# ready = not ready
		ready = True
		print("Ready status (roomba node): ", ready)
		is_ready = ready  # or some condition that evaluates Roomba's readiness
		self.ready_status_publisher.roomba_status_updater(is_ready)


	def main(self):
		try:
			print("Step 1")
			time.sleep(1.0)
			ready_status_publisher.roomba_status_updater(ready=True)
			self.change_roomba_status()
			print("Step 2")
			time.sleep(1.0)
			print("Step 3")
		except Exception as error:
			self.get_logger().error(f"Error in main: {error}") # Error logging



if __name__ == '__main__':
	# rclpy.init()

	ready_status_publisher = ReadyStatusPublisher()

	ready_status_subscriber = ReadyStatusSubscriber()
	exec = MultiThreadedExecutor(8)
	exec.add_node(ready_status_subscriber)


	keycom = KeyCommander([
		(KeyCode(char='u'), ready_status_subscriber.main),
	])
	print(" Press 'U' to intitiate launch")

	try:
		exec.spin()  # execute Roomba callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		ready_status_subscriber.destroy_node()
		rclpy.try_shutdown()
