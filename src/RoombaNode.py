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
		
		# Services
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





	def publish_robot_status(self):
		self.ready_status_publisher_node.publish_ready_status()
		self.get_logger().info('Published the updated ready status')


	def display_robot_statuses(self):
		# DUBUGGING: Used in key commander to display the robot states on keypress in the terminal
		try: 
			self.ready_status_publisher_node.display_robot_statuses()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_roomba_true(self):
		try: 
			print("Setting ROOMBA to: True")
			self.ready_status_publisher_node.set_ready_status(roomba_status=True)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging


	def set_roomba_false(self):
		try: 
			print("Setting ROOMBA to: False")
			self.ready_status_publisher_node.set_ready_status(roomba_status=False)
			self.publish_robot_status()
		except:
			self.get_logger().error(f"Error in display: {error}") # Error logging

	
	def send_request(self, robot1, robot2):
		request = CheckReadiness.Request()
		request.robot1 = robot1
		request.robot2 = robot2
		future = self.client.call_async(request)
		return future

	def check_robot_readiness(self, robot1, robot2):
		future = self.send_request(robot1, robot2)
		rclpy.spin_until_future_complete(self, future)
		if future.result() is not None:
			readiness = future.result().ready
			if readiness:
				self.get_logger().info(f'{robot1} and {robot2} are ready.')
			else:
				self.get_logger().info(f'Waiting for {robot1} and {robot2} to be ready.')
		else:
			self.get_logger().error(f'Exception while calling service: {future.exception()}')

	def main(self):
		print("\n Waiting for beaker to pick up dice block.")
		self.wait_for_ready({'roomba': True, 'beaker': False})
		print("Navigation code goes here. ")
		pass

	def main_wait_for_base2_ready(self):
		self.set_roomba_true() # NOTE: Always set robot status before calling wait_for_ready()
		# self.ready_status_publisher_node.publish_ready_status()
		print("Waiting for roomba and beaker robots ready at base 2")
		self.wait_for_ready({'roomba': True, 'beaker': True})
		print("Robots are ready at base 2")
		pass

	def test_readiness(self):
		future = self.send_request('roomba', 'beaker')
		rclpy.spin_until_future_complete(self, future)

		if future.result() is not None:
			readiness = future.result().ready
			if readiness:
				self.get_logger().info(f'{robot1} and {robot2} are ready.')
			else:
				self.get_logger().info(f'Waiting for {robot1} and {robot2} to be ready.')
		else:
			self.get_logger().error('Exception while calling service: %r' % future.exception())


	


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
		(KeyCode(char='d'), roomba.display_robot_statuses),
		(KeyCode(char='t'), roomba.set_roomba_true),
		(KeyCode(char='f'), roomba.set_roomba_false),
		(KeyCode(char='w'), roomba.main_wait_for_base2_ready),
		(KeyCode(char='r'), roomba.test_readiness),

	])
	# print(" Press 'u' to intitiate main routine") # This will be the navigation code
	print(" Press 'u' to intitiate main routine") # TESTING W/ PASSING VARS (KEY COMM ISSUES)
	print(" Press 'd' to display all robot states in the text file")
	print(" Press 't' to set the roomba's status as 'True'")
	print(" Press 'f' to set the roomba's status as 'False'")
	print(" Press 'w' to wait for wait_for_base2_ready routine at base 2'")
	print(" Press 'r' to test the readiness of roomba and beaker")


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
