import time, sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from pynput.keyboard import KeyCode
from key_commander import KeyCommander

from my_interfaces.msg import ReadyStatus
from my_interfaces.srv import UpdateStatus  # Assuming you have an Update service

class RobotClientNode(Node):
	def __init__(self):
		super().__init__('beaker_node')

		# Subscribe to the published topic to get the latest ready status 
  		# This is for verifying robot statuses in check_robot_status() function
		self.ready_status_subscription_ = self.create_subscription(
			ReadyStatus, 'robot_ready_status', self.ready_status_callback, 10)

		# Client for updating the status of a robot
		self.update_status_client = self.create_client(UpdateStatus, 'update_robot_status')

		while not self.update_status_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
		print("UpdateStatus service is available.")

		# Initialize the variable that stores the robot ready status information
		self.latest_ready_status = None


	def ready_status_callback(self, msg):
		"""
		Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		"""
		self.latest_ready_status = msg
		# print(f"Updated status: {msg.roomba_base2, msg.beaker, msg.beaker_conv, msg.bunsen_conv, msg.bunsen, msg.roomba_base3}")


	def update_robot_status(self, robot_name, status):
		"""
		Update the status of a robot using the UpdateStatus service
		"""
		if not self.update_status_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available...')
			return
		req = UpdateStatus.Request()
		req.robot_name = robot_name
		req.status = status
		future = self.update_status_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		if future.result() is not None:
			self.get_logger().info(f'Successfully updated {robot_name} status to {status}')
		else:
			self.get_logger().error('Failed to update status.')


	def check_robot_status(self, robot_name, expected_status):
		"""
		Check the current status of a robot and log the result
		"""
		self.get_logger().info(f"Checking {robot_name} status...")
		current_status = getattr(self.latest_ready_status, robot_name)
		if current_status == expected_status:
			self.get_logger().info(f"{robot_name} status is as expected: {expected_status}")
		else:
			self.get_logger().info(f"{robot_name} status mismatch. Expected: {expected_status}, got: {current_status}")

