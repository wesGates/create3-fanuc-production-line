import time
import sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node
from pynput.keyboard import KeyCode
import my_interfaces
from my_interfaces.msg import ReadyStatus
from my_interfaces.srv import UpdateStatus, CheckRobotStatus


class ReadyStatusPublisherNode(Node):
	def __init__(self):
		super().__init__('ready_status_publisher')
		self.ready_status_publisher_ = self.create_publisher(ReadyStatus, 'robot_ready_status', 10)
		self.robot_statuses = {
			'roomba_base2': False,
			'beaker': False,
			'beaker_conv': False,
			'bunsen_conv': False,
			'bunsen': False,
			'roomba_base3': False
		}
		self.timer = self.create_timer(1.0, self.publish_ready_status)
		
		# Activating 
		self.update_status_srv = self.create_service(UpdateStatus, 'update_robot_status', self.update_status_callback)

		self.check_status_srv = self.create_service(CheckRobotStatus, 'check_robot_status', self.check_status_callback)
		
		self.latest_ready_status = ReadyStatus()
		print("Publisher is online!")


	def update_status_callback(self, request, response):
		robot_name = request.robot_name
		if robot_name in self.robot_statuses:
			self.robot_statuses[robot_name] = request.status
			response.success = True
		else:
			response.success = False
		return response


	def check_status_callback(self, request, response):
		"""
		Callback function for the CheckStatus service.
		"""
		self.get_logger().info(f"Received request to check {request.robot_name}'s status.")

		# Wait for the status to become the expected status
		while True:
			# Assuming self.latest_ready_status is updated elsewhere in this node
			current_status = getattr(self.latest_ready_status, request.robot_name, None)

			if current_status is None:
				self.get_logger().error(f"{request.robot_name} status is not found.")
				response.status_matched = False
				break

			if current_status == request.expected_status:
				response.status_matched = True
				break

			# Sleep for a short duration to avoid busy waiting
			time.sleep(0.1)

		return response


	def publish_ready_status(self):
		# Prepare the message
		msg = ReadyStatus()
		msg.roomba_base2 = self.robot_statuses['roomba_base2']
		msg.beaker = self.robot_statuses['beaker']
		msg.beaker_conv = self.robot_statuses['beaker_conv']
		msg.bunsen_conv = self.robot_statuses['bunsen_conv']
		msg.bunsen = self.robot_statuses['bunsen']
		msg.roomba_base3 = self.robot_statuses['roomba_base3']

		# Update the latest status before publishing
		self.latest_ready_status = msg

		# Publish the message
		self.get_logger().info("Published: ", msg)
		self.ready_status_publisher_.publish(msg)


# The following is used for spinning up the node for testing w/o using ros2 run
def main(args=None):
	rclpy.init(args=args)
	ready_status_publisher_node = ReadyStatusPublisherNode()

	rclpy.spin(ready_status_publisher_node)

	# Shutdown the ROS client library for Python
	ready_status_publisher_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()