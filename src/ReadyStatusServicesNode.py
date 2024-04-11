import time
import sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node
from pynput.keyboard import KeyCode
import my_interfaces
from my_interfaces.msg import ReadyStatus
from my_interfaces.srv import UpdateStatus, CheckRobotStatus


class ReadyStatusServicesNode(Node):
	def __init__(self):
		super().__init__('ready_status_service_node')
		
		# Initialize all ready status values
		self.robot_statuses = {
			'roomba_base2': False,
			'beaker': False,
			'beaker_conv': False,
			'bunsen_conv': False,
			'bunsen': False,
			'roomba_base3': False
		}
		
		# Activating services
		self.update_status_srv = self.create_service(UpdateStatus, 'update_robot_status', self.update_status_callback)
		self.check_status_srv = self.create_service(CheckRobotStatus, 'check_robot_status', self.check_status_callback)
		
		self.latest_ready_status = ReadyStatus()
		self.get_logger().info(f"Services are activated.")


	def update_status_callback(self, request, response):
		robot_name = request.robot_name
		if robot_name in self.robot_statuses:
			self.robot_statuses[robot_name] = request.status
			# Update the latest status
			setattr(self.latest_ready_status, robot_name, request.status)
			response.success = True
			self.get_logger().info(f"{robot_name}'s status updated to {request.status}")
		else:
			response.success = False
			self.get_logger().error(f"Failed to update {robot_name}'s status.")
		return response
	

	def check_status_callback(self, request, response):
		"""
		Callback function for the CheckStatus service.
		"""
		self.get_logger().info(f"Received request to check {request.robot_name}'s status.")

		# Directly check the current status without waiting
		current_status = getattr(self.latest_ready_status, request.robot_name, None)

		if current_status is None:
			self.get_logger().error(f"{request.robot_name} status is not found.")
			response.status_matched = False
		else:
			response.status_matched = (current_status == request.expected_status)
			self.get_logger().info(f"{request.robot_name} status check: {response.status_matched}")

		print("Check_status_callback response: ", response)
		return response




""" Use this to spin up the node separately from the other nodes in a separate terminal"""

def main(args=None):
    rclpy.init(args=args)
    ready_status_services_node = ReadyStatusServicesNode()

    try:
        # Spin the node to continuously call the callbacks
        rclpy.spin(ready_status_services_node)
    except KeyboardInterrupt:
        ready_status_services_node.get_logger().info('Node was interrupted and is shutting down.')
    finally:
        # Clean up the node resources
        ready_status_services_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()