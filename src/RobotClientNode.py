import time, sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from pynput.keyboard import KeyCode
from key_commander import KeyCommander

from my_interfaces.msg import ReadyStatus
from my_interfaces.srv import UpdateStatus, CheckRobotStatus

from ReadyStatusServicesNode import ReadyStatusServicesNode


class RobotClientNode(Node):
	def __init__(self, robot_name):
		super().__init__(f'{robot_name}_client_node')
		self.robot_name = robot_name

		# Client for updating the status of a robot
		self.update_status_client = self.create_client(UpdateStatus, 'update_robot_status')
		while not self.update_status_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('UpdateStatus not available, waiting again...')
		self.get_logger().info("UpdateStatus service is available.")

		# Client for checking the status of a robot
		self.check_status_client = self.create_client(CheckRobotStatus, 'check_robot_status')
		while not self.check_status_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('CheckRobotStatus service not available, waiting again...')
		self.get_logger().info("CheckRobotStatus service is available.")

		# Initialize the variable that stores the robot ready status information
		self.latest_ready_status = None

		self.get_logger().info(f"{self.robot_name} Client Node started.")


	def ready_status_callback(self, msg):
		"""
		Update the latest ready statuses with the message received from the 'robot_ready_status' topic
		"""
		self.latest_ready_status = msg
		print(f"{self.robot_name} Updated status: {msg.roomba_base2, msg.beaker, msg.beaker_conv, msg.bunsen_conv, msg.bunsen, msg.roomba_base3}")




	def update_robot_status(self, robot_name, status):
		"""
		Update the status of a robot using the UpdateStatus service
		"""
		self.get_logger().info(f"{self.robot_name} Requesting to set {robot_name}'s status to {status}...")

		if not self.update_status_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().error(f"{self.robot_name} UpdateStatus service is not available.")
			return False
	
		req = UpdateStatus.Request()
		req.robot_name = robot_name
		req.status = status

		future = self.update_status_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)

		if future.result() is not None and future.result().success:
			self.get_logger().info(f"{robot_name}'s status successfully updated to {status}.")
			return True
		else:
			self.get_logger().error(f"Failed to update {robot_name}'s status.")
			return False


	def wait_for_specific_status(self, robot_name, expected_status):
		self.get_logger().info(f"Waiting for {robot_name} to reach status {expected_status}...")
		while rclpy.ok():
			req = CheckRobotStatus.Request()
			req.robot_name = robot_name
			req.expected_status = expected_status

			future = self.check_status_client.call_async(req)

			while rclpy.ok():
				rclpy.spin_once(self)
				if future.done():
					if future.result().status_matched:
						self.get_logger().info(f"{robot_name} reached the expected status.")
						return
					else:
						break  # Status not matched, break inner loop to send a new request
				# Optionally, add a sleep here to avoid high-frequency requests
			time.sleep(1)  # Throttle the requests to avoid spamming



	###################################################################################
	""" TESTING STUFF """
	###################################################################################

#####  Example functions used during development #####
	""" Functions for testing - setting statuses to true"""
	def set_beaker_true(self):
		self.update_robot_status('beaker', True)
	
	def set_beaker_conv_true(self):
		self.update_robot_status('beaker_conv', True)

	def set_bunsen_conv_true(self):
		self.update_robot_status('bunsen_conv', True)

	def set_bunsen_true(self):
		self.update_robot_status('bunsen', True)

	def set_roomba_base2_true(self):
		self.update_robot_status('roomba_base2', True)

	def set_roomba_base3_true(self):
		self.update_robot_status('roomba_base3', True)

	""" Functions for testing - setting statuses to False """
	def set_beaker_false(self):
		self.update_robot_status('beaker', False)
	
	def set_beaker_conv_false(self):
		self.update_robot_status('beaker_conv', False)

	def set_bunsen_conv_false(self):
		self.update_robot_status('bunsen_conv', False)

	def set_bunsen_false(self):
		self.update_robot_status('bunsen', False)

	def set_roomba_base2_false(self):
		self.update_robot_status('roomba_base2', False)

	def set_roomba_base3_false(self):
		self.update_robot_status('roomba_base3', False)
########################################################


	def wait_test(self):
		print("\nTEST 1: BEAKER TO TRUE\n")
		tester_client_node.wait_for_specific_status('beaker', True)

		print("\nTEST 2: BEAKER TO FALSE\n")
		tester_client_node.wait_for_specific_status('beaker', False)

		print("\nTEST 3: BEAKER_CONV TO TRUE\n")
		tester_client_node.wait_for_specific_status('beaker_conv', True)

		print("\nTEST 4: BEAKER_CONV TO FALSE\n")
		tester_client_node.wait_for_specific_status('beaker_conv', False)

		print("\nTEST 5: ROOMBA_BASE3 TO TRUE\n")
		tester_client_node.wait_for_specific_status('roomba_base3', True)

		print("\nTEST COMPLETE\n")

########################################################

def main(args=None):
	pass


if __name__ == '__main__':
	rclpy.init()

	# ready_status_service_node = ReadyStatusPublisherNode()
	tester_client_node = RobotClientNode('tester_1')
	exec = MultiThreadedExecutor(4)

	exec.add_node(tester_client_node)
	# exec.add_node(ready_status_service_node)


	keycom = KeyCommander([
		# (KeyCode(char='w'), tester_client_node.wait_test),
		(KeyCode(char='t'), tester_client_node.set_roomba_base2_true),
		(KeyCode(char='f'), tester_client_node.set_roomba_base2_false),

		(KeyCode(char='y'), tester_client_node.set_beaker_true),
		(KeyCode(char='g'), tester_client_node.set_beaker_false),
		
		(KeyCode(char='u'), tester_client_node.set_beaker_conv_true),
		(KeyCode(char='h'), tester_client_node.set_beaker_conv_false),

		(KeyCode(char='i'), tester_client_node.set_bunsen_conv_true),
		(KeyCode(char='j'), tester_client_node.set_bunsen_conv_false),

		(KeyCode(char='o'), tester_client_node.set_bunsen_true),
		(KeyCode(char='k'), tester_client_node.set_bunsen_false),

		(KeyCode(char='p'), tester_client_node.set_roomba_base3_true),
		(KeyCode(char='l'), tester_client_node.set_roomba_base3_false),
	])

	# Display the key command options to the user
	print("Press buttons to do stuff")


	try:
		exec.spin()
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		tester_client_node.destroy_node()
		rclpy.shutdown()