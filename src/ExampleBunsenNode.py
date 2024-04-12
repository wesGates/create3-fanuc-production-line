# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# key_commander stuff
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

# Python Packages
import sys
sys.path.append("../dependencies/")
import time
from math import pi
from collections import deque
import json
from datetime import datetime

# Threading
from rclpy.executors import MultiThreadedExecutor
from threading import RLock
import threading

# Broker
import brokerSender
from brokerSender import mqttc

# Node Imports
from RobotClientNode import RobotClientNode

# Globals
topic = "vandalrobot"

# Initialize and connect to other nodes
rclpy.init()
namespace = 'bunsen'

# Makes instance of the universal robot node that sets/ waits on statuses
bunsen_status_client = RobotClientNode(namespace)

class BunsenNode(Node):
	def __init__(self, namespace):
		super().__init__('bunsen_node')

		self.bunsen_status_client = bunsen_status_client

		# Variable Initialization
		self.latest_dock_status = True #########!!!!!!!!!!!!! Was None


	# NOTE: Change this report for your specific robot
	def reportSender(self, label="Undefined", action="Undefined",
					isAtBase1=False, isAtBase2=False, isAtBase3=False,
					isReady=False, isMoving=False, iswithDice=False
					):
		""""
		Used to send a report at the beginning and end of every action.
		Defaults to False.
		"""

		self.record_pose() # This should update the recorded pose whenever the report is sent
		# time.sleep(1)

		data = {
			"messageType": "Report",
			"node": "roomba",
			"nodeId": "gatesroomba12",
			"productLine": "moscow",
			"roombareport": {
				"isDock": self.latest_dock_status,
				"action": action,
				"label":label,
				"isReady": isReady,
				"withDice": iswithDice,
				"isAtBase1": isAtBase1,
				"isAtBase2": isAtBase2,
				"isAtBase3": isAtBase3,
				"isMoving": isMoving,
				"position": [

							int(self.latest_pose_stamped.pose.position.x*1000),
							int(self.latest_pose_stamped.pose.position.y*1000),
							int(self.latest_pose_stamped.pose.position.z*1000)],

				"Fault": {
				}
			},
			"date": str(datetime.now())
		}
		
		mqttc.publish(topic, json.dumps(data))


	def bunsen_test(self):
		" Uses an instance of the RobotClientNode to change statuses and wait for other statuses"
		try:
			# Step 1: Set BunsenConv's status to True
			self.get_logger().info("Setting BunsenConv's status to True...")
			self.bunsen_status_client.update_robot_status('bunsen_conv', True)

			# Step 2: Wait for BeakerConv to become True
			self.get_logger().info("Waiting for BeakerConv's status to become True...")
			self.bunsen_status_client.wait_for_specific_status('beaker_conv', True)
			self.get_logger().info("BeakerConv's status is now True.")

			# Step 3: Set BunsenConv's status to False
			self.get_logger().info("Setting BunsenConv's status to False...")
			self.bunsen_status_client.update_robot_status('bunsen_conv', False)

			# Step 4: Set Bunsen's status to True
			self.get_logger().info("Setting Bunsen's status to True...")
			self.bunsen_status_client.update_robot_status('bunsen', True)

			# Step 5: Set Bunsen's status to False
			self.get_logger().info("Setting Bunsen's status to False...")
			self.bunsen_status_client.update_robot_status('bunsen', False)

		except Exception as error:
			# Assuming there's a way to signal error audibly or visually for Bunsen
			self.get_logger().error(f"Error in BunsenTest: {error}")



def stop_all(exec,roomba,rclpy):
	while not brokerSender.stop_all_message :
				# print(brokerSender.start_all_message)
				pass
	
	exec.shutdown()
	roomba.destroy_node()
	rclpy.try_shutdown()


if __name__ == '__main__':
	# rclpy.init()

	bunsen = BunsenNode(namespace)

	exec = MultiThreadedExecutor(3) # No. Nodes + 1

	exec.add_node(bunsen)
	exec.add_node(bunsen_status_client)
	
	# NOTE: This node should be spun up in another terminal beforehand
	# exec.add_node(ready_status_publisher_node) 
	

	# # NOTE: Uncomment to enable broker stuff
 	# Establish start and stop messaging from the broker
	# broker_thread = threading.Thread(target=mqttc.loop_start)
	# broker_thread.start()
	
	# stop_thread = threading.Thread(target=stop_all,args=(exec,bunsen,rclpy))
	# stop_thread.start()


	keycom = KeyCommander([
		(KeyCode(char='b'), bunsen.test),

	])
	print(" Press 'b' to intitiate bunsen test")


	try:
		exec.spin()  # execute bunsen callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
		bunsen.destroy_node()
		bunsen_status_client.destroy_node()
		rclpy.shutdown()
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		rclpy.try_shutdown()


