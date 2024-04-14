# Python Packages
import sys
import time
from math import pi
from collections import deque
import json
from datetime import datetime
sys.path.append("../dependencies/")

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# key_commander stuff
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

# Threading
from rclpy.executors import MultiThreadedExecutor
from threading import RLock
import threading

# # Broker: NOTE: Enable this for implementation
# import brokerSender
# from brokerSender import mqttc

# Node Imports
from RobotClientNode import RobotClientNode

# Globals
topic = "vandalrobot"

# Initialize and connect to other nodes
rclpy.init()
namespace = 'roomba'

# Makes instance of the universal robot node that sets/ waits on statuses
roomba_status_client = RobotClientNode(namespace)

class RoombaNode(Node):
	def __init__(self, namespace):
		super().__init__('roomba_node')

		"""" This is everything that needs to be put into your chosen node"""

		self.roomba_status_client = roomba_status_client

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


	def roomba_test(self):
		" Uses an instance of the RobotClientNode to change statuses and wait for other statuses"
		try:
			
			# Move to base2 at beaker

			# Step 1-1: Set the status of roomba_base2 to True after docking
			self.get_logger().info("Setting roomba_base2 status to True...")
			self.roomba_status_client.update_robot_status('roomba_base2', True)

			# Step 1-2: Wait for beaker to become True, indicating it's in position for the dice pickup
			self.get_logger().info("Waiting for beaker status to become True")
			self.roomba_status_client.wait_for_specific_status('beaker', True)

			# Step 1-3: Wait for beaker to become False, indicating beaker has the dice block and has moved away
			self.get_logger().info("Waiting for beaker status to become False...")
			self.roomba_status_client.wait_for_specific_status('beaker', False)
			self.get_logger().info("beaker status is now False.")

			# Step 1-4: Set roomba_base2 status to False
			self.get_logger().info("Setting roomba_base2 status to False...")
			self.roomba_status_client.update_robot_status('roomba_base2', False)
		
			# Undock

			# Move to base3 at bunsen

			# Step 2-1: After docking, wait for bunsen to become True before setting roomba_base3 to true
			self.get_logger().info("Waiting for bunsen status to become True...")
			self.roomba_status_client.wait_for_specific_status('bunsen', True)
			self.get_logger().info("bunsen status is now True.")

			# Step 2-2: Tell bunsen that the dice block can be delivered
			self.get_logger().info("Setting roomba_base3 status to True...")
			self.roomba_status_client.update_robot_status('roomba_base3', True)

			# Step 2-3: Wait for bunsen to indicate that the dice block was delivered, and has moved away
			self.get_logger().info("Waiting for bunsen status to become False...")
			self.roomba_status_client.wait_for_specific_status('bunsen', False)
			self.get_logger().info("bunsen status is now False.")

			# Step 2-4: Reset roomba_base3 to False
			self.get_logger().info("Setting roomba_base3 status to False...")
			self.roomba_status_client.update_robot_status('roomba_base3', False)

			# Undock and travel

			# Step 3: Print statement indicating the transition
			self.get_logger().info("Success! Traveling from base 3 to base 1.")
			print("\nYou may run the test function again on specific button press ")


		except Exception as error:
			self.get_logger().error(f"Error in test: {error}")



def stop_all(exec,roomba,rclpy):
	while not brokerSender.stop_all_message :
				# print(brokerSender.start_all_message)
				pass
	
	exec.shutdown()
	roomba.destroy_node()
	rclpy.try_shutdown()


if __name__ == '__main__':
	# rclpy.init()

	roomba = RoombaNode(namespace)

	exec = MultiThreadedExecutor(3) # No. Nodes + 1

	exec.add_node(roomba)
	exec.add_node(roomba_status_client)
	
	# NOTE: This node should be spun up in another terminal beforehand
	# exec.add_node(ready_status_publisher_node) 
	

	# # NOTE: Uncomment to enable broker stuff
 	# Establish start and stop messaging from the broker
	# broker_thread = threading.Thread(target=mqttc.loop_start)
	# broker_thread.start()
	
	# stop_thread = threading.Thread(target=stop_all,args=(exec,roomba,rclpy))
	# stop_thread.start()


	keycom = KeyCommander([
		(KeyCode(char='v'), roomba.roomba_test),

	])
	print(" Press 'v' to intitiate roomba test")


	try:
		exec.spin()  # execute roomba callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
		roomba.destroy_node()
		roomba_status_client.destroy_node()
		rclpy.shutdown()
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		rclpy.try_shutdown()

