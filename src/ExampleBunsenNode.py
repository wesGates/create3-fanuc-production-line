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
namespace = 'bunsen'

# Makes instance of the universal robot node that sets/ waits on statuses
bunsen_status_client = RobotClientNode(namespace)

class BunsenNode(Node):
	def __init__(self, namespace):
		super().__init__('bunsen_node')

		"""" This is everything that needs to be put into your chosen node"""

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
			# Step 1: Set bunsen conveyor status to indicate it is ready for dice block pickup
			self.get_logger().info("Setting bunsen_conv's status to True...")
			self.bunsen_status_client.update_robot_status('bunsen_conv', True)


			# Step 2-1: Wait for beaker_conv signal that dice block can be picked up
			self.get_logger().info("Waiting for beaker_conv's status to become True...")
			self.bunsen_status_client.wait_for_specific_status('beaker_conv', True)

			# Step 2-2: Pick up dice block
			# Step 2-3: Place dice block and check if the first sensor sees it
			# -- If dice block is absent, throw fault
			# -- Otherwise, start conveyor
			# Step 2-4: Wait until dice block is seen by second sensor or until timeout, then...
			# Step 2-5: Pick up dice block

			# Step 2-6: Reset bunsen_conv to false to realease beaker from waiting
			self.get_logger().info("Setting bunsen_conv's status to False...")
			self.bunsen_status_client.update_robot_status('bunsen_conv', False)


			# Dice flip

			# Step 3-1: Set Bunsen's status to True after reaching the position above base3
			self.get_logger().info("Setting Bunsen's status to True...")
			self.bunsen_status_client.update_robot_status('bunsen', True)

			# Step 3-2: Wait for roomba to arrive at base3
			self.get_logger().info("Waiting for roomba_base3's status to become True...")
			self.bunsen_status_client.wait_for_specific_status('roomba_base3', True)

			# Step 3-3: Deliver dice block and move clear of the roomba

			# Step 3-4: Set Bunsen's status to False to indicate that the roomba can move away
			self.get_logger().info("Setting Bunsen's status to False...")
			self.bunsen_status_client.update_robot_status('bunsen', False)

			# Step 4:
			self.get_logger().info("Moving Bunsen back to home position.")
			print("\nYou may run the test function again on specific button press ")


		except Exception as error:
			# Assuming there's a way to signal error audibly or visually for Bunsen
			self.get_logger().error(f"Error in bunsen_test: {error}")



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
		(KeyCode(char='n'), bunsen.bunsen_test),

	])
	
	print(" Press 'n' to intitiate bunsen test")


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


