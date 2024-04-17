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
# from brokerSender import mqttcb

# Node Imports
from RobotClientNode import RobotClientNode

# Globals
topic = "vandalrobot"

# Initialize and connect to other nodes
rclpy.init()
namespace = 'beaker'

# Makes instance of the universal robot node that sets/ waits on statuses
beaker_status_client = RobotClientNode(namespace)

class BeakerNode(Node):
	def __init__(self, namespace):
		super().__init__('beaker_node')

		"""" This is everything that needs to be put into your chosen node"""

		self.beaker_status_client = beaker_status_client

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


	def beaker_test(self):
		try:
			# Step 1: Set Beaker's status to True to indicate it is ready for dice block pickup
			self.get_logger().info("Setting Beaker's status to True...")
			self.beaker_status_client.update_robot_status('beaker', True)

			# Step 2: Wait for roomba_base2 to become True before dice block pickup
			self.get_logger().info("Waiting for roomba_base2 status to become True...")
			self.beaker_status_client.wait_for_specific_status('roomba_base2', True)

			# Step 3: Set Beaker's status to False to tell the roomba that it can move away
			self.get_logger().info("Setting Beaker's status to False...")
			self.beaker_status_client.update_robot_status('beaker', False)


			# Step 4-1: Check for bunsen_conv True before placing the dice block
			self.get_logger().info("Waiting for bunsen_conv status to become True...")
			self.beaker_status_client.wait_for_specific_status('bunsen_conv', True)

			# Step 4-2: Place dice block
			# Step 4-3: Check if dice block is seen by the first conveyor sensor (maybe move away first)
			# -- If dice block is absent, throw fault
			# -- Otherwise, start conveyor
			# Step 4-4: Wait until dice block is in position or until timeout, then...

			# Step 4-5: Indicate to bunsen that the diceblock is detected by the second conveyor sensor
			self.get_logger().info("Setting beaker_conv's status to True...")
			self.beaker_status_client.update_robot_status('beaker_conv', True)

			# Step 4-6: Wait for bunsen_conveyor goes false to ensure the diceblock pickup was successful
			self.get_logger().info("Waiting for bunsen_conv status to become False...")
			self.beaker_status_client.wait_for_specific_status('bunsen_conv', False)

			# Step 4-7: Reset beaker_conveyor's status to False
			self.get_logger().info("Setting beaker_conv's status to False...")
			self.beaker_status_client.update_robot_status('beaker_conv', False)


			# Step 5: Set BeakerConv's status to False, assuming all went well with bunsen
			self.get_logger().info("Setting BeakerConv's status to False...")
			self.beaker_status_client.update_robot_status('beaker_conv', False)

			# Step 6:
			self.get_logger().info("Success! Moving Beaker back to home position.")
			print("\nYou may run the test function again on specific button press ")

		except Exception as error:
			# Assuming there's a way to signal error audibly or visually for Beaker
			self.get_logger().error(f"Error in beaker_test: {error}")




def stop_all(exec,roomba,rclpy):
	while not brokerSender.stop_all_message :
				# print(brokerSender.start_all_message)
				pass
	
	exec.shutdown()
	roomba.destroy_node()
	rclpy.try_shutdown()


if __name__ == '__main__':
	# rclpy.init()

	beaker = BeakerNode(namespace)

	exec = MultiThreadedExecutor(3) # No. Nodes + 1

	exec.add_node(beaker)
	exec.add_node(beaker_status_client)
	
	# NOTE: This node should be spun up in another terminal beforehand
	# exec.add_node(ready_status_publisher_node) 
	

	# # NOTE: Uncomment to enable broker stuff
 	# Establish start and stop messaging from the broker
	# broker_thread = threading.Thread(target=mqttc.loop_start)
	# broker_thread.start()
	
	# stop_thread = threading.Thread(target=stop_all,args=(exec,beaker,rclpy))
	# stop_thread.start()


	keycom = KeyCommander([
		(KeyCode(char='b'), beaker.beaker_test),

	])
	
	print(" Press 'b' to intitiate beaker test")


	try:
		exec.spin()  # execute beaker callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
		beaker.destroy_node()
		beaker_status_client.destroy_node()
		rclpy.shutdown()
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		rclpy.try_shutdown()


