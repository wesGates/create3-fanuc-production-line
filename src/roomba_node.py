# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.action.client import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Bool 


import time
import sys
sys.path.append("../dependencies/")

# Create3 Packages
import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, RotateAngle, Dock, NavigateToPosition, AudioNoteSequence
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # Import Odometry
from irobot_create_msgs.srv import ResetPose
from irobot_create_msgs.msg import AudioNoteVector, AudioNote, IrIntensityVector, IrOpcode
from builtin_interfaces.msg import Duration

# key_commander stuff
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

# Python Packages
import random, time
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
from dock_status_node import DockStatusMonitorNode
from ir_status_node import IrMonitorNode
from odometry_node import OdomNode
from RobotClientNode import RobotClientNode

# Globals
start_note = [ AudioNote(frequency=600, max_runtime=Duration(sec=0, nanosec= 50000000))]
end_note =   [ AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec= 50000000))]
rand_note =  [ AudioNote(frequency=350, max_runtime=Duration(sec=0, nanosec= 150000000))]
ready_notes = [
	AudioNote(frequency=583, max_runtime=Duration(sec=0, nanosec= 50000000)),
	AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec=100000000))
]
error_notes = [
	AudioNote(frequency=583, max_runtime=Duration(sec=0, nanosec= 50000000)),
	AudioNote(frequency=350, max_runtime=Duration(sec=0, nanosec=100000000))
]

topic = "vandalrobot"

# class BrokerNode(Node):
# 	def __init__(self, stop_all)
# 	# Establish start and stop messaging from the broker
# 	broker_thread = threading.Thread(target=mqttc.loop_start)
# 	broker_thread.start()
	
# 	stop_thread = threading.Thread(target=stop_all,args=(exec,roomba,rclpy))
# 	stop_thread.start()


class RoombaInfo(Node):
	def __init__(self, namespace, roomba_status_client, dock_sensor, odometry_sensor):
		super().__init__('roomba_info_node')
		self.roomba_status_client = roomba_status_client
		self.dock_sensor = dock_sensor
		self.odometry_sensor = odometry_sensor

		self.latest_dock_status = False
		self.latest_pose_stamped = PoseStamped()


		# Subscriptions: 
		self.dock_status_sub_ = self.create_subscription(Bool, f'/{namespace}/check_dock_status', 
												self.dock_status_callback, 10)

		self.current_pose_sub_ = self.create_subscription(PoseStamped, f'/{namespace}/pose_stamped', 
												self.pose_callback, qos_profile_sensor_data)

		# Services:
		self.reset_pose_srv = self.create_client(ResetPose, f'/{namespace}/reset_pose')
		while not self.reset_pose_srv.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('ResetPose service not available, waiting again...')


	def dock_status_callback(self, msg):
		"""Update the internal RobotState with the latest dock status."""
		self.latest_dock_status = msg
		self.get_logger().info(f"Received /is_docked status: {self.latest_dock_status}")


	def pose_callback(self, msg):
		"""Update the internal RobotState with the latest pose."""
		self.latest_pose_stamped = msg # NavigateToPosition action needs the whole PoseStamped msg
		# self.get_logger().info(f"Received stamped pose status: {self.latest_pose_stamped}")


	def request_odometry_update(self):
		self.odometry_sensor.publish_odometry()

	def request_dock_update(self):
		self.dock_sensor.publish_dock_status()

	def get_dock_status(self):
		return self.latest_dock_status

	def get_pose_stamped(self):
		return self.latest_pose_stamped


	def reset_pose(self):
		"""Public method to reset the robot's pose estimate."""
		try:
			request = ResetPose.Request()
			future = self.reset_pose_srv.call_async(request)
			rclpy.spin_until_future_complete(self, future)
			if future.result() is not None:
				self.get_logger().info('ResetPose service called successfully.')
				return True
			else:
				self.get_logger().error('No response from ResetPose service.')
				return False
		except Exception as e:
			self.get_logger().error(f'Failed to call ResetPose service: {e}')
			return False
		


class Roomba(Node):
	def __init__(self, namespace, roomba_info):
		super().__init__('roomba_node')
		self.roomba_info = roomba_info

		# Action clients:
		self.dock_ac = ActionClient(self, Dock, f'/{namespace}/dock')
		self.undock_ac = ActionClient(self, Undock, f'/{namespace}/undock')
		self.drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance')
		self.nav_to_pos_ac = ActionClient(self, NavigateToPosition, f'/{namespace}/navigate_to_position')
		self.rotate_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle')
		self.audio_ac = ActionClient(self, AudioNoteSequence, f'/{namespace}/audio_note_sequence')
		

	def reportSender(self, label="Undefined", messageType="Report", action="Undefined",
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
			"messageType": messageType,
			"node": "roomba",
			"nodeId": "gatesroomba12",
			"productLine": "moscow",
			"roombareport": {
				"isDock": str(roomba_info.latest_dock_status),
				"action": action,
				"label":label,
				"isReady": isReady,
				"withDice": iswithDice,
				"isAtBase1": isAtBase1,
				"isAtBase2": isAtBase2,
				"isAtBase3": isAtBase3,
				"isMoving": isMoving,
				"position": [

							int(roomba_info.latest_pose_stamped.pose.position.x*1000),
							int(roomba_info.latest_pose_stamped.pose.position.y*1000),
							int(roomba_info.latest_pose_stamped.pose.position.z*1000)],

				"Fault": {
				}
			},
			"date": str(datetime.now())
		}
		
		mqttc.publish(topic, json.dumps(data))


	def chirp(self, sent_notes):
		"""
		This function plays a given chirp tone.
		"""
		self.audio_ac.wait_for_server()

		goal = AudioNoteSequence.Goal()
		goal.iterations = 1
		goal.note_sequence = AudioNoteVector(notes=sent_notes)  # Wrap the notes in an AudioNoteVector

		# Send the goal
		self.audio_ac.send_goal_async(goal)
		print("chirped")


	##### Methods for movements #####
	def undock(self):
		print("Undocking started")
		self.chirp(start_note)
		self.undock_ac.wait_for_server()
		undock_goal = Undock.Goal()
		self.undock_ac.send_goal(undock_goal)

		time.sleep(1)
		self.chirp(end_note)

	def dock(self):
		print("Docking started")
		self.chirp(start_note)

		self.dock_ac.wait_for_server()
		dock_goal = Dock.Goal()
		self.dock_ac.send_goal(dock_goal)
		roomba_info.request_dock_update()
		time.sleep(1)
		self.chirp(end_note)


	def record_pose(self):
		"""
		Request an odometry update, then record the latest pose.
		"""
		# Trigger the odometry update
		roomba_info.request_odometry_update()

		# Wait for a short period to allow data to be published and processed
		time.sleep(1)  # Adjust timing based on system responsiveness

		# Fetch the updated pose
		current_pose = roomba_info.latest_pose_stamped

		if current_pose and current_pose.pose:
			self.recorded_pose = current_pose
			self.get_logger().info(f"PoseStamped recorded: X={current_pose.pose.position.x}, Y={current_pose.pose.position.y}")
		else:
			self.get_logger().error("No current pose available to record.")


	def drive_amnt(self, distance):
		print("Driving amount:", distance, "m")

		self.chirp(start_note)
		self.drive_ac.wait_for_server()
		drive_goal = DriveDistance.Goal()
		drive_goal.distance = distance
		self.drive_ac.send_goal(drive_goal)
		
		time.sleep(1)  # Consider using async

		self.chirp(end_note)


	def rotate_amnt(self, angle):
		print("Rotating amount:", angle, "rad")
		self.chirp(start_note)
		self.rotate_ac.wait_for_server()
		rotate_goal = RotateAngle.Goal()
		rotate_goal.angle = angle
		self.rotate_ac.send_goal(rotate_goal) # !!!!!! Getting stuck here!
		

	def navigate_to_recorded_pose(self):
		"""
		Navigate back to the stored pose.
		"""
		self.chirp(start_note)
		
		print("Navigating back to home position from Odometry reading (PoseStamped)")
		if self.recorded_pose is not None:
			goal_msg = NavigateToPosition.Goal()
			goal_msg.goal_pose = self.recorded_pose
			goal_msg.achieve_goal_heading = True
			goal_msg.max_translation_speed = 0.3
			goal_msg.max_rotation_speed = 1.9

			self.nav_to_pos_ac.wait_for_server()
			self.nav_to_pos_ac.send_goal(goal_msg)
			self.chirp(ready_notes)
		else:
			self.chirp(error_notes)
			self.get_logger().error("No pose has been recorded.")


	def takeoff(self):
		try:
		
			# Waits for start message from the broker before starting the circuit
			while not brokerSender.start_all_message :
				# print(brokerSender.start_all_message)
				print("Waiting for start_all message from the broker...")
				time.sleep(1.0)
				pass

			print("Start all message received from the broker!")

			# Define the process labels to be forwarded to the broker
			roomba_label_1_1 = "Traveling from base1 to base2."
			roomba_label_1_2 = "Waiting to deliver the dice block to beaker."
			roomba_label_1_3 = "Dice block is delivered to beaker"

			roomba_label_2_1 = "Traveling from base2 to base3."
			roomba_label_2_2 = "Waiting to receive the dice block from bunsen."
			roomba_label_2_3 = "Dice block is received from bunsen."

			roomba_label_3 = "Traveling from base3 to base1."
			roomba_label_4 = "Finished cycle."

			##############################################################################################
			""" Simulated circuit for testing out status checking """			
			""" Run a simulated BeakerNode and BunsenNode for toggling ready statuses """			
			##############################################################################################			

			## Actions for process 1: Navigating from base1 to base 2 ###
			# Undock and reset pose
			self.reportSender(roomba_label_1_1, action="undock_start", isAtBase1=True, isMoving=False)
			self.undock()
			self.reportSender(roomba_label_1_1, action="undock_done", isAtBase1=False, isMoving=True)
			# self.roomba_info.reset_pose()

			# rotate amount
			self.reportSender(roomba_label_1_1, action="rotate_start", isMoving=True)
			self.rotate_amnt(-pi/5)
			self.reportSender(roomba_label_1_1, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_1_1, action="drive_start", isMoving=True)
			self.drive_amnt(2.15)
			self.reportSender(roomba_label_1_1, action="drive_done", isMoving=True)
   
			# rotate amount to face the dock
			self.reportSender(roomba_label_1_1, action="rotate_start", isMoving=True)
			self.rotate_amnt(pi/5)
			self.reportSender(roomba_label_1_1, action="rotate_done", isMoving=True)
			############################

			# dock
			self.reportSender(roomba_label_1_1, action="dock_start", isMoving=True)
			self.dock()
			self.reportSender(roomba_label_1_2, action="dock_done", isMoving=False, isAtBase2=True)			


			# AFTER DOCKING AT BASE2
			#####################################################################
			# Step 1-1: Set the status of roomba_base2 to True after docking
			self.get_logger().info("Setting roomba_base2 status to True...")
			roomba_info.roomba_status_client.update_robot_status('roomba_base2', True)

			# Step 1-2: Wait for beaker to become True, indicating it's in position for the dice pickup
			self.get_logger().info("Waiting for beaker status to become True")
			roomba_info.roomba_status_client.wait_for_specific_status('beaker', True)

			# Step 1-3: Wait for beaker to become False, indicating beaker has the dice block and has moved away
			self.get_logger().info("Waiting for beaker status to become False...")
			roomba_info.roomba_status_client.wait_for_specific_status('beaker', False)
			self.get_logger().info("beaker status is now False.")

			# Step 1-4: Set roomba_base2 status to False
			self.get_logger().info("Setting roomba_base2 status to False...")
			roomba_info.roomba_status_client.update_robot_status('roomba_base2', False)
			#####################################################################

			# Tell the broker the dice block is delivered
			self.reportSender(roomba_label_1_3)

			### Actions for process 2: Navigating to base3 from base2 ###
			self.reportSender(roomba_label_2_1, action="undock_start", isAtBase2=True, isMoving=False)
			self.undock()
			self.reportSender(roomba_label_2_1, action="undock_done", isAtBase2=False, isMoving=True)

			# drive amount
			self.reportSender(roomba_label_2_1, action="drive_start", isMoving=True)
			self.drive_amnt(2.65)
			self.reportSender(roomba_label_2_1, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_2_1, action="rotate_start", isMoving=True)
			self.rotate_amnt(pi/2)
			self.reportSender(roomba_label_2_1, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_2_1, action="drive_start", isMoving=True)
			self.drive_amnt(2.7)
			self.reportSender(roomba_label_2_1, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_2_1, action="rotate_start", isMoving=True)
			self.rotate_amnt(pi/2)
			self.reportSender(roomba_label_2_1, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_2_1, action="drive_start", isMoving=True)
			self.drive_amnt(2.3)
			self.reportSender(roomba_label_2_1, action="drive_done", isMoving=True)

			# # rotate amount (attempt to accelerate docking)
			# self.reportSender(roomba_label_2, action="rotate_start", isMoving=True)
			# self.rotate_amnt(-pi/3)
			# self.reportSender(roomba_label_2, action="rotate_done", isMoving=True)

			# dock
			self.reportSender(roomba_label_2_1, action="dock_start", isMoving=True)
			self.dock()
			self.reportSender(roomba_label_2_2, action="dock_done", isMoving=False, isAtBase3=True)


			# AFTER DOCKING AT BASE3
			#####################################################################
			# Step 2-1: After docking, wait for bunsen to become True before setting roomba_base3 to true
			self.get_logger().info("Waiting for bunsen status to become True...")
			roomba_info.roomba_status_client.wait_for_specific_status('bunsen', True)
			self.get_logger().info("bunsen status is now True.")

			# Step 2-2: Tell bunsen that the dice block can be delivered
			self.get_logger().info("Setting roomba_base3 status to True...")
			roomba_info.roomba_status_client.update_robot_status('roomba_base3', True)

			# Step 2-3: Wait for bunsen to indicate that the dice block was delivered, and has moved away
			self.get_logger().info("Waiting for bunsen status to become False...")
			roomba_info.roomba_status_client.wait_for_specific_status('bunsen', False)
			self.get_logger().info("bunsen status is now False.")

			# Step 2-4: Reset roomba_base3 to False
			self.get_logger().info("Setting roomba_base3 status to False...")
			roomba_info.roomba_status_client.update_robot_status('roomba_base3', False)
			
			#####################################################################
			
			# Tell broker the dice block has been received from bunsen
			self.reportSender(roomba_label_2_3)

			### Actions for process 3: Navigating to base1 from base3 ###
			self.reportSender(roomba_label_3, action="undock_start", isAtBase3=True, isMoving=False)
			self.undock()
			self.reportSender(roomba_label_3, action="undock_done", isAtBase3=False, isMoving=True)
   

			#####################################################################
			# AFTER LEAVING BASE3
			# Step 3: Print statement indicating the transition
			self.get_logger().info("Success! Traveling from base 3 to base 1.")
			print("\nYou may run the test function again on specific button press ")
			#####################################################################


			# drive amount
			self.reportSender(roomba_label_3, action="drive_start", isMoving=True)
			self.drive_amnt(2.60)
			self.reportSender(roomba_label_3, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_3, action="rotate_start", isMoving=True)
			self.rotate_amnt(-pi/2)
			self.reportSender(roomba_label_3, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_3, action="drive_start", isMoving=True)
			self.drive_amnt(2.60)
			self.reportSender(roomba_label_3, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_3, action="rotate_start", isMoving=True)
			self.rotate_amnt(-pi/4)
			self.reportSender(roomba_label_3, action="rotate_done", isMoving=True)

			# drive amount
			self.reportSender(roomba_label_3, action="drive_start", isMoving=True)
			self.drive_amnt(2.00)
			self.reportSender(roomba_label_3, action="drive_done", isMoving=True)

			# rotate amount
			self.reportSender(roomba_label_3, action="rotate_start", isMoving=True)
			self.rotate_amnt(3*pi/4)
			self.reportSender(roomba_label_3, action="rotate_done", isMoving=True)

			# dock
			self.reportSender(roomba_label_3, action="dock_start", isMoving=True)
			self.dock()
			self.reportSender(roomba_label_4, action="dock_done", isMoving=False, isAtBase1=True)
			self.reportSender(roomba_label_4, messageType = "stop", action="dock_done", isMoving=False, isAtBase1=True)


		except Exception as error:
			roomba.chirp(error_notes)
			self.get_logger().error(f"Error in takeoff: {error}")



def stop_all(exec,roomba,rclpy):
	while not brokerSender.stop_all_message :
				# print(brokerSender.start_all_message)
				pass
	
	exec.shutdown()
	roomba.destroy_node()
	rclpy.try_shutdown()


if __name__ == '__main__':
	rclpy.init()
	namespace = 'create3_05AE'
	
	# Initialize RobotState first

	dock_sensor = DockStatusMonitorNode(namespace)
	odometry_sensor = OdomNode(namespace)
	roomba_status_client = RobotClientNode(namespace)

	roomba_info = RoombaInfo(namespace, roomba_status_client, dock_sensor, odometry_sensor)
	
	roomba = Roomba(namespace, roomba_info)

	exec = MultiThreadedExecutor(7) # Acer laptop and Surface have 8 threads
	exec.add_node(dock_sensor) # 1
	exec.add_node(odometry_sensor) # 1
	exec.add_node(roomba_status_client) # 1
	exec.add_node(roomba_info) # 2 (one for each service in the roomba_status_client)
	exec.add_node(roomba) # 1 
	# + 1 for broker thread

	# time.sleep(0.1)
	roomba.chirp(ready_notes)


# def BrokerNode(Node):
	# Establish start and stop messaging from the broker
	broker_thread = threading.Thread(target=mqttc.loop_start)
	broker_thread.start()
	
	stop_thread = threading.Thread(target=stop_all,args=(exec,roomba,rclpy))
	stop_thread.start()


	# exec.add_node(broker_thread)

	keycom = KeyCommander([
		(KeyCode(char='1'), roomba.takeoff),

	])
	print(" Press '1' to intitiate launch")

	try:
		exec.spin()  # execute Roomba callbacks until shutdown or destroy is called
	except KeyboardInterrupt:
		print("KeyboardInterrupt, shutting down.")
		roomba.destroy_node()
		roomba_status_client.destroy_node()
		dock_sensor.destroy_node()
		odometry_sensor.destroy_node()
		roomba_status_client.destroy_node()
		rclpy.shutdown()
	except Exception as error:
		print(f"Unexpected error: {error}")
	finally:
		exec.shutdown()
		rclpy.try_shutdown()


