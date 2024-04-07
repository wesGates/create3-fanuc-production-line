import sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node
from pynput.keyboard import KeyCode
import my_interfaces
from my_interfaces.msg import ReadyStatus


class ReadyStatusPublisherNode(Node):
	""" Initialization and other methods remain unchanged from RoombaNode"""


	def __init__(self):
		super().__init__('ready_status_publisher')
		self.ready_status_publisher_ = self.create_publisher(ReadyStatus, 'robot_ready_status', 10)
		self.status_file_path = 'robot_status.txt'


	def publish_ready_status(self):
		"""
		Publish the status of robots by reading their statuses from the robot_status.txt file.
		The order is roomba_base2, beaker, beaker_conv, bunsen_conv, bunsen, roomba_base3
		"""
		print("\n Publisher is publishing ready statuses...")

		with open(self.status_file_path, 'r') as file:
			statuses = file.read().strip().split(',')

		# Creating the message to publish
		msg = ReadyStatus()
		msg.roomba_base2 = statuses[0] == 'True'
		msg.beaker = statuses[1] == 'True'
		msg.beaker_conv = statuses[2] == 'True'
		msg.bunsen_conv = statuses[3] == 'True'
		msg.bunsen = statuses[4] == 'True'
		msg.roomba_base3 = statuses[5] == 'True'


		# Publishing the message
		self.ready_status_publisher_.publish(msg)

		# print(f"DEBUGGING: Published statuses - Roomba: {msg.roomba}, Beaker: {msg.beaker}, Bunsen: {msg.bunsen}")

		
	def display_robot_statuses(self):
		"""
		Display the current status of Roomba, Beaker, and Bunsen by reading the status file.
		The order is roomba_base2, beaker, beaker_conv, bunsen_conv, bunsen, roomba_base3
		"""

		print("display current robot statuses:")
		with open(self.status_file_path, 'r') as file:
			statuses = file.read().strip().split(',')
			print(	f"\n roomba_base2: {statuses[0]}, " + 
		 			f"\n beaker: 	{statuses[1]}, " +
					f"\n beaker_conv: 	{statuses[2]}, " +
					f"\n bunsen_conv: 	{statuses[3]}, " +
					f"\n bunsen: 	{statuses[4]}, " +
					f"\n roomba_base3:  {statuses[5]}	")


	def set_ready_status(self, 
					  roomba_status_base2=None, 
					  beaker_status=None, beaker_conv_status=None,
					  bunsen_conv_status=None, bunsen_status=None,
					  roomba_status_base3=None):
		"""
		Update the status of Roomba, Beaker, and Bunsen in the status file.
		The order is in the direction that the dice block goese:
		roomba, beaker, beaker_conv, bunsen_conv, bunsen
		"""
		with open(self.status_file_path, 'r') as file:
			statuses = file.read().strip().split(',')

		# print("DEBUG: STATUS BEFORE WRITE")
		# print(statuses)

		# Update statuses based on the input parameters
		if roomba_status_base2 is not None:
			statuses[0] = str(roomba_status_base2)
		if beaker_status is not None:
			statuses[1] = str(beaker_status)
		if beaker_conv_status is not None:
			statuses[2] = str(beaker_conv_status)
		if bunsen_conv_status is not None:
			statuses[3] = str(bunsen_conv_status)
		if bunsen_status is not None:
			statuses[4] = str(bunsen_status)
		if roomba_status_base3 is not None:
			statuses[5] = str(roomba_status_base3)

		with open(self.status_file_path, 'w') as file:
			file.write(','.join(statuses) + '\n')

		# print("DEBUG: STATUS AFTER WRITE")
		# print(statuses)