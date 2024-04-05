import sys
sys.path.append("../dependencies/")

import rclpy
from rclpy.node import Node
from pynput.keyboard import KeyCode
import my_interfaces
from my_interfaces.msg import ReadyStatus


class ReadyStatusPublisherNode(Node):
	# Initialization and other methods remain unchanged


	def __init__(self):
		super().__init__('ready_status_publisher')
		self.ready_status_publisher_ = self.create_publisher(ReadyStatus, 'robot_ready_status', 10)
		self.status_file_path = 'robot_status.txt'
		self.initialize_status_file()


	def initialize_status_file(self):
		"""
		Initialize the status file with default boolean values for Roomba, Beaker, and Bunsen.
		The order is Roomba, Beaker, Bunsen.
		"""
		with open(self.status_file_path, 'w') as file:
			file.write("False,False,False\n")


	def publish_ready_status(self):
		"""
		Publish the status of robots by reading their statuses from the status file.
		The order is Roomba, Beaker, Bunsen.
		"""
		print("\nPublishing ready statuses...")

		with open(self.status_file_path, 'r') as file:
			statuses = file.read().strip().split(',')

		# Creating the message to publish
		msg = ReadyStatus()
		msg.roomba = statuses[0] == 'True'
		msg.beaker = statuses[1] == 'True'
		msg.bunsen = statuses[2] == 'True'

		# Publishing the message
		self.ready_status_publisher_.publish(msg)
		print(f"Published statuses - Roomba: {msg.roomba}, Beaker: {msg.beaker}, Bunsen: {msg.bunsen}")

		
	def display_robot_statuses(self):
		"""
		Display the current status of Roomba, Beaker, and Bunsen by reading the status file.
		The order is Roomba, Beaker, Bunsen.
		"""
		print("display current robot statuses:")
		with open(self.status_file_path, 'r') as file:
			statuses = file.read().strip().split(',')
			print(f"Roomba: {statuses[0]}, Beaker: {statuses[1]}, Bunsen: {statuses[2]}")


	def set_ready_status(self, roomba_status=None, beaker_status=None, bunsen_status=None):
		"""
		Update the status of Roomba, Beaker, and Bunsen in the status file.
		The order is Roomba, Beaker, Bunsen.
		"""
		with open(self.status_file_path, 'r') as file:
			statuses = file.read().strip().split(',')

		print("STATUS BEFORE WRITE")
		print(statuses)

		# Update statuses based on the input parameters
		if roomba_status is not None:
			statuses[0] = str(roomba_status)
		if beaker_status is not None:
			statuses[1] = str(beaker_status)
		if bunsen_status is not None:
			statuses[2] = str(bunsen_status)

		with open(self.status_file_path, 'w') as file:
			file.write(','.join(statuses) + '\n')

		print("STATUS AFTER WRITE")
		print(statuses)