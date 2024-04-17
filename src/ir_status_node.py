# // Roomba_skeleton
# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from numpy import uint8


# Create3 Packages
from irobot_create_msgs.msg import  IrOpcode

# Python Packages

from std_msgs.msg import String # Some topics have specific datatypes

class IrMonitorNode(Node):
	def __init__(self, namespace):
		super().__init__('ir_sensor')
		
		# Publishers
		self.opcode_publisher_ = self.create_publisher(String, 'ir_opcode_number', 10)

		# Subscriptions
		self.opcode_subscriber = self.create_subscription(IrOpcode, f'/{namespace}/ir_opcode', 
														  self.ir_opcode_listener, qos_profile_sensor_data)
		
		# Variable Initialization for storing published information
		self.current_opcode = None


	def ir_opcode_listener(self, msg):
		""" 
		Constantly updates the opcode message from the /ir_opcode topic.
		This includes a bunch of info that will be filtered out in the publisher.
		"""
		self.current_opcode = msg.opcode # Containes all the opcode information (a lot of stuff we dont need.


	def publish_ir_opcode(self):
		""" This method publishes the opcode value as a UInt8 message to all subscribers """
		msg = String()
		msg.data = str(self.current_opcode)

		self.opcode_publisher_.publish(msg)
		return self.current_opcode



		 
