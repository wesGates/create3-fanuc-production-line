# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
# ROS message typing
from std_srvs.srv import Trigger

import sys
sys.path.append("../dependencies/")
# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, SchunkGripper, JointPose, Conveyor
from fanuc_interfaces.msg import CurGripper, CurCartesian, CurJoints, ProxReadings, IsMoving

import my_interfaces
from my_interfaces.msg import ReadyStatus
from my_interfaces.srv import CheckReadiness
from ReadyStatusPublisherNode import ReadyStatusPublisherNode
from ReadinessTrackerNode import ReadinessTrackerNode

import time
from time import sleep

from datetime import datetime
import json

import threading

import brokerSender
from brokerSender import mqttc

topic = "vandalrobot"

# Node that listens to the fanuc topic
class FanucTopic(Node):
	def __init__(self, namespace):
		super().__init__("robotTopics")

		self.label = ""
		self.action = ""
		self.isHome = False
		self.ready = True
		self.withDice = False
		self.isMoving = False
		self.isGripped = False

		self.isBeltMoving = False
		self.beltSensorRear = False
		self.beltSensorFront = False
		self.convReady = True

		self.position = [[],[]]

		self.fault = ""
		

		# Create a Service node for sending data to 'Main', could be a Topic also
		self.gripperSrv = self.create_service(Trigger, 'check_gripper', self.service_callback)
		self.cartSrv = self.create_service(Trigger, 'check_cartesian', self.service_callback)
		self.jointSrv = self.create_service(Trigger, 'check_joint', self.service_callback)
		self.convSrv = self.create_service(Trigger, 'check_conveyor', self.service_callback)
		self.moveSrv = self.create_service(Trigger, 'check_moving', self.service_callback)
		
		# This subscribes to the Fanuc topic
		self.gripperSubs = self.create_subscription(CurGripper, f'/{namespace}/grip_status', self.gripListener, qos_profile_sensor_data)
		self.cartSubs = self.create_subscription(CurCartesian, f'/{namespace}/cur_cartesian', self.cartListener, qos_profile_sensor_data)
		self.jointSubs = self.create_subscription(CurJoints, f'/{namespace}/cur_joints', self.jointListener, qos_profile_sensor_data)
		self.convSubs = self.create_subscription(ProxReadings, f'/{namespace}/prox_readings', self.convListener, qos_profile_sensor_data)
		self.moveSubs = self.create_subscription(IsMoving, f'/{namespace}/is_moving', self.moveListener, qos_profile_sensor_data)

		# This will hold the current value from the Fanuc topic
		self.curValue = False # Temporary default value
		
	def gripListener(self, msg):
		"""
		This will be called everytime the subscriber recieves a new message from the topic
		"""
		self.curValue = msg.open

	def cartListener(self, msg):
		self.position[0] = list(msg.pose)

	def jointListener(self, msg):
		self.position[1] = list(msg.joints)

	def convListener(self, msg):
		self.beltSensorFront = msg.left
		self.beltSensorRear = msg.right

	def moveListener(self, msg):
		self.isMoving = msg.moving
		
	def service_callback(self, request, response):
		"""
		This will run when we send a service request
		"""
		response.message = str(self.curValue)
		return response
	
	def ready_status_callback(self, msg):
		"""
		The other nodes don't have this function, yet. 
		I think they will need it in order to do their own wait_for_all_ready operations.
		"""
		# Update the latest ready statuses with the message received from the 'robot_ready_status' topic

		# DEBUGGING
		print("DEBUG: Start of RoombaNode callback...")
		print("msg.roomba_base2 : 	", msg.roomba_base2)
		print("msg.beaker : 	", msg.beaker)
		print("msg.beaker_conv:",msg.beaker_conv)
		print("msg.bunsen_conv:",msg.bunsen_conv)
		print("msg.bunsen : 	", msg.bunsen)
		print("msg.roomba_base3 : 	", msg.roomba_base3)
		print("End of RoombaNode callback \n")

		self.latest_ready_status = msg
		statuses = [None,None,None,None,None,None]
		statuses[0] = msg.roomba_base2
		statuses[1] = msg.beaker
		statuses[2] = msg.beaker_conv
		statuses[3] = msg.bunsen_conv
		statuses[4] = msg.bunsen
		statuses[5] = msg.roomba_base3

		statuses = [
            str(msg.roomba_base2),
            str(msg.beaker),
            str(msg.beaker_conv),
            str(msg.bunsen_conv),
            str(msg.bunsen),
            str(msg.roomba_base3),
        ]

		with open(self.status_file_path, 'w') as file:
			file.write(','.join(statuses) + '\n')


# 'Main' node
class FanucActions(Node):
	def __init__(self, namespace):
		super().__init__("robotActions")
		self.namespace = namespace

		# Actions, note their callback groups
		self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
		self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper') 
		self.joint_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose') 
		self.conv_ac = ActionClient(self, Conveyor, f'/{namespace}/conveyor')
		
		# Sercice Client
		self.gripService = self.create_client(Trigger,'/check_gripper')
		self.cartService = self.create_client(Trigger,'/check_cartesian')
		self.jointService = self.create_client(Trigger,'/check_joint')
		self.convService = self.create_client(Trigger,'/check_conveyor')
		self.moveService = self.create_client(Trigger,'/check_moving')
		
	def reportSender(self):
		""""
		Used to send a report at the beginning and end of every action.
		Defaults to False.
		"""
		if self.namespace == 'beaker':
			data = {
			"messageType": "Report",
			"node": "crx10",
			"nodeId": "bryancrx10_1",
			"productLine": "moscow",
			"crx10report":{
				"label":FanucTopic.label,
				"action":FanucTopic.action,
				"isHome":FanucTopic.isHome,
				"CRX10Ready":FanucTopic.ready,
				"withDice":FanucTopic.withDice,
				"ismoving":FanucTopic.isMoving,
				"isGripped":FanucTopic.isGripped,
				"position":FanucTopic.position,
				"isBeltmoving":FanucTopic.isBeltMoving,
				"lineSensorBackBelt":FanucTopic.beltSensorRear,
				"lineSensorFrontBelt":FanucTopic.beltSensorFront,
				"convReady":FanucTopic.convReady,
				"Fault": FanucTopic.fault
			},
			"date": str(datetime.now())
		}
		if self.namespace == 'bunsen':
			data = {
			"messageType": "Report",
			"node": "crx10",
			"nodeId": "bryancrx10_2",
			"productLine": "moscow",
			"crx10report":{
				"label":FanucTopic.label,
				"action":FanucTopic.action,
				"isHome":FanucTopic.isHome,
				"CRX10Ready":FanucTopic.ready,
				"withDice":FanucTopic.withDice,
				"ismoving":FanucTopic.isMoving,
				"isGripped":FanucTopic.isGripped,
				"position":FanucTopic.position,
				"isBeltmoving":FanucTopic.isBeltMoving,
				"lineSensorBackBelt":FanucTopic.beltSensorRear,
				"lineSensorFrontBelt":FanucTopic.beltSensorFront,
				"convReady":FanucTopic.convReady,
				"Fault": FanucTopic.fault
			},
			"date": str(datetime.now())
		}
	
		mqttc.publish(topic, json.dumps(data))
		
		
	def cartMove(self, x, y, z, w, p, r):
		FanucTopic.action = "cartMove_start"
		self.reportSender()

		self.cart_ac.wait_for_server()
		cart_goal = CartPose.Goal()
		cart_goal.x = x
		cart_goal.y = y
		cart_goal.z = z
		cart_goal.w = w
		cart_goal.p = p
		cart_goal.r = r
		self.cart_ac.send_goal(cart_goal)

		FanucTopic.action = "cartMove_done"
		self.reportSender()
		
	def jointMove(self, j1, j2, j3, j4, j5, j6):
		FanucTopic.action = "jointMove_start"
		self.reportSender()

		self.joint_ac.wait_for_server()
		joint_goal = JointPose.Goal()
		joint_goal.joint1 = j1
		joint_goal.joint2 = j2
		joint_goal.joint3 = j3
		joint_goal.joint4 = j4
		joint_goal.joint5 = j5
		joint_goal.joint6 = j6
		self.joint_ac.send_goal(joint_goal)

		FanucTopic.action = "jointMove_done"
		self.reportSender()

	def convMove(self, direction):
		FanucTopic.action = "convMove_start"
		self.reportSender()

		if direction == 'forward' or direction == 'reverse':
			isBeltMoving = True
		elif direction == 'stop':
			isBeltMoving = False
		self.conv_ac.wait_for_server()
		conv_goal = Conveyor.Goal()
		conv_goal.command = direction
		self.conv_ac.send_goal(conv_goal)

		FanucTopic.action = "convMove_done"
		self.reportSender()
		
	def schunkMove(self, type):
		FanucTopic.action = "gripperMove_start"
		self.reportSender()

		if type == 'open':
			isGripped = False
		elif type == 'close':
			isGripped = True
		self.schunk_ac.wait_for_server()
		schunk_goal = SchunkGripper.Goal()
		schunk_goal.command = type
		self.schunk_ac.send_goal(schunk_goal)

		FanucTopic.action = "gripperMove_done"
		self.reportSender()

	def taskBeaker(self):
		self.schunkMove('open')
		
		self.cartMove(302.556, -539.279, -83.733, -179.284, -2.058, -120.535)           # Avoid table
		self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)               # Home
		
		while not brokerSender.start_all_message:                                      # Wait for start signal
			pass

		FanucTopic.label = "Picking up dice block at base 2"

		self.cartMove(302.556, -539.279, -83.733, -179.284, -2.058, -120.535)           # Avoid table
		#self.cartMove(-118.402, -561.427, -1012.155, -177.467, -2.058, -149.770)
		self.jointMove(-87.14, 89.965, -103.104, 3.492, 12.337, -69.112)                # Dock 2 approach

		self.nodeBeaker.check_roomba_base2()

		#self.jointMove(-88.794, 97.111, -108.691, 6.410, 20.199, -64.552)              # Pickup (old)
		self.jointMove(-87.365, 95.745, -103.319, 3.542, 13.117, -69.11)                # Pickup
		
		FanucTopic.withDice = True
		self.schunkMove('close')

		self.jointMove(-87.14, 89.965, -103.104, 3.492, 12.337, -69.112)                # Dock 2 approach

		self.nodeBeaker.set_beaker_false()

		FanucTopic.label = "Dice block flip"

		self.cartMove(302.556, -539.279, -83.733, -179.284, -2.058, -120.535)           # Avoid table
		self.cartMove(656.348, -144.366, -195.534, 127.834, -51.798, -31.558)           # Angled drop

		self.schunkMove('open')

		self.cartMove(656.348, -54.366, -195.534, 127.834, -51.798, -31.558)            # Move away
		self.cartMove(653.397, -161.278, -121.553, 179.158, -1.101, -61.980)            # Table approach
		self.cartMove(653.397, -161.278, -185.0, 179.158, -1.101, -61.980)              # Pickup dock 2

		FanucTopic.label = "Moving dice block to conveyor 1"
		self.schunkMove('close')

		self.cartMove(791.903, 643.183, -53.923, 178.571, 0.950, 30.197)                # Conveyor 1 approach
		self.cartMove(791.903, 643.183, -183.923, 178.571, 0.950, 30.197)               # Conveyor 1 drop

		self.schunkMove('open')
		FanucTopic.withDice = False

		self.cartMove(791.903, 643.183, -53.923, 178.571, 0.950, 30.197)                # Conveyor 1 approach

		FanucTopic.label = "Moving conveyor 1"

		self.nodeBeaker.set_beaker_conv_false()

		self.convMoveBlock()

		self.nodeBeaker.set_beaker_conv_true()                                          # Ready at conveyor 1

		FanucTopic.label = "Moving to home position"

		self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)                # Home

		'''request = Trigger.Request() # Make a request object
		while not self.gripService.wait_for_service(timeout_sec=1.0):
			pass # Wait for service to be ready
		result = self.gripService.call(request).message # Send request and block until given a response

		print(result)'''


	def taskBunsen(self):
		self.jointMove(57.792, 11.879, -30.824, -9.696, -38.583, -63.616)
		self.cartMove(568.058, -75.22, 19.527, -179.637, 1.597, 29.106)                 # Home
		self.schunkMove('open')

		while not brokerSender.start_all_message:                                       # Wait for start signal
			pass

		FanucTopic.label = "Picking up dice block at conveyor 1"

		self.cartMove(75.396, -831.926, -151.291, -178.373, -0.332, 26.7)               # Conveyor 1 approach end
		
		self.nodeBunsen.check_beaker_conv()                                             # Wait for ready at conveyor 1

		self.cartMove(75.396, -831.926, -196.291, -178.373, -0.332, 26.7)               # Conveyor 1 pickup end

		FanucTopic.label = "Moving dice block from conveyor 1 to conveyor 2"
		FanucTopic.withDice = True
		self.schunkMove('close')

		self.cartMove(75.396, -831.926, -92.912, -178.373, -0.332, 26.7)                # Conveyor 1 approach end
		self.cartMove(757.469, -580.225, -92.912, 179.72, -0.016, 27.315)               # Conveyor 2 approach start
		self.cartMove(765.349, -578.817, -197.217, -178.602, 2.568, 28.922)             # Conveyor 2 drop start

		FanucTopic.withDice = False
		self.schunkMove('open')

		self.cartMove(757.469, -580.225, -92.912, 179.72, -0.016, 27.315)               # Conveyor 2 approach start

		FanucTopic.label = "Moving conveyor 2"

		self.nodeBunsen.set_bunsen_conv_false()                                         # Set bunsen conv false
		self.convMoveBlock()
		self.nodeBunsen.set_bunsen_conv_true()                                          # Set bunsen conv true

		FanucTopic.label = "Picking up dice block at conveyor 2"

		#self.cartMove(79.57, -577.628, -92.912, -179.983, 0.628, 26.238)                # Conveyor 2 approach end
		self.jointMove(-64.732, 20.007, -51.662, 3.396, -43.347, 90.833) #-89.167

		# Wait for ready at conveyor 2

		#self.cartMove(79.57, -577.628, -197.217, -179.983, 0.628, 26.238)               # Conveyor 2 pickup end
		self.jointMove(-66.542, 22.921, -55.153, 0.066, -38.991, 94.762) #-85.238

		FanucTopic.withDice = True
		self.schunkMove('close')

		#self.cartMove(79.57, -577.628, -92.912, -179.983, 0.628, 26.238)                # Conveyor 2 approach end
		#self.jointMove(-64.732, 20.007, -51.662, 3.396, -43.347, 90.833) #-89.167
		self.jointMove(-67.273, 10.367, -42.081, 4.478, -49.211, 90.262)

		FanucTopic.label = "Dice block flip"

		self.cartMove(656.348, -144.366, -195.534, 127.834, -51.798, -31.558)           # Angled drop

		self.schunkMove('open')

		self.cartMove(656.348, -54.366, -195.534, 127.834, -51.798, -31.558)            # Move away
		self.cartMove(653.397, -161.278, -121.553, 179.158, -1.101, -61.980)            # Table approach
		self.cartMove(653.397, -161.278, -185.0, 179.158, -1.101, -61.980)              # Pickup

		FanucTopic.label = "Moving dice block to base 3"
		self.schunkMove('close')

		#self.cartMove(363.496, 609.175, -83.366, 179.96, -6.23, 16.058)                 # Avoid table
		self.jointMove(57.792, 11.879, -30.824, -9.696, -38.583, -63.616)
		self.jointMove(109.054, 88.629, -98.042, -15.913, 7.339, -66.288)               # Dock 3 approach

		self.nodeBunsen.check_roomba_base3()                                            # Check create3 ready status

		self.jointMove(109.056, 95.089, -97.762, -15.906, 6.841, -65.237)               # Dock 3 drop

		FanucTopic.withDice = False
		self.schunkMove('open')

		self.jointMove(109.054, 88.629, -98.042, -15.913, 7.339, -66.288)               # Dock 3 approach

		self.nodeBunsen.set_bunsen_false()                                              # Ready for create3 to leave dock 3

		#self.cartMove(363.496, 609.175, -83.366, 179.96, -6.23, 16.058)                 # Avoid table
		self.jointMove(57.792, 11.879, -30.824, -9.696, -38.583, -63.616)
		self.cartMove(568.058, -75.22, 19.527, -179.637, 1.597, 29.106)                 # Home


	def convMoveBlock(self):
		self.convMove('forward')
		timeStart = time.time()
		while beltSensorRear == False:
			if time.time()- timeStart > 10:
				FanucTopic.fault = "No dice block detected on conveyor"
				self.reportSender()
				break
		self.convMove('stop')

	def test(self):
		#self.convMove('forward')
		#self.schunkMove('open')
		#self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)
		sleep(2)
		#self.convMove('stop')
		#self.schunkMove('close')

		#self.reportSender()
		#if beltSensorFront == True:
		#self.convMoveBlock()
		#self.nodeBeaker.check_roomba_base2()
		#self.cartMove(79.57, -577.628, -92.912, -179.983, 0.628, 26.238)
		#self.jointMove(-64.732, 20.007, -51.662, 3.396, -43.347, 90.833)


def stop_all(exec,robot,rclpy):
	while not brokerSender.stop_all_message :
		pass
	exec.shutdown()
	robot.destroy_node()
	rclpy.try_shutdown()


if __name__ == '__main__':
	#rclpy.init()
	# Create our 2 nodes
	mainBeaker = FanucActions('beaker')
	listenerBeaker = FanucTopic('beaker')
	mainBunsen = FanucActions('bunsen')
	listenerBunsen = FanucTopic('bunsen')

	exec = MultiThreadedExecutor(7)

	broker_thread = threading.Thread(target=mqttc.loop_start)
	broker_thread.start()
 
	stop_threadBeaker = threading.Thread(target=stop_all,args=(exec,mainBeaker,rclpy))
	stop_threadBeaker.start()
	stop_threadBunsen = threading.Thread(target=stop_all,args=(exec,mainBunsen,rclpy))
	stop_threadBunsen.start()

	# Add our nodes
	exec.add_node(mainBeaker)
	exec.add_node(listenerBeaker)
	exec.add_node(mainBunsen)
	exec.add_node(listenerBunsen)
	# mqttc.loop_start()???
	
	# This allows us to start the function once the node is spinning
	keycom = KeyCommander([(KeyCode(char='s'), mainBeaker.taskBeaker),])
	keycom = KeyCommander([(KeyCode(char='d'), mainBunsen.taskBunsen),])
	keycom = KeyCommander([(KeyCode(char='e'), mainBeaker.test),])
	keycom = KeyCommander([(KeyCode(char='r'), mainBunsen.test),])
	print("Ready")
	exec.spin() # Start executing the nodes
	rclpy.shutdown()
