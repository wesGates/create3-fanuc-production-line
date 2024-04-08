# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
#from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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

from BeakerNode import BeakerNode

import time
from time import sleep

from datetime import datetime
import json

from brokerSender import mqttc

#namespace = 'beaker'
topic = "vandalrobot"

label = ""
action = ""
isHome = False
ready = True
withDice = False
isMoving = False
isGripped = False

isBeltMoving = False
beltSensorRear = False
beltSensorFront = False
convReady = True

position = [[],[]]

fault = ""

nodeBeaker = BeakerNode()

# Node that listens to the fanuc topic
class FanucTopic(Node):
    def __init__(self, namespace):
        super().__init__("robotTopics")
        

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
        global position
        position[0] = list(msg.pose)

    def jointListener(self, msg):
        global position
        position[1] = list(msg.joints)

    def convListener(self, msg):
        global beltSensorFront
        beltSensorFront = msg.left
        global beltSensorRear
        beltSensorRear = msg.right

    def moveListener(self, msg):
        global isMoving 
        isMoving = msg.moving
    	
    def service_callback(self, request, response):
        """
        This will run when we send a service request
        """
        response.message = str(self.curValue)
        return response


# 'Main' node
class FanucActions(Node):
    def __init__(self, namespace):
        super().__init__("robotActions")
        if namespace == 'beaker':
            self.nodeBeaker = nodeBeaker

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

        data = {
            "messageType": "Report",
            "node": "crx10",
            "nodeId": "bryancrx10_1",
            "productLine": "moscow",
            "crx10report":{
                "label":label,
                "action":action,
                "isHome":isHome,
                "CRX10Ready":ready,
                "withDice":withDice,
                "ismoving":isMoving,
                "isGripped":isGripped,
                "position":position,
                "isBeltmoving":isBeltMoving,
                "lineSensorBackBelt":beltSensorRear,
                "lineSensorFrontBelt":beltSensorFront,
                "convReady":convReady,
                "Fault": fault
            },
            "date": str(datetime.now())
        }
    
        #mqttc.publish(topic, json.dumps(data))
        
        
    def cartMove(self, x, y, z, w, p, r):
        self.cart_ac.wait_for_server()
        cart_goal = CartPose.Goal()
        cart_goal.x = x
        cart_goal.y = y
        cart_goal.z = z
        cart_goal.w = w
        cart_goal.p = p
        cart_goal.r = r
        self.cart_ac.send_goal(cart_goal)
        global action
        action = "cartMove"
        self.reportSender()
        
    def jointMove(self, j1, j2, j3, j4, j5, j6):
        self.joint_ac.wait_for_server()
        joint_goal = JointPose.Goal()
        joint_goal.joint1 = j1
        joint_goal.joint2 = j2
        joint_goal.joint3 = j3
        joint_goal.joint4 = j4
        joint_goal.joint5 = j5
        joint_goal.joint6 = j6
        self.joint_ac.send_goal(joint_goal)
        global action
        action = "jointMove"
        self.reportSender()

    def convMove(self, direction):
        if direction == 'forward' or direction == 'reverse':
            isBeltMoving = True
        elif direction == 'stop':
            isBeltMoving = False
        self.conv_ac.wait_for_server()
        conv_goal = Conveyor.Goal()
        conv_goal.command = direction
        self.conv_ac.send_goal(conv_goal)
        global action
        action = "convMove"
        self.reportSender()
    	
    def schunkMove(self, type):
        if type == 'open':
            isGripped = False
        elif type == 'close':
            isGripped = True
        self.schunk_ac.wait_for_server()
        schunk_goal = SchunkGripper.Goal()
        schunk_goal.command = type
        self.schunk_ac.send_goal(schunk_goal)
        global action
        action = "gripperMove"
        self.reportSender()

    def taskBeaker(self):
        global label, action, fault, withDice
        self.schunkMove('open')
        
        self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)               # Home
        
        # Wait for start signal

        label = "Picking up dice block at base 2"

        self.cartMove(302.556, -539.279, -83.733, -179.284, -2.058, -120.535)           # Avoid table
        #self.cartMove(-118.402, -561.427, -1012.155, -177.467, -2.058, -149.770)
        self.jointMove(-87.14, 89.965, -103.104, 3.492, 12.337, -69.112)                # Dock 2 approach

        #self.nodeBeaker.check_roomba_base2()

        #self.jointMove(-88.794, 97.111, -108.691, 6.410, 20.199, -64.552)              # Pickup (old)
        self.jointMove(-87.365, 95.745, -103.319, 3.542, 13.117, -69.11)                # Pickup
        
        withDice = True
        self.schunkMove('close')

        label = "Dice block flip"

        self.cartMove(302.556, -539.279, -83.733, -179.284, -2.058, -120.535)           # Avoid table
        self.cartMove(656.348, -144.366, -195.534, 127.834, -51.798, -31.558)           # Angled drop

        self.schunkMove('open')

        self.cartMove(656.348, -54.366, -195.534, 127.834, -51.798, -31.558)            # Move away
        self.cartMove(653.397, -161.278, -121.553, 179.158, -1.101, -61.980)            # Table approach
        self.cartMove(653.397, -161.278, -185.0, 179.158, -1.101, -61.980)              # Pickup dock 2

        label = "Moving dice block to conveyor 1"
        self.schunkMove('close')

        self.cartMove(791.903, 643.183, -53.923, 178.571, 0.950, 30.197)                # Conveyor 1 approach
        self.cartMove(791.903, 643.183, -183.923, 178.571, 0.950, 30.197)               # Conveyor 1 drop

        self.schunkMove('open')
        withDice = False

        self.cartMove(791.903, 643.183, -53.923, 178.571, 0.950, 30.197)                # Conveyor 1 approach

        label = "Moving conveyor 1"

        #self.nodeBeaker.set_beaker_conv_false()

        self.convMoveBlock()

        #self.nodeBeaker.set_beaker_conv_true()                                          # Ready at conveyor 1

        label = "Moving to home position"

        self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)               # Home

        '''request = Trigger.Request() # Make a request object
        while not self.gripService.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        result = self.gripService.call(request).message # Send request and block until given a response

        print(result)'''


    def taskBunsen(self):
        global label, action, fault, withDice

        self.cartMove() # Home

        # Wait for start signal

        label = "Picking up dice block at conveyor 1"

        self.cartMove() # Conveyor 1 approach end
        
        # Wait for ready at conveyor 1

        self.cartMove() # Conveyor 1 pickup end

        label = "Moving dice block from conveyor 1 to conveyor 2"
        withDice = True
        self.schunkMove('close')

        self.cartMove() # Conveyor 1 approach end
        self.cartMove() # Conveyor 2 approach start
        self.cartMove() # Conveyor 2 drop start

        withDice = False
        self.schunkMove('open')

        label = "Moving conveyor 2"

        # Set bunsen conv false
        self.convMoveBlock()
        # Set bunsen conv true

        label = "Picking up dice block at conveyor 2"

        self.cartMove() # Conveyor 2 approach start
        self.cartMove() # Conveyor 2 approach end

        # Wait for ready at conveyor 2

        self.cartMove() # Conveyor 2 pickup end

        withDice = True
        self.schunkMove('close')

        self.cartMove() # Conveyor 2 approach end

        label = "Dice block flip"

        self.cartMove(656.348, -144.366, -195.534, 127.834, -51.798, -31.558)           # Angled drop

        self.schunkMove('open')

        self.cartMove(656.348, -54.366, -195.534, 127.834, -51.798, -31.558)            # Move away
        self.cartMove(653.397, -161.278, -121.553, 179.158, -1.101, -61.980)            # Table approach
        self.cartMove(653.397, -161.278, -185.0, 179.158, -1.101, -61.980)              # Pickup

        label = "Moving dice block to base 3"
        self.schunkMove('close')

        self.cartMove() # Avoid table
        self.jointMove() # Dock 3 approach

        # Check create3 ready status

        self.jointMove() # Dock 3 drop

        withDice = False
        self.schunkMove('open')

        self.jointMove() # Dock 3 approach

        # Ready for create3 to leave dock 3

        self.cartMove() # Avoid table
        self.cartMove() # Home


    def convMoveBlock(self):
        global fault
        self.convMove('forward')
        timeStart = time.time()
        while beltSensorRear == False:
            if time.time()-timeStart > 10:
               fault = "No dice block detected on conveyor"
               self.reportSender()
               break
        self.convMove('stop')

    def test(self):
        self.convMove('forward')
        #self.schunkMove('open')
        #self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)
        sleep(2)
        self.convMove('stop')
        #self.schunkMove('close')

        #self.reportSender()
        #if beltSensorFront == True:
        #self.convMoveBlock()
        #self.nodeBeaker.check_roomba_base2()
        #self.cartMove(791.903, 643.183, -53.923, 178.571, 0.950, 30.197)



if __name__ == '__main__':
    #rclpy.init()
    # Create our 2 nodes
    mainBeaker = FanucActions('beaker')
    listenerBeaker = FanucTopic('beaker')
    mainBunsen = FanucActions('bunsen')
    listenerBunsen = FanucActions('bunsen')

    exec = MultiThreadedExecutor(6) # Give it 2 threads for 2 nodes
    # Add our nodes
    exec.add_node(mainBeaker)
    exec.add_node(listenerBeaker)
    exec.add_node(mainBunsen)
    exec.add_node(listenerBunsen)
    mqttc.loop_start()
    
    # This allows us to start the function once the node is spinning
    keycom = KeyCommander([
		(KeyCode(char='s'), mainBunsen.test),
		])
    keycom = KeyCommander([
		(KeyCode(char='r'), mainBeaker.test),
		])
    print("Ready")
    exec.spin() # Start executing the nodes
    rclpy.shutdown()
