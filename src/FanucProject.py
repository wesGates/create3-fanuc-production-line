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
        #print(msg.pose)

    def jointListener(self, msg):
        global position
        position[1] = list(msg.joints)

    def convListener(self, msg):
        global beltSensorFront
        beltSensorFront= msg.left
        global beltSensorRear
        beltSensorRear= msg.right

        if beltSensorFront == True:
            FanucActions.convMoveBlock(FanucActions)

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
            "CRX10report":{
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
                "Fault": {
                }
            },
            "date": str(datetime.now())
        }
    
        mqttc.publish(topic, json.dumps(data))
        
        
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
        action = "gripperMove"
        self.reportSender()

    def taskBeaker(self):
        self.schunkMove('open')
        
        self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)               # Home
        
        # Wait for start signal

        label = "Picking up dice block at base 2"

        self.cartMove(302.556, -539.279, -83.733, -179.284, -2.058, -120.535)           # Avoid table
        #self.cartMove(-118.402, -561.427, -1012.155, -177.467, -2.058, -149.770)
        #self.cartMove(671.175, -141.571, -151.539, 142.619, -43.295, -41.702)           # Create3 approach
        self.jointMove(-88.794, 97.111, -108.691, 6.410, 20.199, -64.552)               # Pickup
        
        withDice = True
        self.schunkMove('close')

        label = "Dice block flip"

        self.cartMove(302.556, -539.279, -83.733, -179.284, -2.058, -120.535)           # Avoid table
        self.cartMove(671.175, -141.571, -181.539, 142.619, -43.295, -41.702)           # Angled drop

        self.schunkMove('open')

        self.cartMove(671.175, -141.571, -151.539, 142.619, -43.295, -41.702)           # Move away
        #self.cartMove(671.175, -141.571, -151.539, 142.619, -43.295, -41.702)           # Table approach
        #self.cartMove(671.175, -141.571, -151.539, 142.619, -43.295, -41.702)           # Pickup

        label = "Moving dice block to conveyor 1"

        #self.cartMove(671.175, -141.571, -151.539, 142.619, -43.295, -41.702)           # Conveyor approach
        #self.cartMove(671.175, -141.571, -151.539, 142.619, -43.295, -41.702)           # Conveyor drop

        self.convMoveBlock()

        label = "Moving to home position"

        #self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)               # Home

        '''request = Trigger.Request() # Make a request object
        while not self.gripService.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        result = self.gripService.call(request).message # Send request and block until given a response

        print(result)'''

    def convMoveBlock(self):
        label = "Moving conveyor 1"
        self.convMove('forward')
        time = datetime.now()
        while beltSensorRear == False:
            dtime = datetime.now()-time
            if dtime.total_seconds > datetime.timedelta(seconds=3):
               break
        self.convMove('stop')

    def test(self):
        #self.convMove('forward')
        #self.schunkMove('open')
        #self.cartMove(618.352, 1.623, -83.733, -179.284, -2.058, -59.679)
        #sleep(2)
        #self.convMove('stop')
        #self.schunkMove('close')

        #self.reportSender()
        #if beltSensorFront == True:
        self.convMoveBlock()
            
        



if __name__ == '__main__':
    rclpy.init()
    # Create our 2 nodes
    mainBeaker = FanucActions('beaker')
    listenerBeaker = FanucTopic('beaker')
    #mainBunsen = FanucActions('bunsen')
    #listenerBunsen = FanucActions('bunsen')

    exec = MultiThreadedExecutor(2) # Give it 2 threads for 2 nodes
    # Add our nodes
    exec.add_node(mainBeaker)
    exec.add_node(listenerBeaker)
    mqttc.loop_start()
    
    # This allows us to start the function once the node is spinning
    keycom = KeyCommander([
		(KeyCode(char='s'), mainBeaker.test),
		])
    print("S")
    exec.spin() # Start executing the nodes
    rclpy.shutdown()
