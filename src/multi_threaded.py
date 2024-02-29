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
from fanuc_interfaces.action import CartPose, SchunkGripper
from fanuc_interfaces.msg import CurGripper

namespace = 'bunsen'

# Node that listens to the fanuc topic
class FanucTopic(Node):
    def __init__(self, namespace):
        super().__init__("robotTopics")

        # Create a Service node for sending data to 'Main', could be a Topic also
        self.srv = self.create_service(Trigger, 'check_gripper', self.service_callback)
        
        # This subscribes to the Fanuc /grip_status topic
        self.subscriber_ = self.create_subscription(CurGripper, f'/{namespace}/grip_status', 
            self.listener, qos_profile_sensor_data)
        # This will hold the current value from the Fanuc topic
        self.curValue = False # Temporary default value
        
    def listener(self, msg):
        """
        This will be called everytime the subscriber recieves a new message from the topic
        """
        self.curValue = msg.open
    	
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
        
        # Sercice Client
        self.service = self.create_client(Trigger,'/check_gripper')

    def demo(self):
        print("Moving Robot!")
        self.cart_ac.wait_for_server() # Wait till its ready
        cart_goal = CartPose.Goal() # Make Goal
        # Add all coordinates 
        cart_goal.x = 110.77
        cart_goal.y = 672.0
        cart_goal.z = -102.75
        cart_goal.w = 170.0
        cart_goal.p = 0.0
        cart_goal.r = 30.0
        # Send_goal is blocking
        self.cart_ac.send_goal(cart_goal)

        print("Reading current gripper...")
        request = Trigger.Request() # Make a request object
        while not self.service.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        result = bool(self.service.call(request).message) # Send request and block until given a response

        print("Doing opposite of gripper state of", result)
        if(result == False): # If the grippers closed
            self.schunk_ac.wait_for_server()
            schunk_goal = SchunkGripper.Goal()
            schunk_goal.command = 'open'
            self.schunk_ac.send_goal(schunk_goal)
        elif(result == True):
            self.schunk_ac.wait_for_server()
            schunk_goal = SchunkGripper.Goal()
            schunk_goal.command = 'close'
            self.schunk_ac.send_goal(schunk_goal)



if __name__ == '__main__':
    rclpy.init()
    # Create our 2 nodes
    main = FanucActions(namespace)
    listener = FanucTopic(namespace)

    exec = MultiThreadedExecutor(2) # Give it 2 threads for 2 nodes
    # Add our nodes
    exec.add_node(main)
    exec.add_node(listener)
    
    # This allows us to start the function once the node is spinning
    keycom = KeyCommander([
		(KeyCode(char='s'), main.demo),
		])
    print("S")
    exec.spin() # Start executing the nodes
    rclpy.shutdown()
