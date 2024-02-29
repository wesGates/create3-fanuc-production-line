# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import sys
sys.path.append("../dependencies/")
# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

from time import sleep

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, JointPose

namespace = 'bunsen'

class FanucSingle(Node):
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
		
    def demo(self):
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

        # Joints
        print("Running Joint test")
        self.joints_ac.wait_for_server() # Wait till its ready
        joint_goal = JointPose.Goal() # Make Goal
        # Add all joints
        joint_goal.joint1 = 90.0
        joint_goal.joint2 = 18.0
        joint_goal.joint3 = -41.0
        joint_goal.joint4 = -2.0
        joint_goal.joint5 = -48.0
        joint_goal.joint6 = -148.0
        # send_goal_async is nonblocking and allows us to get feedback when in a single thread
        future = self.joints_ac.send_goal_async(joint_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback) # This will run when the server accepts the goal 

#------- Helper functions -------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback) # This will run when the goal is done

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))

    def feedback_callback(self, feedback_msg): # This function will run when feedback is recieved from the server
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.distance_left))



if __name__ == '__main__':
    rclpy.init()
	
    fanuc = FanucSingle(namespace)
    # This allows us to start the function once the node is spinning
    keycom = KeyCommander([
		(KeyCode(char='s'), fanuc.demo),
		])
    print("S")
    rclpy.spin(fanuc) # Start executing the node
    rclpy.shutdown()
