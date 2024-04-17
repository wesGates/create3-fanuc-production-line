# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import sys
sys.path.append("../dependencies/")
# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

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
        cart_goal.x = 447.6
        cart_goal.y = 99.8
        cart_goal.z = 43.0
        cart_goal.w = 177.3
        cart_goal.p = 2.4
        cart_goal.r = -73.9
        # Send_goal is blocking
        self.cart_ac.send_goal(cart_goal)

        self.joints_ac.wait_for_server() # Wait till its ready
        joint_goal = JointPose.Goal() # Make Goal
        # Add all joints
        joint_goal.joint1 = -7.6
        joint_goal.joint2 = 35.4
        joint_goal.joint3 = -48.5
        joint_goal.joint4 = -169.0
        joint_goal.joint5 = -18.5
        joint_goal.joint6 = 114.2
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
