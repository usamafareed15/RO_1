#!/usr/bin/env python3

import rospy
import math
from turtlesim.msg import Pose
from com760cw1_b01006947.msg import *
from com760cw1_b01006947.srv import *

class FollowerA:
    def __init__(self):
        rospy.init_node('follower_a_controller')
        
        self.turtle_name = 'b01006947FollowerA'
        self.pose = None
        
        # Subscribe to own pose
        rospy.Subscriber(f'/{self.turtle_name}/pose', Pose, self.pose_callback)
        
        # Subscribe to leader instructions
        rospy.Subscriber('/leader_instructions', b01006947LeaderInstruction, self.instruction_callback)
        
        rospy.loginfo(f"{self.turtle_name} controller initialized")
    
    def pose_callback(self, pose_msg):
        """Store the current pose"""
        self.pose = pose_msg
    
    def instruction_callback(self, instruction_msg):
        """Process instructions from the leader"""
        if instruction_msg.instructionID == "formation":
            # Extract target position for this follower
            target_x = instruction_msg.followerA_x
            target_y = instruction_msg.followerA_y
            
            rospy.loginfo(f"Received formation instruction: Move to ({target_x:.2f}, {target_y:.2f})")
            
            # Call PD service to move to target position
            self.move_to_position(target_x, target_y)
    
    def move_to_position(self, x, y):
        """Move to specified position using PD service"""
        try:
            rospy.wait_for_service('pd_service', timeout=5)
            pd_service = rospy.ServiceProxy('pd_service', b01006947ServicePD)
            
            rospy.loginfo(f"Moving to position ({x:.2f}, {y:.2f})")
            response = pd_service(self.turtle_name, x, y, 0.1)
            
            if response.success:
                rospy.loginfo("Successfully reached position")
            else:
                rospy.logwarn("Failed to reach position")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        follower = FollowerA()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass