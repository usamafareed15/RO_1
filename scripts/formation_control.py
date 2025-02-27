#!/usr/bin/env python3

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from com760cw1_b01006947.msg import *

class FormationControl:
    def __init__(self):
        rospy.init_node('formation_control')
        
        # Initialize pose for leader or wait for it
        self.leader_pose = None
        self.leader_name = 'b01006947Leader'
        
        # Subscribe to leader pose topic
        rospy.Subscriber(f'/{self.leader_name}/pose', Pose, self.pose_callback)
        
        # Create publisher for leader instructions
        self.instruction_pub = rospy.Publisher(
            '/leader_instructions', b01006947LeaderInstruction, queue_size=10)
        
        rospy.loginfo("Formation control initialized. Waiting for pose data...")
        
        # Wait until we have leader pose data
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown() and self.leader_pose is None:
            rate.sleep()
        
        rospy.loginfo("Received leader pose data")
        
        # Start with formation command
        self.send_formation_command()
    
    def pose_callback(self, pose_msg):
        """Store the current pose for the leader"""
        self.leader_pose = pose_msg
    
    def send_formation_command(self):
        """Send the formation command to position followers around the leader"""
        if self.leader_pose is None:
            rospy.logerr("Cannot send formation command: Leader pose unknown")
            return
        
        # Get leader pose
        leader_x = self.leader_pose.x
        leader_y = self.leader_pose.y
        leader_theta = self.leader_pose.theta
        
        # Calculate desired follower positions (1 meter from leader)
        # FollowerA: 1 meter to the left of leader
        followerA_x = leader_x - 1.0 * math.sin(leader_theta)
        followerA_y = leader_y + 1.0 * math.cos(leader_theta)
        
        # FollowerB: 1 meter behind the leader
        followerB_x = leader_x - 1.0 * math.cos(leader_theta)
        followerB_y = leader_y - 1.0 * math.sin(leader_theta)
        
        # FollowerC: 1 meter to the right of leader
        followerC_x = leader_x + 1.0 * math.sin(leader_theta)
        followerC_y = leader_y - 1.0 * math.cos(leader_theta)
        
        # Create and publish instruction message
        instruction = b01006947LeaderInstruction()
        instruction.instructionID = "formation"
        instruction.followerA_x = followerA_x
        instruction.followerA_y = followerA_y
        instruction.followerB_x = followerB_x
        instruction.followerB_y = followerB_y
        instruction.followerC_x = followerC_x
        instruction.followerC_y = followerC_y
        
        self.instruction_pub.publish(instruction)
        rospy.loginfo(f"Sent formation command: A({followerA_x:.2f}, {followerA_y:.2f}), " +
                     f"B({followerB_x:.2f}, {followerB_y:.2f}), " +
                     f"C({followerC_x:.2f}, {followerC_y:.2f})")

if __name__ == '__main__':
    try:
        formation_control = FormationControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass