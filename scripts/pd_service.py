#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Import your custom service by its generated Python module
from com760cw1_b01006947.srv import *

class PDService:
    def __init__(self):
        rospy.init_node('pd_service_node')
        self.poses = {}  # Store poses of all turtles
        self.publishers = {}  # Store publishers for all turtles
        
        rospy.loginfo("PD Service starting up...")
        
        # Create service
        self.service = rospy.Service('pd_service', b01006947ServicePD, self.handle_pd_request)
        rospy.loginfo("PD Service started - ready to control turtles")
        
        # Wait for turtles to be spawned before proceeding
        rospy.sleep(2)
        
        # Get list of all turtles by checking active topics
        topics = rospy.get_published_topics()
        turtle_names = set()
        
        for topic, msg_type in topics:
            if 'pose' in topic and msg_type == 'turtlesim/Pose':
                turtle_name = topic.split('/')[1]
                turtle_names.add(turtle_name)
        
        # Set up subscribers and publishers for each turtle
        for turtle_name in turtle_names:
            rospy.Subscriber(f'/{turtle_name}/pose', Pose, 
                            lambda msg, name=turtle_name: self.pose_callback(msg, name))
            self.publishers[turtle_name] = rospy.Publisher(
                f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
            rospy.loginfo(f"Set up control for turtle: {turtle_name}")
            
        # The additional subscriber logic for debug purposes
        rospy.Subscriber("/b01006947Leader/pose", Pose, self.debug_pose_callback)
    
    def debug_pose_callback(self, data):
        """Debug callback to log leader pose"""
        rospy.loginfo(f"Leader at x={data.x:.2f} y={data.y:.2f} theta={data.theta:.2f}")
    
    def pose_callback(self, pose_msg, turtle_name):
        """Store the current pose of each turtle"""
        self.poses[turtle_name] = pose_msg
    
    def handle_pd_request(self, req):
        """Handle the PD service request to move a turtle to a goal position"""
        turtle_name = req.turtle_name
        goal_x = req.goal_x
        goal_y = req.goal_y
        tolerance = req.tolerance
        
        if turtle_name not in self.poses:
            rospy.logerr(f"Turtle {turtle_name} not found!")
            return b01006947ServicePDResponse(success=False)
        
        if turtle_name not in self.publishers:
            rospy.logerr(f"No publisher for turtle {turtle_name}!")
            return b01006947ServicePDResponse(success=False)
        
        rospy.loginfo(f"Moving {turtle_name} to position ({goal_x}, {goal_y}) with tolerance {tolerance}")
        
        # Implement PD control loop
        rate = rospy.Rate(10)  # 10 Hz
        
        # PD control parameters
        kp_linear = 1.0  # Proportional gain for linear velocity
        kp_angular = 4.0  # Proportional gain for angular velocity
        
        # Main control loop
        max_iterations = 300  # Safety limit to prevent infinite loops
        iteration = 0
        
        while not rospy.is_shutdown() and iteration < max_iterations:
            # Get current pose
            current_pose = self.poses[turtle_name]
            
            # Calculate distance to goal
            distance = math.sqrt((goal_x - current_pose.x)**2 + 
                                (goal_y - current_pose.y)**2)
            
            # Check if we've reached the goal
            if distance < tolerance:
                rospy.loginfo(f"{turtle_name} reached the goal!")
                
                # Set orientation to match the leader's
                if 'Follower' in turtle_name and 'b01006947Leader' in self.poses:
                    leader_pose = self.poses['b01006947Leader']
                    target_angle = leader_pose.theta  # Match leader's orientation
                    
                    # Calculate error between current and target angle
                    angle_error = self.normalize_angle(target_angle - current_pose.theta)
                    
                    # Apply rotation to reach target angle
                    rospy.loginfo(f"Setting {turtle_name} orientation to match the leader: {target_angle:.2f} radians")
                    
                    while abs(angle_error) > 0.05 and not rospy.is_shutdown() and iteration < max_iterations:
                        cmd = Twist()
                        cmd.angular.z = kp_angular * angle_error
                        self.publishers[turtle_name].publish(cmd)
                        rate.sleep()
                        
                        # Update current pose and angle error
                        current_pose = self.poses[turtle_name]
                        angle_error = self.normalize_angle(target_angle - current_pose.theta)
                        iteration += 1
                
                # Stop the turtle
                stop_cmd = Twist()
                self.publishers[turtle_name].publish(stop_cmd)
                
                return b01006947ServicePDResponse(success=True)
            
            # Calculate the heading error
            target_angle = math.atan2(goal_y - current_pose.y, 
                                      goal_x - current_pose.x)
            angle_error = self.normalize_angle(target_angle - current_pose.theta)
            
            # Calculate PD control outputs
            linear_velocity = kp_linear * distance
            angular_velocity = kp_angular * angle_error
            
            # Create and publish velocity command
            cmd = Twist()
            cmd.linear.x = min(linear_velocity, 2.0)  # Limit max speed
            cmd.angular.z = angular_velocity
            
            self.publishers[turtle_name].publish(cmd)
            
            rate.sleep()
            iteration += 1
        
        # If we exited the loop due to max iterations, consider it a failure
        if iteration >= max_iterations:
            rospy.logwarn(f"Failed to reach the goal within {max_iterations} iterations!")
            
            # Stop the turtle
            stop_cmd = Twist()
            self.publishers[turtle_name].publish(stop_cmd)
            
            return b01006947ServicePDResponse(success=False)
    
    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        pd_service = PDService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass