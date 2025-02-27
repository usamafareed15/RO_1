#!/usr/bin/env python3

import rospy
import random
import subprocess
from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Empty

def setup_turtles():
    """
    Setup initial configuration for leader-follower formation control:
    1. Kill default turtle
    2. Set random background color
    3. Spawn leader turtle at center
    4. Spawn follower turtles at random positions
    """
    rospy.init_node('setup_turtles')
    rospy.loginfo("Setting up turtles for formation control...")
    
    # Wait for turtlesim to be fully initialized
    rospy.sleep(1)
    
    # Kill the default turtle
    try:
        rospy.wait_for_service('/kill', timeout=5)
        kill_turtle = rospy.ServiceProxy('/kill', Kill)
        kill_turtle("turtle1")
        rospy.loginfo("Killed default turtle (turtle1)")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Failed to kill default turtle: {e}")
    
    # Set random background color by changing ROS parameters
    try:
        # Generate random RGB values (avoiding very dark or very light colors)
        r = random.randint(30, 225)
        g = random.randint(30, 225)
        b = random.randint(30, 225)
        
        # Set the background color parameters
        rospy.set_param('/turtlesim/background_r', r)
        rospy.set_param('/turtlesim/background_g', g)
        rospy.set_param('/turtlesim/background_b', b)
        
        # Clear the background to apply the new color
        rospy.wait_for_service('/clear', timeout=5)
        clear_background = rospy.ServiceProxy('/clear', Empty)
        clear_background()
        
        rospy.loginfo(f"Set random background color (RGB: {r}, {g}, {b})")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Failed to set background color: {e}")
    
    # Spawn leader turtle at center (5.5, 5.5)
    leader_name = 'b01006947Leader'
    try:
        rospy.wait_for_service('/spawn', timeout=5)
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(5.5, 5.5, 0.0, leader_name)
        rospy.loginfo(f"Spawned leader turtle '{leader_name}' at center (5.5, 5.5)")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Failed to spawn leader turtle: {e}")
    
    # Set leader pen to off initially
    try:
        rospy.wait_for_service(f'/{leader_name}/set_pen', timeout=5)
        set_pen = rospy.ServiceProxy(f'/{leader_name}/set_pen', SetPen)
        set_pen(255, 0, 0, 2, 1)  # Red pen (off initially)
        rospy.loginfo(f"Set {leader_name} pen to red (initially off)")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Failed to set leader pen: {e}")
    
    # Spawn follower turtles at random positions
    follower_names = [
        'b01006947FollowerA',
        'b01006947FollowerB',
        'b01006947FollowerC'
    ]
    
    for follower_name in follower_names:
        try:
            # Generate random position (avoiding too close to borders)
            x = random.uniform(2.0, 9.0)
            y = random.uniform(2.0, 9.0)
            theta = random.uniform(0.0, 6.28)  # 0 to 2π
            
            spawn_turtle(x, y, theta, follower_name)
            rospy.loginfo(f"Spawned follower turtle '{follower_name}' at ({x:.2f}, {y:.2f})")
            
            # Set follower pen off initially
            rospy.wait_for_service(f'/{follower_name}/set_pen', timeout=5)
            set_pen = rospy.ServiceProxy(f'/{follower_name}/set_pen', SetPen)
            set_pen(255, 255, 255, 2, 1)  # White pen (off initially)
            rospy.loginfo(f"Set {follower_name} pen to white (initially off)")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"Failed to setup follower {follower_name}: {e}")
    
    rospy.loginfo("Turtle setup completed successfully!")

if __name__ == '__main__':
    try:
        setup_turtles()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupted Exception")