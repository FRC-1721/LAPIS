#!/usr/bin/env python

import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

ip = "roboRIO-1721-FRC"
#ip = "localhost" # For debugging
print ("Starting NetworkTables using IP: ", ip)
NetworkTables.initialize(server = ip)
table = NetworkTables.getTable("ROS")

if __name__ == "__main__":
    rospy.init_node("robot_control")
    
    turn_p = 0.1 # values for turning
    turn_i = 0.1
    turn_d = 0.1 
    
    drive_p = 0.1  # values for driving
    drive_i = 0.1 
    drive_d = 0.1 
    
    deadzone_angle = 90 # angle for deadzone
    def deadzone_falloff(error_angle): # falloff of deadzone (as the robot nears its target heading its speed will be multiplied
        error_angle = error_angle + 90
        #TODO map error angle between deadzone_angle
        output = sin(error_angle)
        if output >= 0: # Thro cannot be negative here
            return output
        else:
            return 0
    
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():  # Runs as long as node is running
        current_target_heading = 32 #TODO Get heading!
        current_target_distance = 9 #TODO Get distance!
        
        current_robot_world_heading = 30 #TODO Get current heading of robot relative to the world
        current_target_world_heading = 90 #TODO Get current heading of target (The ending direction the robot should face)