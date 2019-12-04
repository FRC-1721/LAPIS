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
    
    turn_p = 0.01 # values for turning
    turn_i = 0.01
    turn_d = 0.01 
    
    drive_p = 0.01  # values for driving
    drive_i = 0.01 
    drive_d = 0.01 
    
    deadzone_angle = 90 # angle for deadzone
    
    def pid_drive(distance_error, instant_heading_error): # Instant heading is the desired heading at that moment along the spline
        current_drive_i = drive_i + current_drive_i
        current_turn_i = turn_i + current_turn_i
        
        thro = (distance_error * (dirve_p / (drive_d * distance_error))) + current_drive_i
        thro = thro * deadzone_falloff(instant_heading_error) # if not facing the correct way dont drive forward
        #TODO Make robot drive backwards by using following algos in reverse
        
        steerage = (instant_heading_error * (turn_p / (turn_d * instant_heading_error))) + current_turn_i
        
        if thro > 1:
            thro = 1
            print ("Max Power!")
        
        table.putNumber("coprocessorPort", thro - steerage)
        table.putNumber("coprocessorStarboard", (thro + steerage) * -1)

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
        
        rate.sleep()
