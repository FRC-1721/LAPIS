#!/usr/bin/env python

"""
    Copyright (c) 2019, Concord Robotics Inc 
    All rights reserved. 

    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met: 

     * Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer. 
     * Redistributions in binary form must reproduce the above copyright 
       notice, this list of conditions and the following disclaimer in the 
       documentation and/or other materials provided with the distribution. 
     * Neither the name of Concord Robotics Inc nor the names of its 
       contributors may be used to endorse or promote products derived from 
       this software without specific prior written permission. 

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    POSSIBILITY OF SUCH DAMAGE. 
"""

import rospy
import tf.transformations

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

from geometry_msgs.msg import Twist
from networktables import NetworkTables

rospy.init_node("robot_control")

ip = rospy.get_param("~ip", "10.17.21.2")
print ("Starting NetworkTables(Robot Control) using IP: ", ip)
NetworkTables.initialize(server = ip)
table = NetworkTables.getTable("ROS")

max_speed = rospy.get_param("~max_speed", 2) # The max speed of the robot in m/s
max_spin = rospy.get_param("~max_spin", 9) # The max turn speed of the robot in rad/s (rpm * 2)

def callback(msg):
    thro = (msg.linear.x / max_speed) # Scale the robot based off its max total speed
    steerage = (msg.angular.z / max_spin)
    print("Thro:" + str(thro) + ", Steerage:" + str(steerage) + "\r")
    if(abs(thro) > 1) or (abs(steerage) > 1):
        print("Overspeed!")

    table.putNumber("coprocessorPort", (thro + steerage) * -1) # Set port wheels
    table.putNumber("coprocessorStarboard", (thro - steerage)) # Set starboard wheels

def start_listener():
    rospy.init_node('cmd_vel_hungry_toaster')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    start_listener()
