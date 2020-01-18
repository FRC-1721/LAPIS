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

def clamp(speed, minspeed, maxspeed):
    output_speed = max(min(maxspeed, speed), minspeed)
    if speed != output_speed:
        logging.warning("Overspeed!")
    return output_speed

class robot_control:
    def callback(self, msg):
        thro = msg.linear.x # Scale the robot based off its max total speed
        steerage = msg.angular.z # Rads/s

        # This equation by ka2qfx
        steerage = steerage / (0.5 * self.wheel_base / 39.34) # The new value of steerage(ms) is equal to the old value of (steerage(rads/s) / (0.5 * wheelbase / 39.34)

        thro = clamp(thro, self.max_speed * -1, self.max_speed)
        steerage = clamp(steerage, self.max_spin * -1, self.max_spin)

        self.table.putNumber("coprocessorPort", thro + steerage) # Set port wheels in m/s
        self.table.putNumber("coprocessorStarboard", thro - steerage) # Set starboard wheels in m/s

        logging.debug("Sent commands " + str(thro + steerage) + ", " + str(thro - steerage) + ", (Port) (Starboard)")

    def start_listener(self):
        rospy.init_node('cmd_vel_hungry_toaster')
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.max_speed = rospy.get_param("~max_speed", 1) # The max speed of the robot in m/s
        self.max_spin = rospy.get_param("~max_spin", 0.5) # The max turn speed of the robot in rad/s (rpm * 2)
        self.wheel_base = rospy.get_param("~wheel_base", 1) # The robot's wheelbase in meters
        
        self.ip = rospy.get_param("~ip", "10.17.21.2")
        logging.info("Starting NetworkTables(Robot Control) using IP: " + self.ip)
        NetworkTables.initialize(server = self.ip)
        NetworkTables.setServer([(ip, 5800)])
        self.table = NetworkTables.getTable("ROS")

        
if __name__ == '__main__':
    r = robot_control()
    r.start_listener()
    
    rospy.spin()