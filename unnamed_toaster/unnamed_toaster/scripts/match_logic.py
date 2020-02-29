#!/usr/bin/env python

"""
    Copyright (c) 2019-2020 Concord Robotics Inc.
    All right reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
    OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
    LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
    OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
    ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String

from unnamed_toaster.unnamed_toaster.scripts.shooter_math import ShooterMath


class MatchLogic:

    def __init__(self):
        # Publishers and Subscribers
        self.turret_command_pub = rospy.Publisher("turret_command", Float64, queu_size=1)
        self.target_sub = rospy.Subscriber("target_point", PointStamped, ShooterMath.calculate_azimuth) # Setup to calculate the target
        self.mode_sub = rospy.Subscriber("robot_mode", String, MatchLogic.update_mode) # Setup to run mode logic on mode update

        # Setup TF2
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Other
        self.robot_mode = "No Mode"
    
    def update_mode(self, msg): # Update the mode.
        self.robot_mode = msg.data

    def run(self):
        # Mode specific Logic
        if self.robot_mode == "Teleop":
            #self.table.putString("ROSStatus", "ROS, Teleop has started.")
            self.Teleop()

        elif self.robot_mode == "Autonomous":
            #self.table.putString("ROSStatus", "ROS is in autonomous mode.")
            self.Autonomous()

        elif self.robot_mode == "Disabled":
            #self.table.putString("ROSStatus", "ROS disabled, robot in disabled mode")
            self.Disabled()

        elif self.robot_mode == "Test":
            #self.table.putString("ROSStatus", "ROS Operating in test mode.")
            self.Test()
            
        elif self.robot_mode == "NoMode":
            #self.table.putString("ROSStatus", "ROS waiting for match to start or reboot.")
            pass
        else:
            #self.table.putString("ROSStatus", "ROS, No mode or robot is not ready")
            rospy.logerr("No mode or robot is not ready")

    def Teleop(self):
        pass
    
    def Autonomous(self):
        # Do at same time
        # - Backup to cross the line
        # - Move turret to close enough
        # Enable Shooter
        # Fire when ready
        pass
    
    def Disabled(self):
        pass

    def Test(self):
        pass

if __name__ == "__main__":
    rospy.init_node("match_logic")
    match_logic = MatchLogic()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        match_logic.run()

        # Sleep
        rate.sleep()
