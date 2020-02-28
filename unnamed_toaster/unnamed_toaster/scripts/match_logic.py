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
from std_msgs.msg import Float64

from unnamed_toaster.unnamed_toaster.scripts.shooter_math import ShooterMath


class MatchLogic:

    def __init__(self, table):
        # Publishers and Subscribers
        self.turret_command_pub = rospy.Publisher("turret_command", Float64, queu_size=1)
        self.target_sub = rospy.Subscriber("target_point", PointStamped, ShooterMath.calculate_azimuth)

        self.rate = rospy.get_param("~rate", 50) # Get the paramater for the rate

        # Setup TF2
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # NT
        self.table = table

        # Init as node
        rospy.init_node("match_logic")
    
    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Find the robot mode
            robot_mode = self.table.getString("RobotMode", "Waiting for Rio")

            # Mode specific Logic
            if robot_mode == "Teleop":
                self.table.putString("ROSStatus", "ROS, Teleop has started.")
                self.Teleop()

            elif robot_mode == "Autonomous":
                self.table.putString("ROSStatus", "ROS is in autonomous mode.")
                self.Autonomous()

            elif robot_mode == "Disabled":
                self.table.putString("ROSStatus", "ROS disabled, robot in disabled mode")
                self.Disabled()

            elif robot_mode == "Test":
                self.table.putString("ROSStatus", "ROS Operating in test mode.")
                self.Test()
                
            elif robot_mode == "NoMode":
                self.table.putString("ROSStatus", "ROS waiting for match to start or reboot.")
                pass
            else:
                self.table.putString("ROSStatus", "ROS, No mode or robot is not ready")
                rospy.logerr("No mode or robot is not ready")

            rate.sleep()

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
