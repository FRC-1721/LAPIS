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

from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float64, String

from shooter_math import ShooterMath



class MatchLogic:

    def __init__(self):
        # Publisher to command turret angle
        self.turret_command_pub = rospy.Publisher("turret_command", Float64, queue_size=1)

        # Publisher to command robot base
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Get the robot mode from network tables
        self.robot_mode = "No Mode"
        self.mode_sub = rospy.Subscriber("robot_mode", String, self.robot_mode_cb)

        # Get our starting position
        self.init_pose = -1.0
        self.init_pose_sub = rospy.Subscriber("init_pose", Float64, self.init_pose_cb)

        # Setup TF2
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.shooter_math = ShooterMath(self.tf_buffer)

    #
    # ROS Callbacks
    #

    def robot_mode_cb(self, msg):
        self.robot_mode = msg.data

    def init_pose_cb(self, msg):
        self.init_pose = msg.data

    #
    # Match Logic
    #

    def run(self):
        # Switch
        modes = {
            "Teleop": self.Teleop,
            "Autonomous": self.Autonomous,
            "Disabled": self.Disabled,
            "Test": self.Test,
            "NoMode": self.NoMode
        }

        mode = modes.get(self.robot_mode, lambda: self.NoMode) # Get the mode
        mode() # Run the mode

    def Teleop(self):
        rospy.loginfo("RobotMode: Teleoperated")

    def Autonomous(self):
        rospy.loginfo("RobotMode: Autonomous")
        rate = rospy.Rate(25.0)

        # Wait until init pose is valid
        while self.init_pose < 0.0:
            rate.sleep()

        # Our starting position is our distance (in meters) from the
        # right side of the field. Field width is 8.21055 meters.
        # The high target is located 2.404364 meters from right side.
        field_y = self.init_pose - 2.404364

        # The starting line is 3.05m from the wall, but we are backwards
        field_x = -3.05

        # Set turret angle
        angle = self.shooter_math.get_turret_angle(field_x, field_y)
        rospy.loginfo("Turret command {}".format(angle))
        turret_command = self.shooter_math.get_turret_msg(angle)
        self.turret_command_pub.publish(turret_command)

        # Back up to cross line
        twist_msg = Twist()
        twist_msg.linear.x = 0.5

        rate = rospy.Rate(25.0)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():

            rospy.loginfo("Move")

            # TODO: add a monitor on laser distance to wall

            # Limit the amount of time we can drive
            if rospy.get_time() - start_time > 1.0:
                self.cmd_vel_pub.publish(Twist())
                break

            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

        # Track with shooter
        while self.robot_mode == "Autonomous":

            rospy.loginfo("SHOOTING AND WAITING UNTIL END OF AUTO")
            rospy.sleep(1)

            # TODO: Enable Shooter when aligned

            #self.turret_command_pub.publish(self.shooter_math.desired_turret_command)
            rate.sleep()

        # TODO: Track and intake balls on the field

    def Disabled(self):
        rospy.loginfo("RobotMode: Disabled")

    def Test(self):
        rospy.loginfo("RobotMode: Test")

    def NoMode(self):
        rospy.loginfo("RobotMode: NoMode, transmitted mode was: " + str(self.robot_mode))

if __name__ == "__main__":
    rospy.init_node("match_logic")
    match_logic = MatchLogic()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        match_logic.run()

        # Sleep
        rate.sleep()
