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
from geometry_msgs.msg import Twist

def clamp(speed, minspeed, maxspeed):
    output_speed = max(min(maxspeed, speed), minspeed)
    return output_speed

class RobotControl:

    def __init__(self, table):
        self.wheel_base = rospy.get_param("~wheel_base", 1.0) # The robot's wheelbase in meters
        self.max_speed = rospy.get_param("~max_speed", 1.5) # The max speed of the robot in m/s
        self.max_spin = rospy.get_param("~max_spin", 1.5) * self.wheel_base / 2.0 # The max turn speed of the robot in rad/s

        self.table = table  # This is an instance of NetworkTableInterface

        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)

    def callback(self, msg):
        thro = msg.linear.x
        steerage = msg.angular.z * self.wheel_base / 2.0

        thro = clamp(thro, self.max_speed * -1, self.max_speed)
        steerage = clamp(steerage, self.max_spin * -1, self.max_spin)

        self.table.putNumber("coprocessorPort", thro + steerage) # Set port wheels in m/s
        self.table.putNumber("coprocessorStarboard", thro - steerage) # Set starboard wheels in m/s
