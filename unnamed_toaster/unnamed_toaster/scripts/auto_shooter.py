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

from math import atan2

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


class AutoShooter:

    def __init__(self):
        self.pub = rospy.Publisher("turret_command", Float64, queue_size=1)

        # Setup TF2
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub = rospy.Subscriber("target_point", PointStamped, self.callback)

    def callback(self, msg):

        # Get transformation between target point frame (laser?) and base_link
        try:
            # TODO if we start moving really fast, might need to use timestamp from message
            transform = self.tf_buffer.lookup_transform(msg.header.frame_id, "base_link", msg.header.stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Error getting transform")
            return

        # Transform point to base_link
        target_point = tf2_geometry_msgs.do_transform_point(msg, transform)

        # Find yaw angle for turret
        angle = atan2(target_point.point.y, target_point.point.x)

        # Command stuff
        command = Float64()
        command.data = angle
        self.pub.publish(command)


if __name__ == "__main__":
	rospy.init_node("auto_shooter")
	auto = AutoShooter()
	rospy.spin()

