#!/usr/bin/env python

"""
  Copyright (c) 2019 Concord Robotics Inc.
  Copyright (c) 2010-2011 Vanadium Labs LLC.
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
import sys

from math import sin, cos

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from networktables import NetworkTables
import logging	# Required
logging.basicConfig(level=logging.DEBUG)


## Based off vanadiumlabs/arbotix_ros::DiffController
class Odom:

    def __init__(self):
        # parameters: rates and geometry
        self.ticks_meter = float(rospy.get_param("~ticks_meter", 50000))  # roughly calibrated on 8/25/2019
        self.base_width = float(rospy.get_param("~base_width", 0.75))

        self.base_frame_id = rospy.get_param("~base_frame_id", "laser")  # was "base_link"
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")

        # internal data
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                     # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()    # time for determining dx/dy

        # subscriptions
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()

    def update(self, left, right):
        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (left - self.enc_left)/self.ticks_meter
            d_right = (right - self.enc_right)/self.ticks_meter
        self.enc_left = left
        self.enc_right = right

        d = (d_left+d_right)/2
        th = (d_right-d_left)/self.base_width
        self.dx = d / elapsed
        self.dr = th / elapsed

        if (d != 0):
            x = cos(th)*d
            y = -sin(th)*d
            self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
            self.y = self.y + (sin(self.th)*x + cos(self.th)*y)
        if (th != 0):
            self.th = self.th + th

        # publish or perish
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
            )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)

if __name__ == "__main__":

    rospy.init_node("odom")

    #ip = rospy.get_param("~ip", "roboRIO-1721-FRC")
    #ip = rospy.get_param("~ip", "localhost") # For testing
    #ip = rospy.get_param("~ip", "10.17.21.55") # First mDNS almost never works on the feild
    if len(sys.argv) > 1: # If there is more than one argument
        ip = sys.argv[1] # Set the ip to the extra argument (remember to count from zero!)
    else: # if no extra argument
        ip = rospy.get_param("~ip", "10.17.21.55") # use the RIO ip by default
    print "Starting NetworkTables using IP: ", ip
    NetworkTables.initialize(server = ip)
    #robotTable = NetworkTables.getTable('ROS')
    robotTable = NetworkTables.getTable('SmartDashboard')

    odom = Odom()

    rate = rospy.Rate(10)  # in Hz
    while not rospy.is_shutdown():  # runs for as long as the node is running
        # Get the encoder counts - have to invert left
        left = -float(robotTable.getNumber('Port','0'))
        right = float(robotTable.getNumber('Starboard','0'))

        # Update and publish odometry
        odom.update(left, right)

        rate.sleep()