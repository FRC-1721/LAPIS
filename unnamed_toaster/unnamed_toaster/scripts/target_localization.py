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

import numpy
import rospy
import tf2_ros
import tf2_geometry_msgs

from math import radians, degrees, tan, cos, sin, isinf
from tf import transformations

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, PoseStamped, PointStamped

from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker, MarkerArray


class TargetLocalization:

    def __init__(self):
        self.arrow_pub = rospy.Publisher("limelight_ray", Marker, queue_size=1)
        self.wall_points_pub = rospy.Publisher("wall_points", MarkerArray, queue_size=1)

        # Setup TF2
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Laser callback
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.laser_msg = None

        # Limelight callback
        self.limelight_sub = rospy.Subscriber("/limelight", PointStamped, self.limelight_callback)


    def limelight_callback(self, msg):
        # We need laser data
        if self.laser_msg:
            laser_msg = self.laser_msg
        else:
            rospy.logerr("Unable to update projection, no laser data")
            return

        # TODO: should we check that laser scan is recent enough?

        tx = msg.point.x
        ty = msg.point.y
        ta = msg.point.z
        orientation = transformations.quaternion_from_euler(0, ty * -1, tx * -1)

        # Publish debug info showing what limelight sees
        arrow_marker = Marker()
        arrow_marker.header.frame_id = msg.header.frame_id
        arrow_marker.header.stamp = msg.header.stamp
        arrow_marker.ns = "target_heading"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.pose.position.x = 0
        arrow_marker.pose.position.y = 0
        arrow_marker.pose.position.z = 0
        arrow_marker.color.a = 1.0 # Don't forget the alpha!
        arrow_marker.color.g = 1.0 # Green!
        arrow_marker.pose.orientation.x = orientation[0]
        arrow_marker.pose.orientation.y = orientation[1]
        arrow_marker.pose.orientation.z = orientation[2]
        arrow_marker.pose.orientation.w = orientation[3]

        # Get transformation between laser and limelight
        try:
            # TODO if we start moving really fast, might need to use timestamp from message
            transform = self.tf_buffer.lookup_transform(msg.header.frame_id, laser_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Error getting transform")
            return

        # Transform a pose at the limelight
        heading = PoseStamped()
        heading.header.frame_id = msg.header.frame_id
        heading.header.stamp = msg.header.stamp
        heading.pose.position.x = 0.0
        heading.pose.position.y = 0.0
        heading.pose.position.z = 0.0
        heading.pose.orientation.x = orientation[0]
        heading.pose.orientation.y = orientation[1]
        heading.pose.orientation.z = orientation[2]
        heading.pose.orientation.w = orientation[3]

        heading_transformed = tf2_geometry_msgs.do_transform_pose(heading, transform)

        # Get heading angle in laser frame
        quaternion = [heading_transformed.pose.orientation.x, \
                      heading_transformed.pose.orientation.y, \
                      heading_transformed.pose.orientation.z, \
                      heading_transformed.pose.orientation.w]
        _, _, yaw = transformations.euler_from_quaternion(quaternion)
        rospy.loginfo("yaw: %f", yaw)

        # Transform points around heading
        start_idx = int((yaw - radians(10) - laser_msg.angle_min) / laser_msg.angle_increment)
        end_idx = int((yaw + radians(10) - laser_msg.angle_min) / laser_msg.angle_increment)
        rospy.loginfo("%d %d", start_idx, end_idx)
        if start_idx < 0:
            rospy.logwarn("Start index invalid %d", start_idx)
            return
        if end_idx >= len(laser_msg.ranges):
            rospy.logwarn("End index invalid %d", end_idx)
            return
        if start_idx >= end_idx:
            rospy.logwarn("Indices are invalid")
            return
        x = list()
        y = list()
        i = start_idx
        avg_distance = 0
        while i < end_idx:
            heading = laser_msg.angle_min + (i * laser_msg.angle_increment)
            distance = laser_msg.ranges[i]
            i += 1
            if isinf(distance):
                continue

            x.append(distance * cos(heading))
            y.append(distance * sin(heading))
            avg_distance += distance

        if len(x) < 3:
            rospy.logwarn("Not enough points to find best fit line (%d)", len(x))
            return

        arrow_marker.scale.x = avg_distance / len(x)
        self.arrow_pub.publish(arrow_marker)

        # Publish the points
        wall_points = MarkerArray()
        for i in range(len(x)):
            marker = Marker()
            marker.header = laser_msg.header
            marker.ns = "scan_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.pose.position.x = x[i]
            marker.pose.position.y = y[i]
            marker.pose.orientation.w = 1
            marker.color.a = 1.0 # Don't forget the alpha!
            marker.color.r = 1.0 # Red!
            wall_points.markers.append(marker)

        # Find a line best fit to points
        # https://docs.scipy.org/doc/numpy/reference/generated/numpy.linalg.lstsq.html
        x = numpy.array(x, dtype=numpy.float64)
        y = numpy.array(y, dtype=numpy.float64)
        A = numpy.vstack([x, numpy.ones(len(x))]).T
        m1, c1 = numpy.linalg.lstsq(A, y)[0]
        rospy.loginfo("m1: %f, c1: %f", m1, c1)

        # Convert heading a lin equation (m2x + c2)
        m2 = tan(yaw)
        c2 = heading_transformed.pose.position.y - (heading_transformed.pose.position.x * m2)
        rospy.loginfo("m2: %f, c2: %f", m2, c2)

        # Find intersection between line (m1x + c1) and ray eminating fom heading_transformed (m2x + c2)
        xi = (c2 - c1) / (m2 - m1)
        yi = m1 * xi + c1

        marker = Marker()
        marker.header = msg.header
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.pose.position.x = xi
        marker.pose.position.y = yi
        marker.pose.orientation.w = 1
        marker.color.a = 1.0 # Don't forget the alpha!
        marker.color.b = 1.0 # Blue!
        wall_points.markers.append(marker)

        self.wall_points_pub.publish(wall_points)


    def laser_callback(self, msg):
        self.laser_msg = msg # save the msg data


if __name__ == "__main__":
    rospy.init_node("target_localization")
    localization = TargetLocalization()
    rospy.spin()
