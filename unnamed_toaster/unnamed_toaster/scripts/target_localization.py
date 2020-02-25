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
        self.target_pub = rospy.Publisher("target_point", PointStamped, queue_size=1)
        self.arrow_pub = rospy.Publisher("limelight_ray", Marker, queue_size=1)
        self.points_pub = rospy.Publisher("wall_points", MarkerArray, queue_size=1)

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
        start_time = rospy.get_time()

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
            transform = self.tf_buffer.lookup_transform(msg.header.frame_id, laser_msg.header.frame_id, laser_msg.header.stamp, rospy.Duration(1.0))
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

        # Generate a set of scan points around the heading
        start_idx = int((yaw - radians(20) - laser_msg.angle_min) / laser_msg.angle_increment)
        end_idx = int((yaw + radians(20) - laser_msg.angle_min) / laser_msg.angle_increment)
        if start_idx < 0:
            rospy.logwarn("Start index invalid %d", start_idx)
            return
        if end_idx >= len(laser_msg.ranges):
            rospy.logwarn("End index invalid %d", end_idx)
            return
        if start_idx >= end_idx:
            rospy.logwarn("Indices are invalid")
            return
        scan_x = list()
        scan_y = list()
        i = start_idx
        avg_distance = 0
        while i < end_idx:
            heading = laser_msg.angle_min + (i * laser_msg.angle_increment)
            distance = laser_msg.ranges[i]
            i += 1
            if isinf(distance):
                continue

            avg_distance += distance
            scan_x.append(distance * cos(heading))
            scan_y.append(distance * sin(heading))

        if len(scan_x) < 3:
            rospy.logwarn("Not enough points along ray")
            return

        avg_distance = avg_distance / len(scan_x)
        arrow_marker.scale.x = avg_distance
        self.arrow_pub.publish(arrow_marker)

        # Generate a series of points along ray from limelight
        limelight_x = list()
        limelight_y = list()
        d = -1.0
        while d <= 1.0:
            distance = avg_distance + d
            limelight_x.append(heading_transformed.pose.position.x + (distance * cos(yaw)))
            limelight_y.append(heading_transformed.pose.position.y + (distance * sin(yaw)))
            d += 0.1

        # Determine which points are closest
        dists = list()
        for i in range(len(scan_x)):
            min_dist = 1000.0
            for j in range(len(limelight_x)):
                dist = (limelight_x[j] - scan_x[i])**2 + (limelight_y[j] - scan_y[i])**2
                if dist < min_dist:
                    min_dist = dist
            dists.append(min_dist)
        min_dist = min(dists)
        x = list()
        y = list()
        for i in range(len(scan_x)):
            if dists[i] < min_dist + 0.05:
                x.append(scan_x[i])
                y.append(scan_y[i])

        if len(x) < 3:
            rospy.logwarn("Not enough close points")

        # Publish the target
        target = PointStamped()
        target.header = laser_msg.header
        target.point.x = sum(x) / len(x)
        target.point.y = sum(y) / len(y)
        target.point.z = 0.0  # TODO, fill this in
        self.target_pub.publish(target)

        # Publish debug information
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

        for i in range(len(limelight_x)):
            marker = Marker()
            marker.header = laser_msg.header
            marker.ns = "limelight_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.pose.position.x = limelight_x[i]
            marker.pose.position.y = limelight_y[i]
            marker.pose.orientation.w = 1
            marker.color.a = 1.0 # Don't forget the alpha!
            marker.color.g = 1.0 # Green!
            wall_points.markers.append(marker)

        marker = Marker()
        marker.header = laser_msg.header
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.pose.position.x = target.point.x
        marker.pose.position.y = target.point.y
        marker.pose.position.z = target.point.z
        marker.pose.orientation.w = 1
        marker.color.a = 1.0 # Don't forget the alpha!
        marker.color.b = 1.0 # Blue!
        wall_points.markers.append(marker)

        self.points_pub.publish(wall_points)

        rospy.loginfo("Found target! (%fs)", rospy.get_time() - start_time)


    def laser_callback(self, msg):
        self.laser_msg = msg # save the msg data


if __name__ == "__main__":
    rospy.init_node("target_localization")
    localization = TargetLocalization()
    rospy.spin()
