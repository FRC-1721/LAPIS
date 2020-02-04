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

from visualization_msgs.msg import Marker


class Limelight:

    def __init__(self):
        self.pub = rospy.Publisher("limelight_marker", Marker, queue_size=1) # Create a publisher for our marker data
        self.closest_pt_pub = rospy.Publisher("closest_point", PointStamped, queue_size=1)

        self.frame_id = rospy.get_param("~frame_id", "limelight")

        # Setup TF2
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize things
        self.arrow_marker = None
        self.wall_point = None
        self.limelight_to_laser_transform = None

        # Lasercallback data
        sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.laser_msg = None

    def update(self, table):
        tx = radians(table.getFloat('tx', 1)) # Convert the float converting the double into
        ty = radians(table.getFloat('ty', 1))
        ta = table.getFloat('ta', 1)

        self.arrow_marker = Marker()
        self.arrow_marker.header.frame_id = self.frame_id
        self.arrow_marker.header.stamp = rospy.Time.now()
        self.arrow_marker.ns = "target_heading"
        self.arrow_marker.id = 0
        self.arrow_marker.type = Marker.ARROW
        self.arrow_marker.scale.y = 0.05
        self.arrow_marker.scale.z = 0.05
        self.arrow_marker.pose.position.x = 0
        self.arrow_marker.pose.position.y = 0
        self.arrow_marker.pose.position.z = 0
        self.arrow_marker.color.a = 1.0 # Don't forget the alpha!
        self.arrow_marker.color.g = 1.0 # Green!
        orientation = transformations.quaternion_from_euler(0, ty * -1, tx * -1)
        self.arrow_marker.pose.orientation.x = orientation[0]
        self.arrow_marker.pose.orientation.y = orientation[1]
        self.arrow_marker.pose.orientation.z = orientation[2]
        self.arrow_marker.pose.orientation.w = orientation[3]

        if self.limelight_to_laser_transform:

            # Cache the values so nothing changes
            msg = self.laser_msg
            transform = self.limelight_to_laser_transform

            # Transform a pose at the limelight
            heading = PoseStamped()
            heading.header.frame_id = self.frame_id
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

            # Transform points around heading
            # TODO: check bounds on start_idx/end_idx
            start_idx = int((yaw - radians(5) - msg.angle_min) / msg.angle_increment)
            end_idx = int((yaw + radians(5) - msg.angle_min) / msg.angle_increment)
            if start_idx < 0:
                rospy.logwarn("Start index invalid")
                return
            if end_idx >= len(msg.ranges):
                rospy.logwarn("End index invalid")
                return
            if start_idx >= end_idx:
                rospy.logwarn("Indices are invalid")
                return
            x = list()
            y = list()
            i = start_idx
            while i < end_idx:
                heading = msg.angle_min + (i * msg.angle_increment)
                distance = msg.ranges[i]
                i += 1
                if isinf(distance):
                    continue

                x.append(distance * cos(heading))
                y.append(distance * sin(heading))

            if len(x) < 3:
                rospy.logwarn("Not enough points")
                return

            # Quick visualization
            distance = msg.ranges[(start_idx + end_idx) / 2]
            self.arrow_marker.scale.x = distance

            # Find a line best fit to points
            # https://docs.scipy.org/doc/numpy/reference/generated/numpy.linalg.lstsq.html
            x = numpy.array(x, dtype=numpy.float64)
            y = numpy.array(y, dtype=numpy.float64)
            A = numpy.vstack([x, numpy.ones(len(x))]).T
            m1, c1 = numpy.linalg.lstsq(A, y)[0]

            # Convert heading a line equation (m2x + c2)
            m2 = tan(yaw)
            c2 = heading_transformed.pose.position.y - (heading_transformed.pose.position.x * m2)

            # Find intersection between line (m1x + c1) and ray eminating fom heading_transformed (m2x + c2)
            xi = (c2 - c1) / (m2 - m1)
            yi = m1 * xi + c1

            ps = PointStamped()
            ps.header = msg.header
            ps.point.x = xi
            ps.point.y = yi
            self.wall_point = ps

    def laser_callback(self, msg):
        self.laser_msg = msg # save the msg data
        self.get_transform() # Update the transform to the laser at the time the laser scan was collected

    def get_transform(self):
        try:
            # TODO if we start moving really fast, might need to use timestamp from message
            self.limelight_to_laser_transform = self.tf_buffer.lookup_transform(self.frame_id, "laser", rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerror("Error getting transform")

    def publish(self):
        if self.arrow_marker:
            self.pub.publish(self.arrow_marker)
        if self.wall_point:
            self.closest_pt_pub.publish(self.wall_point)
