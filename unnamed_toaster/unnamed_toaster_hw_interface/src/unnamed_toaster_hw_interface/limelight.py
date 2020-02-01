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

from math import radians, degrees
from tf import transformations

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, PoseStamped, PointStamped

from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker


class Limelight:

    def __init__(self):
        rospy.init_node("limelight") # Name this node Limelight
        self.pub = rospy.Publisher("limelight_marker", Marker, queue_size=1) # Create a publisher for our marker data

        # Subscriber data
        sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.laser_msg = None

    def update(self, table):
        tx = radians(table.getFloat('tx', 1)) # Convert the float converting the double into
        ty = radians(table.getFloat('ty', 1))
        ta = table.getFloat('ta', 1)

        self.arrow_marker = Marker()
        self.arrow_marker.header.frame_id = "limelight"
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

        try:
          laser_data = self.laser_msg.ranges
          self.arrow_marker.scale.x = laser_data[int(round(90 + degrees(tx)))]
          #print(int(round(90 + degrees(tx))))
          #print(laser_data[int(round(90 + tx))])
        except AttributeError:
          pass
    
    def laser_callback(self, msg):
        self.laser_msg = msg

    def publish(self):
        self.pub.publish(self.arrow_marker)
