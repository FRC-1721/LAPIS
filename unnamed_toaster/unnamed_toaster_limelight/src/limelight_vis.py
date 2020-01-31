#!/usr/bin/env python

"""
    Copyright (c) 2020 Concord Robotics Inc.
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
import logging
from geometry_msgs.msg import Quaternion
from math import radians
from tf import transformations
from visualization_msgs.msg import Marker

from networktables import NetworkTables
import logging	# Required
logging.basicConfig(level=logging.DEBUG)

if __name__ == "__main__":

    rospy.init_node("limelight")
    
    pub = rospy.Publisher("limelight_marker", Marker)

    # FRC mDNS almost never works - 10.17.21.55 is RIO
    ip = rospy.get_param("~ip", "10.17.21.55")
    logging.info("Starting NetworkTables using IP(Limelight): " + ip)
    NetworkTables.initialize(server = ip)
    robotTable = NetworkTables.getTable("limelight")
    
    # Setup Camera Streams
    
    
    rate = rospy.Rate(20)  # in Hz
    while not rospy.is_shutdown():  # runs for as long as the node is running
        tx = radians(float(robotTable.getNumber('tx', '1'))) # Convert the float converting the double into 
        ty = radians(float(robotTable.getNumber('ty', '1')))
        ta = float(robotTable.getNumber('ty', '1'))
        
        marker = Marker()
        #marker.header.frame_id = "limelight"
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_heading"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation = transformations.quaternion_from_euler(tx, ty, 0)
        
        pub.publish(marker)
