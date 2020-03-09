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

from math import radians
from tf import transformations

from geometry_msgs.msg import PointStamped



class Limelight:

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "limelight")
        self.pub = rospy.Publisher("limelight", PointStamped, queue_size=1)
        self.ps = None


    def update(self, table):
        # Convert the float converting the double into
        tx = radians(table.getFloat('tx', 1))
        ty = radians(table.getFloat('ty', 1))
        ta = table.getFloat('ta', 1)

        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.frame_id
        ps.point.x = tx
        ps.point.y = ty
        ps.point.z = ta
        self.ps = ps


    def publish(self):
        if self.ps:
            self.pub.publish(self.ps)
