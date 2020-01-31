#!/usr/bin/env python

"""
    Copyright (c) 2020 Concord Robotics Inc.
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
from std_msgs.msg import Int32

def nt_publisher(): # TODO these need to be updated with real NT table values in the future!
    # Create publisher topics
    starboard_encoder = rospy.Publisher('starboard_encoder', Int32, queue_size=10)
    port_encoder = rospy.Publisher('port_encoder', Int32, queue_size=10)
    # Init node
    rospy.init_node('nt_interface', anonymous=True)
    rate = rospy.Rate(10) # In hz

    while not rospy.is_shutdown():
        nt_value_starboard_encoder = "0"
        nt_value_port_encoder = "0"
        rospy.loginfo("Got values from NT, current index is: " + str(0))
        starboard_encoder.publish(nt_value_starboard_encoder)
        port_encoder.publish(nt_value_port_encoder)
        rate.sleep()

if __name__ == '__main__':
    try:
        nt_publisher()
    except rospy.ROSInterruptException:
        pass # Helps when exiting using Ctrl + C 
