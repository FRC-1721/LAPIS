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

from math import pi

from sensor_msgs.msg import JointState


class Turret:

    def __init__(self, table):
        self.table = table
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=1) # Joint state publisher
        # TODO Add subscriber for turret command 

        # Initialize things
        self.turret_angle = 0.0
        self.offset_ticks = -6663.0         # Calibrated 2/18/2020
        self.ticks_per_rotation = 60468.0   # Calibrated 2/18/2020


    def update(self):
        angle = self.table.getFloat('Turret', 500) # Get the raw turret angle

        if angle > 0: # Bounce if not legal 
            return

        self.turret_angle = ((self.offset_ticks - angle) / self.ticks_per_rotation) * (2 * pi)


    def publish(self):
        msg = JointState()

        msg.header.stamp = rospy.Time.now()
        msg.name.append("turret_joint")
        msg.position.append(self.turret_angle)