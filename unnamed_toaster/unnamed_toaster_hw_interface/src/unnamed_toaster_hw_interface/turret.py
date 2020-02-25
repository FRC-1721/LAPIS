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
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse

from unnamed_toaster_hw_interface.robot_command import RobotCommand

class Turret:

    def __init__(self, table):
        self.table = table
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=1) # Joint state publisher
        self.sub = rospy.Subscriber("turret_command", Float64, self.commandCallback)
        # TODO Add subscriber for turret command

        # Initialize things
        self.raw_angle = 500
        self.turret_angle = 0.0
        self.offset_ticks = -6663.0         # Calibrated 2/18/2020
        self.ticks_per_rotation = 60468.0   # Calibrated 2/18/2020
        self.ticks_per_radians = 9550.0     # Calibrated 2/19/2020

        self.zero_turret = RobotCommand("zero_turret", self.table)
        self.zero_service = rospy.Service("zero_turret", Empty, self.zeroCallback)
        self.enable_turret = RobotCommand("enable_shooter", self.table)


    def commandCallback(self, msg):
        # Enable control of turret
        self.enable_turret.start()

        # Convert the turret angle from radians to ticks
        command = -msg.data * self.ticks_per_radians + self.offset_ticks
        if command < -self.ticks_per_rotation:
            command = -self.ticks_per_rotation
        elif command > 0:
            command = 0

        # Set the turret angle (in ticks)
        self.table.putNumber("coprocessorTurret", command)


    def update(self):
        self.raw_angle = self.table.getFloat('Turret', 500) # Get the raw turret angle

        if self.raw_angle > 0: # Bounce if not legal
            return


        self.turret_angle = (self.offset_ticks - self.raw_angle) / self.ticks_per_radians


    def publish(self):
        msg = JointState()

        msg.header.stamp = rospy.Time.now()
        msg.name.append("turret_joint")
        msg.position.append(self.turret_angle)

        self.pub.publish(msg)


    def zeroCallback(self, req):
        self.zero_turret.start()

        start = rospy.Time.now()
        cycles_zeroed = 0
        while True:
            if abs(self.raw_angle) < 200:
                cycles_zeroed += 1
            if abs(self.turret_angle) < 0.1:
                cycles_zeroed += 1
            if cycles_zeroed > 100:
                break
            if rospy.Time.now() - start > rospy.Duration(5.0):
                break
            rospy.sleep(0.05)
        self.zero_turret.stop()

        return EmptyResponse()

