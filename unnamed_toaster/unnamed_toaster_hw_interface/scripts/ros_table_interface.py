#!/usr/bin/env python

"""
    Copyright (c) 2019-2020, Concord Robotics Inc
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Concord Robotics Inc nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
from unnamed_toaster_hw_interface.network_tables_interface import NetworkTablesInterface
from unnamed_toaster_hw_interface.robot_control import RobotControl
from unnamed_toaster_hw_interface.robot_odom import RobotOdom
from unnamed_toaster_hw_interface.turret import Turret

from unnamed_toaster_hw_interface.robot_command import RobotCommand

class ROSTableInterface:

    def __init__(self):
        ip = rospy.get_param("~ip", "roboRIO-1721-FRC")
        self.table = NetworkTablesInterface("ROS", ip, 5800)

        # Commands
        self.zero_turret = RobotCommand("zeroTurret", self.table)

        # Systems
        self.control = RobotControl(self.table)
        self.turret = Turret(self.table)
        self.odom = RobotOdom()

    def run(self):
        rate = rospy.Rate(50)
        previous_odom_index = 0
        while not rospy.is_shutdown():
            # Odom Get info
            left = self.table.getFloat('Port', 0)
            right = self.table.getFloat('Starboard', 0)
            index = self.table.getInt('rosIndex', 0)
            # Logic for odom sawtooth control
            if index != previous_odom_index:
                self.odom.update(left, right)
            self.odom.publish()
            previous_index = index


            self.turret.update()
            self.turret.publish()
        
            # Sleep
            rate.sleep()


if __name__=="__main__":
    rospy.init_node("ros_table_node")
    table = ROSTableInterface()
    table.run()
