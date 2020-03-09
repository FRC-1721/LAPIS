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
from unnamed_toaster_hw_interface.limelight import Limelight

class DefaultTableInterface:

    def __init__(self):
        # Get the parameter for the IP (fallback to DNS (DNS does not work on the field!))
        ip = rospy.get_param("~ip", "roboRIO-1721-FRC")
        # Get the paramater for the rate
        self.rate = rospy.get_param("~rate", 50)
        # Get the table limelight from the server at "ip"
        self.table = NetworkTablesInterface("limelight", ip)
        self.limelight = Limelight()

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.limelight.update(self.limelight_table)
            self.limelight.publish()

            # Put other logic and data here to run other attached scripts

            # Sleep
            rate.sleep()


if __name__=="__main__":
    rospy.init_node("default_table_node")
    table = DefaultTableInterface()
    table.run()
