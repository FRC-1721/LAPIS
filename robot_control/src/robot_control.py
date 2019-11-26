#!/usr/bin/env python

import rospy
import tf.transformations

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

from geometry_msgs.msg import Twist
from networktables import NetworkTables

ip = rospy.get_param("~ip", "roboRIO-1721-FRC") # Otherwise use mDNS as default
#ip = "localhost" # For debugging
print ("Starting NetworkTables(Robot Control) using IP: ", ip)
NetworkTables.initialize(server = ip)
table = NetworkTables.getTable("ROS")

steerage_p = 0.7

def callback(msg):
    thro = msg.linear.x # x is forward of the robot no matter what
    steerage = msg.angular.z * steerage_p # In radians, equates to speed

    table.putNumber("coprocessorPort", thro - steerage) # Not sure if any of this will work
    table.putNumber("coprocessorStarboard", (thro + steerage) * -1) # Not sure if any of this will work

def start_listener():
    rospy.init_node('cmd_vel_hungry_toaster')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    start_listener()
