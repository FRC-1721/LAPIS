#!/usr/bin/env python

import rospy
import tf.transformations

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

from geometry_msgs.msg import Twist
from networktables import NetworkTables

ip = "roboRIO-1721-FRC"
#ip = "localhost" # For debugging
print ("Starting NetworkTables(Robot Control) using IP: ", ip)
NetworkTables.initialize(server = ip)
table = NetworkTables.getTable("ROS")

def callback(msg):
    thro = msg.linear.x +  msg.linear.y #+ msg.linear.z # Not even sure nav stack will ask for linear z?
    steerage = msg.angular.x # In radians, not quite sure how to use
    #msg.angular.y
    #msg.angular.z

    table.putNumber("coprocessorPort", thro - steerage) # Not sure if any of this will work
    table.putNumber("coprocessorStarboard", (thro + steerage) * -1) # Not sure if any of this will work

def start_listener():
    rospy.init_node('cmd_vel_hungry_toaster')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    start_listener()