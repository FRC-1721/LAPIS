#!/usr/bin/env python

from nav_msgs.msg import Odometry
from networktables import NetworkTables
from math import cos, sin
import rospy
import tf

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

ip = 'roboRIO-1721-FRC.local'	# IP of server (rio)
NetworkTables.initialize(server=ip)

sd = NetworkTables.getTable('SmartDashboard')	# Init smartDashboard

ticks_meter = 10000.0
base_width = 0.75
enc_left = None
enc_right = None


rospy.init_node('fake_odom')	# Contacts ROS and adds fake_odom to the list of nodes
pub = rospy.Publisher('odom', Odometry, queue_size=10)
br = tf.TransformBroadcaster()
rate = rospy.Rate(10)	# in Hz
then = rospy.Time.now()
while not rospy.is_shutdown():	# runs for as long as the node is running
    left = float(sd.getNumber('Port','0'))
    right = float(sd.getNumber('Starboard','0'))

    now = rospy.Time.now()
    elapsed = (now - then).to_sec()
    then = now

    if enc_left == None:
        d_left = 0
        d_right = 0
    else:
        d_left = (left - enc_left)/ticks_meter
        d_right = (right - enc_right)/ticks_meter
    enc_left = left
    enc_right = right

    d = (d_left+d_right)/2
    th = (d_right-d_left)/base_width
    dx = d / elapsed
    dr = th / elapsed

    if (d != 0):
        x = cos(th)*d
        y = -sin(th)*d
        x = x + (cos(th)*x - sin(th)*y)
        y = y + (sin(th)*x + cos(th)*y)
	if (th != 0):
            th = th + th

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "laser"
        odom.pose.pose.orientation.w = 1.0
        #odom.pose.pose.orientation.z = 1.0
        pub.publish(odom)
        br.sendTransform((x, y, 0), (0, 0, sin(th / 2), cos(th / 2)), rospy.Time.now(), "laser", "odom")
rate.sleep()