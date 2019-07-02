#!/usr/bin/env python

from nav_msgs.msg import Odometry
import rospy
import tf

rospy.init_node('fake_odom')	#Contacts ROS and adds fake_odom to the list of nodes
pub = rospy.Publisher('odom', Odometry, queue_size=10)
br = tf.TransformBroadcaster()
rate = rospy.Rate(10)	#in Hz
while not rospy.is_shutdown():	#runs for as long as the node is running
	odom = Odometry()
	odom.header.frame_id = "odom"
	odom.child_frame_id = "laser"
	odom.pose.pose.orientation.w = 1.0
	#odom.pose.pose.orientation.z = 1.0
	pub.publish(odom)
	br.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "laser", "odom")
	rate.sleep()
