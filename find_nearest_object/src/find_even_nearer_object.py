#! /usr/bin/env python
 
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped, PointStamped
 
def callback(msg):
    number_of_laser_points = len(msg.ranges)
    closest_laser = 1000
    closest_laser_angle = 180
    for current_angle in range(number_of_laser_points):
        current_laser = msg.ranges[current_angle]
        if closest_laser > current_laser:
            closest_laser = current_laser
            closest_laser_angle = current_angle
        print("Closest point is " + str(closest_laser) + " at angle " + str(closest_laser_angle))
        
 
rospy.init_node('scan_values')
closest_point = rospy.Publisher("/closest_point", PointStamped, queue_size=1)
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()