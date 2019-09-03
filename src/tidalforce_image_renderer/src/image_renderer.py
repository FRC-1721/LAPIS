#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class laser_pointcloud(): # convert laser to a pointcloud
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher ("/laserPointCloud", pc2, queue_size=1) # point cloud publisher
        self.laserSub = rospy.Subscriber ("/scan", LaserScan, self.laserCallback) # subscribe to laser scan
        print("init")

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data) # create cloud
        self.pcPub.publish (cloud_out) # publish cloud out
        print("scan")

#class pointcloud_image(): # convert pointcloud to image
#    def __init__(self):
#        # init
#
#    def something(self, data):
#        # publish stuff

if __name__ == '__main__':
    rospy.init_node ("laserPC")
    l2pc = laser_pointcloud()
    rospy.spin()
