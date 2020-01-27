#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def nt_publisher():
    # Create publisher topics
    starboard_encoder = rospy.Publusher('starboard_encoder', Double, queue_size=10)
    port_encoder = rospy.Publusher('port_encoder', Double, queue_size=10)
    # Init node
    rospy.init_node('nt_interface', anonymous=True)
    rate = rospy.Rate(10) # In hz

    while not rospy.is_shutdown():
        nt_value_starboard_encoder = "0.0"
        nt_value_port_encoder = "0.0"
        rospy.loginfo("Got values from NT, current index is: " + str(0))
        starboard_encoder.publish(nt_value_starboard_encoder)
        port_encoder.publish(nt_value_port_encoder)
        rate.sleep()

if __name__ == '__main__':
    try:
        nt_publisher()
    except rospy.ROSInterruptException:
        pass # Helps when exiting using Ctrl + C 