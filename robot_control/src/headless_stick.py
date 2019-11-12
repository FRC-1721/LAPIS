#!/usr/bin/env python

import time
import maestro
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

ip = "roboRIO-1721-FRC"
#ip = "localhost" # For debugging
print ("Starting NetworkTables using IP: ", ip)
NetworkTables.initialize(server = ip)
table = NetworkTables.getTable("ROS")

while(true):
    servo_thro = maestro.Controller()
    servo_steer = maestro.Controller()
    
    raw_port = float(table.putNumber("coprocessorPort", '0'))
    raw_starboard = float(table.putNumber("coprocessorStarboard", '0'))
    
    steer = raw_port - raw_starboard
    thro = (raw_port + raw_starboard ) / 2
    
    servo_thro.setTarget(0,(thro * 6000) + 6000)  #set thro
    servo_steer.setTarget(0,(steer * 6000) + 6000)  #set steer
 
servo_thro.close()
servo_steer.close()