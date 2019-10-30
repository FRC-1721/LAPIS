#!/usr/bin/env python

import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

ip = "roboRIO-1721-FRC"
#ip = "localhost"
print ("Starting NetworkTables using IP: ", ip)
NetworkTables.initialize(server = ip)
table = NetworkTables.getTable("ROS")

for i in range(10):
    table.putNumber("coprocessorPort", 0.5)
    table.putNumber("coprocessorPort", 0.5)
    time.sleep(0.5)
    table.putNumber("coprocessorPort", -0.5)
    table.putNumber("coprocessorPort", -0.5)
    time.sleep(0.5)
    table.putNumber("coprocessorPort", 0)
    table.putNumber("coprocessorPort", 0)
    time.sleep(0.5)