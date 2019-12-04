#!/usr/bin/env python
# from robotpy.readthedocs.io


import sys
import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

if len(sys.argv) != 2:
    print("Error: specify an IP to connect to!")
    exit(0)

ip = sys.argv[1]

NetworkTables.initialize(server=ip)
table = NetworkTables.getTable("ROS")

while 1:
    alive = table.getNumber('alive', '-1')
    print("received " + str(alive) + " from networktables")

    time.sleep(1)