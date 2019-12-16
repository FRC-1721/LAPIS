#!/usr/bin/env python

import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize()
table = NetworkTables.getTable("ROS")

alive = 0
i = 0
while 1:
    table.putNumber("alive", alive)
    print("Sent " + str(alive) + " to networktables")

    alive = alive + 1
    time.sleep(1)

    if i == 10:
        i = 1