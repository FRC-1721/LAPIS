#!/usr/bin/env python

import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize()
table = NetworkTables.getTable("SmartDashboard")

fakePortEncoder = 0
fakeStarboardEncoder = 0
speedModifier = 10

while 1:
    table.putNumber("Port", fakePortEncoder)
    table.putNumber("Starboard", fakeStarboardEncoder)

    print(fakePortEncoder)
    print(fakeStarboardEncoder)
    #print(i)

    fakePortEncoder = fakePortEncoder + speedModifier
    fakeStarboardEncoder = fakeStarboardEncoder + speedModifier
    time.sleep(0.05)