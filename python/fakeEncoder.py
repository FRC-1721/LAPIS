#!/usr/bin/env python

import time
from math import sin, cos
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize()
table = NetworkTables.getTable("SmartDashboard")

fakePortEncoder = 10
fakeStarboardEncoder = 10
speedModifier = 1000
i = 1
while 1:
    table.putNumber("Port", fakePortEncoder)
    table.putNumber("Starboard", fakeStarboardEncoder)

    print(fakePortEncoder)
    print(fakeStarboardEncoder)
    #print(i)

    fakePortEncoder = round(speedModifier * (i + (sin(i) + sin(3 * i) + sin(9 * i)))) # Seperate so we can adjust them individually
    fakeStarboardEncoder = round(speedModifier * (i + (sin(i) + sin(3 * i) + sin(9 * i))))
    time.sleep(0.05)
    i = i + 0.005

    if i >= 10:
        i = 1
