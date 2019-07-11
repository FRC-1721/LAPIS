#!/usr/bin/env python

import time
from math import sin, cos
from networktables import NetworkTables
import logging	# Required

logging.basicConfig(level=logging.debug)

NetworkTables.initialize()
table = NetworkTables.getTable("SmartDashboard")

fakePortEncoder = 0
fakeStarboardEncoder = 0
speedModifier = 100
while true:
    table.putNumber("port", fakePortEncoder)
    table.putNumber("Starboard", fakeStarboardEncoder)

    fakePortEncoder = speedModifier * (fakePortEncoder + (sin(fakePortEncoder) + sin(3 * fakePortEncoder) + sin(9 * fakePortEncoder)))
    fakeStarboardEncoder = speedModifier * (fakeStarboardEncoder + (sin(fakeStarboardEncoder) + sin(3 * fakeStarboardEncoder) + sin(9 * fakeStarboardEncoder)))
    time.sleep(0.01)
