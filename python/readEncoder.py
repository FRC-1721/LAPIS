#!/usr/bin/env python3

import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

ip = 'roboRIO-####-FRC.local'	# IP of server (rio)
NetworkTables.initialize(server=ip)

sd = NetworkTables.getTable('SmartDashboard')	# Init smartDashboard
sd.putString('LAPIS State', 'Online')	# Put a number up

while 1:
	portEncoder = sd.getNumber('Port','0')
	starboardEncoder = sd.getNumber('Starboard','0')

	print(portEncoder + starboardEncoder)
	time.sleep(1)
