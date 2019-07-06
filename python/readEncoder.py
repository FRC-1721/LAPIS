#!/usr/bin/env python3

import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

ip = localhost	# IP of server (rio)
NetworkTables.initialize(server=ip)

sd = NetworkTables.getTable('SmartDashboard')	# Init smartDashboard
sd.putString('LAPIS State', 'Online')	# Put a number up

while true:
	portEncoder = sd.getNumber('Port')
	starboardEncoder = sd.getNumber('Starboard')

	print(portEncoder + starboardEncoder)