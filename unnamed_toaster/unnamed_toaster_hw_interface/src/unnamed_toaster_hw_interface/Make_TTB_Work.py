#initialize and imports, do not edit
import sys
import time
from networktables import NetworkTables

import logging
logging.basicConfig(level=logging.DEBUG)

mlserver = NetworkTables
mlserver.initialize(server="10.17.21.2")
mltable = mlserver.getTable("ML")

b1cx = 696969

def get_b1cx():
    return mltable.getNumber('ball1centerX', 0)
