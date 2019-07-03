import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize()
sd = NetworkTables.getTable("SmartDashboard")

i = 0
while True:
    print("dsTime:", sd.getNumber("dsTime", "N/A"))

    sd.putNumber("robotTime", i)
    time.sleep(1)
i += 1