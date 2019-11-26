import pyttsx3
import time
from networktables import NetworkTables

import logging # Required
logging.basicConfig(level=logging.DEBUG)

ip = "10.17.21.2"
hal9000 = pyttsx3.init()

NetworkTables.initialize(server=ip)
table = NetworkTables.getTable("SmartDashboard")

hal9000.say("Robot Audio Service Started")
hal9000.runAndWait()

previous_alert = ""
while True:
    alert = table.getString("alert", "")
    if alert != previous_alert:
        print(alert)
        hal9000.say(alert)
        hal9000.runAndWait()
    else:
        time.sleep(1)
    previous_alert = alert