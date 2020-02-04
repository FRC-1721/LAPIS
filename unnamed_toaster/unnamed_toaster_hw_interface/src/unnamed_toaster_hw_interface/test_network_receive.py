import sys
import time
from networktables import NetworkTables

import logging	# Required
logging.basicConfig(level=logging.DEBUG)

if len(sys.argv) != 2:
    print("Error: specify an IP to connect to!")
    exit(0)

ip = sys.argv[1]

smoothLevel = 10000
memRange = 10

NetworkTables.initialize(server=ip)
table = NetworkTables.getTable("ML")
i = 0
p = 0
ball1memX = 0
ball2memX = 0
tempmem1 = 0
tempmem2 = 0
while 1:
    notEnoughBallage = False
    object1posX = table.getNumberArray('boxes', [0,0,0,0])
    numBall = table.getNumber('nb_objects', 0)
    try:    
        ball1centerX = ((object1posX[2]-object1posX[0])/2 + object1posX[0])/5.33
    except:
        pass

    try:    
        ball2centerX = ((object1posX[6]-object1posX[4])/2 + object1posX[4])/5.33
    except:
        pass

    if (numBall == 1):
        try:     
            ball1memX = ball1centerX 
            i = 0
        except:
            pass
    
    if (numBall == 2):
        try:  
            ball2memX = ball2centerX 
            p = 0
        except:
            pass
        try:
            ball1memX = ball1centerX
            i = 0
        except:
            pass
        try:
            if (tempmem2 >= (ball2memX - memRange) and tempmem2 <= (ball2memX + memRange)):
                pass
            if (tempmem1 >= (ball2memX - memRange ) and tempmem1 <= (ball2memX + memRange )):
                ball1memX = tempmem2
            if (tempmem2 >= (ball1memX - memRange ) and tempmem2 <= (ball1memX + memRange )):
                ball2memX = tempmem1
            if (tempmem1 >= (ball1memX - memRange ) and tempmem1 <= (ball1memX + memRange )):
                pass
        except:
            pass
    if (i >= smoothLevel):
        ball1memX = 0
    else:
        pass

    if (p >= smoothLevel):
        ball2memX = 0
    else:
        pass

    if (numBall >= 1):
        print("The center of ball 1 is at " + str(ball1memX))
        print("The center of ball 2 is at " + str(ball2memX))
    elif (numBall > 2):
        print("Too many balls")
    elif (numBall <= 1):
        if (i <= smoothLevel - 1):
            print("The center of ball 1 is at " + str(ball1memX))
        if (p <= smoothLevel - 1):
            print("The center of ball 2 is at " + str(ball2memX))
        if (p >= smoothLevel or i >= smoothLevel):
            notEnoughBallage = True
    if (notEnoughBallage == True):
        print("Not enough ballage")
    #print(ball1memX)
    p = p + 1
    i = i + 1
    table.putNumber("ball1centerX", ball1memX)
    table.putNumber("ball2centerX", ball2memX)