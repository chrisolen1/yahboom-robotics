import os, sys
sys.path.append("/home/pi/Documents/yahboom-robotics/yahboom")

import yahboom
import time
import multiprocessing

yb = yahboom.Yahboom()

yb.init_odometer()
yb.init_motor()

#p1 = multiprocessing.Process(target=yb.odometer())
#p2 = multiprocessing.Process(target=yb.forward())

#p1.start()
#p2.start()

#time.sleep(2)

#p1.terminate()
#p2.terminate()

while True:

    yb.forward(speed=5)
    #print("going to sleep")
    #time.sleep(1)

    while True:
        print("what's going on")
        yb.odometer()
        print("count is:", yb.counter)

