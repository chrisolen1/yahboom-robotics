import os, sys
sys.path.append("/home/pi/Documents/yahboom-robotics/yahboom")

import yahboom
import time

yb = yahboom.Yahboom()
yb.init_distance_sensor()

while True:
    result = yb.UltraSonicSensor()
    print(result)
    time.sleep(.0000001)


