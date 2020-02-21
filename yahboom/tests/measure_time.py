import os, sys
sys.path.append("/home/pi/Documents/yahboom-robotics/yahboom")

import yahboom
import time

yb = yahboom.Yahboom()
yb.init_motor()
yb.forward(speed=70)

time.sleep(2)

start_time = time.time()

yb.drive_cm(75, 70)

end_time = time.time()

print(end_time - start_time)


