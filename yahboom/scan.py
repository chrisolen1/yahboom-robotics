import yahboom
import time

yb = yahboom.Yahboom()
yb.init_servo()

while True:
    
    yb.rotate_servo("FRONT_SERVO", 55, 135)
    time.sleep(.5)
    yb.rotate_servo("FRONT_SERVO", 135, 55,reverse=True)






