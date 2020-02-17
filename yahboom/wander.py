import yahboom
import time

yb = yahboom.Yahboom()

yb.init_motor()
yb.init_distance_sensor()

while True:

    
    while yb.UltraSonicSensor() > 20 and yb.InfraredSensor()[0] == 1 and yb.InfraredSensor()[1] == 1:
            
        yb.forward(speed=15)  

    yb.stop()
    yb.backward(speed=10)
    time.sleep(2)
    yb.back_spin_left()
    time.sleep(2)

   

