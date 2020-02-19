import yahboom
import time

yb = yahboom.Yahboom()

yb.init_motor()
yb.init_distance_sensor()

def wander(day_mode=False):
    
    while True:
        
        if day_mode==False:

            while yb.UltraSonicSensor() > 25 and yb.InfraredSensor()[0] == 1 and yb.InfraredSensor()[1] == 1:
            
                yb.reinit_motor()
                yb.forward(speed=15)
                time.sleep(1)

            yb.stop()
            yb.backward(speed=10)
            time.sleep(2)
            yb.back_spin_left()
            time.sleep(2)
        
        else:

            while yb.UltraSonicSensor() > 25:

                yb.reinit_motor()
                yb.forward(speed=15)
                time.sleep(1)

            yb.stop()
            yb.backward(speed=10)
            time.sleep(2)
            yb.back_spin_left()
            time.sleep(2)

