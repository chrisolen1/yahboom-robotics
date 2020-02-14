import yahboom
import time


yb = yahboom.Yahboom()

yb.init_motor()
yb.init_servo()
yb.init_distance_sensor()

servos = ["CAMERA_SERVO_V", "FRONT_SERVO", "CAMERA_SERVO_H"]

# reset servos
for i in range(len(servos)):
    yb.reset_servo(servos[i])
    time.sleep(1)

for i in range(len(servos)):
    yb.stop_servo(servos[i])
    time.sleep(1)

while True:

    while yb.UltraSonicSensor() > 20 and yb.InfraredSensor()[0] == 1 and yb.InfraredSensor()[1] == 1:

        yb.forward(speed=15)

    yb.stop()
    yb.backward(speed=10)
    time.sleep(2)
    yb.back_spin_left()
    time.sleep(2)






