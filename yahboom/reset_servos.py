import yahboom
import time

yb = yahboom.Yahboom()
yb.init_servo()

servos = ["FRONT_SERVO","CAMERA_SERVO_V","CAMERA_SERVO_H"]

for i in range(len(servos)):
    yb.reset_servo(servos[i])
    time.sleep(1)

for i in range(len(servos)):
    yb.stop_servo(servos[i])
    time.sleep(1)
