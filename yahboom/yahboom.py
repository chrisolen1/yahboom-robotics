import RPi.GPIO as GPIO
import time
import numpy as np

class Yahboom():

    def __init__(self,
            MOTOR_LEFT_FORWARD = 20,
            MOTOR_LEFT_BACK = 21,
            MOTOR_RIGHT_FORWARD = 19,
            MOTOR_RIGHT_BACK = 26,
            MOTOR_LEFT_PWM = 16,
            MOTOR_RIGHT_PWM = 13,
            FRONT_SERVO = 23,
            CAMERA_SERVO_H = 25,
            CAMERA_SERVO_V = 9,
            DEFAULT_FREQ = 1000):
        
        self.MOTOR_LEFT_FORWARD = MOTOR_LEFT_FORWARD
        self.MOTOR_LEFT_BACK = MOTOR_LEFT_BACK
        self.MOTOR_RIGHT_FORWARD = MOTOR_RIGHT_FORWARD
        self.MOTOR_RIGHT_BACK = MOTOR_RIGHT_BACK
        self.MOTOR_LEFT_PWM = MOTOR_LEFT_PWM
        self.MOTOR_RIGHT_PWM = MOTOR_RIGHT_PWM
        self.FRONT_SERVO = FRONT_SERVO
        self.CAMERA_SERVO_H = CAMERA_SERVO_H
        self.CAMERA_SERVO_V = CAMERA_SERVO_V
        self.DEFAULT_FREQ = DEFAULT_FREQ
    
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def motor_init(self):
        
        GPIO.cleanup()
        global pwm_MOTOR_LEFT_PWM
        global pwm_MOTOR_RIGHT_PWM
        
        GPIO.setup(self.MOTOR_LEFT_PWM,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.MOTOR_LEFT_FORWARD,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.MOTOR_LEFT_BACK,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.MOTOR_RIGHT_PWM,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.MOTOR_RIGHT_FORWARD,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.MOTOR_RIGHT_BACK,GPIO.OUT,initial=GPIO.LOW)
    
        #Set the PWM pin and frequency
        pwm_MOTOR_LEFT_PWM = GPIO.PWM(self.MOTOR_LEFT_PWM, self.DEFAULT_FREQ)
        pwm_MOTOR_RIGHT_PWM = GPIO.PWM(self.MOTOR_RIGHT_PWM, self.DEFAULT_FREQ)
        pwm_MOTOR_LEFT_PWM.start(0)
        pwm_MOTOR_RIGHT_PWM.start(0)

    def motor_reinit(self):
        
        pwm_MOTOR_LEFT_PWM.stop()
        pwm_MOTOR_RIGHT_PWM.stop()
        self.motor_init()

    def forward(self, speed=20):
        
        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def backward(self, speed=20):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.HIGH)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def right(self, speed=20):
        
        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed/3)

    def left(self, speed=20):
        
        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed/3)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)
    
    def spin_right(self, speed=20):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def spin_left(self, speed=20):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def stop(self):
        
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(0)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(0)

    def servo_init(self):

        global pwm_FRONT_SERVO
        global pwm_CAMERA_SERVO_H
        global pwm_CAMERA_SERVO_V

        GPIO.setup(self.FRONT_SERVO,GPIO.OUT)
        GPIO.setup(self.CAMERA_SERVO_H,GPIO.OUT)
        GPIO.setup(self.CAMERA_SERVO_V,GPIO.OUT)

        pwm_FRONT_SERVO = GPIO.PWM(self.FRONT_SERVO, 50)
        pwm_CAMERA_SERVO_H = GPIO.PWM(self.CAMERA_SERVO_H, 50)
        pwm_CAMERA_SERVO_V = GPIO.PWM(self.CAMERA_SERVO_V, 50)

    def angle(self, servo, angle):

       assert angle >= 60 and angle <= 120, "angle must be between 60 and 120"

       if servo=="CAMERA_SERVO_V":
           pwm_CAMERA_SERVO_V.start(angle/12)

       elif servo=="CAMERA_SERVO_H":
           pwm_CAMERA_SERVO_H.start(angle/12)

       else:
           print("invalid servo name")

    def rotate_servo(self, servo, starting_angle, ending_angle):

        assert starting_angle >= 60 and starting_angle <= 120, "starting angle must be between 60 and 120"
        assert ending_angle >= 60 and ending_angle <= 120, "ending angle must be between 60 and 120"

        seq = np.arange(starting_angle/12, ending_angle/12 + .1, .1)
        
        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.start(starting_angle/12)
            for i in range(len(seq)):
                pwm_CAMERA_SERVO_V.ChangeDutyCycle(seq[i])
                time.sleep(.5)
            
        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.start(starting_angle/12)
            for i in range(len(seq)):
                pwm_CAMERA_SERVO_H.ChangeDutyCycle(seq[i])
                time.sleep(.5)
        else:
            print("invalid servo name")

    def reset_servo(self, servo):
        
        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.start(7.5)

        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.start(8)

        else:
            print("invalid servo name")

    def stop_servo(self, servo):

        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.stop()

        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.stop()

        else:
            print("invalid servo name")


