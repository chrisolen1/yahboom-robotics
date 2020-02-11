import RPi.GPIO as GPIO
import time

class Yahboom():

    def __init__(self,
            IN1 = 20,
            IN2 = 21,
            IN3 = 19,
            IN4 = 26,
            ENA = 16,
            ENB = 13,
            FRONT_SERVO = 23,
            CAMERA_SERVO_H = 25,
            CAMERA_SERVO_V = 9,
            DEFAULT_FREQ = 1000):
        
        self.IN1 = IN1
        self.IN2 = IN2
        self.IN3 = IN3
        self.IN4 = IN4
        self.ENA = ENA
        self.ENB = ENB
        self.FRONT_SERVO = FRONT_SERVO
        self.CAMERA_SERVO_H = CAMERA_SERVO_H
        self.CAMERA_SERVO_V = CAMERA_SERVO_V
        self.DEFAULT_FREQ = DEFAULT_FREQ
    
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def motor_init(self):
        
        GPIO.cleanup()
        global pwm_ENA
        global pwm_ENB
        
        GPIO.setup(self.ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.IN4,GPIO.OUT,initial=GPIO.LOW)
    
        #Set the PWM pin and frequency
        pwm_ENA = GPIO.PWM(self.ENA, self.DEFAULT_FREQ)
        pwm_ENB = GPIO.PWM(self.ENB, self.DEFAULT_FREQ)
        pwm_ENA.start(0)
        pwm_ENB.start(0)

    def motor_reinit(self):
        
        pwm_ENA.stop()
        pwm_ENB.stop()
        self.motor_init()

    def forward(self, speed=20):
        
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(speed)
        pwm_ENB.ChangeDutyCycle(speed)

    def backward(self, speed=20):

        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(speed)
        pwm_ENB.ChangeDutyCycle(speed)

    def right(self, speed=20):
        
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(speed)
        pwm_ENB.ChangeDutyCycle(speed)

    def left(self, speed=20):
        
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(speed)
        pwm_ENB.ChangeDutyCycle(speed)
    
    def stop(self):
        
        pwm_ENA.ChangeDutyCycle(0)
        pwm_ENB.ChangeDutyCycle(0)

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

       assert angle >= 30 and angle <= 150, "angle must be between 30 and 150"

       if servo=="CAMERA_SERVO_V":
           pwm_CAMERA_SERVO_V.start(angle/12)

       elif servo=="CAMERA_SERVO_H":
           pwm_CAMERA_SERVO_H.start(angle/12)

       else:
           print("invalid servo name")

    def rotate_servo(self, servo, starting_angle, ending_angle):

        assert starting_angle >= 30 and starting_angle <= 150, "starting angle must be between 30 and 150"
        assert ending_angle >= 30 and ending_angle <= 150, "ending angle must be between 30 and 150"

        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.start(starting_angle/12)
            for i in range(starting_angle/12, ending_angle/12, .1):
                pwm_CAMERA_SERVO_V.ChangeDutyCycle(i)
            
        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.start(starting_angle/12)
            for i in range(starting_angle/12, ending_angle/12, .1):
                pwm_CAMERA_SERVO_H.ChangeDutyCycle(i)

        else:
            print("invalid servo name")

    def reset_servo(self, servo):
        
        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.start(7.5)

        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.start(7.5)

        else:
            print("invalid servo name")

    def stop_servo(self, servo):

        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.stop()

        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.stop()

        else:
            print("invalid servo name")


