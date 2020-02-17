import RPi.GPIO as GPIO
import time
import numpy as np
import math

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
            DEFAULT_FREQ = 1000,
            WHEEL_DIAMETER = 6.65,
            WHEEL_BASE_WIDTH = 16,
            ULTRASONIC_ECHOPIN = 0,
            ULTRASONIC_TRIGGERPIN = 1,
            INFRARED_LEFT = 12,
            INFRARED_RIGHT = 17):

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

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
        
        self.WHEEL_DIAMETER = WHEEL_DIAMETER # in cm
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_DIAMETER * math.pi
        self.WHEEL_BASE_WIDTH = WHEEL_BASE_WIDTH # in cm
        self.WHEEL_BASE_CIRCUMFERENCE = self.WHEEL_BASE_WIDTH * math.pi
    
        self.ULTRASONIC_ECHOPIN = ULTRASONIC_ECHOPIN
        self.ULTRASONIC_TRIGGERPIN = ULTRASONIC_TRIGGERPIN

        self.INFRARED_LEFT = INFRARED_LEFT
        self.INFRARED_RIGHT = INFRARED_RIGHT

    def init_motor(self):
        
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

    def reinit_motor(self):
        
        pwm_MOTOR_LEFT_PWM.stop()
        pwm_MOTOR_RIGHT_PWM.stop()
        self.init_motor()

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
        
        """
        General function for turning right.

        """

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed/3)

    def left(self, speed=20):
        
        """
        General function for turning left. 

        """

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

    def back_left(self, speed=15):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.HIGH)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed/3)
    
    def back_right(self, speed=15):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.HIGH)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed/3)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def back_spin_left(self, speed=15):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def back_spin_right(self, speed=15):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.HIGH)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def steer(self, left_percent, right_percent):
        
        assert left_percent <= 100 and left_percent >= 0, "value must be 0 <= x <= 100"
        assert right_percent <= 100 and right_percent >= 0, "value must be 0 <= x <= 100"

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(left_percent)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(right_percent)
    
    def stop(self):
        
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(0)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(0)

    def init_servo(self):

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

       elif servo=="FRONT_SERVO":
           pwm_FRONT_SERVO.start(angle/12)
       
       else:
           print("invalid servo name")

    def rotate_servo(self, servo, starting_angle, ending_angle, reverse=False):

        assert starting_angle >= 45 and starting_angle <= 135, "starting angle must be between 45 and 135"
        assert ending_angle >= 45 and ending_angle <= 135, "ending angle must be between 45 and 135"

        if reverse==False:
            seq = np.arange(starting_angle/12, ending_angle/12 + .5, .5)

        else:
            seq = np.arange(ending_angle/12, starting_angle/12 + .5, .5)
            seq = seq[len(seq):None:-1]
        
        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.start(starting_angle/12)
            for i in range(len(seq)):
                pwm_CAMERA_SERVO_V.ChangeDutyCycle(seq[i])
                time.sleep(.1)
            
        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.start(starting_angle/12)
            for i in range(len(seq)):
                pwm_CAMERA_SERVO_H.ChangeDutyCycle(seq[i])
                time.sleep(.1)
        
        elif servo=="FRONT_SERVO":
            pwm_FRONT_SERVO.start(starting_angle/12)
            for i in range(len(seq)):
                pwm_FRONT_SERVO.ChangeDutyCycle(seq[i])
                time.sleep(.1)
        
        else:
            print("invalid servo name")

    def reset_servo(self, servo):
        
        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.start(7.5)

        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.start(8)

        elif servo=="FRONT_SERVO":
            pwm_FRONT_SERVO.start(8)
        
        else:
            print("invalid servo name")

    def stop_servo(self, servo):

        if servo=="CAMERA_SERVO_V":
            pwm_CAMERA_SERVO_V.stop()

        elif servo=="CAMERA_SERVO_H":
            pwm_CAMERA_SERVO_H.stop()

        elif servo=="FRONT_SERVO":
            pwm_FRONT_SERVO.stop()
        
        else:
            print("invalid servo name")
                    
    def init_distance_sensor(self):

        GPIO.setup(self.ULTRASONIC_ECHOPIN,GPIO.IN)
        GPIO.setup(self.ULTRASONIC_TRIGGERPIN,GPIO.OUT)
        GPIO.setup(self.INFRARED_LEFT,GPIO.IN)
        GPIO.setup(self.INFRARED_RIGHT,GPIO.IN)
    
    def UltraSonicSensor(self):

        GPIO.output(self.ULTRASONIC_TRIGGERPIN, GPIO.LOW)
        #print("Waiting for sensor to settle")
        time.sleep(.1)

        #print("Calculating distance")

        # set trigger after 0.01ms from HIGH to LOW
        GPIO.output(self.ULTRASONIC_TRIGGERPIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.ULTRASONIC_TRIGGERPIN, GPIO.LOW)

        # save StartTime
        while GPIO.input(self.ULTRASONIC_ECHOPIN) == 0:
            StartTime = time.time()

        while GPIO.input(self.ULTRASONIC_ECHOPIN) == 1:
            StopTime = time.time()

        TimeElapsed = StopTime - StartTime

        distance = (TimeElapsed * 34300) / 2

        return distance
    
    def InfraredSensor(self):

        LeftSensorValue = GPIO.input(self.INFRARED_LEFT)
        RightSensorValue = GPIO.input(self.INFRARED_RIGHT)

        return LeftSensorValue, RightSensorValue
