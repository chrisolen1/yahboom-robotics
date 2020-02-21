import RPi.GPIO as GPIO
import time

import numpy as np
import math

import multiprocessing

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
            ODOMETER = 10,
            ENCODER_TICKS_PER_ROTATION = 6,
            WHEEL_DIAMETER = 6.65,
            WHEEL_BASE_WIDTH = 14,
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
        
        self.ODOMETER = ODOMETER
        self.WHEEL_DIAMETER = WHEEL_DIAMETER # in cm
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_DIAMETER * math.pi
        self.WHEEL_BASE_WIDTH = WHEEL_BASE_WIDTH # in cm
        self.WHEEL_BASE_CIRCUMFERENCE = self.WHEEL_BASE_WIDTH * math.pi
        self.ENCODER_TICKS_PER_ROTATION = ENCODER_TICKS_PER_ROTATION # number of magnet positions
        #self.MOTOR_GEAR_RATIO = 120 # Motor gear ratio # 220 for Nicole's prototype
        #MOTOR_TICKS_PER_DEGREE = ((MOTOR_GEAR_RATIO * ENCODER_TICKS_PER_ROTATION) / 360.0) # encoder ticks per output shaft rotation degree

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
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed+3) # correcting for power imbalance
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def backward(self, speed=20):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.HIGH)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed+3) # correcting for power imbalance
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def right(self, left_speed=20, right_speed=20/3):
        
        """
        General function for turning right.

        """

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(left_speed) # correcting for power imbalance
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(right_speed)

    def left(self, left_speed=20/3, right_speed=20):
        
        """
        General function for turning left. 

        """

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(left_speed) # correcting for power imbalance
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(right_speed)
    
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
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed+3) # correcting for power imbalance
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed/3)
    
    def back_right(self, speed=15):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.HIGH)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle((speed/3)+3) # correcting for power imbalance
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def back_spin_left(self, speed=25):

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(speed)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(speed)

    def back_spin_right(self, speed=25):

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
        time.sleep(.001)

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

    def init_odometer(self):

        GPIO.setup(self.ODOMETER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.odometer(reset_odometer=True)
    
    def odometer(self, reset_odometer=False):
    
        if reset_odometer == True:
            
            self.lastClkState = GPIO.input(self.ODOMETER)
            self.counter = 0
    

        clkState = GPIO.input(self.ODOMETER)
    
        if clkState != self.lastClkState:
            self.counter += 1
            self.tick = True

        else:
            self.tick = False
            
        self.lastClkState = clkState
    
    def drive_cm(self, dist, speed, direction="forward"):

        # number of degrees each wheel needs to turn
        wheelTurnDegrees = (dist/self.WHEEL_CIRCUMFERENCE) * 360
        degreesPerTicks = 360 / self.ENCODER_TICKS_PER_ROTATION
        ticksToTravel = wheelTurnDegrees / degreesPerTicks

        ticksRemaining = ticksToTravel

        self.init_odometer()
        
        try:
            self.init_motor()
        
        except:
            self.reinit_motor()

        start = True

        while start:
        
            if direction == "forward":
                self.forward(speed=speed)

            elif direction == "backward":
                self.backward(speed=speed)

            elif direction == "left":
                self.left(speed=speed)

            else:
                self.right(speed=speed)

            while ticksRemaining > 0:
                        
                self.odometer()
                print("count is:", self.counter)
                
                if self.tick == True:
                    ticksRemaining -= 1
                
                print("ticks remaining:", ticksRemaining)

            self.stop()
            start = False

    def orbit(self, degrees, radius_cm, direction, speed):
        
        assert degrees >= 0, "degrees must be positive"

        # total distance to drive in cm
        inner_drive_distance = math.pi * 2 * radius_cm * degrees / 360
        outer_drive_distance = math.pi * 2 * (radius_cm + self.WHEEL_BASE_WIDTH) * degrees /360
        # degrees each wheel has to turn
        inner_degrees = inner_drive_distance / self.WHEEL_CIRCUMFERENCE * 360
        outer_degrees = outer_drive_distance / self.WHEEL_CIRCUMFERENCE * 360

        if direction == "left":

            # so, left is inner and right is outer
            right_speed = self.pwm_to_cms(speed) # convert from pwm to cm/s
            left_speed = inner_drive_distance * right_speed / outer_drive_distance / 2

        else:

            # so, right is inner and left is outer
            left_speed = self.pwm_to_cms(speed)
            right_speed = inner_drive_distance * left_speed / outer_drive_distance / 3


        degreesPerTicks = 360 / self.ENCODER_TICKS_PER_ROTATION
        
        # rotary encoder is on left wheel, so have to choose inner_degrees, outer_degrees accordingly
        if direction == "left":
            ticksToTravel = inner_degrees / degreesPerTicks # since the encoder will be on the inner

        else:
            ticksToTravel = outer_degrees / degreesPerTicks # since the encoder will be on the outer
        
        ticksRemaining = ticksToTravel

        self.init_odometer()
        
        try:
            self.init_motor()
        
        except:
            self.reinit_motor()

        start = True

        while start:
        
            if direction == "left":
                self.left(left_speed=left_speed, right_speed=right_speed)

            else:
                self.right(left_speed=left_speed, right_speed=right_speed)

            while ticksRemaining > 0:
                        
                self.odometer()
                print("count is:", self.counter)
                
                if self.tick == True:
                    ticksRemaining -= 1
                
                print("ticks remaining:", ticksRemaining)

            self.stop()
            start = False

    def pwm_to_cms(self, pwm):

        # coefficients of fit exp curve
        a = -77.50414018313563
        b = 11.856787257598077
        c = 0.016664634500959574
        d = 97.55455402950095
        
        # output cm/s
        cms = a*np.exp(-c*(pwm-b))+d
        return cms
