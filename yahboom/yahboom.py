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
            WHEEL_BASE_WIDTH = 16):
        
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
        self.WHEEL_BASE_WIDTH = wheel_base_width # in cm
        self.WHEEL_BASE_CIRCUMFERENCE = self.WHEEL_BASE_WIDTH * pi
    
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

    def steer(self, left_percent, right_percent):
        
        assert left_percent <= 100 and left_percent >= 0, "value must be 0 <= x <= 100"
        assert right_percent <= 100 and right_percent >= 0, "value must be 0 <= x <= 100"

        GPIO.output(self.MOTOR_LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_LEFT_BACK, GPIO.LOW)
        GPIO.output(self.MOTOR_RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(self.MOTOR_RIGHT_BACK, GPIO.LOW)
        pwm_MOTOR_LEFT_PWM.ChangeDutyCycle(left_percent)
        pwm_MOTOR_RIGHT_PWM.ChangeDutyCycle(right_percent)
    
    def orbit(self):

        def orbit(self, speed, degrees, radius_cm=0, blocking=True):
        
        """
        Drive in a circle  
        
        :param int degrees: Degrees to steer. **360** for full rotation. Negative for left turn.
        :param int radius_cm: Radius in `cm` of the circle to drive. Default is **0** (turn in place).
        :param boolean blocking = True: Set it as a blocking or non-blocking method.

        """
        assert (degrees <= 0 and degrees >= -360) or (degrees >= 0 and degrees <= 360), "invalid value for degrees"

        radius = radius_cm * 10

        # the total distance to drive in mm
        drive_distance = math.pi * abs(radius) * abs(degrees) / 180 # / 180 is shorter than radius * 2 / 360
        
        # the distance in mm to add to one motor and subtract from the other
        drive_difference = ((self.WHEEL_BASE_CIRCUMFERENCE * 10 * degrees) / 360)
        
        # the number of degrees each wheel needs to turn on average to get the necessary distance 
        distance_degrees = ((drive_distance / self.WHEEL_CIRCUMFERENCE) * 360)
        
        # the difference between motor travel in degrees
        difference_degrees = ((drive_difference / self.WHEEL_CIRCUMFERENCE) * 360)
        
        # the distance each wheel needs to turn
        left_target  = (distance_degrees + difference_degrees)
        right_target = (distance_degrees - difference_degrees)
        
        # if it's a left turn
        if degrees < 0:
            MOTOR_FAST = self.pwm_MOTOR_RIGHT_PWM
            MOTOR_SLOW = self.pwm_MOTOR_LEFT_PWM
            fast_target = right_target
            slow_target = left_target
        
        # if it's a right turn
        else:
            MOTOR_FAST = self.pwm_MOTOR_LEFT_PWM
            MOTOR_SLOW = self.pwm_MOTOR_RIGHT_PWM
            fast_target = left_target
            slow_target = right_target
        
        # determine driving direction from the speed
        #direction = 1
        #speed_with_direction = speed
        #if speed < 0:
        #    direction = -1
        #    speed_with_direction = -speed
        
        # calculate the motor speed for each motor
        fast_speed = speed
        slow_speed = abs((speed * slow_target) / fast_target)
        
        # set the motor speeds
        self.set_motor_limits(MOTOR_FAST, dps = fast_speed)
        self.set_motor_limits(MOTOR_SLOW, dps = slow_speed)
        
        # get the starting position of each motor
        StartPositionLeft = self.get_motor_encoder(self.MOTOR_LEFT)
        StartPositionRight = self.get_motor_encoder(self.MOTOR_RIGHT)
        
        # Set each motor target position
        self.set_motor_position(self.MOTOR_LEFT, (StartPositionLeft + (left_target * direction)))
        self.set_motor_position(self.MOTOR_RIGHT, (StartPositionRight + (right_target * direction)))
        
        if blocking:
            while self.target_reached(
                    StartPositionLeft + (left_target * direction),
                    StartPositionRight + (right_target * direction)) is False:
                time.sleep(0.1)
        
            # reset to original speed once done
            # if non-blocking, then the user is responsible in resetting the speed
            self.set_speed(speed)
        
        return
    
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


