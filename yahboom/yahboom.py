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
            DEFAULT_FREQ = 1000):
        
        self.IN1 = IN1
        self.IN2 = IN2
        self.IN3 = IN3
        self.IN4 = IN4
        self.ENA = ENA
        self.ENB = ENB
        self.DEFAULT_FREQ = DEFAULT_FREQ
    
    def motor_init(self):
        
        GPIO.cleanup()
        global pwm_ENA
        global pwm_ENB
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.IN4,GPIO.OUT,initial=GPIO.LOW)
    
        #Set the PWM pin and frequency is 2000hz
        pwm_ENA = GPIO.PWM(self.ENA, self.DEFAULT_FREQ)
        pwm_ENB = GPIO.PWM(self.ENB, self.DEFAULT_FREQ)
        pwm_ENA.start(0)
        pwm_ENB.start(0)

    def forward(self, speed=50):
        
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(speed)
        pwm_ENB.ChangeDutyCycle(speed)

    def backward(self, speed=50):

        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(speed)
        pwm_ENB.ChangeDutyCycle(speed)

    def stop(self):
        
        pwm_ENA.stop()
        pwm_ENB.stop()
        
