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
            DEFAULT_FREQ = 2000):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.IN1 = IN1
        self.IN2 = IN2
        self.IN3 = IN3
        self.IN4 = IN4
        self.ENA = ENA
        self.ENB = ENB

        self.DEFAULT_FREQ = DEFAULT_FREQ
    
        GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    
        #Set the PWM pin and frequency is 2000hz
        self.pwm_ENA = GPIO.PWM(ENA, DEFAULT_FREQ)
        self.pwm_ENB = GPIO.PWM(ENB, DEFAULT_FREQ)
        self.pwm_ENA.start(0)
        self.pwm_ENB.start(0)

    def forward(self, speed=50):
        
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(speed)
        self.pwm_ENB.ChangeDutyCycle(speed)

    def stop(self):
        
        self.pwm_ENA.stop()
        self.pwm_ENB.stop()
