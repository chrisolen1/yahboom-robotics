import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(10,GPIO.IN,pull_up_down=GPIO.PUD_UP)

while True:
    print("Encoder value:",GPIO.input(10))
    time.sleep(.5)


