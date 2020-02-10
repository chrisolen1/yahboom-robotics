import RPi.GPIO as GPIO
#GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.OUT)
p = GPIO.PWM(25,50)
p.start(9)
p.ChangeDutyCycle(4)
#p.stop()

