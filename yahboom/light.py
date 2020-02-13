import RPi.GPIO as GPIO
import time

#Definition of infrared obstacle avoidance module pin
LdrSensorLeft = 7
LdrSensorRight = 6

#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#infrared obstacle avoidance module pins are initialized into input mode
def init():
    GPIO.setup(LdrSensorLeft,GPIO.IN)
    GPIO.setup(LdrSensorRight,GPIO.IN)
    

	
