import light
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

try:
    print("Here we go")
    light.init()
    print("Initialized!")
    #inf.key_scan()
    print("key scanned!")
    while True:
      #There is obstacle, the indicator light of the infrared obstacle avoidance module is on, and the port level is LOW
      #There is no obstacle, the indicator light of the infrared obstacle avoidance module is off, and the port level is HIGH
      LeftSensorValue  = GPIO.input(light.LdrSensorLeft);
      RightSensorValue = GPIO.input(light.LdrSensorRight);
      print("LeftSensorValue is", LeftSensorValue)
      print("RightSensorValue is", RightSensorValue)
      time.sleep(0.002)
       
except KeyboardInterrupt:
    pass

GPIO.cleanup()
