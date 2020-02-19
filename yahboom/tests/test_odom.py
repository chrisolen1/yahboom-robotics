from RPi import GPIO
import time

def odometer(odometer_pin):

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(odometer_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    global lastClkState
    global counter

    lastClkState = GPIO.input(odometer_pin)
    counter = 0
    
    while True:

        clkState = GPIO.input(odometer_pin)
    
        if clkState != lastClkState:
            counter += 1
        
        lastClkState = clkState

        print("counter",counter)
        time.sleep(.0001)




