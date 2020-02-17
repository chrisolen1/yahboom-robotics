import yahboom
import time

yb = yahboom.Yahboom()

yb.init_motor()

def manual_command(command, duration):

    if command == "backward":
        yb.backward()
        time.sleep(duration)
    else: 
        yb.forward()
        time.sleep(duration)
    yb.stop()

while True:
    print("Input command:")
    command = input()
    print("Input duration in seconds (int):")
    duration = int(input())
    manual_command(command, duration)

