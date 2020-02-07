#!/usr/bin/env python3

import subprocess
import time

from gpiozero import OutputDevice


ON_THRESHOLD = 54  # (degrees Celsius) Fan kicks on at this temperature.
OFF_THRESHOLD = 50  # (degress Celsius) Fan shuts off at this temperature.
SLEEP_INTERVAL = 5  # (seconds) How often we check the core temperature.
GPIO_PIN = 2  # Which GPIO pin you're using to control the fan.


def get_temp():
    """Get the core temperature.

    Run a shell script to get the core temp and parse the output.

    Raises:
        RuntimeError: if response cannot be parsed.

    Returns:
        float: The core temperature in degrees Celsius.
    """
    output = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True)
    temp_str = output.stdout.decode()
    try:
        current_temp = float(temp_str.split('=')[1].split('\'')[0])
        print("Current core temp is: ",current_temp)
        return current_temp
    except (IndexError, ValueError):
        raise RuntimeError('Could not parse temperature output.')

while True:
    get_temp()
    time.sleep(5)
