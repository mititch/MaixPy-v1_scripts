import time
from machine import Pin

# Assign GPIO pins for TMC2208 STEP and DIR
STEP_PIN = 10  # Change to your actual pin
DIR_PIN = 11   # Change to your actual pin

STEP = Pin(STEP_PIN, Pin.OUT)
DIR = Pin(DIR_PIN, Pin.OUT)

# Function to move the stepper motor
# steps: number of steps to move
# direction: 1 for forward, 0 for reverse
# delay: time (in seconds) between steps (controls speed)
def stepper_move(steps, direction=1, delay=0.001):
    DIR.value(1 if direction > 0 else 0)
    for _ in range(steps):
        STEP.value(1)
        time.sleep_us(10)  # Short high pulse (check TMC2208 datasheet, min 1us)
        STEP.value(0)
        time.sleep(delay)

# Example usage:

# Move 1000 steps forward
stepper_move(1000, direction=1, delay=0.001)

# Move 1000 steps backward
stepper_move(1000, direction=0, delay=0.001)
