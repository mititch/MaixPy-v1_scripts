from machine import PWM
from Maix import GPIO
import time

from fpioa_manager import fm

fm.register(6, fm.fpioa.GPIO1)
fm.register(7, fm.fpioa.GPIO2)
fm.register(8, fm.fpioa.GPIO3)
fm.register(9, fm.fpioa.GPIO4)



# Define GPIO pins connected to ULN2003
IN1 = GPIO(GPIO.GPIO1,GPIO.OUT)
IN2 = GPIO(GPIO.GPIO2,GPIO.OUT)
IN3 = GPIO(GPIO.GPIO3,GPIO.OUT)
IN4 = GPIO(GPIO.GPIO4,GPIO.OUT)


import time

# Define PWM pins for ULN2003 inputs
IN1 = PWM(0, freq=1000, duty=0)  # Replace 0 with actual GPIO pin number
IN2 = PWM(1, freq=1000, duty=0)  # Replace 1 with actual GPIO pin number
IN3 = PWM(2, freq=1000, duty=0)  # Replace 2 with actual GPIO pin number
IN4 = PWM(3, freq=1000, duty=0)  # Replace 3 with actual GPIO pin number

# Step sequence for PWM control
step_sequence = [
    [100, 0, 0, 0],  # Duty cycles for each input
    [100, 100, 0, 0],
    [0, 100, 0, 0],
    [0, 100, 100, 0],
    [0, 0, 100, 0],
    [0, 0, 100, 100],
    [0, 0, 0, 100],
    [100, 0, 0, 100]
]

def step_motor(steps, delay):
    for _ in range(steps):
        for step in step_sequence:
            IN1.duty(step[0])  # Set duty cycle for IN1
            IN2.duty(step[1])  # Set duty cycle for IN2
            IN3.duty(step[2])  # Set duty cycle for IN3
            IN4.duty(step[3])  # Set duty cycle for IN4
            time.sleep(delay)

# Rotate motor
step_motor(512, 0.002)  # Adjust steps and delay as needed
