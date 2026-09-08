from Maix import GPIO
from machine import Timer
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

# Step sequence for 28BYJ-48
step_sequence = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]


itr = 0
dirr = 1

def on_timer(timer):
    global itr
    global dirr
    itr += dirr
    if itr < 0:
        itr = 7
    elif itr > 7:
        itr = 0
    step = step_sequence[itr]
    IN1.value(step[0])
    IN2.value(step[1])
    IN3.value(step[2])
    IN4.value(step[3])


tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=1000, unit=Timer.UNIT_US, callback=on_timer, arg=on_timer, start=False, priority=1, div=0)

#tim.stop()

print("period:",tim.period())

tim.start()

while True:
    time.sleep(1)
    dirr = -1
    time.sleep(1)
    dirr = 1


def step_motor(steps, delay):
    for _ in range(steps):
        for step in step_sequence:
            IN1.value(step[0])
            IN2.value(step[1])
            IN3.value(step[2])
            IN4.value(step[3])
            time.sleep_ms(delay)

def step_motor_reverse(steps, delay):
    for _ in range(steps):
        for step in step_sequence[::-1]:  # Reverse the step order
            IN1.value(step[0])
            IN2.value(step[1])
            IN3.value(step[2])
            IN4.value(step[3])
            time.sleep_ms(delay)

while True:

    # Rotate motor
    step_motor(708, 2)  # Adjust steps and delay as needed

    # Call the function to rotate in reverse
    step_motor_reverse(708, 2)  # Adjust as needed

#int numSteps = 708;
