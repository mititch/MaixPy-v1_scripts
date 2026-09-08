from Maix import GPIO
from fpioa_manager import fm
from machine import Timer
import time

# Map Maix Bit pins to DRV8833 inputs (adjust as needed)
fm.register(6, fm.fpioa.GPIO1)  # AIN1
fm.register(7, fm.fpioa.GPIO2)  # AIN2
fm.register(8, fm.fpioa.GPIO3)  # BIN1
fm.register(9, fm.fpioa.GPIO4)  # BIN2
fm.register(10, fm.fpioa.GPIO5)  # STBY

# Define GPIO pins for DRV8833
IN1 = GPIO(GPIO.GPIO1, GPIO.OUT)  # DRV8833 AIN1
IN2 = GPIO(GPIO.GPIO2, GPIO.OUT)  # DRV8833 AIN2
IN3 = GPIO(GPIO.GPIO3, GPIO.OUT)  # DRV8833 BIN1
IN4 = GPIO(GPIO.GPIO4, GPIO.OUT)  # DRV8833 BIN2
IN5 = GPIO(GPIO.GPIO5, GPIO.OUT)  # DRV8833 STBY

# 4-step full-step sequence for bipolar stepper (NEMA 17)
step_sequence = [
    [1, 0, 1, 0],  # Step 1: A+ B+
    [0, 1, 1, 0],  # Step 2: A- B+
    [0, 1, 0, 1],  # Step 3: A- B-
    [1, 0, 0, 1],  # Step 4: A+ B-
]

step_index = 0
step_dir = 1  # 1 for forward, -1 for reverse

# Timer callback for stepping

def step_callback(timer):
    global step_index
    step_index = (step_index + step_dir) % 4
    s = step_sequence[step_index]
    IN1.value(s[0])
    IN2.value(s[1])
    IN3.value(s[2])
    IN4.value(s[3])

IN5.value(1)

# Setup timer for stepper (adjust period for speed)
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC,
            period=4000, unit=Timer.UNIT_US, callback=step_callback)

# Example: alternate direction every 2 seconds, run for N cycles, then stop
max_cycles = 5  # Number of direction changes (edit as needed)
cycles = 0

print("START.")

while cycles < max_cycles:
    time.sleep(2)
    step_dir *= -1
    cycles += 1

# Release motor and stop timer
IN1.value(0)
IN2.value(0)
IN3.value(0)
IN4.value(0)
tim.deinit()
print("Motor released after {} cycles.".format(max_cycles))

