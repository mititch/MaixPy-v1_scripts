from Maix import GPIO
from fpioa_manager import fm
from machine import Timer, PWM
import time

# === Pin Mapping ===
# Adjust these FPIOA pin numbers as needed for your wiring!
fm.register(6, fm.fpioa.GPIO1)   # DRV8833 AIN1
fm.register(7, fm.fpioa.GPIO2)   # DRV8833 AIN2
fm.register(8, fm.fpioa.GPIO3)   # DRV8833 BIN1
fm.register(9, fm.fpioa.GPIO4)   # DRV8833 BIN2
fm.register(10, fm.fpioa.GPIO5)  # DRV8833 STBY

IN1 = GPIO(GPIO.GPIO1, GPIO.OUT)
IN2 = GPIO(GPIO.GPIO2, GPIO.OUT)
IN3 = GPIO(GPIO.GPIO3, GPIO.OUT)
IN4 = GPIO(GPIO.GPIO4, GPIO.OUT)
STBY = GPIO(GPIO.GPIO5, GPIO.OUT)

# === Step Sequence (Full Step) ===
#step_sequence_my = [
#    [1, 1, 0, 0],
#    [0, 1, 1, 0],
#    [0, 0, 1, 1],
#    [1, 0, 0, 1],
#]

step_sequence_their = [
    (1, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 1),
    (1, 0, 0, 1),
]

step_sequence_hs = [
    [1, 0, 0, 0],  # Step 1: Energize first coil
    [1, 1, 0, 0],  # Step 2: Both coils for transition
    [0, 1, 0, 0],  # Step 3: Energize second coil
    [0, 1, 1, 0],  # Step 4: Both coils for transition
    [0, 0, 1, 0],  # Step 5: Energize third coil
    [0, 0, 1, 1],  # Step 6: Both coils for transition
    [0, 0, 0, 1],  # Step 7: Energize fourth coil
    [1, 0, 0, 1],  # Step 8: Both coils for transition
]

step_sequence = step_sequence_their


# Uncomment below for half-step sequence (smoother, slower)
# step_sequence = [
#     [1, 0, 0, 0],  # A+ only
#     [1, 0, 1, 0],  # A+ B+
#     [0, 0, 1, 0],  # B+ only
#     [0, 1, 1, 0],  # A- B+
#     [0, 1, 0, 0],  # A- only
#     [0, 1, 0, 1],  # A- B-
#     [0, 0, 0, 1],  # B- only
#     [1, 0, 0, 1],  # A+ B-
# ]

step_index = 0
step_dir = -1  # 1 for forward, -1 for reverse

def step_motor(timer):
    global step_index
    step_index = (step_index + step_dir) % len(step_sequence)
    s = step_sequence[step_index]
    IN1.value(s[0])
    IN2.value(s[1])
    IN3.value(s[2])
    IN4.value(s[3])

def release_motor():
    IN1.value(0)
    IN2.value(0)
    IN3.value(0)
    IN4.value(0)
    STBY.value(0)

def enable_motor():
    release_motor()
    STBY.value(1)

# === Main ===
enable_motor()
print("DRV8833 Standby released, starting stepping...")

# Set stepping interval (microseconds)
step_period_us = 4000  # 4ms per step (~250 steps/sec)

# Start timer for continuous stepping
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC,
            period=step_period_us, unit=Timer.UNIT_US, callback=step_motor)

# Example: change direction every 2 seconds, run for 10 cycles, then stop
max_cycles = 20
cycles = 0

try:
    while cycles < max_cycles:
        time.sleep(1)
        step_dir *= -1
        cycles += 1
finally:
    tim.deinit()
    release_motor()
    print("Motor stopped and released.")
