import Maix
from Maix import GPIO, I2S, APU
from fpioa_manager import fm
from machine import Timer
import time
import image
import lcd



def threshold_filter(data, threshold):
    filtered_data = [data[0]]
    for i in range(1, len(data)):
        if abs(data[i] - filtered_data[-1]) > threshold:
            filtered_data.append(filtered_data[-1])
        else:
            filtered_data.append(data[i])
    return filtered_data

# close WiFi
fm.register(8,  fm.fpioa.GPIO0)
wifi_en=GPIO(GPIO.GPIO0,GPIO.OUT)
wifi_en.value(0)

# register fpioa for step motor controll
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
step_iterration = 0
step_direction = 0
def on_timer(timer):
    global step_iterration
    global step_direction
    if step_direction == 0:
        return
    step_iterration += step_direction
    if step_iterration < 0:
        step_iterration = 7
    elif step_iterration > 7:
        step_iterration = 0
    step_seq = step_sequence[step_iterration]
    IN1.value(step_seq[0])
    IN2.value(step_seq[1])
    IN3.value(step_seq[2])
    IN4.value(step_seq[3])
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=1000, unit=Timer.UNIT_US, callback=on_timer, arg=on_timer, start=False, priority=1, div=0)

# init LCD
bg = (236, 36, 36)
lcd.init(freq=15000000)

# init APU
time.sleep_ms(500)
APU.init_all(23, 22, 21, 20, 19, 18)
time.sleep_ms(500)

# init LED
APU.init_led(24, 25)
time.sleep_ms(500)


update_treshould = 0
time.sleep_ms(500)
img = image.Image()

tim.start()

directions = [] # Store the last 10 directions

while True:

    detect_dir, power, voc_dir, samples, voc_samples = APU.get_direction()

    directions.append(detect_dir)

    if update_treshould == 10:

        filtered_data = threshold_filter(directions, threshold=3)
        direction = round(sum(filtered_data) / len(filtered_data))
        directions = []
        if direction < 8:
            step_direction = 1
        else:
            step_direction = -1

        # Convert direction to degrees (each step is 22.5 degrees)
        degrees = direction * 22.5
        APU.set_led(int(degrees), 2, 0)
        print("Detected {} sound from: {:.1f}° (sector {}) - {}".format(voc_dir, degrees, direction, power))
        update_treshould = 0

        # Visualize
        img.clear()
        for i in range(255):
            high = int(voc_samples[i] / 2)
            if high > 120:
                high = 120
            if high < -120:
                high = -120
            #img.draw_line(i, 120, i, 120 - high, color=(236, 36, 36))
        for i in range(255):
            high = int(voc_samples[255 + i] / 2)
            if high > 120:
                high = 120
            if high < -120:
                high = -120
            img.draw_line(i, 120, i, 120 - high, color=(36, 36, 236))

        lcd.display(img)

    update_treshould += 1
    # time.sleep_ms(10)
