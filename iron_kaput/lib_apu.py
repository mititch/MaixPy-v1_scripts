import Maix
from Maix import GPIO, I2S, APU
from fpioa_manager import fm
import time
import image
import lcd


# close WiFi
fm.register(8,  fm.fpioa.GPIO0)
wifi_en=GPIO(GPIO.GPIO0,GPIO.OUT)
wifi_en.value(0)

# init LCD
bg = (236, 36, 36)
lcd.init(freq=15000000)

print("Init:")

time.sleep_ms(500)

APU.init_all(23, 22, 21, 20, 19, 18)

time.sleep_ms(500)
print("Init LED")

APU.init_led(24, 25)

time.sleep_ms(500)

print("Run:")

show = 0

time.sleep_ms(500)

img = image.Image()

while True:
    # APU.get_direction() now returns a 6th element, sector_power (16-int list, one entry per
    # direction sector) -- added for future sub-sector interpolation. Not used by this script yet;
    # accepted here only so the unpack doesn't fail.
    direction, power, voc_dir, samples, voc_samples, sector_power = APU.get_direction()
    # Convert direction to degrees (each step is 22.5 degrees)
    if show == 10:
        degrees = direction * 22.5
        APU.set_led(int(degrees), 2, 0)
        print("Detected {} sound from: {:.1f}° (sector {}) - {}".format(voc_dir, degrees, direction, power))
        show = 0

        # Visualize
        img.clear()
        for i in range(255):
            #print(samples[i])
            #time.sleep_ms(500)
            high = int(voc_samples[i] / 2)
            if high > 120:
                high = 120
            if high < -120:
                high = -120
            #img.draw_line(i, 120, i, 120 - high, color=(236, 36, 36))
        for i in range(255):
            #print(voc_samples[i])
            #time.sleep_ms(500)
            high = int(voc_samples[255 + i] / 2)
            if high > 120:
                high = 120
            if high < -120:
                high = -120
            img.draw_line(i, 120, i, 120 - high, color=(36, 36, 236))

        lcd.display(img)


    show += 1
