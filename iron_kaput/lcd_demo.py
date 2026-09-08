import lcd
import image
import time

bg = (236, 36, 36)
lcd.init(freq=15000000)

i = 0
dir = 1

img = image.Image()

while(True):
    time.sleep_ms(1)
    img.clear()
    img.draw_rectangle(i, 50, 50, 50)
    lcd.display(img)

    if dir:
        i += 5
        if i == 270:
            dir = 0
    else:
        i -= 5
        if i == 0:
            dir = 1
