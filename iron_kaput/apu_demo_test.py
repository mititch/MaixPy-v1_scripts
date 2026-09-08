import Maix
from Maix import GPIO, I2S, APU
from fpioa_manager import fm
import time


# close WiFi
fm.register(8,  fm.fpioa.GPIO0)
wifi_en=GPIO(GPIO.GPIO0,GPIO.OUT)
wifi_en.value(0)


print("Clock initialized")

time.sleep_ms(1000)

APU.demo_init(enable_ppl=1, enable_irq=1, reinit_irq=0, reinit_all=1)

time.sleep_ms(1000)
print("Run")
APU.demo_run()
