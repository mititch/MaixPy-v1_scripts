from Maix import MIC_ARRAY, APU
import lcd

lcd.init()
#mic.init()
MIC_ARRAY.init(i2s_d0=23, i2s_d1=22, i2s_d2=21, i2s_d3=20, i2s_ws=19, i2s_sclk=18, sk9822_dat=24, sk9822_clk=25)

#APU.print_settings();

fir_coeffs = [
    0x0000, 0x0000, 0x0000, 0x0000,  # High-pass section
    0x0200, 0x0600, 0x0A00, 0x1000,  # Band-pass section
    0x2000, 0x3000, 0x2000,          # Peak
    0x1000, 0x0A00, 0x0600, 0x0200,  # Band-pass section
    0x0000                           # DC blocking
]


fir_neg = [
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
]

#APU.set_dir_pre_fir(fir_neg)
#APU.set_dir_post_fir(fir_neg)
#APU.configure_direction(4.0, 6, 1)
#APU.reset()

#APU.print_settings();

while True:
    imga = MIC_ARRAY.get_map()
    b = MIC_ARRAY.get_dir(imga)
    a = MIC_ARRAY.set_led(b,(0,0,255))
    imgb = imga.resize(160,160)
    imgc = imgb.to_rainbow(1)
    a = lcd.display(imgc)
    #APU.reset()
    #APU.configure_direction(4.0, 1, 0)
    #APU.init_fpioa(23, 22, 21, 20, 19, 18)
MIC_ARRAY.deinit()
