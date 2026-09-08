from Maix import APU
import time

def init_apu():
    print("Initializing APU...2")
    # 1. Reset APU first to clear any previous state
    APU.reset()
    time.sleep_ms(100)  # Wait for reset to complete
    print("Initializing APU...3")
    # 2. Initialize APU with I2S pins and high gain
    # Default pin configuration for Maix DOCK board:
    # I2S_D0 = 23, I2S_D1 = 22, I2S_D2 = 21, I2S_D3 = 20
    # I2S_WS = 19, I2S_SCLK = 18
    # Using high gain (96) to improve sensitivity
    APU.init(96, 0xFF, 23, 22, 21, 20, 19, 18)
    time.sleep_ms(100)  # Wait for I2S to stabilize

    # 3. Set optimized FIR coefficients for voice frequency range (300Hz - 3.4kHz)
    # These coefficients form a bandpass filter
    fir_coeffs = [
        0x0000, 0x0000, 0x0000, 0x0000,  # High-pass section
        0x0200, 0x0600, 0x0A00, 0x1000,  # Band-pass section
        0x2000, 0x3000, 0x2000,          # Peak
        0x1000, 0x0A00, 0x0600, 0x0200,  # Band-pass section
        0x0000                           # DC blocking
    ]
    APU.set_dir_pre_fir(fir_coeffs)
    APU.set_dir_post_fir(fir_coeffs)

    # 4. Configure direction detection
    # Parameters for user's mic array:
    # - radius: 4.0 cm (actual mic array radius)
    # - mic_num_a_circle: 6 (six microphones in circle)
    # - center: 1 (has center microphone)
    APU.configure_direction(4.0, 6, 1)

    # 5. Enable APU source mode (1: stereo for better channel separation)
    APU.set_source_mode(1)

def start_detection():
    # Clear any previous state
    APU.dir_clear_ready()
    # Start a new detection cycle
    APU.start_direction_detection()

print("APU Direction Detection Demo")
print("---------------------------")
print("Initializing APU...")

# Initialize APU
init_apu()

print("Starting direction detection...")
print("Please make some noise to help calibrate...")

# Start first detection cycle
start_detection()

# Give time for the buffers to fill
time.sleep_ms(1000)  # Wait 1 second for initial buffer fill

print("Ready! Now listening for sounds...")
print("(Make continuous noise to test detection)")

try:
    last_print = time.ticks_ms()
    consecutive_failures = 0

    while True:
        if APU.dir_is_ready():
            # Get direction result
            result = APU.get_direction()
            # Direction values 0-11 correspond to angles:
            # 0: 0°, 1: 30°, 2: 60°, ..., 11: 330°
            angle = result.direction * 30
            print("\nSound detected!")
            print("Direction: %d (%d°)" % (result.direction, angle))
            print("Confidence: %d" % result.confidence)
            print("Power: %d" % result.power)
            print("-" * 20)

            # Reset failure counter on successful detection
            consecutive_failures = 0

            # Start a new detection cycle
            start_detection()

            # Small delay after detection
            time.sleep_ms(100)
        else:
            consecutive_failures += 1

            # If too many failures, try reinitializing
            if consecutive_failures > 1000:  # About 5 seconds
                print("\nNo detections for a while, reinitializing...")
                init_apu()
                start_detection()
                time.sleep_ms(1000)
                consecutive_failures = 0
                print("Ready again! Make some noise...")

            # Print status message periodically
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) > 5000:
                print("Waiting for sound... (make continuous noise to test)")
                last_print = now

            # Very short delay when no detection
            time.sleep_ms(5)

except KeyboardInterrupt:
    # Clean up
    APU.reset()
    print("\nAPU demo stopped")
