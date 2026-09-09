import Maix
from Maix import GPIO, I2S, APU
from fpioa_manager import fm
from machine import Timer
import time
import math
import image
import lcd


SECTOR_COUNT = 16
SECTOR_DEGREES = 360.0 / SECTOR_COUNT  # 22.5
TWO_PI = 2 * math.pi
RAD_PER_SECTOR = TWO_PI / SECTOR_COUNT

# Sectors treated as "aligned" -- step_direction is held at 0 here instead of
# always +-1, so the platform settles once it's close to dead-ahead instead of
# perpetually hunting across the 15/0 boundary. Must stay centered on sector 0
# (dead ahead) for the lock check below to make sense.
LOCK_SECTORS = (15, 0, 1)

# Power-gated VAD (voice/signal-activity detection): only treat a reading as a
# real direction if its `power` clears the adaptive noise floor by this
# margin; otherwise it's ambient noise and shouldn't move the stepper or be
# averaged into the direction estimate.
NOISE_FLOOR_WARMUP_SAMPLES = 20  # readings collected before the floor has a valid initial estimate
NOISE_FLOOR_EMA_ALPHA = 0.15     # adaptation rate applied once per "quiet" window (not per-sample --
                                  # see the note by the window-level update below) -- slow enough to
                                  # track ambient drift (e.g. wind) without chasing transient signal
VAD_MARGIN = 3.0                 # power must exceed noise_floor * VAD_MARGIN to count as signal

# Stopping the stepper (step_direction = 0) on a quiet/VAD-gated window --
# confirmed on hardware (even with 3-window hysteresis) to leave the motor
# stopped for a large fraction of total runtime, which on an already slow,
# cheap motor means negligible net rotation over any reasonable test. The
# original baseline never stops the motor for this reason at all -- it always
# recomputes step_direction from whatever the last 10 raw readings implied
# and keeps driving continuously (100% duty cycle), letting stale/noisy
# readings persist through quiet stretches rather than actively holding.
# Matching that: a quiet window here doesn't touch step_direction at all --
# it just coasts on whatever the last confirmed direction commanded, for as
# long as it takes for real signal to return. LOCK_SECTORS (genuine
# alignment/convergence) is the only thing that should ever deliberately stop
# the motor -- that's a real improvement over the baseline, not something to
# roll back.

# Saturation/clip protection: loud sources up close (e.g. a drone/engine) can
# clip the ADC front-end, corrupting both DOA and (later) classification.
# APU.voc_get_saturation_counter()'s raw 32-bit value is PACKED, per the
# vendor apu.c comment ("high 16 bit is counter, low 16 bit is total"): the
# high 16 bits are the actual clipped-sample count, the low 16 bits are the
# total samples counted since the last reset. Comparing the raw combined
# value against a small threshold (an earlier version of this code did that)
# mostly ends up comparing against the harmless "total" half, since the clip
# half is usually 0 -- confirmed on hardware: it tripped on nearly every
# window regardless of real clipping, collapsing gain to the floor within
# seconds of every boot. Unpack it and gate on an actual clip *rate* instead.
CHANNELS_MASK = 0x3F      # matches init_bf()'s own default channel-enable mask (6 ring mics) --
                          # not a new choice here, just what's already active; only gain changes
INITIAL_GAIN = 1 << 10    # matches APU_AUDIO_GAIN_TEST, the default init_bf() already applies
MIN_GAIN = 1 << 6         # floor -- don't let auto-backoff collapse gain to near-zero
GAIN_BACKOFF_FACTOR = 0.7
SATURATION_CLIP_RATE_LIMIT = 0.05  # trip if >5% of a window's samples clipped -- empirical
                                    # starting point, revisit once real clip-rate behavior is
                                    # observed on hardware under a genuinely loud/close source


def circular_mean_sector(directions, weights=None, round_result=True):
    """Circular mean of a list of sector positions (0..SECTOR_COUNT, floats or
    ints -- e.g. the sub-sector-interpolated values from subsector_interpolate()
    below), returned as a sector position in the same range.

    Sector indices are angles (each step is 22.5 deg around a circle), not a
    linear scale -- a plain arithmetic mean of e.g. [15, 0] gives 7.5 (~169
    deg), which is the wrong side of the circle entirely; the sound is
    actually coming from ~349 deg (sector 15.5). This averages on the unit
    circle (mean of sin/cos, then atan2 back to an angle) so the 15->0
    wraparound is handled correctly.

    `weights`, if given, must be a same-length list of non-negative numbers
    (e.g. each reading's excess power over the noise floor) -- an unweighted
    average over a whole window gives a near-silent/ambient reading exactly
    as much say as a loud, on-target one, which dilutes/washes out the real
    signal whenever only a few readings in the window actually came from the
    source. Weighting lets every raw reading still contribute (so the result
    stays smooth over the full window, unlike gating individual samples
    in/out) while letting the loud, informative readings actually dominate.

    round_result=True (default) rounds to the nearest integer sector, for
    callers that need a discrete sector (e.g. the stepper control logic).
    Pass False to keep the continuous sub-sector value (e.g. for display).
    """
    sum_sin = 0.0
    sum_cos = 0.0
    if weights is None:
        for d in directions:
            angle = d * RAD_PER_SECTOR
            sum_sin += math.sin(angle)
            sum_cos += math.cos(angle)
    else:
        for d, w in zip(directions, weights):
            angle = d * RAD_PER_SECTOR
            sum_sin += w * math.sin(angle)
            sum_cos += w * math.cos(angle)
    mean_angle = math.atan2(sum_sin, sum_cos)
    if mean_angle < 0:
        mean_angle += TWO_PI
    result = mean_angle / RAD_PER_SECTOR
    if round_result:
        return int(round(result)) % SECTOR_COUNT
    return result % SECTOR_COUNT


def subsector_interpolate(sector, sector_power):
    """Estimate a continuous sub-sector angle around the argmax `sector`
    using quadratic (parabolic) interpolation over its two neighbors' power
    in `sector_power` (the 16-element per-sector energy list now returned by
    APU.get_direction()), instead of trusting the raw 22.5-deg-quantized
    sector index.

    Standard 3-point parabolic peak interpolation: fit a parabola through
    (sector-1, p_left), (sector, p_center), (sector+1, p_right) and take its
    vertex offset from `sector`, in units of sectors (nominal range
    [-0.5, 0.5]).
    """
    left = sector_power[(sector - 1) % SECTOR_COUNT]
    center = sector_power[sector]
    right = sector_power[(sector + 1) % SECTOR_COUNT]
    denom = left - 2 * center + right
    if denom == 0:
        # Flat or degenerate samples (e.g. a near-silent window) -- no usable
        # peak shape to interpolate, fall back to the raw sector.
        return float(sector)
    offset = 0.5 * (left - right) / denom
    # A well-formed peak keeps the vertex within +-0.5 sectors of the argmax.
    # A larger offset means center isn't actually the local max (noisy/edge
    # case) -- clamp instead of extrapolating past the neighboring sectors.
    if offset < -0.5:
        offset = -0.5
    elif offset > 0.5:
        offset = 0.5
    return (sector + offset) % SECTOR_COUNT

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

# Saturation counter starts in an unknown state after init_bf() runs its own
# startup sequence -- clear it so the first monitoring window reflects only
# what happens after this point.
APU.voc_reset_saturation_counter()
current_gain = INITIAL_GAIN

update_treshould = 0
time.sleep_ms(500)
img = image.Image()

tim.start()

directions = [] # Store the last 10 raw sub-sector readings (always -- see VAD note below)
powers = []      # Parallel list of each reading's `power`, for the window-level VAD gate

# Adaptive noise floor for the power-gated VAD. Starts uncalibrated (None) and
# is seeded from the average of the first NOISE_FLOOR_WARMUP_SAMPLES readings
# -- deliberately not seeded from a single reading (could be an unlucky loud
# or silent sample) or a fixed constant (ambient level varies by environment
# and drifts, e.g. outdoor wind).
noise_floor = None
noise_floor_warmup = []

while True:

    # APU.get_direction() now returns a 6th element, sector_power (16-int list,
    # one entry per direction sector, one per 22.5-deg sector) -- used below by
    # subsector_interpolate() to get a continuous angle instead of raw 22.5-deg
    # steps.
    detect_dir, power, voc_dir, samples, voc_samples, sector_power = APU.get_direction()

    if noise_floor is None:
        # Still calibrating -- collect samples, don't gate/steer yet.
        noise_floor_warmup.append(power)
        if len(noise_floor_warmup) >= NOISE_FLOOR_WARMUP_SAMPLES:
            # Median, not mean -- the very first reads after boot can be a huge
            # startup transient (observed on hardware: a multi-million seed that
            # then took dozens of seconds to grind back down via the EMA below).
            # A plain mean isn't robust to a couple of extreme outlier samples;
            # the median is, as long as fewer than half the warmup samples are
            # affected.
            noise_floor = sorted(noise_floor_warmup)[len(noise_floor_warmup) // 2]

    # Note: the noise floor is NOT adapted here per-sample anymore -- see the
    # window-level update below. Adapting per-sample (against a permissive
    # per-sample threshold) let the floor chase a genuinely active source
    # upward: individual samples inside a real, sustained signal easily read
    # below noise_floor*VAD_MARGIN too, especially once the floor had already
    # crept up a little, so the floor kept climbing to meet the signal instead
    # of tracking true ambient silence -- confirmed on hardware: noise_floor
    # rose continuously through several straight "tracking" windows, and
    # eventually required an enormous power spike to ever register as signal
    # again. Only ever adapting from whole windows the system judged quiet
    # breaks that feedback loop.

    # Always accumulate every raw reading, regardless of this single sample's
    # power -- matches the original baseline's behavior (it averaged all 10
    # raw reads unconditionally). VAD gating happens below at the *window*
    # level (average power over the whole window vs. threshold), not here
    # per-sample: gating per-sample left as few as 0-2 (often noisy/outlier)
    # readings to average per window, which produced an unsmoothed, jumpy
    # direction estimate instead of the intended smooth tracking -- averaging
    # over all 10 reads is what actually produces a stable estimate.
    directions.append(subsector_interpolate(detect_dir, sector_power))
    powers.append(power)

    if update_treshould == 10:

        window_avg_power = sum(powers) / len(powers)
        window_has_signal = (noise_floor is not None) and (window_avg_power >= noise_floor * VAD_MARGIN)
        window_directions = directions
        window_powers = powers
        directions = []
        powers = []

        if noise_floor is None:
            print("Calibrating noise floor... ({}/{})".format(len(noise_floor_warmup), NOISE_FLOOR_WARMUP_SAMPLES))
        elif window_has_signal:
            # Weight each reading by its excess power over the noise floor,
            # so the (usually few) loud, on-target readings in the window
            # dominate the average instead of being diluted by the (usually
            # more numerous) near-ambient ones -- an unweighted average over
            # the full window gave every reading equal say regardless of
            # whether it actually came from the source, which produced a
            # smooth but wrong (noise-dominated) estimate.
            weights = [max(0.0, p - noise_floor) for p in window_powers]
            # Continuous (sub-sector) value for display/degrees; a rounded
            # sector for the stepper control logic below, which only needs a
            # discrete "which way to turn" decision.
            direction_f = circular_mean_sector(window_directions, weights=weights, round_result=False)
            direction = int(round(direction_f)) % SECTOR_COUNT

            if direction in LOCK_SECTORS:
                step_direction = 0
                lock_state = "locked"
            elif direction < 8:
                step_direction = 1
                lock_state = "tracking"
            else:
                step_direction = -1
                lock_state = "tracking"

            # Point the hardware beamformer at the confirmed direction, so
            # voc_samples becomes a beamformed, SNR-boosted signal aimed at
            # the target instead of a raw omni sum -- this is what
            # classification (§2) should consume.
            APU.enable_voice_output(direction)

            # Convert direction to degrees (each step is 22.5 degrees)
            degrees = direction_f * SECTOR_DEGREES
            APU.set_led(int(degrees), 2, 0)
            print("Detected {} sound from: {:.1f}° (sector {}) - avg_power={:.0f} [{}] noise_floor={:.0f}".format(
                voc_dir, degrees, direction, window_avg_power, lock_state, noise_floor))
        else:
            # This window's average power didn't clear the noise floor.
            # Voice-output beamforming stops immediately -- no motor-momentum
            # concern there, and we no longer have signal evidence for the
            # direction it was pointed at. step_direction is deliberately left
            # untouched: matching the baseline's actual behavior (it never
            # stops the motor at all), the stepper just keeps doing whatever
            # it was last confirmed to do until real signal returns, rather
            # than actively holding on a quiet window. LOCK_SECTORS above is
            # the only place that intentionally stops it.
            APU.disable_voice_output()
            # Safe to adapt here: the *whole window* was judged quiet, so this
            # can't be the source of the upward-chasing feedback loop the old
            # per-sample update caused. (noise_floor is guaranteed set by this
            # point -- the `noise_floor is None` branch above already handled
            # the still-calibrating case.)
            noise_floor += (window_avg_power - noise_floor) * NOISE_FLOOR_EMA_ALPHA
            print("No signal: avg_power={:.0f} < noise_floor={:.0f} * {} -- coasting".format(
                window_avg_power, noise_floor, VAD_MARGIN))

        update_treshould = 0

        # Saturation/clip check -- runs every window regardless of the VAD
        # gate above, since clipping is a front-end/hardware condition, not
        # something that depends on whether this particular window had a
        # confirmed direction.
        raw_sat = APU.voc_get_saturation_counter()
        clip_count = (raw_sat >> 16) & 0xFFFF
        total_count = raw_sat & 0xFFFF
        if total_count > 0:
            clip_rate = clip_count / total_count
            if clip_rate > SATURATION_CLIP_RATE_LIMIT and current_gain > MIN_GAIN:
                current_gain = max(MIN_GAIN, int(current_gain * GAIN_BACKOFF_FACTOR))
                APU.init_apu(current_gain, CHANNELS_MASK)
                # "%" presentation type ({:.1%}) isn't reliably supported by
                # MicroPython's reduced str.format() -- compute the percentage
                # manually instead.
                print("Clip rate {:.1f}% ({}/{}) -- reduced gain to {}".format(
                    clip_rate * 100.0, clip_count, total_count, current_gain))
        APU.voc_reset_saturation_counter()

        # Visualize (runs every window regardless of VAD gate, using this
        # cycle's latest samples)
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
