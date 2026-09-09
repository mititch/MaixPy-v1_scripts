# Unified mic-array streaming app: mode-switchable from the PC (no
# redeploying scripts -- see stream_proto.py's command frames), covering:
#
#   MODE_MICS -- raw I2S, all 4 data lines (8 slots / 7 real mics):
#     - per-slot RMS every block (TYPE_MIC_LEVELS) -- "strength of each mic"
#     - full-quality mono audio for ONE selected slot (TYPE_AUDIO_RAW) --
#       "switch between microphones to record/listen"
#     - PC-side FFT on that selected slot's stream (no on-device FFT --
#       see the k210-chip-hardware skill's dma-and-resource-conflicts.md:
#       hardware FFT claims the same DMA channels this already contends
#       with, for no benefit when the PC has idle CPU + numpy)
#
#   MODE_APU -- the APU beamformer: direction, sector_power[16], saturation
#     (TYPE_APU_TELEMETRY), and FIR/gain tuning from the PC (CMD_SET_FIR /
#     CMD_SET_GAIN). Beamformed *audio* forwarding is NOT implemented here --
#     get_direction()'s samples/voc_samples come back as ~1000 boxed
#     MicroPython ints per call; packing that every ~12ms is the GC-thrash
#     problem the plan's Phase 3 (a zero-copy bytearray read path in
#     lib_apu.c/Maix_apu.c) exists to fix. Telemetry-only until then.
#
# IMPORTANT: disconnect the MaixPy IDE before running this -- see README.md.
#
# UNVERIFIED ON HARDWARE: the MODE_MICS per-block Python loop (de-interleave
# 8 channels + accumulate RMS + extract the selected channel) is real-time
# critical -- if it takes longer than one block's capture time
# (SAMPLES_PER_BLOCK_MICS / SAMPLE_RATE_MICS seconds), samples are lost
# between blocks (see the k210-chip-hardware skill's references/i2s-audio.md
# on why gapless capture requires processing to keep up). This project has
# previously spent real effort chasing a regression that looked fine in
# console logs but only broke on real hardware -- profile this loop on the
# actual board before trusting it, and shrink SAMPLES_PER_BLOCK_MICS /
# raise SAMPLE_RATE_MICS's divisor only after confirming headroom.

import math
import ustruct
from machine import UART
import uos
from fpioa_manager import fm
from Maix import I2S, APU

import stream_proto as proto

# ---- user settings ----
CONSOLE_UART = UART.UART1
CONSOLE_TX_PIN = 10       # spare pins per hardware/demo_uart_loop.py -- outside
CONSOLE_RX_PIN = 11       # the mic-array hat's 18-25 range
CONSOLE_BAUDRATE = 115200
STREAM_BAUDRATE = 1500000  # see README.md's Phase 1 verification for how to
                           # find the real ceiling for a given cable/adapter

# Sipeed 6+1 mic-array hat's default I2S0 pinout
I2S_D0_PIN, I2S_D1_PIN, I2S_D2_PIN, I2S_D3_PIN = 23, 22, 21, 20
I2S_WS_PIN = 19
I2S_SCLK_PIN = 18

# MODE_MICS: all 4 data lines = 8 raw slots (7 populated mics, per the
# 6+1 array). 8kHz/1024-sample blocks (128ms/block) chosen for hot-loop
# headroom margin -- see the UNVERIFIED note above before raising these.
SAMPLE_RATE_MICS = 8000
SAMPLES_PER_BLOCK_MICS = 1024
NUM_SLOTS = 8
WORDS_PER_BLOCK = SAMPLES_PER_BLOCK_MICS * NUM_SLOTS
FRAME_UNPACK_FMT = '<%dh' % (NUM_SLOTS * 2)  # low+high i16 half per 32-bit word, x8 slots
FRAME_BYTES = NUM_SLOTS * 4
AUDIO_FMT = '<%dh' % SAMPLES_PER_BLOCK_MICS

# MODE_APU
APU_GAIN_DEFAULT = 1024   # Q1.10 unity -- the shipped apu_direction_demo.py's
                           # gain=96 looks like an unintended ~20dB attenuation
APU_CHANNEL_MASK = 0x7F   # 7 channels, matching configure_direction(.., 6, True)

# ---- move the REPL off the USB UART, before claiming it for streaming ----
usb_uart = UART.repl_uart()  # grab UARTHS while it's still the REPL's UART

fm.register(CONSOLE_TX_PIN, fm.fpioa.UART1_TX, force=True)
fm.register(CONSOLE_RX_PIN, fm.fpioa.UART1_RX, force=True)
console = UART(CONSOLE_UART, CONSOLE_BAUDRATE, 8, None, 1, timeout=1000, read_buf_len=256)
uos.set_REPLio(console)  # NOT machine.UART.set_repl_uart() -- see stream_proto.py's plan notes
print("console moved to UART1 (pins %d/%d @ %d baud)" % (CONSOLE_TX_PIN, CONSOLE_RX_PIN, CONSOLE_BAUDRATE))

usb_uart.init(baudrate=STREAM_BAUDRATE, bits=8, parity=None, stop=1)
link = proto.FrameWriter(usb_uart)
cmdreader = proto.CommandFrameReader(usb_uart)

# ---- shared state, mutated by handle_command() ----
current_mode = proto.MODE_MICS
mode_change_requested = False
pending_mode = None
selected_mic = 0
selected_dir = 0
apu_gain = APU_GAIN_DEFAULT
apu_ready = False  # True once init_clock/init_fpioa/init_plic have run (once ever)
rx = None          # the I2S object, live only while in MODE_MICS


def poll_and_apply_commands():
    for type_id, payload in cmdreader.poll():
        handle_command(type_id, payload)


def handle_command(type_id, payload):
    global pending_mode, mode_change_requested, selected_mic, selected_dir, apu_gain
    if type_id == proto.CMD_SET_MODE and len(payload) == 1:
        m = payload[0]
        if m in (proto.MODE_MICS, proto.MODE_APU) and m != current_mode:
            pending_mode = m
            mode_change_requested = True
    elif type_id == proto.CMD_SELECT_MIC and len(payload) == 1:
        m = payload[0]
        if 0 <= m < NUM_SLOTS:
            selected_mic = m
            link.status("selected mic %d" % m)
    elif type_id == proto.CMD_SELECT_DIR and len(payload) == 1:
        d = payload[0]
        if 0 <= d < 16:
            selected_dir = d
    elif type_id == proto.CMD_SET_FIR and len(payload) == 35:
        apply_fir(payload)
    elif type_id == proto.CMD_SET_GAIN and len(payload) == 2:
        (apu_gain,) = ustruct.unpack('>H', payload)
        if current_mode == proto.MODE_APU:
            APU.init_apu(apu_gain, APU_CHANNEL_MASK)
        link.status("gain set to %d" % apu_gain)


def apply_fir(payload):
    if not apu_ready:
        link.status("FIR requires MODE_APU at least once before it can be applied")
        return
    target = payload[0]
    coefs = list(ustruct.unpack('>17H', payload[1:]))
    if target == proto.FIR_TARGET_DIR_PRE:
        APU.set_dir_pre_fir(coefs)
    elif target == proto.FIR_TARGET_DIR_POST:
        APU.set_dir_post_fir(coefs)
    elif target == proto.FIR_TARGET_VOICE_PRE:
        APU.set_voice_pre_fir(coefs)
    elif target == proto.FIR_TARGET_VOICE_POST:
        APU.set_voice_post_fir(coefs)
    else:
        return
    link.status("FIR target=%d applied" % target)


# ---- MODE_MICS ----

def enter_mics_mode():
    global rx
    fm.register(I2S_D0_PIN, fm.fpioa.I2S0_IN_D0, force=True)
    fm.register(I2S_D1_PIN, fm.fpioa.I2S0_IN_D1, force=True)
    fm.register(I2S_D2_PIN, fm.fpioa.I2S0_IN_D2, force=True)
    fm.register(I2S_D3_PIN, fm.fpioa.I2S0_IN_D3, force=True)
    fm.register(I2S_WS_PIN, fm.fpioa.I2S0_WS, force=True)
    fm.register(I2S_SCLK_PIN, fm.fpioa.I2S0_SCLK, force=True)

    rx = I2S(I2S.DEVICE_0, sample_points=WORDS_PER_BLOCK)
    # Configure all 4 data lines together, in one pass -- each channel_config()
    # call resets the whole I2S0 instance, so configuring incrementally would
    # wipe the previous channel's setup (see the k210-chip-hardware skill).
    for ch in (rx.CHANNEL_0, rx.CHANNEL_1, rx.CHANNEL_2, rx.CHANNEL_3):
        rx.channel_config(ch, rx.RECEIVER, resolution=I2S.RESOLUTION_16_BIT,
                           cycles=I2S.SCLK_CYCLES_32, align_mode=I2S.STANDARD_MODE)
    rx.set_sample_rate(SAMPLE_RATE_MICS)
    link.status("mode=MICS rate=%d slots=%d block=%d" %
                (SAMPLE_RATE_MICS, NUM_SLOTS, SAMPLES_PER_BLOCK_MICS))


def process_mics_block(block):
    raw = block.to_bytes()
    sumsq = [0] * NUM_SLOTS
    out = []
    off = 0
    sel = selected_mic
    unpack_from = ustruct.unpack_from
    fmt = FRAME_UNPACK_FMT
    stride = FRAME_BYTES
    for _ in range(SAMPLES_PER_BLOCK_MICS):
        vals = unpack_from(fmt, raw, off)
        off += stride
        for ch in range(NUM_SLOTS):
            s = vals[2 * ch]
            sumsq[ch] += s * s
        out.append(vals[2 * sel])

    levels = ustruct.pack('<8f', *(
        math.sqrt(sumsq[ch] / SAMPLES_PER_BLOCK_MICS) for ch in range(NUM_SLOTS)))
    link.send(proto.TYPE_MIC_LEVELS, levels)
    link.send(proto.TYPE_AUDIO_RAW, ustruct.pack(AUDIO_FMT, *out))


def run_mics_mode():
    global mode_change_requested
    pending = None
    while not mode_change_requested:
        poll_and_apply_commands()
        if mode_change_requested:
            break
        tmp = rx.record(WORDS_PER_BLOCK)
        if pending is not None:
            process_mics_block(pending)
        rx.wait_record()
        pending = tmp


# ---- MODE_APU ----

def enter_apu_mode():
    global apu_ready
    if not apu_ready:
        APU.init_clock(45158400)
        APU.init_fpioa(I2S_D0_PIN, I2S_D1_PIN, I2S_D2_PIN, I2S_D3_PIN, I2S_WS_PIN, I2S_SCLK_PIN)
        APU.init_plic(4)
        apu_ready = True
    # init_i2s() resets I2S0 (see enter_mics_mode's comment) -- always redo
    # this and everything downstream of it when (re-)entering APU mode,
    # since a prior MODE_MICS session may have reconfigured I2S0 since.
    APU.init_i2s(44100)
    APU.init_apu(apu_gain, APU_CHANNEL_MASK)
    APU.configure_direction(4.0, 6, True)
    APU.start_direction_detection()
    link.status("mode=APU")


def run_apu_mode():
    global mode_change_requested
    while not mode_change_requested:
        poll_and_apply_commands()
        if mode_change_requested:
            break
        direction, power, voc_dir, _samples, _voc_samples, sector_power = APU.get_direction()
        raw_sat = APU.voc_get_saturation_counter()
        clips = (raw_sat >> 16) & 0xFFFF
        total = raw_sat & 0xFFFF
        # *sector_power must be the LAST argument: this MicroPython port's
        # parser only allows keyword args after a starred unpack in a call
        # (unlike CPython 3.5+/PEP 448, which also allows more positional
        # args) -- "SyntaxError: non-keyword arg after */**" otherwise.
        payload = ustruct.pack('<BiBII16i', direction, power, voc_dir,
                                clips, total, *sector_power)
        link.send(proto.TYPE_APU_TELEMETRY, payload)


# ---- main dispatch ----

def main():
    global current_mode, pending_mode, mode_change_requested
    enter_mics_mode()
    try:
        while True:
            mode_change_requested = False
            if current_mode == proto.MODE_MICS:
                run_mics_mode()
            else:
                run_apu_mode()
            if pending_mode is not None:
                current_mode = pending_mode
                pending_mode = None
                if current_mode == proto.MODE_MICS:
                    enter_mics_mode()
                else:
                    enter_apu_mode()
    except KeyboardInterrupt:
        pass
    finally:
        link.status("device_app: stopped")
        print("stopped")


main()
