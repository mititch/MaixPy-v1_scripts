#!/usr/bin/env python3
"""Record the K210 mic stream (device: device_app.py, MODE_MICS) to a mono
WAV file for whichever mic slot is currently selected, and report the
link's error rate on exit.

Usage:
    python record_wav.py COM5 out.wav [--baud 1500000] [--mic 0] [--seconds 10]

On Linux/macOS, `port` is typically /dev/ttyUSB0 or similar. Ctrl+C stops
early and still finalizes the WAV file. `--mic` sends a CMD_SELECT_MIC
before recording starts; omit it to record whatever the device/dashboard
already has selected.
"""
import argparse
import sys
import time
import wave

from k210_link import K210Link
import wire_format as wf

SAMPLE_RATE = 8000  # must match device_app.py's SAMPLE_RATE_MICS


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('port')
    ap.add_argument('out_wav')
    ap.add_argument('--baud', type=int, default=1500000)
    ap.add_argument('--mic', type=int, default=None, choices=range(wf.NUM_SLOTS),
                     help='select this mic slot (0-7) before recording')
    ap.add_argument('--seconds', type=float, default=None,
                     help='stop automatically after this many seconds (default: run until Ctrl+C)')
    args = ap.parse_args()

    link = K210Link(args.port, baudrate=args.baud)
    link.set_mode(wf.MODE_MICS)
    if args.mic is not None:
        link.select_mic(args.mic)

    wfile = wave.open(args.out_wav, 'wb')
    wfile.setnchannels(1)
    wfile.setsampwidth(2)
    wfile.setframerate(SAMPLE_RATE)

    start = time.time()
    last_report = start
    try:
        for type_id, payload in link.frames():
            if type_id != wf.TYPE_AUDIO_RAW:
                continue
            wfile.writeframes(wf.decode_audio_mono(payload).tobytes())

            now = time.time()
            if now - last_report >= 1.0:
                print("\rframes=%d crc_fail=%d seq_gaps=%d elapsed=%.1fs" % (
                    link.frames_received, link.crc_failures, link.seq_gaps,
                    now - start), end='', file=sys.stderr)
                last_report = now
            if args.seconds is not None and now - start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        wfile.close()
        link.close()
        print()
        print("wrote %s -- frames=%d crc_fail=%d seq_gaps=%d" % (
            args.out_wav, link.frames_received, link.crc_failures, link.seq_gaps))
        if link.crc_failures or link.seq_gaps:
            print("WARNING: link dropped/corrupted data -- try a lower --baud "
                  "on the device (device_app.py's STREAM_BAUDRATE) and here",
                  file=sys.stderr)


if __name__ == '__main__':
    main()
