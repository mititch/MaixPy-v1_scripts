#!/usr/bin/env python3
"""Live playback of the K210 mic stream (device: device_app.py, MODE_MICS)
through the PC's speakers/headphones, for whichever mic slot is selected.

Usage:
    python monitor.py COM5 [--baud 1500000] [--mic 0]

A background thread reads and validates frames off the serial link and
pushes decoded mono blocks into a small queue; sounddevice's audio callback
(its own real-time thread) pulls from that queue. Empty queue -> silence
(a gap, not a stutter); full queue -> drop the newest block rather than let
latency grow unbounded.
"""
import argparse
import queue
import sys
import threading
import time

import numpy as np
import sounddevice as sd

from k210_link import K210Link
import wire_format as wf

SAMPLE_RATE = 8000  # must match device_app.py's SAMPLE_RATE_MICS
BLOCK_QUEUE_SIZE = 32


def reader_thread(link, block_queue, stop_event):
    try:
        for type_id, payload in link.frames():
            if stop_event.is_set():
                return
            if type_id != wf.TYPE_AUDIO_RAW:
                continue
            samples = wf.decode_audio_mono(payload).astype(np.float32) / 32768.0
            try:
                block_queue.put(samples, timeout=1.0)
            except queue.Full:
                pass  # drop this block rather than build unbounded latency
    except Exception as exc:
        print("\nreader thread stopped:", exc, file=sys.stderr)
        stop_event.set()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('port')
    ap.add_argument('--baud', type=int, default=1500000)
    ap.add_argument('--mic', type=int, default=None, choices=range(wf.NUM_SLOTS))
    args = ap.parse_args()

    link = K210Link(args.port, baudrate=args.baud)
    link.set_mode(wf.MODE_MICS)
    if args.mic is not None:
        link.select_mic(args.mic)

    block_queue = queue.Queue(maxsize=BLOCK_QUEUE_SIZE)
    stop_event = threading.Event()
    leftover = np.zeros(0, dtype=np.float32)

    def callback(outdata, frames, time_info, status):
        nonlocal leftover
        if status:
            print(status, file=sys.stderr)
        chunks = [leftover]
        have = len(leftover)
        while have < frames:
            try:
                block = block_queue.get_nowait()
            except queue.Empty:
                break
            chunks.append(block)
            have += len(block)
        data = np.concatenate(chunks) if len(chunks) > 1 else chunks[0]
        if len(data) >= frames:
            outdata[:, 0] = data[:frames]
            leftover = data[frames:]
        else:
            outdata[:len(data), 0] = data
            outdata[len(data):, 0] = 0
            leftover = np.zeros(0, dtype=np.float32)

    reader = threading.Thread(target=reader_thread, args=(link, block_queue, stop_event), daemon=True)
    reader.start()

    print("streaming from %s @ %d baud -- Ctrl+C to stop" % (args.port, args.baud))
    try:
        with sd.OutputStream(samplerate=SAMPLE_RATE, channels=1,
                              dtype='float32', blocksize=512, callback=callback):
            while not stop_event.is_set():
                time.sleep(1.0)
                print("\rframes=%d crc_fail=%d seq_gaps=%d queue=%d   " % (
                    link.frames_received, link.crc_failures, link.seq_gaps,
                    block_queue.qsize()), end='', file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        link.close()
        print()


if __name__ == '__main__':
    main()
