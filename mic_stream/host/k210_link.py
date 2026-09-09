"""Serial link to the K210 running device_app.py: reads framed
device->host data/telemetry, and sends framed host->device commands, both
using wire_format's shared format/constants.

**Every connection launches device_app.py itself, unconditionally.** This
isn't a design choice so much as a hardware fact discovered the hard way:
opening a fresh serial connection to this board resets it via DTR
regardless of what the host does or doesn't send -- there is no way to
"just attach" to an already-running session from a new connection, since
the act of opening one is itself a reset. Given that, the only correct
behavior is for every connect to own the reset: catch the ~200ms
post-reset window before the board's `main.py` (which may be an unrelated,
possibly-misbehaving script -- it commonly is, in practice) takes over,
upload stream_proto.py + device_app.py fresh over the raw MicroPython REPL
protocol, launch device_app.py, and only then switch to the streaming baud
rate -- all on one continuous, never-closed-and-reopened connection.

This means `mpremote` is no longer needed for the normal dashboard/monitor/
record_wav workflow; it remains useful for manual debugging (a plain REPL,
inspecting the filesystem, etc.) but every host tool here is self-contained.
"""

import base64
import struct
import threading
import time
from pathlib import Path

import wire_format as wf

# stream_proto.py / device_app.py live one directory up from this file
# (mic_stream/host/k210_link.py -> mic_stream/).
_DEVICE_DIR = Path(__file__).resolve().parent.parent


def _reset_and_interrupt(ser, settle=3.0):
    """Pulse DTR (the board's hardware reset line, wired through the USB
    bridge chip) and immediately blast Ctrl-C for `settle` seconds. This is
    the only reliable way found to catch `_boot.py`'s ~200ms pre-main.py
    interrupt window against a `main.py` that doesn't yield control back to
    the VM promptly (observed in practice: an old APU test script stuck in
    a tight C-level retry loop that a single post-reset Ctrl-C, e.g. what
    mpremote sends by default, does not reliably interrupt)."""
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(0.05)
    ser.dtr = False
    end = time.time() + settle
    while time.time() < end:
        ser.write(b'\x03')
        ser.read(4096)


def _enter_raw_repl(ser):
    ser.reset_input_buffer()
    ser.write(b'\x01')
    time.sleep(0.3)
    resp = ser.read(500)
    if b'raw REPL' not in resp:
        raise RuntimeError("failed to enter raw REPL: %r" % resp)


def _reset_and_enter_raw_repl(ser, attempts=3):
    """The full reset+catch+enter-raw-REPL dance sometimes doesn't land on
    the first try (real hardware timing variance, not just a flaky Ctrl-A)
    -- retry the *whole* sequence (fresh DTR pulse and all), not just the
    raw-REPL entry step alone."""
    last_exc = None
    for _ in range(attempts):
        _reset_and_interrupt(ser)
        try:
            _enter_raw_repl(ser)
            return
        except RuntimeError as e:
            last_exc = e
    raise RuntimeError("could not reach raw REPL after %d full reset attempts; last error: %s"
                        % (attempts, last_exc))


def _exec_raw(ser, code_bytes, timeout=10.0):
    ser.write(code_bytes)
    ser.write(b'\x04')
    deadline = time.time() + timeout
    resp = bytearray()
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            resp += chunk
            if resp.count(b'\x04') >= 2:
                break
    return bytes(resp)


def _upload(ser, local_path, remote_name):
    """Write `local_path`'s content to `remote_name` on the board, base64
    over the raw REPL. Uses an explicit close() + os.sync() -- on this
    board's SD-card-backed filesystem, `open(x,'w').write(...)` without an
    explicit close silently fails to flush before the next command runs
    (found the hard way: the write "succeeds" with no exception, but a
    read-back immediately afterward returns 0 bytes)."""
    data = open(local_path, 'rb').read()
    b64 = base64.b64encode(data).decode('ascii')
    code = ("import ubinascii, os\n"
            "_f = open('%s','wb')\n"
            "_f.write(ubinascii.a2b_base64('%s'))\n"
            "_f.close()\n"
            "os.sync()\n"
            "print('WROTE', %d)\n" % (remote_name, b64, len(data))).encode()
    resp = _exec_raw(ser, code)
    expected = ("OKWROTE %d" % len(data)).encode()
    if not resp.startswith(expected):
        raise RuntimeError("upload of %s failed, device said: %r" % (remote_name, resp))


class K210Link:
    """`frames()` yields (type_id, payload) for each valid frame received;
    it resyncs on the magic bytes and *counts* (rather than raises on) CRC
    failures and sequence gaps, so a long-running consumer can keep going
    and report a final error rate. `send_command()` is safe to call from a
    different thread than the one iterating `frames()` -- writes are
    serialized with a lock; pyserial's own read/write are independently
    safe on most backends, but the lock also protects the outgoing seq
    counter."""

    def __init__(self, port, baudrate=1500000, timeout=1.0, launch_baudrate=115200):
        import serial  # deferred so this module can be imported for the
                        # constants/crc16 alone without pyserial installed
        # Open at a plain, conservative baud rate first -- this is the
        # connection that will do the reset/upload/launch dance below --
        # then switch the SAME open handle to the streaming baud rate at
        # the end. Never close and reopen for this: that would just
        # trigger another reset (see module docstring).
        self.ser = serial.Serial(port, baudrate=launch_baudrate, timeout=0.3)
        self._launch_device_app()
        self.ser.timeout = timeout
        self.ser.baudrate = baudrate

        self.frames_received = 0
        self.crc_failures = 0
        self.seq_gaps = 0
        self._last_seq = None
        self._out_seq = 0
        self._write_lock = threading.Lock()

    def _launch_device_app(self):
        _reset_and_enter_raw_repl(self.ser)
        _upload(self.ser, _DEVICE_DIR / "stream_proto.py", "stream_proto.py")
        _upload(self.ser, _DEVICE_DIR / "device_app.py", "device_app.py")
        # This never returns a normal raw-REPL completion response --
        # device_app.py's main() loops forever and moves the REPL to UART1
        # partway through, so from here on this connection only ever sees
        # the framed protocol stream, never REPL output again.
        self.ser.write(b"import device_app\n")
        self.ser.write(b'\x04')
        time.sleep(0.05)

    def close(self):
        self.ser.close()

    def frames(self):
        while True:
            self._sync()
            rest = self._read_exact(4)  # type(1) seq(1) len(2)
            type_id, seq, length = struct.unpack('>BBH', rest)
            payload = self._read_exact(length)
            (crc_received,) = struct.unpack('>H', self._read_exact(2))
            if wf.crc16(payload) != crc_received:
                self.crc_failures += 1
                continue
            if self._last_seq is not None:
                expected = (self._last_seq + 1) & 0xFF
                if seq != expected:
                    self.seq_gaps += (seq - expected) & 0xFF
            self._last_seq = seq
            self.frames_received += 1
            yield type_id, payload

    def send(self, type_id, payload=b''):
        with self._write_lock:
            seq = self._out_seq
            self._out_seq = (self._out_seq + 1) & 0xFF
            self.ser.write(wf.pack_frame(type_id, seq, payload))

    def send_command(self, cmd, payload=b''):
        self.send(cmd, payload)

    def set_mode(self, mode):
        self.send_command(wf.CMD_SET_MODE, bytes([mode]))

    def select_mic(self, slot):
        self.send_command(wf.CMD_SELECT_MIC, bytes([slot]))

    def select_dir(self, direction):
        self.send_command(wf.CMD_SELECT_DIR, bytes([direction]))

    def set_fir(self, target, coefs):
        self.send_command(wf.CMD_SET_FIR, wf.pack_fir_command(target, coefs))

    def set_gain(self, gain):
        self.send_command(wf.CMD_SET_GAIN, struct.pack('>H', gain & 0xFFFF))

    def _sync(self):
        """Block until the next two bytes on the wire are the magic
        sequence, discarding everything before it. This is what lets the
        reader recover after a CRC failure, a serial glitch, or attaching
        mid-stream."""
        prev = self._read_exact(1)[0]
        while True:
            cur = self._read_exact(1)[0]
            if (prev << 8) | cur == wf.MAGIC:
                return
            prev = cur

    def _read_exact(self, n):
        data = bytearray()
        while len(data) < n:
            chunk = self.ser.read(n - len(data))
            if not chunk:
                raise TimeoutError("serial read timed out waiting for %d byte(s)" % n)
            data.extend(chunk)
        return bytes(data)
