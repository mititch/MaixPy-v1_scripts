# Framed binary protocol for streaming data from the K210 to a PC over the
# board's USB-UART bridge (UARTHS), once the REPL has been moved off it --
# see device_app.py. Full duplex: the same physical link carries
# device->host data/telemetry frames AND host->device command frames, using
# the identical wire format in both directions (only the type-id ranges
# differ, purely for readability -- there's no actual collision risk since
# TX/RX are separate wires/queues).
#
# Frame layout (big-endian):
#   magic(2)=0xAA55 | type(1) | seq(1, wraps 0-255) | len(2) | payload(len) | crc16(2)
#
# `seq` is a rolling per-frame counter (per direction) the receiver uses to
# detect dropped frames. `crc16` covers the payload only.

import ustruct

MAGIC = 0xAA55

# ---- device -> host frame types ----
TYPE_AUDIO_RAW = 0x01      # mono int16 PCM samples, one selected mic (MODE_MICS)
TYPE_STATUS = 0x02         # short UTF-8 text -- diagnostics, once the REPL is gone
TYPE_APU_TELEMETRY = 0x03  # direction / power / voc_dir / sector_power / saturation
TYPE_MIC_LEVELS = 0x04     # per-slot RMS, all 8 raw channels, every MODE_MICS block

# ---- host -> device command types ----
CMD_SET_MODE = 0x10     # payload: 1 byte, MODE_MICS or MODE_APU
CMD_SELECT_MIC = 0x11   # payload: 1 byte, raw channel slot 0-7 (MODE_MICS)
CMD_SELECT_DIR = 0x12   # payload: 1 byte, direction 0-15 (reserved for Phase 3 beam steering)
CMD_SET_FIR = 0x13      # payload: 1 byte target + 17 x uint16 big-endian taps (MODE_APU only)
CMD_SET_GAIN = 0x14     # payload: 1 x uint16 big-endian, Q1.10 APU gain (MODE_APU only)

MODE_MICS = 0
MODE_APU = 1

FIR_TARGET_DIR_PRE = 0
FIR_TARGET_DIR_POST = 1
FIR_TARGET_VOICE_PRE = 2
FIR_TARGET_VOICE_POST = 3

CRC16_TABLE = (
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601,
    0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0,
    0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81,
    0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941,
    0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01,
    0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0,
    0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081,
    0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00,
    0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
    0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
    0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41,
    0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700,
    0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0,
    0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281,
    0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01,
    0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1,
    0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80,
    0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101,
    0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
    0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481,
    0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801,
    0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1,
    0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581,
    0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
    0x4100, 0x81C1, 0x8081, 0x4040)


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc


class FrameWriter:
    """Writes framed messages straight to a UART. One instance per link."""

    def __init__(self, uart):
        self.uart = uart
        self._seq = 0
        self.frames_sent = 0

    def send(self, type_id, payload):
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        header = ustruct.pack('>HBBH', MAGIC, type_id, seq, len(payload))
        crc = ustruct.pack('>H', crc16(payload))
        # Three sequential writes, not one concatenated buffer: UARTHS is
        # CPU-polled with no DMA, so there's nothing that can interleave
        # between these calls in single-threaded MicroPython, and this
        # avoids an extra full-size copy of a potentially large audio
        # payload on every frame.
        self.uart.write(header)
        self.uart.write(payload)
        self.uart.write(crc)
        self.frames_sent += 1
        return seq

    def status(self, msg):
        self.send(TYPE_STATUS, msg.encode('utf-8'))


class CommandFrameReader:
    """Incremental, non-blocking reader for host->device command frames,
    sharing FrameWriter's wire format. The device must never block waiting
    for host input mid capture-loop, so this accumulates whatever bytes are
    currently available into a small buffer and only completes a frame once
    enough bytes have arrived -- call poll() once per capture block.

    Kept deliberately simple (buffer + rescan) rather than a formal
    byte-by-byte state machine: command frames are tiny and rare (one per
    UI click), so buffer growth and the linear magic-byte scan are cheap
    regardless.
    """

    def __init__(self, uart):
        self.uart = uart
        self._buf = bytearray()

    def poll(self):
        """Returns a list of (type_id, payload) for every frame completed
        since the last call (usually 0 or 1)."""
        n = self.uart.any()
        if n:
            chunk = self.uart.read(n)
            if chunk:
                self._buf.extend(chunk)

        out = []
        while True:
            idx = self._find_magic()
            if idx is None:
                if len(self._buf) > 1:
                    del self._buf[:-1]  # keep a possible half-magic byte
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 6:
                break  # header incomplete, wait for more bytes next poll()
            type_id, _seq, length = ustruct.unpack('>BBH', self._buf[2:6])
            total = 6 + length + 2
            if len(self._buf) < total:
                break  # payload/crc incomplete, wait for more bytes
            payload = bytes(self._buf[6:6 + length])
            (crc_recv,) = ustruct.unpack('>H', self._buf[6 + length:total])
            del self._buf[:total]
            if crc16(payload) == crc_recv:
                out.append((type_id, payload))
            # loop again in case more than one frame was already buffered
        return out

    def _find_magic(self):
        hi = (MAGIC >> 8) & 0xFF
        lo = MAGIC & 0xFF
        b = self._buf
        for i in range(len(b) - 1):
            if b[i] == hi and b[i + 1] == lo:
                return i
        return None
