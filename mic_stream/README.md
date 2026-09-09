# mic_stream — Sipeed 6+1 mic array → PC dashboard over USB

Streams audio and telemetry from the mic array to a PC over the board's USB
port (an onboard USB↔UART bridge chip — the K210 has no native USB device
controller; **which specific bridge chip varies by board/batch, see
"Connecting the board" below** — don't assume CH340 just because that's
what Sipeed's docs and `kflash.py` describe). See
`Acoustic + Visual Source Detection.md`'s prior research and the
`k210-chip-hardware` skill for why the transport is a UART, not real USB,
and why raw I2S capture and the APU beamformer can't run at the same
instant yet.

Kept on its own branch (`mic_stream`), separate from `iron_kaput`.

## What the hardware can actually provide

| You want | Source | Notes |
|---|---|---|
| Direction of sound | APU `sector_power[16]` + `direction` | 22.5° buckets, azimuth only, up to 86 Hz |
| Strength of each mic | Raw I2S, all 4 data lines (7 mics) | Computed on-device (RMS per slot, every block) |
| Switch mics to listen/record | Raw I2S, per-slot selection | One mic at full quality at a time; all 7 levels stream continuously regardless |
| Beamforming | APU `voc_samples` (steered beam) | Telemetry only for now — gapless audio needs the firmware Phase 3 fix (see the plan) |
| FFT / frequency domain | Computed on the PC (numpy) | Deliberately not on-device — avoids yet another claimant on the DMA channels the APU/I2S already contend for |
| FIR filtering | APU 17-tap hardware FIR, 4 paths | **Experimental** — tap fixed-point format is a best-effort guess, not confirmed against a datasheet |
| Clipping/saturation | APU saturation counter | Bit-packed register, unpacked into clip/total counts |

**The one constraint shaping the whole design:** raw-per-mic data and APU
direction/beamforming can't be captured at the same instant yet (shared DMA
channel + shared I2S0 reset — see `dma-and-resource-conflicts.md`). So the
dashboard has **two modes**, switched live from the PC (no redeploying
scripts), not one screen with everything simultaneous — until the firmware
Phase 3 work lands.

## Layout

- **Device** (upload both to the board): `stream_proto.py` (wire protocol),
  `device_app.py` (the app — mode-switchable via commands from the PC).
- **Host**: `host/wire_format.py` (shared constants/decoders),
  `host/k210_link.py` (serial link), `host/dashboard.py` (the GUI — this is
  the main deliverable), `host/record_wav.py` / `host/monitor.py` (CLI
  companions, handy for the verification steps below without needing a
  display), `host/pyproject.toml` + `host/uv.lock` (dependencies, see below).

Dependencies are managed with [uv](https://docs.astral.sh/uv/) (`host/pyproject.toml` + `host/uv.lock`), not plain pip:

```
pip install uv        # one-time, if you don't have it yet
cd host
uv sync
```

`uv sync` also handles a real gotcha on its own: PyQt5 has no wheel for
Python 3.13+ at time of writing (`pyproject.toml` pins
`requires-python = ">=3.10,<3.13"`), so if your system Python is 3.13, `uv`
transparently downloads a compatible 3.12 interpreter into its own managed
store and builds the venv against that — no manual venv/Python-version
juggling. Run host scripts via `uv run python host/dashboard.py ...` (from
`mic_stream/`) or activate `host/.venv` directly.

> **Verified:** `dashboard.py`'s full UI construction (both tabs, all
> widgets, all signal/slot wiring), its frame-handling path fed synthetic
> `TYPE_MIC_LEVELS`/`TYPE_AUDIO_RAW`/`TYPE_APU_TELEMETRY` frames, the
> waveform/FFT/polar refresh methods, tab switching, and the FIR
> design→apply flow were all instantiated and exercised headlessly
> (`QT_QPA_PLATFORM=offscreen`) via a `uv`-managed Python 3.12 venv — this
> is what got PyQt5 actually installable in the first place. What remains
> genuinely unverified is only what needs real hardware or a real display:
> actual rendering/visual layout, and everything in `device_app.py` (see
> its own UNVERIFIED-ON-HARDWARE note).

## Connecting the board

**Physical connection:** a USB-C/micro-USB cable straight from the board to
the PC — that's it, no adapter. The board's USB port is a USB-to-UART
bridge chip (see above: no native USB on the K210), so the PC sees a
**virtual COM port**.

⚠️ **Don't assume which bridge chip without checking.** Sipeed's own
documentation and this firmware's flashing tool (`kflash.py`) describe the
Maix BIT as "CH340/CH552 mode" — but on the actual unit this project was
built and tested against, Device Manager / `usbipd list` showed a
**genuine FTDI FT232** (`USB\VID_0403&PID_6001`, custom string descriptor
`"Sipeed-Debug"`), not a WCH chip at all. Sipeed's own official driver
download (the CH340 one their docs point to) **did not work** on this
board — confirmed by trying it directly. The bridge chip apparently varies
by manufacturing batch/board revision; always check what's actually
enumerating (see below) before hunting for a driver, rather than trusting
either the docs or this README's specific chip name.

- **Windows**: plug in, then Device Manager → look at "Ports (COM & LPT)"
  first (it may already just work — Windows Update sometimes supplies a
  driver automatically). If it instead shows up under "Other devices" /
  "Universal Serial Bus devices" with a warning icon, check the exact
  chip:
  ```powershell
  Get-PnpDevice | Where-Object { $_.Status -eq 'Error' -or $_.FriendlyName -match 'Debug|Serial' } |
    Select-Object FriendlyName, InstanceId, Status
  ```
  The `InstanceId` (`USB\VID_xxxx&PID_xxxx\...`) tells you the real vendor —
  `1A86` = WCH (CH340/CH341/CH343), `0403` = FTDI, `10C4` = Silicon Labs
  (CP210x). Get the driver from that vendor specifically, not by board name.
  - **FTDI, confirmed working path** (this board): download the **Windows
    ARM64** (or plain x64, on a normal PC) package from
    https://ftdichip.com/drivers/vcp-drivers/ — it ships as raw driver
    files (no setup.exe for ARM64), so install it directly:
    ```powershell
    pnputil /add-driver "<extracted-folder>\*.inf" /install
    ```
    (elevation/UAC required). This is the official, WHQL-signed driver —
    no test-signing, no unsigned community driver needed. Confirmed
    working: the device changes from `Status: Error` to two clean entries
    ("USB Serial Converter" + "USB Serial Port (COMn)"), both `Status: OK`,
    and a real COM port appears in
    `HKLM:\HARDWARE\DEVICEMAP\SERIALCOMM` — at which point the MaixPy IDE
    connects normally too.
  - **CH340/CH341, if that's what you actually have**: wch-ic.com's
    CH341SER package — see the git history of this README for the
    ARM64-specific caveats we hit chasing that path on this machine before
    discovering the chip was actually FTDI; skip straight to checking your
    `InstanceId` first to avoid repeating that detour.
- **Linux/macOS**: `ls /dev/ttyUSB*` (Linux, WCH/CP210x — the `ch341`/
  `cp210x` kernel modules are typically already built in) or `ls
  /dev/tty.usbserial-*` (macOS/Linux, FTDI's `ftdi_sio` driver — also
  commonly in-tree) after plugging in. Much less driver drama than Windows
  in general, and none of the ARM64-specific issues above apply.

**You don't need to upload anything manually, or use the IDE/mpremote at
all.** `host/k210_link.py` — used by `dashboard.py`, `monitor.py`, and
`record_wav.py` — does it automatically on every connect: it resets the
board, uploads `stream_proto.py` + `device_app.py` fresh over the raw
MicroPython REPL protocol, and launches `device_app.py`, all on one
continuous connection, before handing you a live frame stream. Just run
one of the host tools (see "Running it" below) and it's fully self-contained.

This isn't a design choice so much as a hardware fact discovered the hard
way: **opening a fresh serial connection to this board resets it via
DTR, regardless of what the host does or doesn't send.** There is no way
to "just attach" to an already-running `device_app.py` session from a new
connection — the act of connecting *is* a reset. Given that, every host
tool owns the reset rather than assuming one didn't just happen: it pulses
DTR itself, then immediately blasts Ctrl-C for ~3 seconds to catch
`_boot.py`'s ~200ms pre-`main.py` interrupt window (this window matters in
practice — the board's `main.py` is whatever was last deployed there
independently of this project, and in testing was often an old, unrelated
script stuck in a tight loop that a single post-reset Ctrl-C, e.g. what
`mpremote`'s default connect sends, did not reliably interrupt; blasting
for several seconds did).

`mpremote` (`pip install mpremote`) remains useful for manual
debugging — a plain REPL, poking at the filesystem, one-off experiments —
just not required for normal use of the tools in this directory anymore.

**Recovery path, if something misbehaves mid-session:** just disconnect
the host tool (Ctrl-C it) and reconnect — every connection is a full fresh
reset + re-upload + re-launch, so there's no persistent bad state to clean
up. `device_app.py` never writes anything to `main.py`, so nothing here
ever risks the MaixPy IDE losing the ability to connect afterward: reset
or power-cycle the board with no host tool running and it comes back up in
the normal, IDE-connectable state.

## Running it

1. **Disconnect the MaixPy IDE** if it's connected — its serial connection
   would hold the COM port, and separately, its debug protocol mode is
   incompatible with this stream (a 2 KB ring buffer drained 64 bytes at a
   time) if it were ever left attached while `device_app.py` runs.
2. On the PC (from `mic_stream/host`, so `uv` finds `pyproject.toml`):
   ```
   uv run python dashboard.py --port COM5
   ```
   (replace `COM5` with the board's port; on Linux/macOS it's typically
   `/dev/ttyUSB0`.) This resets and launches the board automatically (see
   above) — no separate upload step. Two tabs:
   - **Per-Mic** — 8 level meters (7 real mics + 1 unused slot), a mic
     selector, live waveform + FFT for the selected mic, Listen/Record.
   - **Direction / Beamform** — polar `sector_power` plot with the argmax
     sector highlighted, direction/power readout, clip indicator, gain
     control, and the FIR panel (raw 17-tap entry, or an experimental
     bandpass-design helper).

   CLI alternative for scripted testing: `uv run python record_wav.py COM5
   out.wav --mic 0 --seconds 60`, or `uv run python monitor.py COM5 --mic 0`.

## Verification

**Transport (do this first — everything else builds on it)**
1. Play a **1 kHz tone** at the array. Record with `record_wav.py` and open
   the WAV in Audacity — expect a clean 1 kHz peak. Wrong pitch ⇒
   sample-rate mismatch; buzzing ⇒ dropped frames.
2. Run 60 s and check the final report: **`crc_fail=0 seq_gaps=0`** at the
   default 1.5 Mbaud. If either is nonzero, lower `STREAM_BAUDRATE` in
   `device_app.py` and `--baud` on the host together — try 1000000, then
   500000 — to find this cable/adapter's real ceiling.
3. `monitor.py`/the dashboard's Listen button should give recognizable live
   audio with no periodic clicking. Clicking = per-block gaps — see
   `device_app.py`'s UNVERIFIED-ON-HARDWARE note about the MODE_MICS hot
   loop; if this happens, that loop isn't keeping up in real time and
   `SAMPLES_PER_BLOCK_MICS`/`SAMPLE_RATE_MICS` need adjusting before
   anything built on top of it can be trusted (this project has previously
   spent real effort on exactly this class of "looks fine in logs, breaks
   on hardware" issue — see the git history on `iron_kaput`).

**Dashboard**
4. Mic-switch test: clap near one physical mic; that slot's meter should
   read highest. Switch the selector to it and confirm Listen/Record track
   the switch immediately.
5. Switch to the Direction tab; walk around the array clapping — the polar
   plot's highlighted sector and the direction readout should track
   position. Verify the 15↔0 boundary behaves (this is where the original
   `iron_kaput` project's linear-averaging bug lived).
6. Shout close to the array; the clip indicator should go red, then clear
   afterward.
7. FIR: apply the experimental bandpass design with a narrow band, confirm
   *some* audible/measurable change in the direction-search behavior or
   `power` readout — do not assume the filter shape is exactly what was
   requested until this is checked against real hardware (see the panel's
   own warning).

### Verified on real hardware (2026-09-10)

MODE_MICS's full pipeline — I2S capture, per-channel RMS, mono audio
extraction, protocol framing, transmission at 1.5 Mbaud — confirmed
end-to-end via `K210Link` against a real Maix BIT + 6+1 array (FTDI-bridge
board, see "Connecting the board"). Over an 8-second connection:

- `STATUS`: `"mode=MICS rate=8000 slots=8 block=1024"` ✓
- `MIC_LEVELS`: consistently ~18000–19000 on 6 slots, ~0.7 on one slot
  (the 8th, "unused" slot — matches the 6+1 design), one slot varying
  800–2300 — physically coherent, not noise. **Open question, not yet
  investigated:** that varying slot might be picking up the Maix BIT's
  *onboard* single MEMS mic rather than being genuinely unused — its
  `MIC0_WS`/`MIC0_DATA`/`MIC0_BCK` pins (`board/config_maix_bit.py`)
  overlap with the array's D3/WS/SCLK pins, so this is a real, plausible
  hypothesis worth checking, not yet confirmed either way.
- `AUDIO_RAW`: sample count matched frame count × 1024 exactly.
- Error rate: ~1 CRC failure + ~3 seq gaps per ~83 frames received
  (~1–5%, two runs). Small but nonzero — worth re-checking after any
  change to `STREAM_BAUDRATE`, and possibly related to the open question
  above rather than the link itself.

The MODE_MICS hot loop's timing risk (flagged in `device_app.py`'s own
header comment) is therefore resolved: it keeps up in real time. MODE_APU
and the dashboard's own widget behavior against live data remain
unverified — see the checklist above.

## Coming later (see the implementation plan, not yet built here)

- **Phase 3** — firmware C changes (double-buffered `voc_samples` DMA, fixed
  uninitialized-result race, a zero-copy `bytearray` read path) so
  beamformed audio can actually be forwarded and listened to/recorded, and
  so per-mic levels + direction/beamforming can eventually run
  simultaneously instead of as two switched modes.
- **Phase 4** — routing the dashboard's audio through a virtual audio
  device (VB-CABLE/VoiceMeeter) so other Windows apps see the array as a
  microphone; Ethernet (W5500) as an escape hatch if the USB bridge chip's
  bandwidth ceiling ever binds.
