#!/usr/bin/env python3
"""
mic_stream dashboard -- live PC UI for device_app.py.

Two tabs, matching device_app.py's two mutually-exclusive modes (true
simultaneity needs the firmware Phase 3 work -- see the implementation
plan):

  Per-Mic:    7(+1) live level meters, mic switcher, waveform, FFT,
              listen/record for whichever mic is selected.
  Direction:  APU polar sector-power plot, direction/power/saturation
              readout, gain + FIR tuning (FIR is experimental -- see the
              FIR panel's own warning in the UI).

Usage: python dashboard.py [--port COM5] [--baud 1500000]
"""
import argparse
import queue
import sys
import threading
import wave

import numpy as np
import sounddevice as sd
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from k210_link import K210Link
import wire_format as wf

SAMPLE_RATE = 8000       # must match device_app.py's SAMPLE_RATE_MICS
APU_SAMPLE_RATE = 44100  # fixed by the APU (see the k210-chip-hardware skill)
AUDIO_RING_LEN = 4096    # samples kept for the waveform/FFT views
UI_HZ = 20


def design_bandpass_fir(low_hz, high_hz, num_taps=17, sample_rate=APU_SAMPLE_RATE):
    """EXPERIMENTAL: windowed-sinc bandpass -> the device's 17-tap uint16
    FIR wire format. The confirmed API (Maix_apu.c) is 17 unsigned 16-bit
    taps per path; it does NOT document the fixed-point Q-format or sign
    convention, so this assumes a signed Q1.15-style coefficient packed
    into the uint16's two's-complement bit pattern -- a common convention
    for this class of hardware FIR, but UNVERIFIED against a datasheet or
    real hardware. Treat the result as a starting point to test and tune,
    not a calibrated design."""
    n = np.arange(num_taps) - (num_taps - 1) / 2.0

    def sinc_lp(cutoff_hz):
        fc = cutoff_hz / sample_rate
        return 2 * fc * np.sinc(2 * fc * n)

    taps = sinc_lp(high_hz) - sinc_lp(low_hz)
    taps = taps * np.hanning(num_taps)
    peak = np.max(np.abs(taps))
    if peak > 0:
        taps = taps / peak
    fixed = np.round(taps * 32767).astype(np.int64)
    return [int(v) & 0xFFFF for v in fixed]


class Reader(threading.Thread):
    """Background thread: owns the serial link, reads frames, pushes them
    into a thread-safe queue for the Qt main thread to drain on a timer.
    Never touches Qt widgets directly -- cross-thread widget access isn't
    safe in Qt."""

    def __init__(self, link, out_queue):
        super().__init__(daemon=True)
        self.link = link
        self.out_queue = out_queue
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            for type_id, payload in self.link.frames():
                if self._stop.is_set():
                    return
                try:
                    self.out_queue.put_nowait((type_id, payload))
                except queue.Full:
                    pass  # drop rather than backlog
        except Exception as exc:
            print("reader stopped:", exc, file=sys.stderr)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baud, auto_connect=True):
        super().__init__()
        self.setWindowTitle("mic_stream dashboard")
        self.link = None
        self.reader = None
        self.frame_queue = queue.Queue(maxsize=512)

        self.mic_levels = np.zeros(wf.NUM_SLOTS, dtype=np.float32)
        self.audio_ring = np.zeros(AUDIO_RING_LEN, dtype=np.float32)
        self.apu_latest = None

        self.selected_mic = 0
        self.recording = False
        self._wav = None
        self._out_stream = None
        self._play_queue = queue.Queue(maxsize=32)
        self._play_leftover = np.zeros(0, dtype=np.float32)

        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / UI_HZ))

        if auto_connect:
            self._connect(port, baud)

    # ---- connection ----

    def _connect(self, port, baud):
        try:
            self.link = K210Link(port, baudrate=baud)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Connection failed", str(exc))
            return
        self.reader = Reader(self.link, self.frame_queue)
        self.reader.start()
        self.link.set_mode(wf.MODE_MICS)
        self.link.select_mic(self.selected_mic)
        self.status_bar.showMessage("connected to %s @ %d baud" % (port, baud))

    def closeEvent(self, event):
        self._stop_listen()
        if self.recording:
            self.record_btn.setChecked(False)
        if self.reader:
            self.reader.stop()
        if self.link:
            self.link.close()
        event.accept()

    # ---- UI construction ----

    def _build_ui(self):
        self.status_bar = self.statusBar()

        tabs = QtWidgets.QTabWidget()
        self.tabs = tabs
        self.setCentralWidget(tabs)

        tabs.addTab(self._build_mics_tab(), "Per-Mic")
        tabs.addTab(self._build_apu_tab(), "Direction / Beamform")
        tabs.currentChanged.connect(self._on_tab_changed)

    def _build_mics_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(w)

        left = QtWidgets.QVBoxLayout()
        left.addWidget(QtWidgets.QLabel("Mic levels (RMS)"))
        self.level_bars = []
        grid = QtWidgets.QGridLayout()
        for i in range(wf.NUM_SLOTS):
            grid.addWidget(QtWidgets.QLabel("Mic %d" % i), i, 0)
            bar = QtWidgets.QProgressBar()
            bar.setRange(0, 1000)  # RMS 0.0-1.0 (relative to int16 full scale) x1000
            grid.addWidget(bar, i, 1)
            self.level_bars.append(bar)
        left.addLayout(grid)

        left.addWidget(QtWidgets.QLabel("Listen / record:"))
        self.mic_selector = QtWidgets.QComboBox()
        self.mic_selector.addItems(["Mic %d" % i for i in range(wf.NUM_SLOTS)])
        self.mic_selector.currentIndexChanged.connect(self._on_select_mic)
        left.addWidget(self.mic_selector)

        btn_row = QtWidgets.QHBoxLayout()
        self.listen_btn = QtWidgets.QPushButton("Listen")
        self.listen_btn.setCheckable(True)
        self.listen_btn.toggled.connect(self._toggle_listen)
        btn_row.addWidget(self.listen_btn)
        self.record_btn = QtWidgets.QPushButton("Record")
        self.record_btn.setCheckable(True)
        self.record_btn.toggled.connect(self._toggle_record)
        btn_row.addWidget(self.record_btn)
        left.addLayout(btn_row)
        left.addStretch(1)

        layout.addLayout(left, 1)

        right = QtWidgets.QVBoxLayout()
        self.wave_plot = pg.PlotWidget(title="Waveform (selected mic)")
        self.wave_curve = self.wave_plot.plot(pen='y')
        self.wave_plot.setYRange(-1.0, 1.0)
        right.addWidget(self.wave_plot)

        self.fft_plot = pg.PlotWidget(title="FFT (selected mic)")
        self.fft_curve = self.fft_plot.plot(pen='c')
        self.fft_plot.setLabel('bottom', 'Hz')
        right.addWidget(self.fft_plot)

        layout.addLayout(right, 2)
        return w

    def _build_apu_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(w)

        left = QtWidgets.QVBoxLayout()
        self.fig = Figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.canvas = FigureCanvas(self.fig)
        left.addWidget(self.canvas)

        self.apu_readout = QtWidgets.QLabel("no data yet")
        self.apu_readout.setStyleSheet("font-family: monospace;")
        left.addWidget(self.apu_readout)

        self.clip_label = QtWidgets.QLabel("clip: --")
        left.addWidget(self.clip_label)

        gain_row = QtWidgets.QHBoxLayout()
        gain_row.addWidget(QtWidgets.QLabel("Gain (Q1.10, 1024=unity):"))
        self.gain_spin = QtWidgets.QSpinBox()
        self.gain_spin.setRange(0, 65535)
        self.gain_spin.setValue(1024)
        gain_row.addWidget(self.gain_spin)
        gain_btn = QtWidgets.QPushButton("Apply gain")
        gain_btn.clicked.connect(self._apply_gain)
        gain_row.addWidget(gain_btn)
        left.addLayout(gain_row)

        layout.addLayout(left, 1)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(QtWidgets.QLabel(
            "FIR filters (17 taps, hardware, per path) -- EXPERIMENTAL:\n"
            "tap fixed-point format is a best-effort guess (signed Q1.15\n"
            "packed as uint16), not confirmed against a datasheet or real\n"
            "hardware. Verify the effect before trusting it."))

        target_row = QtWidgets.QHBoxLayout()
        target_row.addWidget(QtWidgets.QLabel("Target:"))
        self.fir_target = QtWidgets.QComboBox()
        self.fir_target.addItems([
            "direction pre-FIR", "direction post-FIR",
            "voice pre-FIR", "voice post-FIR"])
        target_row.addWidget(self.fir_target)
        right.addLayout(target_row)

        design_row = QtWidgets.QHBoxLayout()
        design_row.addWidget(QtWidgets.QLabel("Bandpass low (Hz):"))
        self.fir_low = QtWidgets.QSpinBox()
        self.fir_low.setRange(0, APU_SAMPLE_RATE // 2)
        self.fir_low.setValue(80)
        design_row.addWidget(self.fir_low)
        design_row.addWidget(QtWidgets.QLabel("high (Hz):"))
        self.fir_high = QtWidgets.QSpinBox()
        self.fir_high.setRange(0, APU_SAMPLE_RATE // 2)
        self.fir_high.setValue(2000)
        design_row.addWidget(self.fir_high)
        design_btn = QtWidgets.QPushButton("Design (experimental)")
        design_btn.clicked.connect(self._design_fir)
        design_row.addWidget(design_btn)
        right.addLayout(design_row)

        self.fir_taps = []
        taps_grid = QtWidgets.QGridLayout()
        for i in range(17):
            sb = QtWidgets.QSpinBox()
            sb.setRange(0, 65535)
            sb.setValue(0)
            taps_grid.addWidget(sb, i // 6, i % 6)
            self.fir_taps.append(sb)
        right.addLayout(taps_grid)

        apply_fir_btn = QtWidgets.QPushButton("Apply FIR to device")
        apply_fir_btn.clicked.connect(self._apply_fir)
        right.addWidget(apply_fir_btn)
        right.addStretch(1)

        layout.addLayout(right, 1)
        return w

    # ---- mode / tab switching ----

    def _on_tab_changed(self, index):
        if not self.link:
            return
        if index == 0:
            self.link.set_mode(wf.MODE_MICS)
            self.link.select_mic(self.selected_mic)
        else:
            self._stop_listen()
            self.link.set_mode(wf.MODE_APU)

    # ---- Per-Mic tab actions ----

    def _on_select_mic(self, idx):
        self.selected_mic = idx
        if self.link:
            self.link.select_mic(idx)
        if self.recording:
            self.record_btn.setChecked(False)  # switching mic mid-recording would corrupt the file

    def _toggle_listen(self, checked):
        if checked:
            self._start_listen()
        else:
            self._stop_listen()

    def _start_listen(self):
        if self._out_stream is not None:
            return
        self._play_leftover = np.zeros(0, dtype=np.float32)
        while not self._play_queue.empty():
            try:
                self._play_queue.get_nowait()
            except queue.Empty:
                break

        def callback(outdata, frames, time_info, status):
            chunks = [self._play_leftover]
            have = len(self._play_leftover)
            while have < frames:
                try:
                    block = self._play_queue.get_nowait()
                except queue.Empty:
                    break
                chunks.append(block)
                have += len(block)
            data = np.concatenate(chunks) if len(chunks) > 1 else chunks[0]
            if len(data) >= frames:
                outdata[:, 0] = data[:frames]
                self._play_leftover = data[frames:]
            else:
                outdata[:len(data), 0] = data
                outdata[len(data):, 0] = 0
                self._play_leftover = np.zeros(0, dtype=np.float32)

        self._out_stream = sd.OutputStream(samplerate=SAMPLE_RATE, channels=1,
                                            dtype='float32', blocksize=512,
                                            callback=callback)
        self._out_stream.start()

    def _stop_listen(self):
        if self._out_stream is not None:
            self._out_stream.stop()
            self._out_stream.close()
            self._out_stream = None
        self.listen_btn.setChecked(False)

    def _toggle_record(self, checked):
        self.recording = checked
        if checked:
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Record to WAV", "", "WAV files (*.wav)")
            if not path:
                self.record_btn.setChecked(False)
                self.recording = False
                return
            self._wav = wave.open(path, 'wb')
            self._wav.setnchannels(1)
            self._wav.setsampwidth(2)
            self._wav.setframerate(SAMPLE_RATE)
            self.status_bar.showMessage("recording mic %d to %s" % (self.selected_mic, path))
        else:
            if self._wav is not None:
                self._wav.close()
                self._wav = None
            self.status_bar.showMessage("recording stopped")

    # ---- Direction tab actions ----

    def _apply_gain(self):
        if self.link:
            self.link.set_gain(self.gain_spin.value())

    def _design_fir(self):
        low = self.fir_low.value()
        high = self.fir_high.value()
        if low >= high:
            QtWidgets.QMessageBox.warning(self, "Invalid range", "low must be < high")
            return
        coefs = design_bandpass_fir(low, high)
        for sb, v in zip(self.fir_taps, coefs):
            sb.setValue(v)

    def _apply_fir(self):
        if not self.link:
            return
        target = self.fir_target.currentIndex()
        coefs = [sb.value() for sb in self.fir_taps]
        self.link.set_fir(target, coefs)
        self.status_bar.showMessage("FIR sent (target=%d)" % target)

    # ---- frame processing (main-thread timer) ----

    def _tick(self):
        drained = 0
        while drained < 200:  # bound per-tick work regardless of backlog
            try:
                type_id, payload = self.frame_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self._handle_frame(type_id, payload)

        if self.tabs.currentIndex() == 0:
            self._refresh_mics_tab()
        else:
            self._refresh_apu_tab()

    def _handle_frame(self, type_id, payload):
        if type_id == wf.TYPE_MIC_LEVELS:
            levels = wf.decode_mic_levels(payload) / 32768.0
            self.mic_levels = np.clip(levels, 0.0, 1.0)
        elif type_id == wf.TYPE_AUDIO_RAW:
            samples = wf.decode_audio_mono(payload).astype(np.float32) / 32768.0
            n = len(samples)
            ring_len = len(self.audio_ring)
            self.audio_ring = (np.concatenate([self.audio_ring[n:], samples])
                                if n < ring_len else samples[-ring_len:])
            if self.listen_btn.isChecked():
                try:
                    self._play_queue.put_nowait(samples)
                except queue.Full:
                    pass
            if self.recording and self._wav is not None:
                pcm16 = (samples * 32768.0).astype('<i2')
                self._wav.writeframes(pcm16.tobytes())
        elif type_id == wf.TYPE_APU_TELEMETRY:
            self.apu_latest = wf.decode_apu_telemetry(payload)
        elif type_id == wf.TYPE_STATUS:
            self.status_bar.showMessage(payload.decode('utf-8', 'replace'))

    def _refresh_mics_tab(self):
        for bar, level in zip(self.level_bars, self.mic_levels):
            bar.setValue(int(level * 1000))

        self.wave_curve.setData(self.audio_ring[-1024:])

        n = len(self.audio_ring)
        if n >= 256:
            windowed = self.audio_ring * np.hanning(n)
            spectrum = np.abs(np.fft.rfft(windowed))
            freqs = np.fft.rfftfreq(n, d=1.0 / SAMPLE_RATE)
            self.fft_curve.setData(freqs, spectrum)

    def _refresh_apu_tab(self):
        t = self.apu_latest
        if t is None:
            return
        self.apu_readout.setText(
            "direction: %2d (%.1f deg)   power: %10d   voc_dir: %2d" % (
                t['direction'], t['direction'] * 22.5, t['power'], t['voc_dir']))
        clip_pct = (100.0 * t['sat_clips'] / t['sat_total']) if t['sat_total'] else 0.0
        self.clip_label.setText("clip: %.1f%% (%d/%d samples)" % (
            clip_pct, t['sat_clips'], t['sat_total']))
        self.clip_label.setStyleSheet(
            "color: red; font-weight: bold;" if clip_pct > 1.0 else "color: green;")

        sector_power = t['sector_power'].astype(np.float64)
        peak = sector_power.max() if sector_power.max() > 0 else 1.0
        radii = sector_power / peak
        theta = np.arange(wf.NUM_SECTORS) * (2 * np.pi / wf.NUM_SECTORS)
        self.ax.clear()
        colors = ['red' if i == t['direction'] else 'steelblue' for i in range(wf.NUM_SECTORS)]
        self.ax.bar(theta, radii, width=2 * np.pi / wf.NUM_SECTORS * 0.9, color=colors)
        self.ax.set_ylim(0, 1.05)
        self.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default=None, help='serial port, e.g. COM5 or /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=1500000)
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)

    port = args.port
    if not port:
        port, ok = QtWidgets.QInputDialog.getText(None, "Serial port", "Port (e.g. COM5):")
        if not ok or not port:
            sys.exit(0)

    win = MainWindow(port, args.baud)
    win.resize(1100, 650)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
