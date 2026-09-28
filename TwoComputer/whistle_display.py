"""
whistle_display.py

Reusable live "waveform + spectrogram + spectrum" monitor for a
WhistleDetector, factored out of ME_193_HW3's whistle_drive.py so every
script in this project can show the same live view (and each computer's
own accepted frequency range, shaded on the plot) instead of
re-implementing matplotlib plumbing.

matplotlib GUIs must run on the MAIN thread (required on macOS), so the
usual pattern is:

    monitor = WhistleMonitor(title="...", analysis_min_freq=..., analysis_max_freq=...)

    def audio_loop():
        while monitor.running:
            chunk = ...                       # read one chunk from the mic
            is_whistle, freq = detector.update(chunk)
            monitor.push(detector, status_label=..., status_color=...)
            ...                                # do MQTT/drive logic here

    threading.Thread(target=audio_loop, daemon=True).start()
    monitor.run()   # blocks on the main thread until the window is closed
"""

import threading

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from whistle_detector import SAMPLE_RATE, WINDOW_SIZE

plt.style.use("dark_background")

SPEC_HISTORY_SECONDS = 8
SPEC_DB_FLOOR = -40.0
SPEC_DB_CEILING = 40.0

ACCEPTED_RANGE_COLOR = "#22c55e"


class WhistleMonitor:
    def __init__(self, title, analysis_min_freq, analysis_max_freq,
                 max_freq_display=5000, chunk_size=1024):
        self.title = title
        self.analysis_min_freq = analysis_min_freq
        self.analysis_max_freq = analysis_max_freq
        # The accepted range has to actually fit on screen to be shaded.
        self.max_freq_display = max(max_freq_display, analysis_max_freq)
        self.chunk_size = chunk_size

        all_freqs = np.fft.rfftfreq(WINDOW_SIZE, d=1 / SAMPLE_RATE)
        self.display_bin_count = int(np.searchsorted(all_freqs, self.max_freq_display))
        self.spec_history_columns = max(1, int(SPEC_HISTORY_SECONDS / (chunk_size / SAMPLE_RATE)))

        self.lock = threading.Lock()
        self.running = True
        self.state = {
            "waveform": np.zeros(WINDOW_SIZE, dtype=np.float32),
            "freqs": np.array([0.0]),
            "magnitude": np.array([0.0]),
            "peak_freq": 0.0,
            "rms": 0.0,
            "purity": 0.0,
            "rms_threshold": 0.0,
            "whistle_detected": False,
            "status_label": "",
            "status_color": "#ffffff",
            "spec_history": np.full(
                (self.display_bin_count, self.spec_history_columns), SPEC_DB_FLOOR, dtype=np.float32
            ),
        }

    def push(self, detector, status_label="", status_color="#ffffff"):
        """Call once per audio frame, right after detector.update(chunk)."""
        column_db = 20.0 * np.log10(detector.magnitude[:self.display_bin_count] + 1e-9)

        with self.lock:
            self.state["waveform"] = detector.rolling.copy()
            self.state["freqs"] = detector.freqs
            self.state["magnitude"] = detector.magnitude
            self.state["peak_freq"] = detector.smoothed_freq
            self.state["rms"] = detector.rms
            self.state["purity"] = detector.purity
            self.state["rms_threshold"] = detector.rms_threshold
            self.state["whistle_detected"] = detector.currently_whistling
            self.state["status_label"] = status_label
            self.state["status_color"] = status_color
            self.state["spec_history"] = np.roll(self.state["spec_history"], -1, axis=1)
            self.state["spec_history"][:, -1] = column_db

    def _build_plot(self):
        fig, (ax_wave, ax_specgram, ax_spec) = plt.subplots(
            3, 1, figsize=(10, 10), gridspec_kw={"height_ratios": [1, 2.2, 1.3]}
        )
        fig.suptitle(self.title, fontsize=14, fontweight="bold")
        fig.subplots_adjust(hspace=0.55, top=0.90, left=0.09, right=0.97, bottom=0.06)

        t = np.arange(WINDOW_SIZE) / SAMPLE_RATE
        wave_line, = ax_wave.plot(t, np.zeros(WINDOW_SIZE), linewidth=0.7, color="#22d3ee")
        ax_wave.set_ylim(-1, 1)
        ax_wave.set_xlim(0, WINDOW_SIZE / SAMPLE_RATE)
        ax_wave.set_xlabel("Time (s)")
        ax_wave.set_ylabel("Amplitude")
        ax_wave.set_title("Live waveform")
        ax_wave.grid(alpha=0.2)

        specgram_im = ax_specgram.imshow(
            self.state["spec_history"],
            aspect="auto", origin="lower", cmap="inferno",
            extent=[-SPEC_HISTORY_SECONDS, 0, 0, self.max_freq_display],
            vmin=SPEC_DB_FLOOR, vmax=SPEC_DB_CEILING, interpolation="nearest",
        )
        ax_specgram.set_xlabel("Time (s ago)")
        ax_specgram.set_ylabel("Frequency (Hz)")
        ax_specgram.set_title("Live spectrogram")
        fig.colorbar(specgram_im, ax=ax_specgram, pad=0.01, label="Magnitude (dB)")

        ax_specgram.axhspan(self.analysis_min_freq, self.analysis_max_freq,
                             color=ACCEPTED_RANGE_COLOR, alpha=0.18)
        ax_specgram.text(-SPEC_HISTORY_SECONDS + 0.15,
                          (self.analysis_min_freq + self.analysis_max_freq) / 2,
                          "ACCEPTED RANGE", color=ACCEPTED_RANGE_COLOR,
                          fontsize=9, fontweight="bold", ha="left", va="center")

        spec_line, = ax_spec.plot([], [], linewidth=0.9, color="#e5e5e5")
        peak_marker = ax_spec.axvline(0, color="#ef4444", linestyle="--", linewidth=1.5)
        ax_spec.set_xlim(0, self.max_freq_display)
        ax_spec.set_xlabel("Frequency (Hz)")
        ax_spec.set_ylabel("Magnitude")
        ax_spec.set_title("Live spectrum + decision")
        ax_spec.axvspan(self.analysis_min_freq, self.analysis_max_freq,
                         color=ACCEPTED_RANGE_COLOR, alpha=0.12)
        ax_spec.text((self.analysis_min_freq + self.analysis_max_freq) / 2, 0,
                      "ACCEPTED RANGE", color=ACCEPTED_RANGE_COLOR,
                      ha="center", va="bottom", fontsize=8, fontweight="bold", rotation=90)

        status_text = fig.text(0.02, 0.965, "", fontsize=12, family="monospace", va="top")

        return fig, wave_line, specgram_im, spec_line, peak_marker, status_text

    def _update_plot(self, _frame, wave_line, specgram_im, spec_line, peak_marker, status_text):
        with self.lock:
            waveform = self.state["waveform"]
            freqs = self.state["freqs"]
            magnitude = self.state["magnitude"]
            peak_freq = self.state["peak_freq"]
            rms = self.state["rms"]
            purity = self.state["purity"]
            rms_threshold = self.state["rms_threshold"]
            whistle_detected = self.state["whistle_detected"]
            status_label = self.state["status_label"]
            status_color = self.state["status_color"]
            spec_history = self.state["spec_history"]

        wave_line.set_ydata(waveform)
        specgram_im.set_data(spec_history)

        mask = freqs <= self.max_freq_display
        spec_line.set_data(freqs[mask], magnitude[mask])
        ax = spec_line.axes
        if magnitude[mask].size:
            ax.set_ylim(0, max(magnitude[mask].max() * 1.1, 1.0))

        peak_marker.set_xdata([peak_freq, peak_freq])

        status_text.set_color(status_color)
        status_text.set_text(
            f"{status_label:<14s}  {'WHISTLE' if whistle_detected else 'no whistle':<10s}  "
            f"peak={peak_freq:6.0f} Hz   purity={purity:4.2f}   "
            f"rms={rms:5.3f}   threshold={rms_threshold:5.3f}"
        )

        return wave_line, specgram_im, spec_line, peak_marker, status_text

    def run(self):
        """Builds the figure and blocks (plt.show()) until the window is
        closed. Must be called from the main thread. Sets self.running to
        False on close, so a background audio-loop thread can check it."""
        fig, wave_line, specgram_im, spec_line, peak_marker, status_text = self._build_plot()
        ani = FuncAnimation(fig, self._update_plot,
                            fargs=(wave_line, specgram_im, spec_line, peak_marker, status_text),
                            interval=80, cache_frame_data=False)

        def on_close(_event):
            self.running = False

        fig.canvas.mpl_connect("close_event", on_close)
        plt.show()
        self.running = False
