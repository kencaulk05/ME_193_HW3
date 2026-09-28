"""
whistle_detector.py

Reusable whistle-detection core, factored out of ME_193_HW3's
whistle_drive.py so both computers in this two-computer setup can share
the exact same tuned noise-masking logic instead of duplicating it:

  - Frequency-range gate (ignore rumble/ultrasonic content outside
    plausible whistle range).
  - Adaptive, hysteresis-based RMS gate (tracks the ambient noise floor
    and requires a bigger jump above it to *start* counting as a whistle
    than to *continue*, so a borderline signal doesn't flicker).
  - Spectral purity gate (a whistle is close to a pure tone; broadband
    noise like talking/claps spreads energy across many frequencies and
    fails this even when loud enough to pass the RMS gate).
  - Median smoothing of the frequency estimate over the last few
    whistle-only frames, so a single noisy frame's peak can't shift the
    result on its own.

This module only does the math -- no mic capture, no threading, no
MQTT/hardware. Feed it consecutive audio chunks; it maintains its own
rolling window and gate state internally.
"""

from collections import deque

import numpy as np

SAMPLE_RATE = 44100
WINDOW_SIZE = 4096  # samples analyzed per FFT

ANALYSIS_MIN_FREQ = 300
ANALYSIS_MAX_FREQ = 5000

PURITY_THRESHOLD = 0.35
PURITY_BAND_HZ = 60

NOISE_FLOOR_EMA_ALPHA = 0.02
NOISE_FLOOR_MIN = 0.005
NOISE_ENTER_MULTIPLE = 4.0
NOISE_EXIT_MULTIPLE = 2.5

FREQ_SMOOTHING_FRAMES = 5


def analyze_buffer(buffer):
    """Returns (rms, freqs, magnitude, peak_freq, purity) for one buffer."""
    rms = float(np.sqrt(np.mean(buffer.astype(np.float64) ** 2)))

    window = np.hanning(len(buffer))
    fft_values = np.fft.rfft(buffer * window)
    freqs = np.fft.rfftfreq(len(buffer), d=1 / SAMPLE_RATE)
    magnitude = np.abs(fft_values)

    valid = (freqs >= ANALYSIS_MIN_FREQ) & (freqs <= ANALYSIS_MAX_FREQ)
    if not np.any(valid):
        return rms, freqs, magnitude, 0.0, 0.0

    valid_freqs = freqs[valid]
    valid_mag = magnitude[valid]
    peak_idx = np.argmax(valid_mag)
    peak_freq = float(valid_freqs[peak_idx])

    near_peak = np.abs(valid_freqs - peak_freq) <= PURITY_BAND_HZ
    peak_energy = float(np.sum(valid_mag[near_peak] ** 2))
    total_energy = float(np.sum(valid_mag ** 2)) + 1e-12
    purity = peak_energy / total_energy

    return rms, freqs, magnitude, peak_freq, purity


class WhistleDetector:
    """Feed consecutive audio chunks via update(chunk); tracks its own
    rolling window, adaptive noise floor, and hysteresis state."""

    def __init__(self):
        self.rolling = np.zeros(WINDOW_SIZE, dtype=np.float32)
        self.noise_floor = NOISE_FLOOR_MIN
        self.currently_whistling = False
        self.recent_peak_freqs = deque(maxlen=FREQ_SMOOTHING_FRAMES)

        # Diagnostics from the most recent update() call.
        self.rms = 0.0
        self.purity = 0.0
        self.rms_threshold = NOISE_FLOOR_MIN * NOISE_ENTER_MULTIPLE
        self.raw_peak_freq = 0.0
        self.smoothed_freq = 0.0

    def update(self, chunk):
        """chunk: 1D float32 array of new mono samples (any length <=
        WINDOW_SIZE). Returns (is_whistle, smoothed_freq)."""
        chunk = np.asarray(chunk, dtype=np.float32)
        self.rolling = np.roll(self.rolling, -len(chunk))
        self.rolling[-len(chunk):] = chunk

        rms, freqs, magnitude, peak_freq, purity = analyze_buffer(self.rolling)

        rms_threshold = self.noise_floor * (
            NOISE_EXIT_MULTIPLE if self.currently_whistling else NOISE_ENTER_MULTIPLE
        )
        is_whistle = (rms >= rms_threshold) and (purity >= PURITY_THRESHOLD)
        self.currently_whistling = is_whistle

        if not is_whistle:
            self.noise_floor = (1 - NOISE_FLOOR_EMA_ALPHA) * self.noise_floor + NOISE_FLOOR_EMA_ALPHA * rms
            self.noise_floor = max(self.noise_floor, NOISE_FLOOR_MIN)

        if is_whistle:
            self.recent_peak_freqs.append(peak_freq)
            smoothed_freq = float(np.median(self.recent_peak_freqs))
        else:
            self.recent_peak_freqs.clear()
            smoothed_freq = peak_freq

        self.rms = rms
        self.purity = purity
        self.rms_threshold = rms_threshold
        self.raw_peak_freq = peak_freq
        self.smoothed_freq = smoothed_freq

        return is_whistle, smoothed_freq
