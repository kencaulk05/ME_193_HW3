"""
whistle_drive.py

Listens to the microphone in real time, classifies the whistle pitch into a
driving command, and publishes that command over MQTT for game_logic.py to
execute on the actual robot. This script never touches the hub directly --
see the README for why (BLE only allows one connection to a hub at a time,
so only game_logic.py owns that connection).

Shows a live two-panel plot: the raw waveform on top, the frequency
spectrum (with the pitch bands shaded and the detected peak marked) on the
bottom, plus a text readout of the live decision.

Install first:
    pip install pyaudio numpy matplotlib

Usage:
    python whistle_drive.py
    python whistle_drive.py --list-devices     # find your mic's device index
    python whistle_drive.py --device 2
Close the plot window to quit.
"""

import argparse
import threading
import time
from collections import deque

import numpy as np
import pyaudio
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from mqttlib import MQTTClient
import game_config as cfg

# ---------------------------------------------------------------------------
# Audio settings
# ---------------------------------------------------------------------------

SAMPLE_RATE = 44100
CHUNK = 1024            # samples read from the mic per read() call
WINDOW_SIZE = 4096       # samples analyzed per FFT (several chunks' worth,
                         # for better frequency resolution than one CHUNK alone)

PLOT_MAX_FREQ = 5000    # spectrum/spectrogram frequency-axis limit

# --- Spectrogram history ---
# How much scrolling time history the spectrogram shows at once. Each
# column is one audio-thread iteration (one CHUNK read, ~CHUNK/SAMPLE_RATE
# seconds), so this many seconds of columns are kept and scrolled left as
# new ones arrive on the right.
SPEC_HISTORY_SECONDS = 8

# Loudness range (dB) mapped to the spectrogram's colormap. Tune these if
# the display looks all-dark (raise SPEC_DB_FLOOR toward SPEC_DB_CEILING)
# or all-bright (lower SPEC_DB_CEILING) for your mic's actual gain.
SPEC_DB_FLOOR = -40.0
SPEC_DB_CEILING = 40.0

# Precomputed once: which FFT bins fall within the displayed frequency
# range, since WINDOW_SIZE/SAMPLE_RATE never change at runtime.
_ALL_FREQS = np.fft.rfftfreq(WINDOW_SIZE, d=1 / SAMPLE_RATE)
_DISPLAY_BIN_COUNT = int(np.searchsorted(_ALL_FREQS, PLOT_MAX_FREQ))
_SPEC_HISTORY_COLUMNS = max(1, int(SPEC_HISTORY_SECONDS / (CHUNK / SAMPLE_RATE)))

# ---------------------------------------------------------------------------
# Noise masking -- how we tell "a whistle" apart from silence/background noise
# ---------------------------------------------------------------------------

# Ignore very low rumble (footsteps, HVAC, handling noise) and anything
# above typical whistle range when searching for the dominant pitch.
ANALYSIS_MIN_FREQ = 300
ANALYSIS_MAX_FREQ = 5000

# RMS amplitude gate: below this, treat the buffer as silence regardless of
# what the FFT says (quiet background hum can still have a "peak" that
# means nothing). Calibrate by watching the printed RMS value in a quiet
# room vs. while whistling.
SILENCE_RMS = 0.02

# Spectral purity gate: whistles are close to pure tones, so almost all
# their energy sits in one narrow peak. Talking, claps, and general room
# noise spread energy across many frequencies. purity = (energy right
# around the peak) / (total energy in the analysis range). Require a high
# purity to reject broadband noise even when it's loud enough to pass the
# RMS gate.
PURITY_THRESHOLD = 0.35
PURITY_BAND_HZ = 60  # width around the peak counted as "the peak's energy"

# ---------------------------------------------------------------------------
# Pitch bands -> driving commands
# ---------------------------------------------------------------------------
# CALIBRATE THESE against your own whistle range: run
#   python whistle_recorder.py
# a few times, whistle low/medium/high/very-high, and read off the printed
# "Dominant frequency" each time. Typical whistling covers roughly 500-4000
# Hz, but everyone's range differs -- these are starting points, not truth.

BAND_STOP_MAX = 700       # freq below this -> stop
BAND_LEFT_MAX = 1300      # freq below this (and above BAND_STOP_MAX) -> turn left
BAND_RIGHT_MAX = 2000     # freq below this (and above BAND_LEFT_MAX) -> turn right
BAND_FORWARD_MAX = 3200   # freq below this (and above BAND_RIGHT_MAX) -> speed up / forward
# freq above BAND_FORWARD_MAX -> CMD_GOAL (the special "I scored" whistle) --
# deliberately a hard-to-hit-by-accident range at the very top of most
# people's whistling register.

BAND_COLORS = {
    cfg.CMD_STOP: "#888888",
    cfg.CMD_LEFT: "#3b82f6",
    cfg.CMD_RIGHT: "#f59e0b",
    cfg.CMD_FORWARD: "#22c55e",
    cfg.CMD_GOAL: "#ef4444",
}


def classify_frequency(freq_hz):
    if freq_hz < BAND_STOP_MAX:
        return cfg.CMD_STOP
    elif freq_hz < BAND_LEFT_MAX:
        return cfg.CMD_LEFT
    elif freq_hz < BAND_RIGHT_MAX:
        return cfg.CMD_RIGHT
    elif freq_hz < BAND_FORWARD_MAX:
        return cfg.CMD_FORWARD
    else:
        return cfg.CMD_GOAL


# ---------------------------------------------------------------------------
# Debounce / fail-safe timing
# ---------------------------------------------------------------------------

DEBOUNCE_FRAMES = 3        # consecutive matching classifications needed to commit
NO_WHISTLE_TIMEOUT = 1.0   # seconds with no valid whistle before we force STOP

# ---------------------------------------------------------------------------
# Shared state between the audio thread and the plotting (main) thread
# ---------------------------------------------------------------------------

lock = threading.Lock()
state = {
    "waveform": np.zeros(WINDOW_SIZE, dtype=np.float32),
    "freqs": np.array([0.0]),
    "magnitude": np.array([0.0]),
    "peak_freq": 0.0,
    "rms": 0.0,
    "purity": 0.0,
    "whistle_detected": False,
    "raw_command": None,       # this frame's classification, or None
    "effective_command": cfg.CMD_STOP,  # what we've actually published
    "spec_history": np.full((_DISPLAY_BIN_COUNT, _SPEC_HISTORY_COLUMNS), SPEC_DB_FLOOR, dtype=np.float32),
}
running = True


def analyze_buffer(buffer):
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


def audio_thread_fn(device_index, mqtt_client):
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paFloat32, channels=1, rate=SAMPLE_RATE,
                    input=True, input_device_index=device_index,
                    frames_per_buffer=CHUNK)

    rolling = np.zeros(WINDOW_SIZE, dtype=np.float32)
    recent_classifications = deque(maxlen=DEBOUNCE_FRAMES)
    stable_command = None
    last_whistle_time = time.monotonic()
    last_published = None
    spec_history = state["spec_history"].copy()

    print("Listening... whistle to drive. Ctrl+C or close the plot window to quit.")

    try:
        while running:
            data = stream.read(CHUNK, exception_on_overflow=False)
            chunk = np.frombuffer(data, dtype=np.float32)

            rolling = np.roll(rolling, -len(chunk))
            rolling[-len(chunk):] = chunk

            rms, freqs, magnitude, peak_freq, purity = analyze_buffer(rolling)
            is_whistle = (rms >= SILENCE_RMS) and (purity >= PURITY_THRESHOLD)
            raw_command = classify_frequency(peak_freq) if is_whistle else None

            now = time.monotonic()
            if is_whistle:
                last_whistle_time = now

            recent_classifications.append(raw_command)
            if (len(recent_classifications) == DEBOUNCE_FRAMES
                    and all(c == raw_command for c in recent_classifications)
                    and raw_command is not None):
                stable_command = raw_command

            # Fail-safe: force STOP if nothing valid has been heard recently
            # -- see the README's answer to "what if no whistle is detected?"
            if now - last_whistle_time > NO_WHISTLE_TIMEOUT:
                effective_command = cfg.CMD_STOP
            else:
                effective_command = stable_command or cfg.CMD_STOP

            if effective_command != last_published:
                mqtt_client.publish(cfg.DRIVE_TOPIC, effective_command)
                print(f"[whistle] -> {effective_command}  "
                      f"(peak={peak_freq:.0f}Hz purity={purity:.2f} rms={rms:.3f})")
                last_published = effective_command

            # Scroll this frame's magnitude spectrum (cropped to the display
            # range, converted to dB) into the spectrogram history as the
            # newest column.
            column_db = 20.0 * np.log10(magnitude[:_DISPLAY_BIN_COUNT] + 1e-9)
            spec_history = np.roll(spec_history, -1, axis=1)
            spec_history[:, -1] = column_db

            with lock:
                state["waveform"] = rolling.copy()
                state["freqs"] = freqs
                state["magnitude"] = magnitude
                state["peak_freq"] = peak_freq
                state["rms"] = rms
                state["purity"] = purity
                state["whistle_detected"] = is_whistle
                state["raw_command"] = raw_command
                state["spec_history"] = spec_history
                state["effective_command"] = effective_command
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


# ---------------------------------------------------------------------------
# Live plot
# ---------------------------------------------------------------------------

plt.style.use("dark_background")

# Shared by both the spectrogram (horizontal bands) and the spectrum panel
# (vertical bands) -- one definition, so the two views can never disagree
# about which frequency does what.
BAND_EDGES = [0, BAND_STOP_MAX, BAND_LEFT_MAX, BAND_RIGHT_MAX, BAND_FORWARD_MAX, PLOT_MAX_FREQ]
BAND_LABELS = [cfg.CMD_STOP, cfg.CMD_LEFT, cfg.CMD_RIGHT, cfg.CMD_FORWARD, cfg.CMD_GOAL]


def build_plot():
    fig, (ax_wave, ax_specgram, ax_spec) = plt.subplots(
        3, 1, figsize=(10, 10), gridspec_kw={"height_ratios": [1, 2.2, 1.3]}
    )
    fig.suptitle("Whistle Drive -- Live Audio Monitor", fontsize=14, fontweight="bold")
    fig.subplots_adjust(hspace=0.55, top=0.90, left=0.09, right=0.97, bottom=0.06)

    # --- Waveform ---
    t = np.arange(WINDOW_SIZE) / SAMPLE_RATE
    wave_line, = ax_wave.plot(t, np.zeros(WINDOW_SIZE), linewidth=0.7, color="#22d3ee")
    ax_wave.set_ylim(-1, 1)
    ax_wave.set_xlim(0, WINDOW_SIZE / SAMPLE_RATE)
    ax_wave.set_xlabel("Time (s)")
    ax_wave.set_ylabel("Amplitude")
    ax_wave.set_title("Live waveform")
    ax_wave.grid(alpha=0.2)

    # --- Spectrogram: scrolling time/frequency/loudness history ---
    specgram_im = ax_specgram.imshow(
        state["spec_history"],
        aspect="auto", origin="lower", cmap="inferno",
        extent=[-SPEC_HISTORY_SECONDS, 0, 0, PLOT_MAX_FREQ],
        vmin=SPEC_DB_FLOOR, vmax=SPEC_DB_CEILING, interpolation="nearest",
    )
    ax_specgram.set_xlabel("Time (s ago)")
    ax_specgram.set_ylabel("Frequency (Hz)")
    ax_specgram.set_title("Live spectrogram")
    fig.colorbar(specgram_im, ax=ax_specgram, pad=0.01, label="Magnitude (dB)")

    # Label each frequency band with the command it drives, right on the
    # spectrogram -- so "what would the car do at this frequency" is
    # answered directly on the history you're watching, not just the
    # current-instant panel below.
    for lo, hi, label in zip(BAND_EDGES[:-1], BAND_EDGES[1:], BAND_LABELS):
        ax_specgram.axhspan(lo, hi, color=BAND_COLORS[label], alpha=0.18)
        ax_specgram.text(-SPEC_HISTORY_SECONDS + 0.15, (lo + hi) / 2, label,
                          color=BAND_COLORS[label], fontsize=9, fontweight="bold",
                          ha="left", va="center")

    # --- Current-instant spectrum + decision ---
    spec_line, = ax_spec.plot([], [], linewidth=0.9, color="#e5e5e5")
    peak_marker = ax_spec.axvline(0, color="#ef4444", linestyle="--", linewidth=1.5)
    ax_spec.set_xlim(0, PLOT_MAX_FREQ)
    ax_spec.set_xlabel("Frequency (Hz)")
    ax_spec.set_ylabel("Magnitude")
    ax_spec.set_title("Live spectrum + decision")

    for lo, hi, label in zip(BAND_EDGES[:-1], BAND_EDGES[1:], BAND_LABELS):
        ax_spec.axvspan(lo, hi, color=BAND_COLORS[label], alpha=0.12)
        ax_spec.text((lo + hi) / 2, 0, label, color=BAND_COLORS[label],
                     ha="center", va="bottom", fontsize=8, fontweight="bold", rotation=90)

    status_text = fig.text(0.02, 0.965, "", fontsize=12, family="monospace", va="top")

    return fig, wave_line, specgram_im, spec_line, peak_marker, status_text


def update_plot(_frame, wave_line, specgram_im, spec_line, peak_marker, status_text):
    with lock:
        waveform = state["waveform"]
        freqs = state["freqs"]
        magnitude = state["magnitude"]
        peak_freq = state["peak_freq"]
        rms = state["rms"]
        purity = state["purity"]
        whistle_detected = state["whistle_detected"]
        effective_command = state["effective_command"]
        spec_history = state["spec_history"]

    wave_line.set_ydata(waveform)

    specgram_im.set_data(spec_history)

    mask = freqs <= PLOT_MAX_FREQ
    spec_line.set_data(freqs[mask], magnitude[mask])
    ax = spec_line.axes
    ax.set_ylim(0, max(magnitude[mask].max() * 1.1, 1.0))

    peak_marker.set_xdata([peak_freq, peak_freq])

    color = BAND_COLORS.get(effective_command, "#ffffff")
    status_text.set_color(color)
    status_text.set_text(
        f"command: {effective_command:<8}  "
        f"{'WHISTLE' if whistle_detected else 'no whistle':<10}  "
        f"peak={peak_freq:6.0f} Hz   purity={purity:4.2f}   rms={rms:5.3f}"
    )

    return wave_line, specgram_im, spec_line, peak_marker, status_text


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def list_devices():
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info.get("maxInputChannels", 0) > 0:
            print(f"{i}: {info['name']}  (inputs: {info['maxInputChannels']})")
    p.terminate()


def parse_args():
    parser = argparse.ArgumentParser(description="Whistle-controlled driving commands over MQTT")
    parser.add_argument("--device", type=int, default=None,
                        help="Input device index (see --list-devices)")
    parser.add_argument("--list-devices", action="store_true",
                        help="List available input devices and exit")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.list_devices:
        list_devices()
        return

    global running
    mqtt_client = MQTTClient()
    mqtt_client.connect()
    print(f"Publishing driving commands to '{cfg.DRIVE_TOPIC}'")

    audio_thread = threading.Thread(target=audio_thread_fn, args=(args.device, mqtt_client), daemon=True)
    audio_thread.start()

    fig, wave_line, specgram_im, spec_line, peak_marker, status_text = build_plot()
    ani = FuncAnimation(fig, update_plot,
                        fargs=(wave_line, specgram_im, spec_line, peak_marker, status_text),
                        interval=80, cache_frame_data=False)

    def on_close(_event):
        global running
        running = False

    fig.canvas.mpl_connect("close_event", on_close)

    try:
        plt.show()
    finally:
        running = False
        audio_thread.join(timeout=2)
        mqtt_client.publish(cfg.DRIVE_TOPIC, cfg.CMD_STOP)  # leave the robot stopped on exit
        time.sleep(0.2)
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
