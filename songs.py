"""
A tiny cross-platform tone player, built on pyaudio (which you already need
for the whistle mic input) rather than an OS-specific beep function -- so it
works the same way on both teammates' laptops.

legoeducation's hub can also beep(), but only single tones, not a melody --
see beep_hub_flourish() at the bottom if you want the physical robot to
chime in too, as a bonus touch.
"""

import time

import numpy as np
import pyaudio

SAMPLE_RATE = 44100
FADE_MS = 8  # short fade in/out on each note so notes don't click/pop


def _tone(freq_hz, duration_s, volume=0.4):
    n_samples = int(SAMPLE_RATE * duration_s)
    t = np.arange(n_samples) / SAMPLE_RATE
    wave = np.sin(2 * np.pi * freq_hz * t).astype(np.float32)

    fade_samples = int(SAMPLE_RATE * FADE_MS / 1000)
    fade_samples = min(fade_samples, n_samples // 2)
    if fade_samples > 0:
        fade_in = np.linspace(0, 1, fade_samples)
        fade_out = np.linspace(1, 0, fade_samples)
        wave[:fade_samples] *= fade_in
        wave[-fade_samples:] *= fade_out

    return (wave * volume)


def play_song(notes):
    """notes: list of (freq_hz, duration_s) tuples. freq_hz=0 is a rest."""
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paFloat32, channels=1, rate=SAMPLE_RATE, output=True)
    try:
        for freq_hz, duration_s in notes:
            if freq_hz <= 0:
                stream.write(np.zeros(int(SAMPLE_RATE * duration_s), dtype=np.float32).tobytes())
            else:
                stream.write(_tone(freq_hz, duration_s).tobytes())
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


# --- The two songs -----------------------------------------------------------
# Simple, distinct melodic shapes: victory rises, defeat falls. Swap these
# for whatever you like -- just keep them clearly different from each other
# so a listener can tell them apart without seeing the screen.

SUCCESS_SONG = [
    (523, 0.12),  # C5
    (659, 0.12),  # E5
    (784, 0.12),  # G5
    (1047, 0.25),  # C6, held
]

DEATH_SONG = [
    (392, 0.15),  # G4
    (349, 0.15),  # F4
    (330, 0.15),  # E4
    (262, 0.35),  # C4, held, low and final
]


def play_success_song():
    play_song(SUCCESS_SONG)


def play_death_song():
    play_song(DEATH_SONG)


# --- Optional bonus: have the physical hub chime in too ---------------------

def beep_hub_flourish(hub, success):
    """hub: anything with a legoeducation-style beep(frequency, duration_ms).
    Best-effort -- swallows errors so a hub hiccup never blocks the song."""
    try:
        if success:
            hub.beep(frequency=1047, duration=200)
        else:
            hub.beep(frequency=262, duration=400)
    except Exception as exc:
        print(f"[songs] hub beep failed (non-fatal): {exc}")


if __name__ == "__main__":
    print("Playing success song...")
    play_success_song()
    time.sleep(0.3)
    print("Playing death song...")
    play_death_song()
