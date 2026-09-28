"""
remote_computer.py

Runs on the SECOND computer. Listens to its own microphone; whenever it
hears a whistle, publishes "TURNING" to the shared turn topic
(config.TURN_TOPIC); otherwise publishes "NOT_TURNING". Never touches the
LEGO hub at all -- only main_computer.py does that.

Shows a live waveform + spectrogram + spectrum monitor (see
whistle_display.py) so you can see what this computer's mic is picking up
and whether it's within the accepted frequency range.

Usage:
    python remote_computer.py
    python remote_computer.py --list-devices     # find your mic's device index
    python remote_computer.py --device 2
Close the plot window (or Ctrl+C) to quit.
"""

import argparse
import threading
import time

import numpy as np
import pyaudio

from whistle_detector import WhistleDetector
from whistle_display import WhistleMonitor
from mqttlib import MQTTClient
import config as cfg

SAMPLE_RATE = 44100
CHUNK = 1024

# Accepted whistle frequency range for THIS computer only. Kept wide for
# now (covers essentially all normal whistling); narrow this once you want
# this computer to only respond to a specific pitch register. This is
# intentionally independent from main_computer.py's own
# ANALYSIS_MIN_FREQ/MAX_FREQ -- the two can be tuned to different ranges.
ANALYSIS_MIN_FREQ = 300
ANALYSIS_MAX_FREQ = 5000


def list_devices():
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info.get("maxInputChannels", 0) > 0:
            print(f"{i}: {info['name']}  (inputs: {info['maxInputChannels']})")
    p.terminate()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publishes a turn-clockwise signal whenever this mic hears a whistle")
    parser.add_argument("--device", type=int, default=None,
                         help="Input device index (see --list-devices)")
    parser.add_argument("--list-devices", action="store_true",
                         help="List available input devices and exit")
    return parser.parse_args()


def connect_mqtt():
    mqtt_client = MQTTClient()
    print(f"[remote] Connecting to MQTT broker ({mqtt_client.broker})...")
    attempt = 0
    while True:
        attempt += 1
        mqtt_client.connect()
        if mqtt_client.is_connected():
            break
        print(f"[remote] Still not connected (attempt {attempt}) -- retrying...")
        mqtt_client.disconnect()
    print(f"[remote] Connected to MQTT broker ({mqtt_client.broker}).")
    return mqtt_client


def main():
    args = parse_args()
    if args.list_devices:
        list_devices()
        return

    mqtt_client = connect_mqtt()
    print(f"[remote] Publishing turn state to '{cfg.TURN_TOPIC}'")

    detector = WhistleDetector(analysis_min_freq=ANALYSIS_MIN_FREQ, analysis_max_freq=ANALYSIS_MAX_FREQ)
    monitor = WhistleMonitor(title="Remote Computer - Whistle Monitor",
                              analysis_min_freq=ANALYSIS_MIN_FREQ,
                              analysis_max_freq=ANALYSIS_MAX_FREQ,
                              chunk_size=CHUNK)

    def audio_loop():
        last_published = None
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paFloat32, channels=1, rate=SAMPLE_RATE,
                         input=True, input_device_index=args.device, frames_per_buffer=CHUNK)

        print("Listening... whistle to signal TURNING. Close the window (or Ctrl+C) to quit.")
        try:
            while monitor.running:
                data = stream.read(CHUNK, exception_on_overflow=False)
                chunk = np.frombuffer(data, dtype=np.float32)

                is_whistle, freq = detector.update(chunk)
                state = cfg.MSG_TURNING if is_whistle else cfg.MSG_NOT_TURNING
                monitor.push(detector, status_label=state,
                             status_color="#f59e0b" if is_whistle else "#888888")

                if state != last_published:
                    mqtt_client.publish(cfg.TURN_TOPIC, state)
                    print(f"[remote] -> {state}  "
                          f"(peak={freq:.0f}Hz purity={detector.purity:.2f} rms={detector.rms:.3f})")
                    last_published = state
        finally:
            mqtt_client.publish(cfg.TURN_TOPIC, cfg.MSG_NOT_TURNING)
            time.sleep(0.2)  # give the "stop turning" message time to actually send
            stream.stop_stream()
            stream.close()
            p.terminate()

    audio_thread = threading.Thread(target=audio_loop, daemon=True)
    audio_thread.start()

    try:
        monitor.run()  # blocks on the main thread until the window is closed
    finally:
        monitor.running = False
        audio_thread.join(timeout=3)
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
