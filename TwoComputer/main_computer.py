"""
main_computer.py

Runs on the MAIN computer -- the one physically connected to the LEGO car.
Owns the Double Motor over BLE (the only script in this pair that ever
should, same reasoning as ME_193_HW3: BLE only allows one connection to a
hub at a time) and drives it by blending two independent whistle signals:

  - Its OWN microphone: whistle -> forward, no whistle -> stop.
  - The REMOTE computer's whistle (remote_computer.py), delivered over MQTT
    on config.TURN_TOPIC: while "TURNING", adds a clockwise differential on
    top of whatever the local forward/stop state already is -- so
    forward+turning arcs to the right, and stopped+turning pivots in place.

If the remote signal goes stale (remote_computer.py crashed, network
dropped, etc.) for more than REMOTE_SIGNAL_TIMEOUT, this fails safe to
"not turning" rather than getting stuck mid-turn.

Usage:
    python main_computer.py
    python main_computer.py --list-devices     # find your mic's device index
    python main_computer.py --device 2
    python main_computer.py --simulate         # no hardware needed, just prints
Ctrl+C to quit (motors always stop and disconnect cleanly).
"""

import argparse
import threading
import time

import legoeducation as le
import numpy as np
import pyaudio
from lelib import doubleMotor

import config as cfg
from mqttlib import MQTTClient
from whistle_detector import WhistleDetector
from whistle_display import WhistleMonitor

SAMPLE_RATE = 44100
CHUNK = 1024

# Accepted whistle frequency range for THIS computer only. Kept wide for
# now (covers essentially all normal whistling); narrow this once you want
# this computer to only respond to a specific pitch register. This is
# intentionally independent from remote_computer.py's own
# ANALYSIS_MIN_FREQ/MAX_FREQ -- the two can be tuned to different ranges.
ANALYSIS_MIN_FREQ = 300
ANALYSIS_MAX_FREQ = 5000

DRIVE_SPEED = 50    # forward speed, 0-100, applied to both wheels
TURN_BIAS = 30      # added to the left wheel / subtracted from the right
                    # while turning -- bigger = tighter/faster turn

# If no message arrives on the turn topic for this long, treat the remote
# side as "not turning" instead of getting stuck mid-turn.
REMOTE_SIGNAL_TIMEOUT = 2.0

# The two physical motors are usually mounted mirrored on the chassis --
# same fix as ME_193_HW3's game_logic.py. Flip whichever side turns out to
# be wrong once you test it.
INVERT_LEFT_MOTOR = False
INVERT_RIGHT_MOTOR = True


class Car:
    def __init__(self, simulate=False):
        self.simulate = simulate
        self.dm = None

    def connect(self):
        if self.simulate:
            print("[car] SIMULATE mode: not connecting to real hardware.")
            return
        self.dm = doubleMotor()
        print("[car] Waiting for the Double Motor (must match the card color/"
              "serial in config.py) -- power it on, tap the card, and bring "
              "it in range.")
        attempt = 0
        while True:
            attempt += 1
            try:
                self.dm.connect(card_color=cfg.CARD_COLOR, card_serial=cfg.CARD_SERIAL)
                break
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(f"[car] Still not connected (attempt {attempt}): {exc} -- retrying...")
        print("[car] Connected.")

    def drive(self, left_speed, right_speed):
        # Signed speed, default (CLOCKWISE) direction -- the pattern
        # confirmed working on real hardware for this project.
        left_speed = float(-left_speed if INVERT_LEFT_MOTOR else left_speed)
        right_speed = float(-right_speed if INVERT_RIGHT_MOTOR else right_speed)
        if self.simulate:
            print(f"[car][sim] L={left_speed:+.0f} R={right_speed:+.0f}")
            return
        for motor_side, speed in ((le.MOTOR_LEFT, left_speed), (le.MOTOR_RIGHT, right_speed)):
            if abs(speed) < 1:
                self.dm.motor_stop(motor=motor_side)
            else:
                self.dm.motor_run(motor=motor_side, speed=speed, blocking=False)

    def stop(self):
        self.drive(0, 0)

    def disconnect(self):
        if self.simulate or self.dm is None:
            return
        try:
            self.dm.motor_stop(motor=le.MOTOR_BOTH)
            self.dm.disconnect()
        except Exception as exc:
            print(f"[car] Error while disconnecting: {exc}")


def mix(local_forward, remote_turning):
    """Blends the two independent signals into wheel speeds.

    local_forward=False, remote_turning=False -> stopped
    local_forward=True,  remote_turning=False -> straight forward
    local_forward=False, remote_turning=True  -> pivot in place (clockwise)
    local_forward=True,  remote_turning=True  -> arcs forward, clockwise
    """
    base = DRIVE_SPEED if local_forward else 0.0
    bias = TURN_BIAS if remote_turning else 0.0
    left = max(-100.0, min(100.0, base + bias))
    right = max(-100.0, min(100.0, base - bias))
    return left, right


def list_devices():
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info.get("maxInputChannels", 0) > 0:
            print(f"{i}: {info['name']}  (inputs: {info['maxInputChannels']})")
    p.terminate()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drives the car from this computer's own whistle plus a remote computer's turn signal")
    parser.add_argument("--device", type=int, default=None,
                         help="Input device index (see --list-devices)")
    parser.add_argument("--list-devices", action="store_true",
                         help="List available input devices and exit")
    parser.add_argument("--simulate", action="store_true",
                         help="Run without hardware; just print what would happen.")
    return parser.parse_args()


def connect_mqtt():
    mqtt_client = MQTTClient()
    print(f"[main] Connecting to MQTT broker ({mqtt_client.broker})...")
    attempt = 0
    while True:
        attempt += 1
        mqtt_client.connect()
        if mqtt_client.is_connected():
            break
        print(f"[main] Still not connected (attempt {attempt}) -- retrying...")
        mqtt_client.disconnect()
    print(f"[main] Connected to MQTT broker ({mqtt_client.broker}).")
    return mqtt_client


def main():
    args = parse_args()
    if args.list_devices:
        list_devices()
        return

    car = Car(simulate=args.simulate)
    car.connect()

    lock = threading.Lock()
    remote_turning = False
    last_remote_message_time = 0.0  # 0 = "never received" -> treated as stale/not-turning

    def on_turn_message(topic, payload):
        nonlocal remote_turning, last_remote_message_time
        with lock:
            remote_turning = (payload == cfg.MSG_TURNING)
            last_remote_message_time = time.monotonic()

    mqtt_client = connect_mqtt()
    mqtt_client.subscribe(cfg.TURN_TOPIC, on_turn_message)
    print(f"[main] Listening for remote turn signal on '{cfg.TURN_TOPIC}'")

    detector = WhistleDetector(analysis_min_freq=ANALYSIS_MIN_FREQ, analysis_max_freq=ANALYSIS_MAX_FREQ)
    monitor = WhistleMonitor(title="Main Computer - Whistle Monitor",
                              analysis_min_freq=ANALYSIS_MIN_FREQ,
                              analysis_max_freq=ANALYSIS_MAX_FREQ,
                              chunk_size=CHUNK)

    def audio_loop():
        last_left, last_right = None, None
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paFloat32, channels=1, rate=SAMPLE_RATE,
                         input=True, input_device_index=args.device, frames_per_buffer=CHUNK)

        print("Whistle to drive forward. Remote whistle adds a clockwise turn. "
              "Close the window (or Ctrl+C) to quit.")
        try:
            while monitor.running:
                data = stream.read(CHUNK, exception_on_overflow=False)
                chunk = np.frombuffer(data, dtype=np.float32)

                local_forward, freq = detector.update(chunk)

                with lock:
                    turning = remote_turning
                    stale = (last_remote_message_time == 0.0
                              or time.monotonic() - last_remote_message_time > REMOTE_SIGNAL_TIMEOUT)
                if stale:
                    turning = False

                left, right = mix(local_forward, turning)
                status_label = (f"{'FORWARD' if local_forward else 'STOP'}/"
                                 f"{'TURN' if turning else 'straight'}")
                monitor.push(detector, status_label=status_label,
                             status_color="#22c55e" if local_forward else "#888888")

                if (left, right) != (last_left, last_right):
                    print(f"[main] local={'FORWARD' if local_forward else 'STOP':7s}  "
                          f"remote={'TURNING' if turning else 'not turning':11s}  "
                          f"-> L={left:+.0f} R={right:+.0f}")
                    car.drive(left, right)
                    last_left, last_right = left, right
        finally:
            car.stop()
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
        car.disconnect()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
