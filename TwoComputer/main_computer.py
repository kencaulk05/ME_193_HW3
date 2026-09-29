"""
main_computer.py

Runs on the MAIN computer -- the one physically connected to the LEGO car.
Owns the Double Motor over BLE (the only script in this pair that ever
should, same reasoning as ME_193_HW3: BLE only allows one connection to a
hub at a time) and drives it by blending two independent whistle signals:

  - Its OWN microphone: a whistle in the FORWARD band drives forward, no
    whistle (or anything outside a recognized band) stops.
  - The REMOTE computer's whistle (remote_computer.py), delivered over MQTT
    on config.TURN_TOPIC: while "TURNING", adds a clockwise differential on
    top of whatever the local forward/stop state already is -- so
    forward+turning arcs to the right, and stopped+turning pivots in place.

On top of driving, this also plays the "ball"/"goalie" World Cup game from
the assignment, over MQTT on config.GAME_TOPIC:

  - --role ball: owns the forward Color Sensor. If the goalie gets close
    enough (reflection crosses PROXIMITY_REFLECTION_THRESHOLD), shuts down,
    publishes MSG_BALL_TAGGED, and plays the death song. A second,
    higher-pitched whistle band (GOAL_MIN_FREQ-GOAL_MAX_FREQ, distinct from
    the FORWARD band and from remote_computer.py's own range) is a one-shot
    "I scored" signal -- publishes MSG_BALL_SCORED and plays the success
    song instead.
  - --role goalie: no sensor. Reacts to the ball's messages instead --
    MSG_BALL_TAGGED means the goalie won (success song), MSG_BALL_SCORED
    means the goalie lost (death song).

Driving itself is NOT gated on the referee's "start" message (so you can
test/position the car beforehand) -- only the tagged/scored game events
check game_active, matching ME_193_HW3's game_logic.py.

If the remote turn signal goes stale (remote_computer.py crashed, network
dropped, etc.) for more than REMOTE_SIGNAL_TIMEOUT, this fails safe to
"not turning" rather than getting stuck mid-turn.

Usage:
    python main_computer.py --role ball
    python main_computer.py --role goalie
    python main_computer.py --role ball --calibrate   # print reflection() only
    python main_computer.py --list-devices            # find your mic's device index
    python main_computer.py --device 2
    python main_computer.py --role ball --simulate    # no hardware needed, just prints
Close the plot window (or Ctrl+C) to quit (motors always stop and
disconnect cleanly).
"""

import argparse
import threading
import time

import legoeducation as le
import numpy as np
import pyaudio
from lelib import doubleMotor, colorSensor

import config as cfg
import songs
from mqttlib import MQTTClient
from whistle_detector import WhistleDetector
from whistle_display import WhistleMonitor

SAMPLE_RATE = 44100
CHUNK = 1024

# --- Local whistle sub-bands (this computer's own mic) -----------------------
# FORWARD drives the car; GOAL is a distinct, one-shot "I scored" event, not
# a drive speed. Deliberately kept separate from remote_computer.py's own
# accepted range so a shared whistle in the room doesn't ambiguously trigger
# both computers at once -- re-tune all three scripts together if that ever
# changes. Kept wide-ish for now; narrow later as needed.
FORWARD_MIN_FREQ = 1000
FORWARD_MAX_FREQ = 2000
GOAL_MIN_FREQ = 3200
GOAL_MAX_FREQ = 5000

# Overall range the local detector searches -- must cover both sub-bands
# above (the detector's own noise/purity gates operate over this whole
# span; classify_local() below picks which sub-band a detected whistle
# actually falls into).
ANALYSIS_MIN_FREQ = FORWARD_MIN_FREQ
ANALYSIS_MAX_FREQ = GOAL_MAX_FREQ

DRIVE_SPEED = 50    # forward speed, 0-100, applied to both wheels
TURN_BIAS = 30      # added to the left wheel / subtracted from the right
                    # while turning -- bigger = tighter/faster turn

# If no message arrives on the turn topic for this long, treat the remote
# side as "not turning" instead of getting stuck mid-turn.
REMOTE_SIGNAL_TIMEOUT = 2.0

# Proximity/"tagged" threshold for the ball's forward Color Sensor.
# CALIBRATE THIS: run with --role ball --calibrate (no driving/game logic,
# just prints live reflection() readings) while a teammate's hand/robot
# approaches from the front at roughly the distance you want to count as
# "caught," and set this just above the resting (nobody-nearby) reading.
PROXIMITY_REFLECTION_THRESHOLD = 60

# The two physical motors are usually mounted mirrored on the chassis --
# same fix as ME_193_HW3's game_logic.py. Flip whichever side turns out to
# be wrong once you test it.
INVERT_LEFT_MOTOR = False
INVERT_RIGHT_MOTOR = True


def classify_local(freq):
    """Returns 'FORWARD', 'GOAL', or None (whistle heard, but outside both
    recognized bands -- treated as a no-op/stop) for this computer's own mic."""
    if FORWARD_MIN_FREQ <= freq <= FORWARD_MAX_FREQ:
        return "FORWARD"
    if GOAL_MIN_FREQ <= freq <= GOAL_MAX_FREQ:
        return "GOAL"
    return None


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


class ProximitySensor:
    """Ball role only: the forward-facing Color Sensor used to detect the
    goalie getting close enough to "tag" the ball out."""

    def __init__(self, simulate=False):
        self.simulate = simulate
        self.sensor = None

    def connect(self):
        if self.simulate:
            print("[sensor] SIMULATE mode: not connecting to real hardware.")
            return
        self.sensor = colorSensor()
        print("[sensor] Waiting for the Color Sensor (must match the card color/"
              "serial in config.py) -- power it on, tap the card, and bring "
              "it in range.")
        attempt = 0
        while True:
            attempt += 1
            try:
                self.sensor.connect(card_serial=cfg.CARD_SERIAL, card_color=cfg.CARD_COLOR)
                break
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(f"[sensor] Still not connected (attempt {attempt}): {exc} -- retrying...")
        print("[sensor] Connected.")

    def reflection(self):
        if self.simulate:
            return 0
        return self.sensor.reflection()

    def disconnect(self):
        if self.simulate or self.sensor is None:
            return
        try:
            self.sensor.disconnect()
        except Exception as exc:
            print(f"[sensor] Error while disconnecting: {exc}")


def mix(local_forward, remote_turning):
    """Blends the two independent driving signals into wheel speeds.

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
        description="Drives the car from this computer's own whistle plus a remote computer's turn "
                    "signal, and plays the ball/goalie World Cup game over MQTT")
    parser.add_argument("--role", required=True, choices=["ball", "goalie"])
    parser.add_argument("--device", type=int, default=None,
                         help="Input device index (see --list-devices)")
    parser.add_argument("--list-devices", action="store_true",
                         help="List available input devices and exit")
    parser.add_argument("--simulate", action="store_true",
                         help="Run without hardware; just print what would happen.")
    parser.add_argument("--calibrate", action="store_true",
                         help="Ball role only: just print live reflection() readings, no driving/game logic.")
    return parser.parse_args()


def run_calibration(sensor):
    print("Calibration mode: printing reflection() readings. Ctrl+C to stop.")
    print("Watch this while a hand/robot approaches from the front, and set")
    print("PROXIMITY_REFLECTION_THRESHOLD in this file just above the resting value.")
    try:
        while True:
            print(f"reflection: {sensor.reflection()}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


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

    sensor = None
    if args.role == "ball":
        sensor = ProximitySensor(simulate=args.simulate)
        sensor.connect()

    if args.calibrate:
        if args.role != "ball":
            print("--calibrate only applies to --role ball (that's the side with the sensor).")
            return
        run_calibration(sensor)
        car.disconnect()
        return

    lock = threading.Lock()
    remote_turning = False
    last_remote_message_time = 0.0  # 0 = "never received" -> treated as stale/not-turning
    game_active = False
    game_over = False

    def on_turn_message(topic, payload):
        nonlocal remote_turning, last_remote_message_time
        with lock:
            remote_turning = (payload == cfg.MSG_TURNING)
            last_remote_message_time = time.monotonic()

    def on_game_message(topic, payload):
        nonlocal game_active
        if payload == cfg.MSG_START:
            with lock:
                game_active = True
            print("[game] START received -- game is live.")
        elif payload == cfg.MSG_BALL_TAGGED:
            print(f"[game] Received: {payload}")
            if args.role == "goalie":
                songs.play_success_song()
        elif payload == cfg.MSG_BALL_SCORED:
            print(f"[game] Received: {payload}")
            if args.role == "goalie":
                songs.play_death_song()

    def handle_tagged(mqtt_client):
        nonlocal game_over
        print("[game] Tagged! Goalie got too close.")
        with lock:
            game_over = True
        car.stop()
        mqtt_client.publish(cfg.GAME_TOPIC, cfg.MSG_BALL_TAGGED)
        songs.play_death_song()

    def handle_score(mqtt_client):
        nonlocal game_over
        print("[game] Scored! Whistle signaled a goal.")
        with lock:
            game_over = True
        car.stop()
        mqtt_client.publish(cfg.GAME_TOPIC, cfg.MSG_BALL_SCORED)
        songs.play_success_song()

    mqtt_client = connect_mqtt()
    mqtt_client.subscribe(cfg.TURN_TOPIC, on_turn_message)
    mqtt_client.subscribe(cfg.GAME_TOPIC, on_game_message)
    print(f"[main] Role: {args.role}")
    print(f"[main] Listening for remote turn signal on '{cfg.TURN_TOPIC}'")
    print(f"[main] Listening for game messages on '{cfg.GAME_TOPIC}'")

    detector = WhistleDetector(analysis_min_freq=ANALYSIS_MIN_FREQ, analysis_max_freq=ANALYSIS_MAX_FREQ)
    monitor = WhistleMonitor(
        title=f"Main Computer ({args.role}) - Whistle Monitor",
        analysis_min_freq=ANALYSIS_MIN_FREQ, analysis_max_freq=ANALYSIS_MAX_FREQ,
        chunk_size=CHUNK,
        bands=[
            (FORWARD_MIN_FREQ, FORWARD_MAX_FREQ, "FORWARD", "#22c55e"),
            (GOAL_MIN_FREQ, GOAL_MAX_FREQ, "GOAL", "#ef4444"),
        ],
    )

    def audio_loop():
        last_left, last_right = None, None
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paFloat32, channels=1, rate=SAMPLE_RATE,
                         input=True, input_device_index=args.device, frames_per_buffer=CHUNK)

        print("Whistle (FORWARD band) to drive forward, (GOAL band) to signal a goal. "
              "Remote whistle adds a clockwise turn. Close the window (or Ctrl+C) to quit.")
        try:
            while monitor.running:
                data = stream.read(CHUNK, exception_on_overflow=False)
                chunk = np.frombuffer(data, dtype=np.float32)

                is_whistle, freq = detector.update(chunk)
                local_class = classify_local(freq) if is_whistle else None
                local_forward = (local_class == "FORWARD")

                with lock:
                    turning = remote_turning
                    stale = (last_remote_message_time == 0.0
                              or time.monotonic() - last_remote_message_time > REMOTE_SIGNAL_TIMEOUT)
                    active = game_active
                    over = game_over
                if stale:
                    turning = False

                # Ball-only: proximity tag check.
                if args.role == "ball" and active and not over:
                    if sensor.reflection() > PROXIMITY_REFLECTION_THRESHOLD:
                        handle_tagged(mqtt_client)
                        over = True

                # Ball-only: GOAL whistle check (one-shot -- handle_score()
                # sets game_over, which blocks this from firing again).
                if args.role == "ball" and local_class == "GOAL" and active and not over:
                    handle_score(mqtt_client)
                    over = True

                # Once tagged/scored, stay stopped/non-responsive for the
                # rest of the round rather than keep driving.
                if over:
                    local_forward = False
                    turning = False

                left, right = mix(local_forward, turning)
                status_label = (f"[{args.role.upper()}] {local_class or 'STOP'}/"
                                 f"{'TURN' if turning else 'straight'}"
                                 f"{' (GAME OVER)' if over else ''}")
                monitor.push(detector, status_label=status_label,
                             status_color="#22c55e" if local_forward else "#888888")

                if (left, right) != (last_left, last_right):
                    print(f"[main] local={local_class or 'STOP':7s}  "
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
        if sensor:
            sensor.disconnect()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
