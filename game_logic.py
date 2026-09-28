"""
game_logic.py

Owns the actual hardware connection (this is the ONLY script that should
ever call dm.connect() -- BLE only allows one active connection to a given
hub, so whistle_drive.py never touches the motors directly; it just
publishes commands over MQTT for this script to execute).

Responsibilities:
  - Subscribe to STEER_TOPIC and THROTTLE_TOPIC (one whistler per channel,
    each with their own laptop and their own tuned pitch bands) and mix
    the two most recent commands together into actual wheel speeds.
  - Subscribe to GAME_TOPIC for the referee's "start" message and for the
    opponent's/your own win-loss announcements.
  - If role == "ball": poll the forward-facing Color Sensor. A goalie
    getting close enough spikes the reflectivity reading -- that's how
    "tagged out" is detected. Also watch for the CMD_GOAL special whistle
    (a distinct command, not a drive instruction) meaning "I reached the
    goal."
  - Fail safe: if no drive command arrives for a while (whistle_drive.py
    crashed, MQTT hiccup, etc.), stop the motors rather than keep running
    whatever was last commanded.

Usage:
    python game_logic.py --role ball
    python game_logic.py --role goalie
    python game_logic.py --role ball --simulate   # no hardware needed
Ctrl+C to quit.
"""

import argparse
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import legoeducation as le
from lelib import doubleMotor, colorSensor

from mqttlib import MQTTClient
import game_config as cfg
import songs

# ---------------------------------------------------------------------------
# Tunable settings
# ---------------------------------------------------------------------------



# Proximity/"tagged" threshold for the ball's forward Color Sensor.
# CALIBRATE THIS: run with --simulate off, print reflection() values (see
# the --calibrate flag below) while a teammate's hand/robot approaches from
# the front at roughly the distance you want to count as "caught," and set
# this just above the resting (no-one-nearby) reading.
PROXIMITY_REFLECTION_THRESHOLD = 60

# If a channel has been silent this long, it fails safe to a neutral
# default -- protects against that whistle_drive.py crashing or an MQTT
# hiccup leaving the robot running blind. Each channel gets its OWN timer
# and its OWN safe default, since they're two independent people/laptops
# now and either one can go quiet without the other noticing:
#   throttle silent -> defaults to STOP (stop moving, full safety default)
#   steer silent    -> defaults to STRAIGHT (stop turning, but don't fight
#                      whatever the throttle channel is still commanding)
DRIVE_COMMAND_TIMEOUT = 2.0  # seconds

DRIVE_SPEED = 50   # base forward speed, 0-100 (throttle = FORWARD)
TURN_BIAS = 25     # how much the steer command biases the two wheels apart,
                   # on top of whatever the throttle channel is doing

# Live sensor plot (ball role only): how many recent reflection() samples to
# keep on screen, and how often the plot redraws.
PLOT_HISTORY_LEN = 200
PLOT_UPDATE_INTERVAL_MS = 100

# The two physical motors are usually mounted mirrored on the chassis --
# see arm_race.py/apriltag_center.py for the same fix. Flip whichever side
# turns out to be wrong once you test it.
INVERT_LEFT_MOTOR = False
INVERT_RIGHT_MOTOR = True


# ---------------------------------------------------------------------------
# Motor control
# ---------------------------------------------------------------------------

class Car:
    def __init__(self, simulate=False):
        self.simulate = simulate
        self.dm = None

    def connect(self):
        if self.simulate:
            print("[car] SIMULATE mode: not connecting to real hardware.")
            return
        self.dm = doubleMotor()
        print("[car] Scanning for the Double Motor hub...")
        self.dm.connect()
        print("[car] Connected.")

    def drive(self, left_speed, right_speed):
        left_speed = float(-left_speed if INVERT_LEFT_MOTOR else left_speed)
        right_speed = float(-right_speed if INVERT_RIGHT_MOTOR else right_speed)
        if self.simulate:
            print(f"[car][sim] L={left_speed:+.0f} R={right_speed:+.0f}")
            return
        self._drive_side(le.MOTOR_LEFT, left_speed)
        self._drive_side(le.MOTOR_RIGHT, right_speed)

    def _drive_side(self, motor_side, speed):
        if abs(speed) < 1:
            self.dm.motor_stop(motor=motor_side)
        else:
            direction = (le.MOTOR_MOVE_DIRECTION_CLOCKWISE if speed > 0
                         else le.MOTOR_MOVE_DIRECTION_COUNTERCLOCKWISE)
            self.dm.motor_run(direction=direction, motor=motor_side,
                               speed=float(abs(speed)), blocking=False)

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


def compute_wheel_speeds(throttle_command, steer_command):
    """Mix the two most recent independent commands into wheel speeds --
    same idea as apriltag_chase.py's steer+approach mixing: a base speed
    common to both wheels, with a steering bias added to one and
    subtracted from the other.

    Note steering still applies even at zero throttle (STOP + LEFT spins
    the car in place, one wheel forward and one back) -- that's
    deliberate, so the steerer can still reposition the car while stopped,
    rather than steering being locked out unless the throttle side is
    actively driving forward.
    """
    base_speed = DRIVE_SPEED if throttle_command == cfg.CMD_FORWARD else 0

    if steer_command == cfg.CMD_LEFT:
        bias = -TURN_BIAS
    elif steer_command == cfg.CMD_RIGHT:
        bias = TURN_BIAS
    else:  # CMD_STRAIGHT
        bias = 0

    return base_speed - bias, base_speed + bias


def apply_combined_drive(car, state):
    left, right = compute_wheel_speeds(state.last_throttle_command, state.last_steer_command)
    car.drive(left, right)


# ---------------------------------------------------------------------------
# Light sensor (ball role only)
# ---------------------------------------------------------------------------

class ProximitySensor:
    def __init__(self, simulate=False):
        self.simulate = simulate
        self.sensor = None

    def connect(self):
        if self.simulate:
            print("[sensor] SIMULATE mode: not connecting to real hardware.")
            return
        self.sensor = colorSensor()
        print("[sensor] Scanning for the Color Sensor...")
        self.sensor.connect(card_serial=None)
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


# ---------------------------------------------------------------------------
# Live sensor plot (ball role only) -- shows reflection() over time against
# PROXIMITY_REFLECTION_THRESHOLD, so a "tagged" detection is visible the
# instant it happens instead of only showing up as a console line.
# ---------------------------------------------------------------------------

def build_sensor_plot():
    fig, ax = plt.subplots(figsize=(9, 5))
    line, = ax.plot([], [], color="#3b82f6", linewidth=1.5)
    ax.axhline(PROXIMITY_REFLECTION_THRESHOLD, color="red", linestyle="--",
               linewidth=1.2, label=f"threshold ({PROXIMITY_REFLECTION_THRESHOLD})")
    ax.set_xlim(0, PLOT_HISTORY_LEN)
    ax.set_ylim(0, 100)
    ax.set_xlabel("Sample")
    ax.set_ylabel("Reflection")
    ax.set_title("Live proximity sensor")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    status_text = fig.text(0.02, 0.96, "", fontsize=13, family="monospace", va="top")
    return fig, ax, line, status_text


def run_live_plot(state, lock, running, get_status_line=lambda: ""):
    """Blocks (like plt.show()) until the plot window is closed or Ctrl+C.

    Reads `state["reflection"]`/`state["history"]`/`state["detected"]`,
    written elsewhere under `lock`, and sets `running["value"] = False` when
    the window closes so whatever thread is producing that state can stop.
    """
    fig, ax, line, status_text = build_sensor_plot()

    def update(_frame):
        with lock:
            history = list(state["history"])
            detected = state["detected"]
            reflection = state["reflection"]

        line.set_data(range(len(history)), history)

        color = "#ef4444" if detected else "#22c55e"
        status_text.set_color(color)
        status_text.set_text(
            f"{'OBJECT DETECTED' if detected else 'clear':<16}  reflection={reflection:5.1f}   "
            + get_status_line()
        )

        if not running["value"]:
            plt.close(fig)
        return line, status_text

    ani = FuncAnimation(fig, update, interval=PLOT_UPDATE_INTERVAL_MS, cache_frame_data=False)

    def on_close(_event):
        running["value"] = False

    fig.canvas.mpl_connect("close_event", on_close)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        running["value"] = False


# ---------------------------------------------------------------------------
# Game state machine
# ---------------------------------------------------------------------------

class GameState:
    def __init__(self, role):
        self.role = role
        self.game_active = False
        self.game_over = False
        # Two independent channels, two independent commands, two
        # independent "last heard from" timestamps -- see the fail-safe
        # note on DRIVE_COMMAND_TIMEOUT above for why they're separate.
        self.last_throttle_command = cfg.CMD_STOP
        self.last_throttle_time = time.monotonic()
        self.last_steer_command = cfg.CMD_STRAIGHT
        self.last_steer_time = time.monotonic()


def on_throttle_message(topic, payload, car, state):
    state.last_throttle_time = time.monotonic()

    if payload == cfg.CMD_GOAL:
        if state.role == "ball" and state.game_active and not state.game_over:
            handle_score(car, state, mqtt_client=state.mqtt_client)
        return  # an event, not a speed -- don't fall through to the mixer

    state.last_throttle_command = payload
    apply_combined_drive(car, state)


def on_steer_message(topic, payload, car, state):
    state.last_steer_time = time.monotonic()
    state.last_steer_command = payload
    apply_combined_drive(car, state)


def on_game_message(topic, payload, state):
    if payload == cfg.MSG_START:
        state.game_active = True
        print("[game] START received -- game is live.")
    elif payload == cfg.MSG_BALL_TAGGED:
        print(f"[game] Received: {payload}")
        if state.role == "goalie":
            songs.play_success_song()
    elif payload == cfg.MSG_BALL_SCORED:
        print(f"[game] Received: {payload}")
        if state.role == "goalie":
            songs.play_death_song()


def handle_tagged(car, state, mqtt_client):
    print("[game] Tagged! Goalie got too close.")
    state.game_over = True
    car.stop()
    mqtt_client.publish(cfg.GAME_TOPIC, cfg.MSG_BALL_TAGGED)
    songs.play_death_song()


def handle_score(car, state, mqtt_client):
    print("[game] Scored! Whistle signaled a goal.")
    state.game_over = True
    car.stop()
    mqtt_client.publish(cfg.GAME_TOPIC, cfg.MSG_BALL_SCORED)
    songs.play_success_song()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Execute whistle-driven commands and referee the game over MQTT")
    p.add_argument("--role", required=True, choices=["ball", "goalie"])
    p.add_argument("--simulate", action="store_true",
                    help="Run without hardware; just print what would happen.")
    p.add_argument("--calibrate", action="store_true",
                    help="Ball role only: show a live plot of reflection() readings, no driving/game logic.")
    return p.parse_args()


def run_calibration(sensor):
    print("Calibration mode: live plot of reflection() readings.")
    print("Close the plot window (or Ctrl+C) to stop.")
    print("Watch it while a hand/robot approaches from the front, and set")
    print("PROXIMITY_REFLECTION_THRESHOLD in this file just above the resting value.")

    lock = threading.Lock()
    state = {"reflection": 0.0, "history": deque(maxlen=PLOT_HISTORY_LEN), "detected": False}
    running = {"value": True}

    def poll_loop():
        while running["value"]:
            reflection = sensor.reflection()
            detected = reflection > PROXIMITY_REFLECTION_THRESHOLD
            with lock:
                state["reflection"] = reflection
                state["history"].append(reflection)
                state["detected"] = detected
            time.sleep(0.05)

    poll_thread = threading.Thread(target=poll_loop, daemon=True)
    poll_thread.start()

    run_live_plot(state, lock, running)

    running["value"] = False
    poll_thread.join(timeout=2)


def game_loop(args, car, sensor, state, mqtt_client, running, plot_state=None, plot_lock=None):
    """The fail-safe + sensor-polling loop, run either directly (goalie, no
    plot) or on a background thread while the main thread owns the live plot
    (ball role -- see main())."""
    while running["value"]:
        now = time.monotonic()

        # Fail-safe: each channel gets its OWN timeout and its OWN neutral
        # default, since either laptop can go quiet independently of the
        # other now (see the note on DRIVE_COMMAND_TIMEOUT above).
        changed = False
        if (now - state.last_throttle_time > DRIVE_COMMAND_TIMEOUT
                and state.last_throttle_command != cfg.CMD_STOP):
            print("[game] No throttle command recently -- failing safe to STOP.")
            state.last_throttle_command = cfg.CMD_STOP
            changed = True
        if (now - state.last_steer_time > DRIVE_COMMAND_TIMEOUT
                and state.last_steer_command != cfg.CMD_STRAIGHT):
            print("[game] No steer command recently -- failing safe to STRAIGHT.")
            state.last_steer_command = cfg.CMD_STRAIGHT
            changed = True
        if changed:
            apply_combined_drive(car, state)

        # Ball-only: poll the forward light sensor for "tagged out."
        if args.role == "ball":
            reflection = sensor.reflection()
            detected = reflection > PROXIMITY_REFLECTION_THRESHOLD
            if plot_state is not None:
                with plot_lock:
                    plot_state["reflection"] = reflection
                    plot_state["history"].append(reflection)
                    plot_state["detected"] = detected
            if state.game_active and not state.game_over and detected:
                handle_tagged(car, state, mqtt_client)

        time.sleep(0.05)


def main():
    args = parse_args()

    car = Car(simulate=args.simulate)
    car.connect()

    sensor = None
    if args.role == "ball":
        sensor = ProximitySensor(simulate=args.simulate)
        sensor.connect()

    if args.calibrate:
        if args.role != "ball":
            print("--calibrate only applies to --role ball (that's the side with the sensor).")
            sys.exit(1)
        run_calibration(sensor)
        car.disconnect()
        sensor.disconnect()
        return

    state = GameState(role=args.role)
    mqtt_client = MQTTClient()
    mqtt_client.connect()
    state.mqtt_client = mqtt_client

    mqtt_client.subscribe(cfg.THROTTLE_TOPIC, lambda t, p: on_throttle_message(t, p, car, state))
    mqtt_client.subscribe(cfg.STEER_TOPIC, lambda t, p: on_steer_message(t, p, car, state))
    mqtt_client.subscribe(cfg.GAME_TOPIC, lambda t, p: on_game_message(t, p, state))

    print(f"[game] Role: {args.role}")
    print(f"[game] Listening for throttle commands on '{cfg.THROTTLE_TOPIC}'")
    print(f"[game] Listening for steer commands on '{cfg.STEER_TOPIC}'")
    print(f"[game] Listening for game messages on '{cfg.GAME_TOPIC}'")
    print("Waiting for 'start'...")

    running = {"value": True}

    try:
        if args.role == "ball":
            # Run the fail-safe/sensor loop on a background thread so the
            # main thread is free to drive the live "object detected" plot.
            plot_lock = threading.Lock()
            plot_state = {"reflection": 0.0, "history": deque(maxlen=PLOT_HISTORY_LEN), "detected": False}

            loop_thread = threading.Thread(
                target=game_loop,
                args=(args, car, sensor, state, mqtt_client, running, plot_state, plot_lock),
                daemon=True,
            )
            loop_thread.start()

            def status_line():
                return (f"throttle={state.last_throttle_command:<8} steer={state.last_steer_command:<8}  "
                        f"game_active={state.game_active}  game_over={state.game_over}")

            run_live_plot(plot_state, plot_lock, running, get_status_line=status_line)

            running["value"] = False
            loop_thread.join(timeout=2)
        else:
            game_loop(args, car, sensor, state, mqtt_client, running)
    except KeyboardInterrupt:
        pass
    finally:
        running["value"] = False
        car.stop()
        car.disconnect()
        if sensor:
            sensor.disconnect()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()