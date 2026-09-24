"""
game_logic.py

Owns the actual hardware connection (this is the ONLY script that should
ever call dm.connect() -- BLE only allows one active connection to a given
hub, so whistle_drive.py never touches the motors directly; it just
publishes commands over MQTT for this script to execute).

Responsibilities:
  - Subscribe to DRIVE_TOPIC and turn whistle_drive.py's commands into
    actual motor speeds.
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
import time

import legoeducation as le
from lelib import doubleMotor, colorSensor

from mqttlib import MQTTClient
import game_config as cfg
import songs

# ---------------------------------------------------------------------------
# Tunable settings
# ---------------------------------------------------------------------------

DRIVE_SPEED = 50         # forward speed, 0-100
TURN_SPEED = 40          # per-wheel speed while turning in place

# Proximity/"tagged" threshold for the ball's forward Color Sensor.
# CALIBRATE THIS: run with --simulate off, print reflection() values (see
# the --calibrate flag below) while a teammate's hand/robot approaches from
# the front at roughly the distance you want to count as "caught," and set
# this just above the resting (no-one-nearby) reading.
PROXIMITY_REFLECTION_THRESHOLD = 60

# If no drive command has arrived in this long, stop the motors -- protects
# against whistle_drive.py crashing or an MQTT hiccup leaving the robot
# running blind.
DRIVE_COMMAND_TIMEOUT = 2.0  # seconds

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


def apply_drive_command(car, command):
    if command == cfg.CMD_FORWARD:
        car.drive(DRIVE_SPEED, DRIVE_SPEED)
    elif command == cfg.CMD_LEFT:
        car.drive(-TURN_SPEED, TURN_SPEED)
    elif command == cfg.CMD_RIGHT:
        car.drive(TURN_SPEED, -TURN_SPEED)
    elif command == cfg.CMD_STOP:
        car.stop()
    # CMD_GOAL is handled separately in on_drive_message -- it's an event,
    # not a speed to apply.


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
# Game state machine
# ---------------------------------------------------------------------------

class GameState:
    def __init__(self, role):
        self.role = role
        self.game_active = False
        self.game_over = False
        self.last_drive_command = cfg.CMD_STOP
        self.last_drive_time = time.monotonic()


def on_drive_message(topic, payload, car, state):
    state.last_drive_time = time.monotonic()

    if payload == cfg.CMD_GOAL:
        if state.role == "ball" and state.game_active and not state.game_over:
            handle_score(car, state, mqtt_client=state.mqtt_client)
        return  # never a literal drive speed, don't fall through to apply_drive_command

    state.last_drive_command = payload
    apply_drive_command(car, payload)


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
                    help="Ball role only: just print live reflection() readings, no driving/game logic.")
    return p.parse_args()


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
        return

    state = GameState(role=args.role)
    mqtt_client = MQTTClient()
    mqtt_client.connect()
    state.mqtt_client = mqtt_client

    mqtt_client.subscribe(cfg.DRIVE_TOPIC, lambda t, p: on_drive_message(t, p, car, state))
    mqtt_client.subscribe(cfg.GAME_TOPIC, lambda t, p: on_game_message(t, p, state))

    print(f"[game] Role: {args.role}")
    print(f"[game] Listening for drive commands on '{cfg.DRIVE_TOPIC}'")
    print(f"[game] Listening for game messages on '{cfg.GAME_TOPIC}'")
    print("Waiting for 'start'...")

    try:
        while True:
            now = time.monotonic()

            # Fail-safe: whistle_drive.py went quiet -> stop.
            if now - state.last_drive_time > DRIVE_COMMAND_TIMEOUT and state.last_drive_command != cfg.CMD_STOP:
                print("[game] No drive command recently -- failing safe to STOP.")
                car.stop()
                state.last_drive_command = cfg.CMD_STOP

            # Ball-only: poll the forward light sensor for "tagged out."
            if args.role == "ball" and state.game_active and not state.game_over:
                if sensor.reflection() > PROXIMITY_REFLECTION_THRESHOLD:
                    handle_tagged(car, state, mqtt_client)

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.disconnect()
        if sensor:
            sensor.disconnect()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
