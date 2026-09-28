"""
test_drive.py

Standalone hardware smoke test: connects to the Double Motor (using the
card color/serial in game_config.py) and drives it through a short
forward/turn/backward sequence -- completely independent of
whistle_drive.py, MQTT, and the game state machine, so you can confirm the
car itself connects and actually drives before debugging anything upstream
of it.

Usage:
    python test_drive.py
Ctrl+C to stop early (motors are always stopped and disconnected cleanly).
"""

import time

import legoeducation as le
from lelib import doubleMotor

import game_config as cfg

DRIVE_SPEED = 50
TURN_SPEED = 40
STEP_SECONDS = 1.5

# Mirrors game_logic.py's motor mounting fix -- flip whichever side turns
# out to be wrong once you see the actual behavior here.
INVERT_LEFT_MOTOR = False
INVERT_RIGHT_MOTOR = True


def drive(dm, left_speed, right_speed):
    """Signed speed, default (CLOCKWISE) direction -- the pattern lelib.py's
    own comments confirm was tested working on real hardware."""
    left_speed = -left_speed if INVERT_LEFT_MOTOR else left_speed
    right_speed = -right_speed if INVERT_RIGHT_MOTOR else right_speed
    for motor_side, speed in ((le.MOTOR_LEFT, left_speed), (le.MOTOR_RIGHT, right_speed)):
        if abs(speed) < 1:
            dm.motor_stop(motor=motor_side)
        else:
            dm.motor_run(motor=motor_side, speed=float(speed), blocking=False)


def stop(dm):
    drive(dm, 0, 0)


def connect(dm):
    print("[test_drive] Waiting for the Double Motor (card color/serial from "
          "game_config.py) -- power it on, tap the card, and bring it in range.")
    attempt = 0
    while True:
        attempt += 1
        try:
            dm.connect(card_color=cfg.CARD_COLOR, card_serial=cfg.CARD_SERIAL)
            break
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            print(f"[test_drive] Still not connected (attempt {attempt}): {exc} -- retrying...")
    print("[test_drive] Connected.")


def main():
    dm = doubleMotor()
    connect(dm)

    try:
        steps = [
            ("forward", DRIVE_SPEED, DRIVE_SPEED),
            ("stop", 0, 0),
            ("turn left", -TURN_SPEED, TURN_SPEED),
            ("stop", 0, 0),
            ("turn right", TURN_SPEED, -TURN_SPEED),
            ("stop", 0, 0),
            ("backward", -DRIVE_SPEED, -DRIVE_SPEED),
            ("stop", 0, 0),
        ]
        for label, left, right in steps:
            print(f"[test_drive] {label}: L={left:+.0f} R={right:+.0f}")
            drive(dm, left, right)
            time.sleep(STEP_SECONDS)
    except KeyboardInterrupt:
        print("[test_drive] Interrupted.")
    finally:
        stop(dm)
        dm.disconnect()
        print("[test_drive] Stopped and disconnected.")


if __name__ == "__main__":
    main()
