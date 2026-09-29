# Two-Computer Whistle Control

Two separate computers, each with their own microphone, jointly drive one
LEGO car over MQTT:

- **Main computer** (`main_computer.py`) is physically connected to the
  Double Motor over BLE. Its own whistle drives the car (in a FORWARD
  frequency band; a separate, higher GOAL band signals scoring instead of
  driving -- see "World Cup game" below), and it also subscribes to a
  shared MQTT topic for a "turning" signal from the other computer.
- **Remote computer** (`remote_computer.py`) never touches the hub at
  all. It just listens to its own microphone and publishes "TURNING" /
  "NOT_TURNING" to that shared topic whenever it hears a whistle.

The two signals are **blended**, not exclusive:

| Local (own whistle) | Remote (other computer's whistle) | Result |
|---|---|---|
| stop | not turning | stopped |
| forward | not turning | straight forward |
| stop | turning | pivot in place (clockwise) |
| forward | turning | arcs forward, clockwise |

Both scripts share the exact same tuned whistle-detection logic (adaptive
noise floor, hysteresis, spectral purity gate, frequency smoothing) via
`whistle_detector.py`, factored out of the ME_193_HW3 project so neither
script has to duplicate that ~150 lines of noise-masking logic. Both also
show a live waveform + spectrogram + spectrum monitor (`whistle_display.py`)
with the accepted frequency range shaded in green, so you can see what
each mic is picking up and whether it falls inside that range.

## Setup (on both computers)

This folder lives under a path containing a `:` (`MATLAB:Code For Tufts`),
and Python's `venv` refuses to create an environment inside a path with a
colon in it (colon is the `PATH` separator). So the virtual environment
lives outside that path, in your home directory:

```bash
python3 -m venv ~/.venvs/me193twocomputer
source ~/.venvs/me193twocomputer/bin/activate
pip install -r requirements.txt
```

`pyaudio` needs the `portaudio` C library installed first if `pip
install` fails to build it:
```bash
brew install portaudio
```

## Configure

Open `config.py` and set:
- `TEAM_NAME` to something unique (avoids colliding with anyone else's
  traffic on the public `test.mosquitto.org` broker -- this defines
  `TURN_TOPIC`, e.g. `ME193/{TEAM_NAME}/turn`).
- `CARD_COLOR` / `CARD_SERIAL` to match the LEGO connection card tapped to
  the Double Motor (only needed on the main computer).

Both computers need the same `config.py` (same `TEAM_NAME`) so they're
talking on the same MQTT topic.

## World Cup game (ball/goalie)

`main_computer.py` also plays the assignment's ball/goalie game over MQTT
on `config.GAME_TOPIC` ("ME193/Rogers", fixed by the assignment):

- **`--role ball`** connects the forward-facing Color Sensor too. If the
  goalie's car gets close enough (`reflection()` crosses
  `PROXIMITY_REFLECTION_THRESHOLD`), the car shuts down, publishes
  `MSG_BALL_TAGGED`, and plays the death song. Whistling in the **GOAL**
  band (separate from the FORWARD band -- see Tuning) is a one-shot "I
  scored" signal: publishes `MSG_BALL_SCORED` and plays the success song
  instead. Either event ends the round -- the car stops responding to
  further whistles until you restart the script.
- **`--role goalie`** has no sensor; it just reacts to the ball's
  messages -- `MSG_BALL_TAGGED` (goalie won) plays the success song,
  `MSG_BALL_SCORED` (goalie lost) plays the death song.

Driving itself isn't gated on the referee's `"start"` message (so you can
test/position the car beforehand) -- only the tagged/scored events check
that the game is actually active.

Agree on the exact `MSG_BALL_TAGGED`/`MSG_BALL_SCORED` strings with your
opponent team before the real match (see the placeholders in `config.py`),
and calibrate `PROXIMITY_REFLECTION_THRESHOLD` first:
```bash
python3 main_computer.py --role ball --calibrate
```
This just prints live `reflection()` readings with no driving/game logic
-- watch the numbers while a hand/robot approaches from the front at the
distance you want to count as "tagged," and set the threshold in
`main_computer.py` just above the resting (nobody-nearby) value.

## Run

On the remote computer:
```bash
source ~/.venvs/me193twocomputer/bin/activate
python3 remote_computer.py
# --list-devices first if you need to pick a specific mic
```

On the main computer (needs the Double Motor connected, and the Color
Sensor too if `--role ball`):
```bash
source ~/.venvs/me193twocomputer/bin/activate
python3 main_computer.py --role ball      # or --role goalie
# --simulate to test the logic without hardware
```

Order doesn't matter much -- each side retries its MQTT connection until
confirmed, and `main_computer.py` treats a missing/stale remote signal as
"not turning" (see `REMOTE_SIGNAL_TIMEOUT`) rather than getting stuck
mid-turn if the remote computer isn't running yet or drops out.

Each script opens its own live monitor window; close that window (or
Ctrl+C) to quit -- the mic-reading loop runs on a background thread, so
closing the window is what actually stops everything cleanly (motors
stopped, MQTT disconnected).

## Tuning

- `remote_computer.py`'s `ANALYSIS_MIN_FREQ`/`ANALYSIS_MAX_FREQ` -- the
  remote computer's own accepted whistle range for the turn signal.
- `main_computer.py`'s `FORWARD_MIN_FREQ`/`FORWARD_MAX_FREQ` and
  `GOAL_MIN_FREQ`/`GOAL_MAX_FREQ` -- two separate sub-bands on the main
  computer's own mic (FORWARD drives; GOAL is the one-shot scoring
  signal). Both are deliberately kept clear of `remote_computer.py`'s own
  range so a shared whistle in the room can't ambiguously trigger both
  computers -- re-tune all three together if you change any of them. Each
  band is shaded and labeled on that script's own monitor window.
- `DRIVE_SPEED`, `TURN_BIAS` in `main_computer.py` -- forward speed and how
  much the turn signal biases the two wheels apart. Bigger `TURN_BIAS` =
  tighter/faster turn.
- `REMOTE_SIGNAL_TIMEOUT` -- how long without a message before the remote
  side is treated as stale/not-turning.
- `PROXIMITY_REFLECTION_THRESHOLD` in `main_computer.py` -- see the
  calibration steps above.
- Other whistle detection constants (noise floor, hysteresis, purity
  threshold) live in `whistle_detector.py` -- see ME_193_HW3's README for
  the full explanation of what each one does and why.
- `INVERT_LEFT_MOTOR` / `INVERT_RIGHT_MOTOR` in `main_computer.py` --
  flip whichever side turns out backwards once you test on real hardware
  (the two motors are usually mounted mirrored on the chassis).
