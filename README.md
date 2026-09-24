# Whistle-Controlled Soccer

Drive a LEGO Double Motor car by whistling at it, then play a World-Cup-style
game over MQTT: a "ball" car tries to whistle its way into the goal, a
"goalie" car tries to get close enough to the ball's forward light sensor
to tag it out first.

## Why two separate scripts

BLE only allows **one active connection to a given hub at a time**. If the
whistle listener and the game/motor logic were both trying to `connect()`
to the same Double Motor as two independent processes, the second one
would fail. So:

- **`whistle_drive.py`** — owns only the microphone. Listens, classifies
  the whistle's pitch, and **publishes** a driving command over MQTT. Never
  touches the hub.
- **`game_logic.py`** — owns the actual BLE connection (motors + light
  sensor). **Subscribes** to that command channel and executes whatever
  `whistle_drive.py` publishes, and separately handles the cross-team game
  messages on `ME193/Rogers`.

Since MQTT doesn't care whether publisher and subscriber are on the same
machine, this same split is also exactly how the 4-point version works:
each teammate runs their own `whistle_drive.py` on their own laptop, both
publishing to the same team drive topic, while a single `game_logic.py`
(on whichever laptop is actually paired with the hub) executes commands
from either.

## Files

- `game_config.py` — shared topic names and command strings both scripts import, so they can't drift out of sync.
- `songs.py` — cross-platform tone player (built on pyaudio) for the death/victory songs.
- `whistle_drive.py` — real-time mic listener + live signal/decision plot + MQTT publisher.
- `game_logic.py` — hub connection, motor control, light sensor polling, game state machine.
- `mqttlib.py`, `lelib.py` — provided course helper libraries.

## Setup

```bash
pip install -r requirements.txt
```

1. Open `game_config.py` and set `TEAM_NAME` to something unique (avoids
   colliding with other teams on the public `test.mosquitto.org` broker).
2. Agree on `MSG_BALL_TAGGED` / `MSG_BALL_SCORED` with your opponent team
   before the actual game, and update those two constants to match.

## Calibrating your whistle bands

Pitch varies a lot person to person. Use the provided `whistle_recorder.py`
to check your own range: whistle low, medium, high, and very high, and read
off the printed "Dominant frequency" each time. Then adjust the `BAND_*`
constants near the top of `whistle_drive.py` to match your actual range —
the defaults are just starting points.

## Calibrating the proximity threshold

```bash
python game_logic.py --role ball --calibrate
```
This just prints live `reflection()` readings without driving or playing
anything. Watch the numbers while a hand/robot approaches the sensor from
the distance you want to count as "tagged," and set
`PROXIMITY_REFLECTION_THRESHOLD` in `game_logic.py` just above the resting
(nobody-nearby) value.

## Running it

Two terminals (or two laptops):
```bash
python whistle_drive.py --list-devices   # if you need to pick a mic
python whistle_drive.py
```
```bash
python game_logic.py --role ball      # or --role goalie
```
`--simulate` on `game_logic.py` tests everything (MQTT, state machine,
songs) without hardware connected.

---

## Reflection questions

### 1. Describe the policy — how does it make decisions?

It's a **frequency-band classifier with debouncing and a fail-safe timeout**,
not a continuous controller — every ~23ms audio chunk gets folded into a
rolling FFT window, and the dominant frequency (ignoring rumble below
300Hz) gets bucketed into one of five bands:

| Frequency range | Command |
|---|---|
| below 700 Hz | `STOP` |
| 700–1300 Hz | `LEFT` |
| 1300–2000 Hz | `RIGHT` |
| 2000–3200 Hz | `FORWARD` |
| above 3200 Hz | `GOAL` (special "I scored" signal, not a drive speed) |

A classification only "commits" once the **same band has been seen for 3
consecutive frames** (debouncing), so a whistle sliding through the middle
frequencies on its way up doesn't cause a flicker of spurious left/right
commands. The committed command gets published over MQTT only when it
actually changes, so the channel isn't flooded with duplicate messages —
`game_logic.py` just keeps acting on the last command it received.

### 2. What does your code do if no whistle is detected?

Two layers, matching the pattern from earlier assignments:

- **In `whistle_drive.py`**: if nothing above the noise/silence gates has
  been heard for more than `NO_WHISTLE_TIMEOUT` (1 second), it stops
  waiting for the debounce to naturally decay and explicitly forces the
  published command to `STOP`, regardless of what the last stable command
  was.
- **In `game_logic.py`**: a second, independent fail-safe — if no drive
  message arrives at all for `DRIVE_COMMAND_TIMEOUT` (2 seconds), it stops
  the motors itself. This covers the case the first layer can't: if
  `whistle_drive.py` crashes, loses its mic, or the MQTT connection drops
  entirely, there's nothing left to *publish* a STOP — so the executor
  independently notices the silence and fails safe on its own.

### 3. How did you try to mask out unwanted noise?

Three separate gates, all of which have to pass before a frame counts as
"a whistle" at all:

1. **Frequency range gate** — only search for a peak between 300–5000 Hz,
   which cuts out low rumble (footsteps, HVAC, handling the mic) and
   ignores anything above typical whistle range.
2. **Amplitude (RMS) gate** — quiet background noise can still have *some*
   dominant frequency, so a loudness floor (`SILENCE_RMS`) rejects anything
   too quiet to plausibly be a deliberate whistle.
3. **Spectral purity gate** — this is the one that actually distinguishes a
   whistle from other *loud* sounds, like talking or a clap. A whistle is
   close to a pure tone, so almost all of its energy sits in one narrow
   peak. We compute `purity = energy within ±60Hz of the peak / total
   energy in the analysis range` and require it to clear a threshold
   (0.35). Broadband sounds spread their energy across many frequencies
   and fail this even when they're loud enough to pass the RMS gate.

On top of the gates, the debounce requirement (3 consecutive matching
frames) acts as a second, temporal layer of noise rejection — a single
stray frame that slips past all three gates still can't flip the
committed command on its own.
