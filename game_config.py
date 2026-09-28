"""
Shared constants for the whistle-controlled soccer game.

Both whistle_drive.py and game_logic.py import from here, so the topic
names and command strings can never accidentally drift apart between the
two files (or between your laptop and your teammate's).
"""

# --- Identify your team -----------------------------------------------------
# Change this to something unique to your group so your local drive channel
# doesn't collide with another team's on the public test.mosquitto.org
# broker (anyone can publish/subscribe to any topic there).
TEAM_NAME = "changeme-team"

# --- MQTT topics -------------------------------------------------------------
# The shared cross-team game channel (given in the assignment).
GAME_TOPIC = "ME193/Rogers"

# Two separate channels, one per person, since everyone's natural whistle
# range is different -- trying to share one 5-band scheme meant whoever's
# range didn't reach a given band just couldn't produce that command. Each
# person now runs their own whistle_drive.py --channel {steer,throttle}
# with their own (independently tunable) two/three-band scheme, and
# game_logic.py mixes the two most recent commands together continuously.
# Using MQTT for this (rather than, say, a local socket) is what lets the
# two scripts run as fully separate processes -- even on two different
# laptops -- since MQTT doesn't care whether publisher and subscriber are
# on the same machine.
STEER_TOPIC = f"ME193/{TEAM_NAME}/steer"
THROTTLE_TOPIC = f"ME193/{TEAM_NAME}/throttle"

# --- Game messages on GAME_TOPIC ---------------------------------------------
# The referee's start signal (given in the assignment -- don't change this
# one, it has to match what's actually published on the day).
MSG_START = "start"

# PLACEHOLDERS -- agree on the exact strings with your opponent team before
# the actual game, then update these two lines (and tell them to match).
MSG_BALL_TAGGED = "BALL_TAGGED"   # ball published this: goalie caught it
MSG_BALL_SCORED = "BALL_SCORED"  # ball published this: it reached the goal

# --- Throttle commands, on THROTTLE_TOPIC ------------------------------------
CMD_STOP = "STOP"
CMD_FORWARD = "FORWARD"
# Not a speed -- a distinct "special command" whistle that means "I just
# drove into the goal." Only meaningful for the ball role. Lives on the
# throttle channel since it's the same person/pitch-direction ("high") as
# forward, just an even higher, deliberately-hard-to-hit-by-accident band.
CMD_GOAL = "GOAL"

# --- Steering commands, on STEER_TOPIC ----------------------------------------
CMD_LEFT = "LEFT"
CMD_RIGHT = "RIGHT"
CMD_STRAIGHT = "STRAIGHT"  # neutral/no-turn -- the steerer's "middle" band