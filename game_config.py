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

# Your own team's private channel: whistle_drive.py publishes driving
# commands here, game_logic.py subscribes and executes them. Using MQTT for
# this (rather than, say, a local socket) is what lets the two scripts run
# as fully separate processes -- even on two different laptops for the
# 4-point version, since MQTT doesn't care whether publisher and subscriber
# are on the same machine.
DRIVE_TOPIC = f"ME193/{TEAM_NAME}/drive"

# --- Game messages on GAME_TOPIC ---------------------------------------------
# The referee's start signal (given in the assignment -- don't change this
# one, it has to match what's actually published on the day).
MSG_START = "start"

# PLACEHOLDERS -- agree on the exact strings with your opponent team before
# the actual game, then update these two lines (and tell them to match).
MSG_BALL_TAGGED = "BALL_TAGGED"   # ball published this: goalie caught it
MSG_BALL_SCORED = "BALL_SCORED"  # ball published this: it reached the goal

# --- Driving commands on DRIVE_TOPIC -----------------------------------------
# whistle_drive.py publishes these; game_logic.py maps them to motor speeds.
CMD_STOP = "STOP"
CMD_FORWARD = "FORWARD"
CMD_LEFT = "LEFT"
CMD_RIGHT = "RIGHT"
# Not a driving speed -- a distinct "special command" whistle that means
# "I just drove into the goal." Only meaningful for the ball role.
CMD_GOAL = "GOAL"
