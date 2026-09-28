"""
Shared constants for the two-computer whistle control setup.

remote_computer.py and main_computer.py both import from here, so the
topic name and message strings can't drift apart between the two
machines.
"""

import legoeducation as le

# --- Identify this setup -----------------------------------------------------
# Change this to something unique so this pair's channel doesn't collide
# with anyone else's on the public test.mosquitto.org broker (anyone can
# publish/subscribe to any topic there).
TEAM_NAME = "changeme-team"

# The remote computer publishes here; the main computer subscribes.
TURN_TOPIC = f"ME193/{TEAM_NAME}/turn"

MSG_TURNING = "TURNING"
MSG_NOT_TURNING = "NOT_TURNING"

# --- Main computer's LEGO connection card ------------------------------------
# Fill these in with the color/serial printed on your LEGO connection card
# (only the main computer needs this -- the remote one never touches the
# hub). Set both to None to instead connect to the first advertising
# Double Motor found, if you only have one nearby.
CARD_COLOR = le.LEGO_COLOR_PURPLE
CARD_SERIAL = 6056
