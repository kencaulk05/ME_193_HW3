"""
Diagnostic: scans for nearby Double Motor / Color Sensor hubs with NO card
filter, to check whether the hub is discoverable at all before blaming
CARD_COLOR/CARD_SERIAL.

If this finds your hub but `game_logic.py` (which filters by card) doesn't
connect, that confirms a card mismatch, not a Bluetooth/range problem --
most likely the connection card has never actually been tapped against
the hub (that's what makes it start advertising that card's identity),
or the color/serial in game_config.py doesn't match the physical card.

Usage:
    python card_scan.py
"""

import legoeducation as le


def scan_and_report(label, device):
    print(f"Scanning for any {label} (5s, no card filter)...")
    found = device.search(timeout=5)
    if not found:
        print(f"  No {label} found at all -- check power/range/Bluetooth "
              f"before worrying about the card.\n")
        return
    print(f"  Found {len(found)} {label}(s): {found}\n")


def main():
    scan_and_report("Double Motor", le.DoubleMotor())
    scan_and_report("Color Sensor", le.ColorSensor())
    print("If a device was found here but game_logic.py's filtered connect()")
    print("still times out, re-tap your connection card against the hub --")
    print("that's what makes it advertise that card's color+serial.")


if __name__ == "__main__":
    main()
