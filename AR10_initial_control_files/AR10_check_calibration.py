#!/usr/bin/env python3

"""
Check calibration script for AR10 hand using the Python 3 AR10 class.
"""

import time
import sys
from AR10_class import AR10


def check_calibration(hand: AR10, joint: int) -> None:
    """
    Move a joint through standard targets and compare set vs calibrated positions.
    Prints joint number, set position, actual calibrated position, and error.
    """
    # For joint 9, position joint 8 first
    if joint == 9:
        hand.move(8, 5500)
        hand.wait_for_hand()

    print()
    # Sweep through target values
    for target in range(4500, 8000, 500):
        hand.move(joint, target)
        hand.wait_for_hand()
        time.sleep(2.0)

        set_position = hand.get_set_position(joint)
        actual_position = hand.get_position(joint)
        error = abs(set_position - actual_position)

        print(
            f"joint = {joint}  "
            f"set position = {set_position}  "
            f"position = {actual_position}  "
            f"error = {error}"
        )

    # Move to final max position
    hand.move(joint, 7950)
    hand.wait_for_hand()
    if joint == 9:
        hand.move(8, 7950)
        hand.wait_for_hand()


def main() -> None:
    """Create AR10 instance and check calibration for joints 0-9."""
    hand = AR10()
    hand.open_hand()
    hand.wait_for_hand()

    for joint in range(10):
        check_calibration(hand, joint)

    hand.close()


if __name__ == "__main__":
    main()
