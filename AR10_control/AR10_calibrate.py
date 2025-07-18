#!/usr/bin/env python3

"""
Calibration script for AR10 hand using the Python 3 AR10 class.
"""

import time
import sys
from AR10_class import AR10


def calibrate_joint(hand: AR10, joint: int) -> tuple[float, float]:
    """
    Calibrate one joint by sweeping target positions and fitting a linear model.
    Returns (y_intercept, slope).
    """
    n_points = 0
    sum_x = 0.0
    sum_y = 0.0
    sum_xy = 0.0
    sum_xx = 0.0

    # For joint 9, move joint 8 into position first
    if joint == 9:
        hand.move(8, 5500)
        hand.wait_for_hand()

    # Sweep targets and record raw read positions
    for target in range(4500, 8000, 500):
        hand.move(joint, target)
        hand.wait_for_hand()
        time.sleep(2.0)

        position = hand.get_read_position(joint)

        n_points += 1
        sum_x += position
        sum_y += target
        sum_xy += position * target
        sum_xx += position * position

    # Compute linear fit: slope and intercept
    denom = (sum_x * sum_x) - (n_points * sum_xx)
    if denom == 0:
        raise ValueError("Denominator zero while calibrating joint {}".format(joint))
    slope = ((sum_x * sum_y) - (n_points * sum_xy)) / denom
    y_intercept = (sum_y - (slope * sum_x)) / n_points

    # Move to max and finalize
    hand.move(joint, 7950)
    hand.wait_for_hand()
    if joint == 9:
        hand.move(8, 7950)
        hand.wait_for_hand()

    return y_intercept, slope


def main() -> None:
    # create AR10 hand object
    hand = AR10()

    # open calibration file for writing
    with open("calibration_file", "w", newline='') as cal_file:
        hand.open_hand()

        for joint in range(10):
            y_intercept, slope = calibrate_joint(hand, joint)
            print(f"joint = {joint}  y intercept = {y_intercept:.6f}  slope = {slope:.6f}")
            cal_file.write(f"{joint}\t{y_intercept}\t{slope}\n")

    # cleanup
    hand.close()


if __name__ == "__main__":
    main()
