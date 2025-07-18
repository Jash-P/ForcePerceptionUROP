#!/usr/bin/env python3

"""
Calibration script for AR10 hand using the Python 3 AR10 class.
"""

import time
import sys
from AR10_class import AR10


def main() -> None:
    # create AR10 hand object
    hand = AR10()

    hand.hold_tennis_ball()

    # cleanup
    hand.close()


if __name__ == "__main__":
    main()
