#!/usr/bin/env python3

"""
Active8 Robots, AR10 hand demonstration (Python 3 version).
"""

import time
import sys
from AR10_class import AR10


def main() -> None:
    # create hand object
    hand = AR10()

    # initialize hand
    hand.open_hand()

    # menu loop
    while True:
        # clear screen
        print("\033c\n")
        print("Active8 Robots, AR10 Hand Demonstration")
        print("=======================================")
        print()
        print(" S = Set Speed             O = Open Hand")
        print(" A = Set Acceleration      C = Close Hand")
        print("                           F = Flex Finger")
        print("                           M = Move Joint")
        print("                           H = Hold Tennis Ball")
        print("                           G = Hold Golf Ball")
        print(" E = Exit                  D = Demonstrate Range of Motions ")
        print()

        option = input("Enter option >> ").strip().upper()

        if option == "S":
            # set global speed
            while True:
                try:
                    speed = int(input("Enter Speed (0 - 60) >> "))
                    if 0 <= speed <= 60:
                        hand.change_speed(speed)
                        break
                    else:
                        print("Invalid Speed")
                except ValueError:
                    print("Invalid Speed")
        elif option == "A":
            # set global acceleration
            while True:
                try:
                    acceleration = int(input("Enter Acceleration (0 - 255) >> "))
                    if 0 <= acceleration <= 255:
                        hand.change_acceleration(acceleration)
                        break
                    else:
                        print("Invalid Acceleration")
                except ValueError:
                    print("Invalid Acceleration")
        elif option == "O":
            # open the hand
            hand.open_hand()
        elif option == "C":
            hand.close_hand()
        elif option == "F":
            # flex a specific finger
            print("T = thumb")
            print("1 = first finger")
            print("2 = second finger")
            print("3 = third finger")
            print("4 = fourth finger")
            while True:
                finger = input("Enter Finger >> ").strip().upper()
                mapping = {"T": 4, "1": 0, "2": 1, "3": 2, "4": 3}
                if finger in mapping:
                    hand.open_hand()
                    hand.flex_finger(mapping[finger])
                    break
                else:
                    print("Invalid Finger")
        elif option == "M":
            # move a specific joint
            while True:
                try:
                    joint = int(input("Enter Joint (0 - 9) >> "))
                    if 0 <= joint <= 9:
                        break
                    else:
                        print("Invalid Joint")
                except ValueError:
                    print("Invalid Joint")

            while True:
                try:
                    target = int(input("Enter Position (256 - 3850) >> "))
                    if 256 <= target <= 3850:
                        break
                    else:
                        print("Invalid Position")
                except ValueError:
                    print("Invalid Position")

            hand.move(joint, target)
            hand.wait_for_hand()
        elif option == "H":
            hand.hold_tennis_ball()
        elif option == "G":
            hand.hold_golf_ball()
        elif option == "D":
            hand.demo()
        elif option == "E":
            break
        else:
            print("Invalid Option")
            time.sleep(2)

    # cleanup
    time.sleep(1)
    hand.close()


if __name__ == "__main__":
    main()
