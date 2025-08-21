#!/usr/bin/env python

"""
Active8 Robots, AR10 hand demonstration.
"""

from AR10_class import AR10
import time
import sys
import os
import random
import serial
import subprocess

def main():
    # create hand object
    hand = AR10()

    hand.open_hand()

    # menu loop
    while True:
        print "\033c\n"
        print "Active8 Robots, AR10 Hand Demonstration"
        print "======================================="
        print
        print
        print " S = Set Speed             O = Open Hand"
        print " A = Set Acceleration      C = Close Hand"
        print "                           F = Flex Finger"
        print "                           M = Move Joint"
        print "                           H = Hold Tennis Ball"
        print "                           G = Hold Golf Ball"
        print " E = Exit                  D = Demonstrate Range of Motions "

        print
        option = raw_input("Enter option >> ").upper()

        if option == "S":
            while True:
                try:
                    speed = int(raw_input("Enter Speed (0 - 60) >> "))
                    if speed <= 60 and speed >= 0:
                        break
                    else:
                        print "Invalid Speed"
                except ValueError:
                    print "Invalid Speed"
                hand.set_speed(speed)
        elif option == "A":
            while True:
                try:
                    acceleration = int(raw_input("Enter Acceleration (0 - 60) >> "))
                    if acceleration <= 255 and acceleration >= 0:
                        break
                    else:
                        print "Invalid Acceleration"
                except ValueError:
                    print "Invalid Acceleration"
                hand.set_acceleration(acceleration)
        elif option == "O":        # open the hand
            hand.open_hand()
        elif option == "C":
            hand.close_hand()
        elif option == "T":
            print "A = all fingers"
            print "1 = first finger"
            print "2 = second finger"
            print "3 = third finger"
            while True:
                finger = raw_input("Enter Finger >> ").upper()
                if finger == "A":    # touch fingers
                    hand.open_hand()
                    for finger in range(0, 3):
                        hand.touch_finger(finger)
                    break
                elif finger == "1":
                    hand.open_hand()
                    hand.touch_finger(0)
                    break
                elif finger == "2":
                    hand.open_hand()
                    hand.touch_finger(1)
                    break
                elif finger == "3":
                    hand.open_hand()
                    hand.touch_finger(2)
                    break
                else:
                    print "Invalid Finger"
        elif option == "F":
            print "T = thumb"
            print "1 = first finger"
            print "2 = second finger"
            print "3 = third finger"
            print "4 = forth finger"
            while True:
                finger = raw_input("Enter Finger >> ").upper()
                if finger == "T":    # flex fingers
                    hand.open_hand()
                    hand.flex_finger(0)
                    break
                elif finger == "1":
                    hand.open_hand()
                    hand.flex_finger(4)
                    break
                elif finger == "2":
                    hand.open_hand()
                    hand.flex_finger(3)
                    break
                elif finger == "3":
                    hand.open_hand()
                    hand.flex_finger(2)
                    break
                elif finger == "4":
                    hand.open_hand()
                    hand.flex_finger(1)
                    break
                else:
                    print "Invalid Finger"
        elif option == "M":
            # find valid joint
            while True:
                try:
                    joint = int(raw_input("Enter Joint (0 - 9) >> "))
                    if joint <= 9 and joint >= 0:
                        break
                    else:
                        print "Invalid Joint"
                except ValueError:
                    print "Invalid Joint"

            # find valid position
            while True:
                try:
                    target = int(raw_input("Enter Position (256 - 3850) >> "))
                    if target <= 3850 and target >= 256:
                        break
                    else:
                        print "Invalid Position"
                except ValueError:
                    print "Invalid Position"

            # move joint to target position
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
            print "Invalid Option"
            time.sleep(2)

    time.sleep(1)

    hand.close()

if __name__ == "__main__":
    main()