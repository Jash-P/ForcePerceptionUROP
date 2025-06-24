#!/usr/bin/env python
#Active8 Robots, AR10 hand posing tool
#Beta release 1.2
#Written by Nick Hornsey
#Last edited on 1/11/16

from ros_ar10_class import ar10 # necessary imports in order for the program to run
import time
import sys
import os
import random
import serial
import subprocess
import argparse

def main():

    parser = argparse.ArgumentParser() #defines argument parser
    grip_group = parser.add_mutually_exclusive_group(required=True) #adds group of arguments "pose_group"
    grip_group.add_argument("-d", help = "grips round object",action='store_true') #argument to open hand
    args = parser.parse_args()

    if args.d:
	hand = ar10()
	print ('Rotating Thumb ...')
	hand.move(0,6000)
	hand.wait_for_hand()
        dia = input("Enter diameter (mm) >> ")
	if dia <=20:
	    hand.close_hand()
	else:
	    x=4500+((dia-20)*50)
	    hand.move(2,x)
	    hand.move(3,x)
	    hand.move(4,x)
	    hand.move(5,x)
	    hand.move(6,x)
	    hand.move(7,x)
	    hand.move(8,x)
	    hand.move(9,x)
  
	hand.close()

if __name__ == "__main__":
    main()
