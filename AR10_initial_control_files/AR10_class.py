#!/usr/bin/env python3
"""
Active8 Robots, AR10 hand class (Python 3 version).
Includes pololu serial communication, speed, acceleration,
and movement together with demonstration facilities.
"""

import time
import sys
import random
import serial
import csv

class AR10:
    def __init__(self, port='/dev/ttyACM0', device_number=0x0C, baudrate=9600, timeout=1):
        self.speed = 20
        self.acceleration = 10

        self.intercept = []
        self.slope = []

        # Open the command port
        self.usb = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        # Pololu protocol prefix: 0xAA, device number
        self.pololu_command = bytes([0xAA, device_number])

        # Read the calibration file
        try:
            with open('calibration_file', newline='') as f:
                reader = csv.reader(f, delimiter='\t')
                for row in reader:
                    self.intercept.append(float(row[1]))
                    self.slope.append(float(row[2]))
        except FileNotFoundError:
            print("Calibration file missing")
            print("Please run AR10_calibrate.py")

    def close(self):
        """Cleanup by closing USB serial port"""
        self.usb.close()

    def change_speed(self, speed):
        """Change speed setting"""
        self.speed = speed

    def set_speed(self, channel):
        """Set speed of a channel"""
        lsb = self.speed & 0x7F
        msb = (self.speed >> 7) & 0x7F
        cmd = self.pololu_command + bytes([0x07, channel, lsb, msb])
        self.usb.write(cmd)

    def change_acceleration(self, acceleration):
        """Change acceleration setting"""
        self.acceleration = acceleration

    def set_acceleration(self, channel, acceleration):
        """Set acceleration of a channel"""
        lsb = acceleration & 0x7F
        msb = (acceleration >> 7) & 0x7F
        cmd = self.pololu_command + bytes([0x09, channel, lsb, msb])
        self.usb.write(cmd)

    def set_target(self, channel, target):
        """Set channel to a specified target value"""
        lsb = target & 0x7F
        msb = (target >> 7) & 0x7F
        cmd = self.pololu_command + bytes([0x04, channel, lsb, msb])
        self.usb.write(cmd)

    def joint_to_channel(self, joint):
        """Convert joint number to channel number"""
        return joint + 10

    def get_set_position(self, joint):
        """Get the last target position sent (quarter-microseconds)"""
        channel = self.joint_to_channel(joint)
        cmd = self.pololu_command + bytes([0x10, channel])
        self.usb.write(cmd)
        data = self.usb.read(2)
        if len(data) < 2:
            raise IOError("Timeout reading set position")
        lsb, msb = data[0], data[1]
        return (msb << 8) + lsb

    def get_read_position(self, channel):
        """Read the Maestro's reported position on a channel"""
        cmd = self.pololu_command + bytes([0x90, channel])
        self.usb.write(cmd)
        data = self.usb.read(2)
        if len(data) < 2:
            raise IOError("Timeout reading position")
        lsb, msb = data[0], data[1]
        return (msb << 8) + lsb

    def get_position(self, channel):
        """Get calibrated position value"""
        raw = self.get_read_position(channel)
        calibrated = self.intercept[channel] + (self.slope[channel] * raw)
        return int(calibrated)

    def get_moving_state(self):
        """Return True if the Maestro is still moving"""
        cmd = self.pololu_command + bytes([0x13, 0x01])
        self.usb.write(cmd)
        data = self.usb.read(1)
        if not data:
            raise IOError("Timeout reading moving state")
        return data[0] != 0

    def run_script(self, sub_number):
        """Run a subroutine in the active Maestro script"""
        cmd = self.pololu_command + bytes([0x27, sub_number])
        self.usb.write(cmd)

    def stop_script(self):
        """Stop the current Maestro script"""
        cmd = self.pololu_command + bytes([0x24])
        self.usb.write(cmd)

    def move(self, joint, target):
        """Move a joint to a target, with speed and acceleration settings"""
        channel = self.joint_to_channel(joint)
        # clamp target
        target = max(4200, min(7950, target))
        # apply speed and acceleration
        self.set_speed(channel)
        self.set_acceleration(channel, self.acceleration)
        self.set_target(channel, target)

    def wait_for_hand(self):
        """Block until all motions complete"""
        while self.get_moving_state():
            time.sleep(0.25)

    # hand gestures
    def open_hand(self):
        """Open all joints to maximum"""
        for j in range(10):
            self.move(j, 8000)
        time.sleep(1.0)
        self.wait_for_hand()

    def close_hand(self):
        """Close fingers then thumb"""
        for j in range(2, 10):
            self.move(j, 2500)
        time.sleep(2.0)
        self.move(0, 5000)
        time.sleep(1.0)
        self.move(1, 6500)
        self.wait_for_hand()

    def hold_golf_ball(self):
        """Pose to hold a golf ball"""
        positions = {0:5700, 1:8000, 6:4500, 7:7700, 8:4500, 9:7900}
        for j, pos in positions.items():
            self.move(j, pos)
        self.wait_for_hand()

    def hold_tennis_ball(self):
        """Pose to hold a tennis ball"""
        positions = {0:5500, 1:8000, 2:5000, 3:5000, 4:5500,
                     5:7400, 6:5300, 7:7400, 8:5200, 9:7500}
        for j, pos in positions.items():
            self.move(j, pos)
        self.wait_for_hand()

    def test(self):
        """Sweep a finger through a range of positions"""
        for pos in range(1500, 2000, 100):
            print(pos)
            self.move(6, pos)
            self.wait_for_hand()
            time.sleep(2.0)

    def flex_finger(self, finger):
        """Flex and release a single finger"""
        if not (0 <= finger <= 4):
            print(f"ERROR in flex_finger: finger = {finger}")
            sys.exit(1)
        # determine joints for finger
        base = 2 * finger
        first = base
        second = base + 1
        # flex
        self.move(first, 300)
        self.move(second, 800 if finger == 4 else 300)
        self.wait_for_hand()
        time.sleep(1.0)
        # release
        self.move(first, 3850)
        self.move(second, 3850)
        self.wait_for_hand()

    def demo(self):
        """Run through a demo sequence of opening/closing"""
        sequence = [
            ([0,1],2500), ([0,1],8000), ([9,8],2500), ([7,6],2500),
            ([5,4],2500), ([3,2],2500), ([2,3],8000), ([4,5],8000),
            ([6,7],8000), ([8,9],8000)
        ]
        for group, pos in sequence:
            for j in group:
                self.move(j, pos)
            time.sleep(3.0)
        self.wait_for_hand()
