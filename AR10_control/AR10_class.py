#!/usr/bin/env python3
"""
Active8 Robots, AR10 hand class (Python 3 version) using send_to_nano for all command transmissions.
"""

import time
import sys
import csv
from nanoAP_test import send_to_nano

class AR10:
    def __init__(self, device_number=0x00):
        self.speed = 20
        self.acceleration = 10

        self.intercept = []
        self.slope = []

        # Pololu protocol prefix: 0xAA, device number
        self.pololu_prefix = bytes([0xAA, device_number])

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
        """Cleanup if needed"""
        pass  # no persistent connections

    def _send(self, command: bytes) -> None:
        """Helper to send a command packet via send_to_nano"""
        send_to_nano(command)

    def change_speed(self, speed: int) -> None:
        """Change speed setting"""
        self.speed = speed

    def set_speed(self, channel: int) -> None:
        """Set speed of a channel"""
        lsb = self.speed & 0x7F
        msb = (self.speed >> 7) & 0x7F
        cmd = self.pololu_prefix + bytes([0x07, channel, lsb, msb])
        self._send(cmd)

    def change_acceleration(self, acceleration: int) -> None:
        """Change acceleration setting"""
        self.acceleration = acceleration

    def set_acceleration(self, channel: int, acceleration: int) -> None:
        """Set acceleration of a channel"""
        lsb = acceleration & 0x7F
        msb = (acceleration >> 7) & 0x7F
        cmd = self.pololu_prefix + bytes([0x09, channel, lsb, msb])
        self._send(cmd)

    def set_target(self, channel: int, target: int) -> None:
        """Set channel to a specified target value"""
        lsb = target & 0x7F
        msb = (target >> 7) & 0x7F
        cmd = self.pololu_prefix + bytes([0x04, channel, lsb, msb])
        self._send(cmd)

    def joint_to_channel(self, joint: int) -> int:
        """Convert joint number to channel number"""
        return joint + 10

    def get_moving_state(self) -> bool:
        """Return True if the Maestro is still moving"""
        cmd = self.pololu_prefix + bytes([0x13, 0x01])
        # no response over send_to_nano; assume commands only
        self._send(cmd)
        return True  # placeholder, cannot query state via send only

    def run_script(self, sub_number: int) -> None:
        """Run a subroutine in the active Maestro script"""
        cmd = self.pololu_prefix + bytes([0x27, sub_number])
        self._send(cmd)

    def stop_script(self) -> None:
        """Stop the current Maestro script"""
        cmd = self.pololu_prefix + bytes([0x24])
        self._send(cmd)

    def move(self, joint: int, target: int) -> None:
        """Move a joint to a target, with speed and acceleration settings"""
        channel = self.joint_to_channel(joint)
        target = max(4200, min(7950, target))
        self.set_speed(channel)
        self.set_acceleration(channel, self.acceleration)
        self.set_target(channel, target)

    def wait_for_hand(self) -> None:
        """Block until all motions complete"""
        # unable to poll; simple delay
        time.sleep(1)

    # hand gestures
    def open_hand(self) -> None:
        """Open all joints to maximum"""
        for j in range(10):
            self.move(j, 8000)
        time.sleep(1.0)

    def close_hand(self) -> None:
        """Close fingers then thumb"""
        for j in range(2, 10):
            self.move(j, 2500)
        time.sleep(2.0)
        self.move(0, 5000)
        time.sleep(1.0)
        self.move(1, 6500)

    def hold_golf_ball(self) -> None:
        """Pose to hold a golf ball"""
        positions = {0:5700, 1:8000, 6:4500, 7:7700, 8:4500, 9:7900}
        for j, pos in positions.items():
            self.move(j, pos)

    def hold_tennis_ball(self) -> None:
        """Pose to hold a tennis ball"""
        positions = {0:5500, 1:8000, 2:5000, 3:5000, 4:5500,
                     5:7400, 6:5300, 7:7400, 8:5200, 9:7500}
        for j, pos in positions.items():
            self.move(j, pos)

    def demo(self) -> None:
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
