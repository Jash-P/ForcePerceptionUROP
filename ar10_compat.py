# ar10_compat.py
import csv, time, sys
from pathlib import Path
from typing import Optional, List
from maestro_wifi import MaestroWiFiClient

class AR10Compat:
    """
    A near drop-in for the legacy AR10 class, backed by MaestroWiFiClient.
    Differences vs. the original:
      - Uses Wi-Fi TCP via ESP32C3 rather than USB serial.
      - set_target expects quarter-microseconds (like the original AR10);
        we convert to microseconds for the Wi-Fi bridge.
      - get_set_position/get_read_position return quarter-microseconds,
        matching Pololu's GET POSITION behavior.
    """
    def __init__(self, client: MaestroWiFiClient,
                 calibration_file: str = "calibration_file",
                 joint_channel_offset: int = 10):
        self.m = client
        self.speed = 20
        self.acceleration = 10
        self.joint_channel_offset = joint_channel_offset

        # Load calibration (tab-delimited): assume columns [*, intercept, slope, *]
        self.intercept: List[float] = []
        self.slope: List[float] = []
        try:
            with open(calibration_file, "r", newline="") as f:
                r = csv.reader(f, delimiter="\t")
                for row in r:
                    # defensive parse; skip malformed rows
                    if len(row) >= 3:
                        self.intercept.append(float(row[1]))
                        self.slope.append(float(row[2]))
            # pad to 24 channels if needed
            while len(self.intercept) < 24: self.intercept.append(0.0)
            while len(self.slope) < 24: self.slope.append(1.0)
        except Exception:
            print("Calibration file missing or unreadable; using defaults (intercept=0, slope=1", file=sys.stderr)
            self.intercept = [0.0]*24
            self.slope = [1.0]*24

    # --- lifecycle ---
    def close(self):
        # nothing to close; the caller owns the MaestroWiFiClient context
        pass

    # --- helpers matching legacy names ---
    def change_speed(self, speed: int):
        self.speed = int(speed)

    def set_speed(self, channel: int):
        # Send current self.speed to <channel>
        return self.m.set_speed(int(channel), int(self.speed))

    def change_acceleration(self, acceleration: int):
        self.acceleration = int(acceleration)

    def set_acceleration(self, channel: int, acceleration: Optional[int] = None):
        val = self.acceleration if acceleration is None else int(acceleration)
        return self.m.set_accel(int(channel), val)

    def set_target(self, channel: int, target_qus: int):
        # legacy API passes quarter-microseconds (e.g., 8000 -> 2000 us)
        us = int(round(target_qus / 4.0))
        return self.m.set_target(int(channel), us)

    def joint_to_channel(self, joint: int) -> int:
        return int(joint) + int(self.joint_channel_offset)

    def get_set_position(self, joint: int) -> int:
        ch = self.joint_to_channel(joint)
        _ch, raw_qus, us = self._get_raw(ch)
        return raw_qus

    def get_read_position(self, channel: int) -> int:
        _ch, raw_qus, us = self._get_raw(channel)
        return raw_qus

    def get_position(self, channel: int) -> int:
        # calibrated engineering units from raw ADC-ish units
        raw = self.get_read_position(channel)  # quarter-microseconds
        # original code used: intercept[channel] + slope[channel]*read_position
        val = self.intercept[channel] + self.slope[channel] * raw
        return int(round(val))

    def get_moving_state(self) -> bool:
        return bool(self.m.get_moving())

    def run_script(self, subNumber: int, param: Optional[int] = None):
        return self.m.run_script(subNumber, param)

    def stop_script(self):
        return self.m.stop_script()

    # --- high-level motions mirrored from the legacy class ---
    def move(self, joint: int, target_qus: int):
        ch = self.joint_to_channel(joint)
        # clamp to legacy bounds (quarter-microseconds)
        target_qus = max(4200, min(7950, int(target_qus)))
        self.set_speed(ch)
        self.set_acceleration(ch, self.acceleration)
        self.set_target(ch, target_qus)

    def wait_for_hand(self, poll_s: float = 0.25):
        # poll Maestro moving state (true while any servo still moving)
        while self.get_moving_state():
            time.sleep(poll_s)

    # sequences (unchanged values from your sample; edit to taste)
    def open_hand(self):
        self.move(0, 8000); self.move(1, 8000)
        time.sleep(1.0)
        for joint in range(2, 10):
            self.move(joint, 8000)
        self.wait_for_hand()

    def close_hand(self):
        for j in range(2, 10): self.move(j, 2500)
        time.sleep(2.0)
        self.move(0, 5000)
        time.sleep(1.0)
        self.move(1, 6500)
        self.wait_for_hand()

    def hold_golf_ball(self):
        self.move(0, 5700); self.move(1, 8000)
        self.move(6, 4500); self.move(7, 7700)
        self.move(8, 4500); self.move(9, 7900)
        self.wait_for_hand()

    def hold_tennis_ball(self):
        self.move(0, 5500); self.move(1, 8000)
        self.move(2, 5000); self.move(3, 5000)
        self.move(4, 5500); self.move(5, 7400)
        self.move(6, 5300); self.move(7, 7400)
        self.move(8, 5200); self.move(9, 7500)
        self.wait_for_hand()

    def test(self):
        for pos in range(1500, 2000, 100):
            print(pos)
            self.move(6, pos)
            self.wait_for_hand()
            time.sleep(2.0)

    def flex_finger(self, finger: int):
        if finger < 0 or finger > 4:
            raise ValueError(f"finger out of range: {finger}")
        if finger == 4:   # thumb
            self.move(2*finger, 300); self.move(2*finger+1, 800)
        else:
            self.move(2*finger, 300); self.move(2*finger+1, 300)
        self.wait_for_hand(); time.sleep(1.0)
        self.move(2*finger, 3850); self.move(2*finger+1, 3850)
        self.wait_for_hand()

    def demo(self):
        seq = [
            (0,2500),(1,2500),
            (0,8000),(1,8000),
            (9,2500),(8,2500),
            (7,2500),(6,2500),
            (5,2500),(4,2500),
            (3,2500),(2,2500),
            (2,8000),(3,8000),
            (4,8000),(5,8000),
            (6,8000),(7,8000),
            (8,8000),(9,8000)
        ]
        for j,t in seq:
            self.move(j,t); time.sleep(3.0)
        self.wait_for_hand()

    # --- private ---
    def _get_raw(self, ch: int):
        ch_i, raw, us = self.m.get_position(int(ch))
        return ch_i, raw, us
