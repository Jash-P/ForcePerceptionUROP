#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 + EIT with sensor_init.py-grade contact detection:
- Async serial reader + robust CSV parsing for EIT frames
- RobustEITDetector (MAD-scale, EMA + median prefilter, top-K robust z, winsorization)
- Coarse→fine guarded descent (warn→confirm with debounce, refractory, DZ guard, optional force bump)
- 1s pause after contact before sampling "after" EIT
Then:
  • Build ΔEIT[256] at contact vs hover
  • Run stage1 ensemble to infer XYZ
  • Move by (desired - predicted): XY at hover, then continuous lower to goal Z (EIT-guarded)
  • Report final pose error and log a row

Requires: ur-rtde, pyserial, numpy, joblib
"""

import math
import time
import csv, os, random, threading, statistics, glob
from collections import deque
from datetime import datetime

import numpy as np
import serial
from joblib import load
from infer_stage1_xyz import predict_xyz_delta_eit  # keep next to this file

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# ---------------------------- USER SETTINGS ---------------------------- #

ROBOT_IP = "169.254.150.50"

# TCP at sensor centre (meters, axis–angle)
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion & dwell
SPEED = 0.05
ACCEL = 0.05
HOVER_LIFT_Z = 0.050
EIT_BEFORE_DWELL = 0.30
POST_CONTACT_LOG_PAUSE = 1.0  # pause before logging AFTER frame

# Active descent (coarse / fine) — mirrored from sensor_init.py
DESCENT_SPEED   = 0.040
DESCENT_ACCEL   = 0.040
STEP_Z_COARSE   = 0.0010     # 1.0 mm
STEP_Z_FINE     = 0.00025    # 0.25 mm after WARN
STEP_SETTLE     = 0.010
NO_CONTACT_EXTRA_Z = 0.015   # go a bit below target if no contact

# Floor guard (base frame)
MIN_BASE_Z = -0.390
FLOOR_MARGIN = 0.010
CLAMP_BELOW_FLOOR = False

# EIT serial (auto-detect if empty)
EIT_PORT = "/dev/ttyACM0"
EIT_BAUD = 115200
EIT_TIMEOUT = 0.2
EIT_SNIFF_SECS = 5.0

# Inference model
ENSEMBLE_PATH = "./outputs_stage1_tuned/eit_stage1_xyz_ensemble.joblib"
DELTA_EIT_DIM = 256

# Desired contact (base frame)
DESIRED_XYZ = [-0.006, -0.493, -0.222+0.350]   # meters

# Initial randomization around desired XY (±range in meters)
RAND_INIT_X_RANGE = 0.030   # ±30 mm
RAND_INIT_Y_RANGE = 0.030   # ±30 mm

# >>> Start-Z offset <<<
# Starting Z = desired_z + START_Z_OFFSET. Positive values start above the goal.
START_Z_OFFSET = 0.050

# Orientation choice
USE_CURRENT_ORIENT   = False
EXPLICIT_START_POSE  = [-0.030, -0.500, 0.200, 0.47, -1.50, 0.65]  # use only [3:6]

# Continuous lower-to-endpoint (after EIT) — params
ENDPOINT_CONT_Z_SPEED   = 0.010  # m/s downward
ENDPOINT_CONT_CMD_DT    = 0.20   # s per segment
ENDPOINT_CONT_STOP_RAMP = 0.20   # s decel ramp

# Logging
LOG_CSV_PATH = "eit_infer_xyz_randomstart_runlog_functional.csv"
LOG_WRENCH   = True

# ------------------------- Robust EIT detection ------------------------ #
# (Copied 1:1 from your sensor_init.py)

BASELINE_FRAMES          = 50
EMA_ALPHA                = 0.35
PREFILT_MEDIAN_WINDOW    = 3
TOP_K_PERCENT            = 20
WARN_SCORE               = 4.0
CONFIRM_SCORE            = 5.5
DEBOUNCE_CONSEC          = 5
DZ_GUARD_ENABLE          = 0.003
REFRACTORY_SEC           = 0.5

SCALE_ABS_FLOOR         = 0.02
MAD_SCALE_PCTL_FLOOR    = 50
MIN_VALID_CHANS_RATIO   = 0.60
Z_HUGE_CLIP             = 50.0

HOVER_NOISE_FRAMES   = 80
HOVER_NOISE_PCTL     = 95
NOISE_MARGIN         = 1.0
CONFIRM_EXTRA        = 0.75

COARSE_WARN_CONSEC   = 2

REQUIRE_FORCE_BUMP       = False
FORCE_BUMP_N             = 3
FORCE_BUMP_DELTA         = 10.0
FORCE_ABS_MIN            = 10.0

# ------------------------------ math utils ----------------------------- #

def deg2rad(d): return d * math.pi / 180.0

def rvec_to_rotmat(rvec):
    rx, ry, rz = rvec
    t = math.sqrt(rx*rx + ry*ry + rz*rz)
    if t < 1e-12: return ((1,0,0),(0,1,0),(0,0,1))
    ux, uy, uz = rx/t, ry/t, rz/t
    c = math.cos(t); s = math.sin(t); C = 1.0 - c
    return (
        (c+ux*ux*C,      ux*uy*C - uz*s, ux*uz*C + uy*s),
        (uy*ux*C + uz*s, c+uy*uy*C,      uy*uz*C - ux*s),
        (uz*ux*C - uy*s, uz*uy*C + ux*s, c+uz*uz*C     )
    )

def rotmat_to_rvec(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    ct = max(min((tr - 1.0)/2.0, 1.0), -1.0)
    t = math.acos(ct)
    if t < 1e-12: return (0,0,0)
    denom = 2.0*math.sin(t)
    rx = (R[2][1] - R[1][2]) / denom
    ry = (R[0][2] - R[2][0]) / denom
    rz = (R[1][0] - R[0][1]) / denom
    return (rx*t, ry*t, rz*t)

# ---------------------------- safety helpers --------------------------- #

def floor_guard(pose):
    zmin = MIN_BASE_Z + FLOOR_MARGIN
    if pose[2] >= zmin - 1e-9: return pose
    if CLAMP_BELOW_FLOOR:
        p = list(pose); p[2] = zmin
        print(f"[FLOOR] Clamped Z from {pose[2]:.3f} to {zmin:.3f}")
        return p
    raise ValueError(f"Target Z={pose[2]:.3f} below floor+margin {zmin:.3f}")

def safe_moveL(rtde_c, rtde_r, target, speed, accel):
    try:
        tgt = floor_guard(target)
    except ValueError as e:
        print(f"[FLOOR ABORT] {e}")
        return False
    return bool(rtde_c.moveL(tgt, speed, accel))

def add_z(pose, dz):
    return [pose[0], pose[1], pose[2] + dz, pose[3], pose[4], pose[5]]

# ----------------------- Async EIT (low-memory) ------------------------ #

def _split_csv_fields(line: str):
    if not line:
        return []
    try:
        row = next(csv.reader([line]))
    except Exception:
        row = [tok.strip() for tok in line.strip().strip('"').split(",")]
    while row and row[-1] == "":
        row.pop()
    return row

def _to_float(s: str):
    s = s.strip().strip('"').strip("'")
    if s and s[0] in "[(" and s[-1] in "])":
        s = s[1:-1]
    return float(s)

def parse_vec_into(line, buf):
    fields = _split_csv_fields(line)
    n = min(len(fields), len(buf))
    try:
        for i in range(n):
            buf[i] = _to_float(fields[i])
    except Exception:
        return 0
    return n

class AsyncEIT:
    __slots__ = ("port","baud","timeout","sniff_secs","ser","lock",
                 "last","cols","_stop","_th")
    def __init__(self, port, baud, timeout, sniff_secs=2.0):
        self.port=port; self.baud=baud; self.timeout=timeout; self.sniff_secs=sniff_secs
        self.ser=None; self.lock=threading.Lock()
        self.last="" ; self.cols=0
        self._stop=False; self._th=None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout, dsrdtr=False, rtscts=False)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        t0 = time.time()
        raw = ""
        while time.time()-t0 < self.sniff_secs:
            line = self.ser.readline()
            if not line: continue
            s = line.decode("utf-8", "ignore").strip()
            if s: raw = s; break
        if raw:
            fields = _split_csv_fields(raw)
            self.cols = len(fields)
            with self.lock: self.last = raw
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    def _reader(self):
        lc = self.last
        while not self._stop:
            try:
                line = self.ser.readline()
                if not line: continue
                s = line.decode("utf-8","ignore").strip()
                if not s: continue
                if self.cols == 0 or len(_split_csv_fields(s)) == self.cols:
                    lc = s
                    with self.lock: self.last = lc
            except Exception:
                time.sleep(0.005)

    def latest(self):
        with self.lock:
            return self.last

    def close(self):
        self._stop=True
        try:
            if self.ser: self.ser.close()
        except Exception:
            pass

# ------------------------ Robust detector class ------------------------ #

class RobustEITDetector:
    def __init__(self, cols):
        self.cols = cols
        self.baseline = [0.0]*cols
        self.scale    = [1.0]*cols
        self.ema      = [0.0]*cols
        self.med_bufs = [deque(maxlen=PREFILT_MEDIAN_WINDOW) for _ in range(cols)]
        self.valid_idx = list(range(cols))
        self._ready = False
        self.last_confirm_t = 0.0
        self.warn_eff = WARN_SCORE
        self.confirm_eff = max(CONFIRM_SCORE, WARN_SCORE + CONFIRM_EXTRA)

    def build_baseline(self, eit, timeout_s=3.0):
        frames = []
        work = [0.0]*self.cols
        t0 = time.time()
        while len(frames) < BASELINE_FRAMES and (time.time()-t0) < timeout_s:
            line = eit.latest()
            if parse_vec_into(line, work) == self.cols:
                frames.append(list(work))
            time.sleep(0.005)
        if not frames:
            self._ready = False
            return False

        mads = []
        for k in range(self.cols):
            col = [f[k] for f in frames]
            med = statistics.median(col)
            self.baseline[k] = med
            abs_dev = [abs(v - med) for v in col]
            mad = statistics.median(abs_dev) if abs_dev else 0.0
            mads.append(mad)

        nz = sorted([m for m in mads if m > 0.0])
        p50 = statistics.median(nz) if nz else 0.0
        scale_floor = max(1.4826 * max(p50, 0.0), SCALE_ABS_FLOOR)

        self.valid_idx.clear()
        for k, mad in enumerate(mads):
            sc = max(1.4826*mad, scale_floor)
            self.scale[k] = sc
            self.ema[k] = self.baseline[k]
            self.med_bufs[k].clear()
            if sc >= SCALE_ABS_FLOOR * 0.99:
                self.valid_idx.append(k)

        min_needed = int(self.cols * MIN_VALID_CHANS_RATIO)
        if len(self.valid_idx) < min_needed:
            print(f"[EIT] Too few valid channels ({len(self.valid_idx)}/{self.cols}); check sensor/cabling.")
            self._ready = False
            return False

        self._ready = True
        return True

    def prefilter(self, vec):
        out = [0.0]*self.cols
        a = EMA_ALPHA
        for k in range(self.cols):
            self.ema[k] = (1-a)*self.ema[k] + a*vec[k]
            self.med_bufs[k].append(self.ema[k])
            out[k] = statistics.median(self.med_bufs[k]) if self.med_bufs[k] else self.ema[k]
        return out

    def score(self, vec):
        if not self._ready or not self.valid_idx:
            return 0.0
        zs = []
        for k in self.valid_idx:
            z = abs(vec[k] - self.baseline[k]) / self.scale[k]
            if z > Z_HUGE_CLIP:
                z = Z_HUGE_CLIP
            zs.append(z)
        if not zs:
            return 0.0
        K = max(1, int(len(zs) * TOP_K_PERCENT / 100.0))
        K = min(K, len(zs))
        return statistics.median(sorted(zs, reverse=True)[:K])

    def can_eval(self, dz_from_baseline):
        return dz_from_baseline is not None and dz_from_baseline >= DZ_GUARD_ENABLE

    def refractory_ok(self):
        return (time.time() - self.last_confirm_t) >= REFRACTORY_SEC

    def mark_confirm(self):
        self.last_confirm_t = time.time()

    def calibrate_hover_noise(self, eit, work_vec, max_time_s=2.0):
        scores = []
        t0 = time.time()
        while len(scores) < HOVER_NOISE_FRAMES and (time.time() - t0) < max_time_s:
            line = eit.latest()
            if parse_vec_into(line, work_vec) == self.cols:
                v = self.prefilter(work_vec)
                s = self.score(v)
                scores.append(s)
            time.sleep(0.005)
        if not scores:
            self.warn_eff = WARN_SCORE
            self.confirm_eff = max(CONFIRM_SCORE, self.warn_eff + CONFIRM_EXTRA)
            return False
        ss = sorted(scores)
        idx = int(round((HOVER_NOISE_PCTL/100.0) * (len(ss)-1)))
        noise_pctl = ss[idx]
        self.warn_eff = max(WARN_SCORE, noise_pctl + NOISE_MARGIN)
        self.confirm_eff = max(CONFIRM_SCORE, self.warn_eff + CONFIRM_EXTRA)
        return True

# -------------------------- Guarded descent ---------------------------- #

def descend_until_contact(rtde_c, rtde_r, hover_pose, target_pose,
                          eit, cols, detector, work_vec):
    """
    Hybrid descent (mirrors sensor_init.py):
      1) coarse stepping until WARN reached stably
      2) fine stepping until CONFIRM reached (debounced)
    Returns (contact_detected, final_pose_z)
    """
    zmin_allowed = MIN_BASE_Z + FLOOR_MARGIN
    z_goal = max(target_pose[2] - NO_CONTACT_EXTRA_Z, zmin_allowed)
    z_baseline = rtde_r.getActualTCPPose()[2]

    warn_thr = getattr(detector, "warn_eff", WARN_SCORE)
    conf_thr = getattr(detector, "confirm_eff", CONFIRM_SCORE)

    z = hover_pose[2]
    warn_consec = 0
    confirm_consec = 0
    contact = False

    p_step = list(target_pose)
    p_step[0:3] = [target_pose[0], target_pose[1], z]

    def measure_score():
        line = eit.latest() if eit else ""
        if parse_vec_into(line, work_vec) != cols:
            return None
        v = detector.prefilter(work_vec)
        return detector.score(v)

    # Coarse stepping
    warn_mode = False
    while z - STEP_Z_COARSE >= z_goal - 1e-12:
        z -= STEP_Z_COARSE
        p_step[2] = z
        if not safe_moveL(rtde_c, rtde_r, p_step, DESCENT_SPEED, DESCENT_ACCEL):
            print(f"[DESCENT] moveL failed at z={z:.4f}")
            break
        time.sleep(STEP_SETTLE)

        dz_from_baseline = z_baseline - z
        if detector.can_eval(dz_from_baseline) and detector.refractory_ok():
            s = measure_score()
            if s is not None and s >= warn_thr:
                warn_consec += 1
                if warn_consec >= COARSE_WARN_CONSEC:
                    print(f"[WARN] at z={z:.4f} score={s:.3f} -> switching to fine steps")
                    warn_mode = True
                    break
            else:
                warn_consec = 0

    # Fine stepping
    if warn_mode and z - STEP_Z_FINE >= z_goal - 1e-12:
        while z - STEP_Z_FINE >= z_goal - 1e-12:
            z -= STEP_Z_FINE
            p_step[2] = z
            if not safe_moveL(rtde_c, rtde_r, p_step, DESCENT_SPEED*0.5, DESCENT_ACCEL*0.5):
                print(f"[DESCENT] fine moveL failed at z={z:.4f}")
                break
            time.sleep(max(0.005, STEP_SETTLE*0.5))

            dz_from_baseline = z_baseline - z
            if not (detector.can_eval(dz_from_baseline) and detector.refractory_ok()):
                confirm_consec = 0
                continue

            s = measure_score()
            if s is None:
                confirm_consec = 0
                continue

            if s >= conf_thr:
                confirm_consec += 1
            else:
                confirm_consec = 0

            if confirm_consec >= DEBOUNCE_CONSEC:
                contact = True
                detector.mark_confirm()
                print(f"[CONTACT] at z={z:.4f} (score={s:.6f})")
                break

    final_pose = list(target_pose)
    final_pose[2] = z
    return contact, final_pose

# -------- Continuous lower to endpoint (EIT-guarded; stop on contact) -------- #

def _sample_motion_window(eit, work_vec, detector):
    line = eit.latest()
    if parse_vec_into(line, work_vec) != detector.cols:
        return None
    v = detector.prefilter(work_vec)
    return detector.score(v)

def continuous_lower_until_contact(rtde_c, rtde_r, hover_pose, goal_pose_absZ,
                                   eit, detector, work_vec,
                                   z_speed=ENDPOINT_CONT_Z_SPEED,
                                   cmd_dt=ENDPOINT_CONT_CMD_DT,
                                   stop_ramp=ENDPOINT_CONT_STOP_RAMP):
    """
    speedL-based continuous descent from hover_pose to goal_pose_absZ (same x,y,rx,ry,rz).
    Uses robust EIT scoring + debounce; stops on confirmed contact or when end_z reached.
    Returns: (contact_detected, reached_pose)
    """
    # enforce same x,y,rx,ry,rz
    for k in (0,1,3,4,5):
        assert abs(hover_pose[k]-goal_pose_absZ[k]) < 1e-9

    zmin_allowed = MIN_BASE_Z + FLOOR_MARGIN
    end_z = max(goal_pose_absZ[2], zmin_allowed)

    vz_cmd = [0.0, 0.0, -abs(z_speed), 0.0, 0.0, 0.0]
    confirm = 0
    contact = False
    z_baseline = rtde_r.getActualTCPPose()[2]

    try:
        while True:
            pose = rtde_r.getActualTCPPose()
            znow = pose[2]
            if znow <= end_z + 1e-6:
                break

            rtde_c.speedL(vz_cmd, ACCEL, cmd_dt)

            s = _sample_motion_window(eit, work_vec, detector)
            if s is not None:
                dz_from_baseline = z_baseline - znow
                if detector.can_eval(dz_from_baseline) and detector.refractory_ok():
                    if s >= detector.confirm_eff:
                        confirm += 1
                    else:
                        confirm = 0
                    if confirm >= DEBOUNCE_CONSEC:
                        contact = True
                        detector.mark_confirm()
                        print(f"[CONTACT-ENDPOINT] z={znow:.4f} score={s:.3f}")
                        break

            time.sleep(0.01)
    finally:
        try:
            rtde_c.speedStop(stop_ramp)
        except Exception:
            pass
        time.sleep(stop_ramp + 0.02)

    reached_pose = rtde_r.getActualTCPPose()
    if reached_pose[2] < end_z:
        reached_pose = list(reached_pose); reached_pose[2] = end_z
    return contact, reached_pose

# --------------------------- Small helpers ----------------------------- #

def auto_pick_port():
    cands = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return cands[0] if cands else None

def to_len(vec, n, pad=0.0):
    if len(vec) == n: return vec
    if len(vec) > n:  return vec[:n]
    return vec + [pad]*(n - len(vec))

def open_csv_logger(path, include_wrench=True, dim=None):
    fields = [
        "timestamp","session_id",
        "desired_x","desired_y","desired_z",
        "start_x","start_y","start_z","start_rx","start_ry","start_rz",
        "contact_x","contact_y","contact_z",
        "pred_x","pred_y","pred_z",
        "goal_x","goal_y","goal_z",
        "final_x","final_y","final_z",
        "err_x","err_y","err_z",
        "eit_raw_before","eit_raw_after"
    ]
    if dim:
        fields += [f"before_{i}" for i in range(dim)]
        fields += [f"after_{i}"  for i in range(dim)]
        fields += [f"delta_{i}"  for i in range(dim)]
    if include_wrench:
        fields += ["Fx","Fy","Fz","Tx","Ty","Tz"]

    exists = os.path.exists(path)
    f = open(path, "a", newline="")
    w = csv.DictWriter(f, fieldnames=fields)
    if not exists:
        w.writeheader()
    return f, w

# ------------------------------- Main --------------------------------- #

def main():
    random.seed(42)

    # Robot
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    rtde_c.setTcp(TCP_OFFSET_AT_SENSOR_CENTER)

    # EIT bring-up (async)
    port = EIT_PORT or auto_pick_port()
    eit = None
    eit_cols = 0
    try:
        if not port:
            raise RuntimeError("No EIT serial candidates found.")
        print(f"EIT serial on {port} @ {EIT_BAUD}.")
        eit = AsyncEIT(port, EIT_BAUD, EIT_TIMEOUT, sniff_secs=EIT_SNIFF_SECS)
        eit.open()
        eit_cols = eit.cols
        if eit_cols <= 0:
            print("[EIT] Unknown column count; detection disabled.")
            return
    except Exception as e:
        print(f"[EIT] Open failed: {e}")
        return

    # Model
    try:
        ensembles = load(ENSEMBLE_PATH)
    except Exception as e:
        print(f"[MODEL] Failed to load ensemble at {ENSEMBLE_PATH}: {e}")
        return

    desired_x, desired_y, desired_z = DESIRED_XYZ

    # Initial pose: random XY around desired; orientation choice
    if USE_CURRENT_ORIENT:
        rx, ry, rz = rtde_r.getActualTCPPose()[3:6]
    else:
        rx, ry, rz = EXPLICIT_START_POSE[3:6]

    start_x = desired_x + random.uniform(-RAND_INIT_X_RANGE, +RAND_INIT_X_RANGE)
    start_y = desired_y + random.uniform(-RAND_INIT_Y_RANGE, +RAND_INIT_Y_RANGE)
    start_z = desired_z + START_Z_OFFSET
    start_pose = [start_x, start_y, start_z, rx, ry, rz]

    # Move to hover
    start_hover = add_z(start_pose, HOVER_LIFT_Z)
    print("Start pose :", [round(v,6) for v in start_pose])
    if not safe_moveL(rtde_c, rtde_r, start_hover, SPEED, ACCEL):
        print("Cannot reach start hover; exiting.")
        return

    # Build detector baseline + calibrate noise at hover
    detector = RobustEITDetector(eit_cols)
    # dwell to sniff a raw "before" line
    eit_raw_before = ""
    tbh = time.time() + EIT_BEFORE_DWELL
    while time.time() < tbh:
        s = eit.latest()
        if s: eit_raw_before = s
        time.sleep(0.01)

    if not detector.build_baseline(eit, timeout_s=5.0):
        print("[EIT] Baseline failed; exiting to avoid false triggers.")
        return
    work_vec = [0.0]*eit_cols
    detector.calibrate_hover_noise(eit, work_vec, max_time_s=2.0)
    print(f"[THR] WARN_EFF={detector.warn_eff:.2f}, CONFIRM_EFF={detector.confirm_eff:.2f}")

    # Guarded descent to contact (coarse→fine)
    contact, contact_pose = descend_until_contact(
        rtde_c, rtde_r, start_hover, start_pose,
        eit, eit_cols, detector, work_vec
    )

    # Pause before "after" logging (mirrors sensor_init.py)
    time.sleep(POST_CONTACT_LOG_PAUSE)
    eit_raw_after = eit.latest() or ""

    # Build BEFORE/AFTER numeric vectors
    vb = [0.0]*eit_cols
    va = [0.0]*eit_cols
    if parse_vec_into(eit_raw_before, vb) < eit_cols:
        vb = list(detector.baseline)
    else:
        vb = list(vb)
    if parse_vec_into(eit_raw_after, va) < eit_cols:
        va = vb[:]  # worst-case: no delta
    else:
        va = list(va)

    # ΔEIT[256] shaping
    vb256 = to_len(vb, DELTA_EIT_DIM, 0.0)
    va256 = to_len(va, DELTA_EIT_DIM, 0.0)
    delta = [va256[i]-vb256[i] for i in range(DELTA_EIT_DIM)]
    delta_eit = np.asarray(delta, dtype=np.float32)[None, :]

    # Inference
    pred_xyz = predict_xyz_delta_eit(delta_eit, ensembles)  # [1,3]
    pred_x, pred_y, pred_z = map(float, pred_xyz[0])
    print(f"[PRED] xyz = {pred_x:.4f}, {pred_y:.4f}, {pred_z:.4f}")

    # Move by (desired - predicted): rise, XY at hover
    dx = desired_x - pred_x
    dy = desired_y - pred_y
    print(f"[MOVE] delta_xy = ({dx:+.4f}, {dy:+.4f}) m; target_z_abs = {desired_z:.4f} m")

    up_from_contact = add_z(contact_pose, HOVER_LIFT_Z)
    safe_moveL(rtde_c, rtde_r, up_from_contact, SPEED, ACCEL)

    hover_to_goal_xy = [
        up_from_contact[0] + dx,
        up_from_contact[1] + dy,
        up_from_contact[2],
        contact_pose[3], contact_pose[4], contact_pose[5]
    ]
    if not safe_moveL(rtde_c, rtde_r, hover_to_goal_xy, SPEED, ACCEL):
        print("[MOVE] Could not move to hover over desired XY.")
        final_pose = rtde_r.getActualTCPPose()
    else:
        # Re-baseline at corrected hover for reliable endpoint contact-stop
        if not detector.build_baseline(eit, timeout_s=3.0):
            print("[EIT] Baseline (post-XY) failed; proceeding without endpoint contact gating.")
            # Fallback: ungated lower (not recommended)
            goal_at_z = [hover_to_goal_xy[0], hover_to_goal_xy[1], DESIRED_XYZ[2],
                         hover_to_goal_xy[3], hover_to_goal_xy[4], hover_to_goal_xy[5]]
            final_pose = continuous_lower_until_contact(  # still uses function; will just never trigger
                rtde_c, rtde_r, hover_to_goal_xy, goal_at_z,
                eit, detector, work_vec,
                z_speed=ENDPOINT_CONT_Z_SPEED,
                cmd_dt=ENDPOINT_CONT_CMD_DT,
                stop_ramp=ENDPOINT_CONT_STOP_RAMP
            )[1]
        else:
            detector.calibrate_hover_noise(eit, work_vec, max_time_s=2.0)
            print(f"[THR2] WARN_EFF={detector.warn_eff:.2f}, CONFIRM_EFF={detector.confirm_eff:.2f}")

            goal_at_z = [
                hover_to_goal_xy[0],
                hover_to_goal_xy[1],
                DESIRED_XYZ[2],  # absolute Z
                hover_to_goal_xy[3], hover_to_goal_xy[4], hover_to_goal_xy[5]
            ]

            # >>> EIT-guarded continuous lower to endpoint; stop on contact <<<
            _contact2, final_pose = continuous_lower_until_contact(
                rtde_c, rtde_r,
                hover_to_goal_xy, goal_at_z,
                eit, detector, work_vec,
                z_speed=ENDPOINT_CONT_Z_SPEED,
                cmd_dt=ENDPOINT_CONT_CMD_DT,
                stop_ramp=ENDPOINT_CONT_STOP_RAMP
            )

    # Report error
    err_x = final_pose[0] - desired_x
    err_y = final_pose[1] - desired_y
    err_z = final_pose[2] - desired_z
    print(f"[RESULT] Final position:  x={final_pose[0]:.4f}, y={final_pose[1]:.4f}, z={final_pose[2]:.4f}")
    print(f"[RESULT] Desired target:  x={desired_x:.4f}, y={desired_y:.4f}, z={desired_z:.4f}")
    print(f"[ERROR ] Pose error:      dx={err_x:+.4f} m, dy={err_y:+.4f} m, dz={err_z:+.4f} m")

    # Log a row (includes raw strings + split vectors)
    csv_f, csv_w = open_csv_logger(LOG_CSV_PATH, include_wrench=LOG_WRENCH, dim=DELTA_EIT_DIM)
    wrench = rtde_r.getActualTCPForce() if LOG_WRENCH else [None]*6
    row = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "session_id": datetime.now().strftime("%Y%m%d-%H%M%S"),
        "desired_x": desired_x, "desired_y": desired_y, "desired_z": desired_z,
        "start_x": start_pose[0], "start_y": start_pose[1], "start_z": start_pose[2],
        "start_rx": start_pose[3], "start_ry": start_pose[4], "start_rz": start_pose[5],
        "contact_x": contact_pose[0], "contact_y": contact_pose[1], "contact_z": contact_pose[2],
        "pred_x": pred_x, "pred_y": pred_y, "pred_z": pred_z,
        "goal_x": final_pose[0], "goal_y": final_pose[1], "goal_z": final_pose[2],
        "final_x": final_pose[0], "final_y": final_pose[1], "final_z": final_pose[2],
        "err_x": err_x, "err_y": err_y, "err_z": err_z,
        "eit_raw_before": (eit_raw_before or ""),
        "eit_raw_after":  (eit_raw_after  or "")
    }
    for i in range(DELTA_EIT_DIM):
        row[f"before_{i}"] = vb256[i]
        row[f"after_{i}"]  = va256[i]
        row[f"delta_{i}"]  = delta[i]
    if LOG_WRENCH and wrench:
        Fx,Fy,Fz,Tx,Ty,Tz = wrench
        row.update({"Fx":Fx,"Fy":Fy,"Fz":Fz,"Tx":Tx,"Ty":Ty,"Tz":Tz})
    csv_w.writerow(row); csv_f.flush(); csv_f.close()

    # NOTE: No return-to-hover. We intentionally STAY at the endpoint.

    # Cleanup
    try: rtde_c.stopScript()
    except Exception: pass
    try:
        if eit: eit.close()
    except Exception: pass
    print("Done.")

if __name__ == "__main__":
    main()
