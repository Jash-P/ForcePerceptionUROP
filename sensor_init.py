#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 EIT localizer — robust contact detection (MAD + hysteresis + gating)

This version integrates:
- Robust CSV parsing for quoted EIT frames so eitb_* / eita_* log reliably.
- Hardened detector (noise floor, channel masking, winsorized z) to prevent mid-air triggers.
- 1s pause after contact before logging "after" EIT for settling.
- Joint-limit avoidance while hovered (360° unwind opposite direction if near limits).
- Auto-resume: on restart, reads existing CSV rows and resumes from the next pose.

Requires: ur-rtde, pyserial
"""

import math
import time
import csv, os, random, threading, statistics
from collections import deque
from datetime import datetime

import serial
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# ---------------------------- USER SETTINGS ---------------------------- #

ROBOT_IP = "169.254.150.50"

# TCP at sensor centre (meters, axis–angle)
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion & dwell
SPEED = 0.05
ACCEL = 0.05
DWELL = 1.0
HOVER_LIFT_Z = 0.050
EIT_BEFORE_DWELL = 0.30

# Active descent (coarse / fine)
DESCENT_SPEED   = 0.040
DESCENT_ACCEL   = 0.040
STEP_Z_COARSE   = 0.0010     # 1.0 mm
STEP_Z_FINE     = 0.00025    # 0.25 mm after WARN
STEP_SETTLE     = 0.010
NO_CONTACT_EXTRA_Z = 0.015   # go a little below target if no contact

# Floor guard (base frame)
MIN_BASE_Z = -0.390
FLOOR_MARGIN = 0.010
CLAMP_BELOW_FLOOR = False

# Logging
LOG_CSV_PATH = "eit_localisation_log.csv"
LOG_WRENCH   = True
LOG_EIT_RAW_STRINGS = False  # keep False to avoid “extra column” chaos in Excel
POST_CONTACT_LOG_PAUSE = 1.0 # seconds to wait before sampling “after” EIT

# EIT serial
EIT_PORT = "/dev/ttyACM0"
EIT_BAUD = 115200
EIT_TIMEOUT = 0.2
EIT_SNIFF_SECS = 5.0

# Grid (meters & degrees)
X_MIN, X_MAX, X_STEP = -0.020, +0.020, 0.001
Y_MIN, Y_MAX, Y_STEP = -0.020, +0.020, 0.001
Z_MIN, Z_MAX, Z_STEP =  0.000, +0.005, 0.001
ROLL_MIN,  ROLL_MAX,  ROLL_STEP  = -10, +10, 1
PITCH_MIN, PITCH_MAX, PITCH_STEP = -10, +10, 1
YAW_MIN,   YAW_MAX,   YAW_STEP   = -10, +10, 1

# Sampling
SELECTION_MODE   = "RANDOM"   # or "ALL"
RANDOM_SAMPLE_N  = 1500
RANDOM_SEED      = 42

# Auto-resume
AUTO_RESUME = True

# ------------------------- Robust EIT detection ------------------------ #
# Thresholds per latest spec / noise-adaptive logic.

BASELINE_FRAMES          = 50
EMA_ALPHA                = 0.35
PREFILT_MEDIAN_WINDOW    = 3
TOP_K_PERCENT            = 20
WARN_SCORE               = 4.0
CONFIRM_SCORE            = 5.5
DEBOUNCE_CONSEC          = 5
DZ_GUARD_ENABLE          = 0.003
REFRACTORY_SEC           = 0.5

# --- EIT noise-floor & channel masking (prevents mid-air false positives) ---
SCALE_ABS_FLOOR         = 0.02   # minimum per-channel scale (in raw EIT units)
MAD_SCALE_PCTL_FLOOR    = 50     # use at least the p50 MAD across channels as the floor
MIN_VALID_CHANS_RATIO   = 0.60   # require >=60% channels deemed "valid" to score
Z_HUGE_CLIP             = 50.0   # winsorize extreme per-channel z before top-K/median

# --- Noise-adaptive thresholding around hover ---
HOVER_NOISE_FRAMES   = 80
HOVER_NOISE_PCTL     = 95
NOISE_MARGIN         = 1.0
CONFIRM_EXTRA        = 0.75

COARSE_WARN_CONSEC   = 2  # stable warn before fine stepping

# Optional force corroboration
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

def rpy_to_rotmat(roll, pitch, yaw):
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    Rz = ((cz,-sz,0),(sz,cz,0),(0,0,1))
    Ry = ((cy,0,sy),(0,1,0),(-sy,0,cy))
    Rx = ((1,0,0),(0,cx,-sx),(0,sx,cx))
    def mm(A,B):
        return (
            (A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0],
             A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1],
             A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2]),
            (A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0],
             A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1],
             A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2]),
            (A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0],
             A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1],
             A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2])
        )
    return mm(mm(Rz,Ry),Rx)

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

def rotmat_to_rpy(R):
    if abs(R[2][0]) < 1.0:
        pitch = math.asin(-R[2][0])
        roll  = math.atan2(R[2][1], R[2][2])
        yaw   = math.atan2(R[1][0], R[0][0])
    else:
        pitch = math.pi/2 if R[2][0] <= -1.0 else -math.pi/2
        roll  = 0.0
        yaw   = math.atan2(-R[0][1], R[1][1])
    return roll, pitch, yaw

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

# ----------------------- Joint-limit avoidance ------------------------- #

# Approximate UR5 joint hard limits (rad) – conservative
JOINT_LIMITS = [
    (-2*math.pi,  2*math.pi),   # base
    (-2*math.pi,  2*math.pi),   # shoulder
    (-2*math.pi,  2*math.pi),   # elbow
    (-2*math.pi,  2*math.pi),   # wrist1
    (-2*math.pi,  2*math.pi),   # wrist2
    (-2*math.pi,  2*math.pi),   # wrist3
]
JOINT_MARGIN = 0.25            # rad; if closer than this to a limit, unwind 2π
JOINT_UNWIND_SPEED = 0.7
JOINT_UNWIND_ACCEL = 0.7

def ensure_joint_margin(rtde_c, rtde_r):
    """If any joint is too close to a limit, unwind 360° in the opposite direction while hovered."""
    try:
        q = list(rtde_r.getActualQ())
    except Exception:
        return
    changed = False
    q_cmd = q[:]
    for i,(lo,hi) in enumerate(JOINT_LIMITS):
        if (q[i] - lo) < JOINT_MARGIN:
            q_cmd[i] = q[i] + 2*math.pi
            changed = True
        elif (hi - q[i]) < JOINT_MARGIN:
            q_cmd[i] = q[i] - 2*math.pi
            changed = True
    if changed:
        rtde_c.moveJ(q_cmd, JOINT_UNWIND_SPEED, JOINT_UNWIND_ACCEL)

# --------------- Robust CSV parsing for quoted EIT frames -------------- #

def _split_csv_fields(line: str):
    """Robustly split a CSV line into fields, handling quotes and stray commas."""
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

def parse_vec(line, buf):
    fields = _split_csv_fields(line)
    n = min(len(fields), len(buf))
    try:
        for i in range(n):
            buf[i] = _to_float(fields[i])
    except Exception:
        return 0
    return n

# ----------------------- Async EIT (low-memory) ------------------------ #

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
    """
    - Baseline: per-channel MEDIAN; scale = max(1.4826*MAD, robust floor).
    - Prefilter: EMA + rolling median.
    - Score: median of top-K% robust z for VALID channels only (mask tiny-variance),
             with winsorization at Z_HUGE_CLIP.
    """
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
            if parse_vec(line, work) == self.cols:
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

        # Robust floor from channel MAD distribution
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
            print(f"[EIT] Too few valid channels ({len(self.valid_idx)}/{self.cols}); "
                  f"check sensor/cabling or relax floors.")
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

    def _score_of_latest(self, eit, work_vec):
        line = eit.latest()
        if parse_vec(line, work_vec) != self.cols:
            return None
        v = self.prefilter(work_vec)
        return self.score(v)

    def calibrate_hover_noise(self, eit, work_vec, max_time_s=2.0):
        scores = []
        t0 = time.time()
        while len(scores) < HOVER_NOISE_FRAMES and (time.time() - t0) < max_time_s:
            s = self._score_of_latest(eit, work_vec)
            if s is not None:
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

# ---------------------- Range sampling (memory-safe) ------------------- #

class UniformGrid1D:
    __slots__ = ("vmin","step","count")
    def __init__(self, vmin, vmax, step):
        self.vmin = vmin
        self.step = step
        c = int(round((vmax - vmin)/step))
        if vmin + c*step < vmax - 1e-12: c += 1
        self.count = c + 1
    def value(self, idx):
        return round(self.vmin + idx*self.step, 9)

def total_count(axes):
    t = 1
    for a in axes: t *= a.count
    return t

def decode_linear_index(idx, axes):
    i0 = idx
    out = [0]*len(axes)
    for k in reversed(range(len(axes))):
        c = axes[k].count
        out[k] = i0 % c
        i0 //= c
    return out

# -------------------------- Guarded descent ---------------------------- #

def descend_until_contact(rtde_c, rtde_r, hover_pose, target_pose,
                          eit, cols, detector, work_vec, last_line_holder):
    """
    Hybrid descent:
      1) coarse stepping until WARN threshold is stably hit
      2) fine stepping with short settle until CONFIRM hit (debounced; optional force)
    Returns (contact_detected, final_pose, steps_taken)
    """
    zmin_allowed = MIN_BASE_Z + FLOOR_MARGIN
    z_goal = max(target_pose[2] - NO_CONTACT_EXTRA_Z, zmin_allowed)

    z_baseline = rtde_r.getActualTCPPose()[2]

    warn_thr = getattr(detector, "warn_eff", WARN_SCORE)
    conf_thr = getattr(detector, "confirm_eff", CONFIRM_SCORE)

    z = hover_pose[2]
    steps = 0
    warn_mode = False
    warn_consec = 0
    confirm_consec = 0
    contact = False

    score_hist = deque(maxlen=12)
    fz_hist    = deque(maxlen=20)
    last_line_holder[0] = ""

    p_step = list(target_pose)
    p_step[0:3] = [target_pose[0], target_pose[1], z]

    def measure_score():
        line = eit.latest() if eit else ""
        if parse_vec(line, work_vec) != cols:
            return None
        v = detector.prefilter(work_vec)
        s = detector.score(v)
        last_line_holder[0] = line
        score_hist.append(s)
        return s

    def update_force_and_check_bump():
        Fx, Fy, Fz, Tx, Ty, Tz = rtde_r.getActualTCPForce()
        afz = abs(Fz)
        fz_hist.append(afz)
        if len(fz_hist) < FORCE_BUMP_N + 2:
            return False
        recent = list(fz_hist)[-FORCE_BUMP_N:]
        prev_med = statistics.median(list(fz_hist)[:-FORCE_BUMP_N])
        return (max(recent) - prev_med) >= FORCE_BUMP_DELTA and max(recent) >= FORCE_ABS_MIN

    # Coarse stepping
    while z - STEP_Z_COARSE >= z_goal - 1e-12:
        z -= STEP_Z_COARSE
        p_step[2] = z
        if not safe_moveL(rtde_c, rtde_r, p_step, DESCENT_SPEED, DESCENT_ACCEL):
            print(f"[DESCENT] moveL failed at z={z:.4f}")
            break
        steps += 1
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
        force_bump_consec = 0
        while z - STEP_Z_FINE >= z_goal - 1e-12:
            z -= STEP_Z_FINE
            p_step[2] = z
            if not safe_moveL(rtde_c, rtde_r, p_step, DESCENT_SPEED*0.5, DESCENT_ACCEL*0.5):
                print(f"[DESCENT] fine moveL failed at z={z:.4f}")
                break
            steps += 1
            time.sleep(max(0.005, STEP_SETTLE*0.5))

            dz_from_baseline = z_baseline - z
            if not (detector.can_eval(dz_from_baseline) and detector.refractory_ok()):
                confirm_consec = 0
                force_bump_consec = 0
                continue

            s = measure_score()
            if s is None:
                confirm_consec = 0
                force_bump_consec = 0
                continue

            force_ok = True
            if REQUIRE_FORCE_BUMP:
                if update_force_and_check_bump():
                    force_bump_consec += 1
                else:
                    force_bump_consec = 0
                force_ok = (force_bump_consec >= FORCE_BUMP_N)

            if s >= conf_thr and force_ok:
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

    try:
        rtde_c.speedStop()
    except Exception:
        pass
    time.sleep(0.02)

    return contact, final_pose, steps

# ------------------------------- Main --------------------------------- #

def _count_completed_rows(csv_path):
    if not os.path.exists(csv_path):
        return 0
    with open(csv_path, "r", newline="") as f:
        # Count non-empty lines minus header
        n = sum(1 for _ in f)
    return max(0, n - 1)

def main():
    random.seed(RANDOM_SEED)

    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)

    # EIT bring-up
    eit = None
    eit_cols = 0
    try:
        print(f"EIT serial on {EIT_PORT} @ {EIT_BAUD}.")
        eit = AsyncEIT(EIT_PORT, EIT_BAUD, EIT_TIMEOUT, sniff_secs=EIT_SNIFF_SECS)
        eit.open()
        eit_cols = eit.cols
        if eit_cols <= 0:
            print("[EIT] Unknown column count; detection disabled.")
    except Exception as e:
        print(f"[EIT] Open failed: {e}")
        eit = None
        eit_cols = 0

    csv_f = None
    try:
        rtde_c.setTcp(TCP_OFFSET_AT_SENSOR_CENTER)

        start_pose = rtde_r.getActualTCPPose()
        R0 = rvec_to_rotmat(start_pose[3:6])
        base_roll, base_pitch, base_yaw = rotmat_to_rpy(R0)

        start_hover = add_z(start_pose, HOVER_LIFT_Z)
        if not safe_moveL(rtde_c, rtde_r, start_hover, SPEED, ACCEL):
            print("Cannot reach start hover; exiting.")
            return

        # Make sure we are not near joint limits before starting grid
        ensure_joint_margin(rtde_c, rtde_r)

        # Axes
        ax = UniformGrid1D(X_MIN, X_MAX, X_STEP)
        ay = UniformGrid1D(Y_MIN, Y_MAX, Y_STEP)
        az = UniformGrid1D(Z_MIN, Z_MAX, Z_STEP)
        ar = UniformGrid1D(ROLL_MIN,  ROLL_MAX,  ROLL_STEP)
        ap = UniformGrid1D(PITCH_MIN, PITCH_MAX, PITCH_STEP)
        ayw= UniformGrid1D(YAW_MIN,   YAW_MAX,   YAW_STEP)
        axes = (ax, ay, az, ar, ap, ayw)

        total = total_count(axes)
        print(f"[GRID] Total poses: {total:,}")

        if SELECTION_MODE.upper() == "ALL":
            selected_indices = None
            stream_all = True
        else:
            k = min(RANDOM_SAMPLE_N, total)
            # Deterministic order (seed) → resume works
            selected_indices = random.sample(range(total), k)
            stream_all = False
            print(f"[GRID] Randomly selected {k} poses (seed={RANDOM_SEED}).")

        # CSV setup
        exists = os.path.exists(LOG_CSV_PATH)
        csv_f = open(LOG_CSV_PATH, "a", newline="")
        base_fields = [
            "timestamp","session_id","index",
            "dx_m","dy_m","dz_m","roll_deg","pitch_deg","yaw_deg",
            "sensor_x","sensor_y","sensor_z","sensor_rx","sensor_ry","sensor_rz",
            "cmd_tcp_x","cmd_tcp_y","cmd_tcp_z","cmd_tcp_rx","cmd_tcp_ry","cmd_tcp_rz",
            "act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz",
        ]
        raw_fields = (["eit_raw_before","eit_raw_after"] if LOG_EIT_RAW_STRINGS else [])
        contact_fields = ["contact_detected","steps_taken","contact_z"]
        split_fields = ([f"eitb_{i}" for i in range(eit_cols)] +
                        [f"eita_{i}" for i in range(eit_cols)]) if eit_cols else []
        wrench_fields = ["Fx","Fy","Fz","Tx","Ty","Tz"] if LOG_WRENCH else []
        fieldnames = base_fields + raw_fields + contact_fields + split_fields + wrench_fields
        writer = csv.DictWriter(csv_f, fieldnames=fieldnames, quoting=csv.QUOTE_MINIMAL, lineterminator="\n")
        if not exists:
            writer.writeheader(); csv_f.flush()

        # Auto-resume: figure out starting index based on completed rows
        start_index_in_log = 1
        if AUTO_RESUME:
            done_rows = _count_completed_rows(LOG_CSV_PATH)
            if done_rows > 0:
                start_index_in_log = done_rows + 1
                print(f"[RESUME] Found {done_rows} completed rows. Resuming at index {start_index_in_log}.")

        session_id = datetime.now().strftime("%Y%m%d-%H%M%S")

        # Prealloc EIT buffers
        work_vec   = [0.0]*eit_cols if eit_cols else []
        baseline   = [0.0]*eit_cols if eit_cols else []
        last_line_holder = [""]

        # Build detector/baseline at first hover
        detector = None
        if eit and eit_cols:
            detector = RobustEITDetector(eit_cols)
            # quick dwell for raw sniff
            tbh = time.time() + EIT_BEFORE_DWELL
            eit_raw_before = ""
            while time.time() < tbh:
                s = eit.latest()
                if s: eit_raw_before = s
                time.sleep(0.01)
            if detector.build_baseline(eit, timeout_s=5.0):
                if parse_vec(eit_raw_before, baseline) < eit_cols:
                    for i in range(eit_cols):
                        baseline[i] = detector.baseline[i]
                print("[EIT] Baseline ready.")
            else:
                print("[EIT] Baseline failed; detection disabled.")
                detector = None

        # Helpers
        def pose_from_indices(idxs):
            dx = ax.value(idxs[0]); dy = ay.value(idxs[1]); dz = az.value(idxs[2])
            r  = ar.value(idxs[3]); p  = ap.value(idxs[4]); y  = ayw.value(idxs[5])
            x = start_pose[0] + dx; yy = start_pose[1] + dy; zz = start_pose[2] + dz
            roll  = base_roll  + deg2rad(r)
            pitch = base_pitch + deg2rad(p)
            yaw   = base_yaw   + deg2rad(y)
            R = rpy_to_rotmat(roll, pitch, yaw)
            rvx, rvy, rvz = rotmat_to_rvec(R)
            tgt = [x, yy, zz, rvx, rvy, rvz]
            return (dx,dy,dz,r,p,y), tgt

        def do_one_pose(idx_in_log, target, deltas):
            target_hover = add_z(target, HOVER_LIFT_Z)
            if not safe_moveL(rtde_c, rtde_r, target_hover, SPEED, ACCEL):
                return False, None

            # Avoid joint limits while hovered
            ensure_joint_margin(rtde_c, rtde_r)

            # Rebuild baseline each pose (robust against slow drift)
            eit_raw_before = ""
            if detector:
                tbh = time.time() + EIT_BEFORE_DWELL
                while time.time() < tbh:
                    s = eit.latest()
                    if s: eit_raw_before = s
                    time.sleep(0.01)
                detector.build_baseline(eit, timeout_s=3.0)
                detector.calibrate_hover_noise(eit, work_vec, max_time_s=2.0)
                print(f"[THR] WARN_EFF={detector.warn_eff:.2f}, CONFIRM_EFF={detector.confirm_eff:.2f}")

            # Descent
            contact = False; steps_taken=0; eit_raw_after=""
            final_pose = target
            if detector:
                contact, final_pose, steps_taken = descend_until_contact(
                    rtde_c, rtde_r, target_hover, target,
                    eit, eit_cols, detector, work_vec, last_line_holder
                )
                # pause to let EIT settle for logging only
                if contact:
                    time.sleep(POST_CONTACT_LOG_PAUSE)
                    eit_raw_after = eit.latest() if eit else ""
                else:
                    eit_raw_after = eit.latest() if eit else ""
            else:
                if not safe_moveL(rtde_c, rtde_r, target, SPEED, ACCEL):
                    return False, None
                time.sleep(DWELL)

            # Log
            actual = rtde_r.getActualTCPPose()
            wrench = rtde_r.getActualTCPForce() if LOG_WRENCH else [None]*6
            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "session_id": session_id,
                "index": idx_in_log,
                "dx_m": deltas[0], "dy_m": deltas[1], "dz_m": deltas[2],
                "roll_deg": deltas[3], "pitch_deg": deltas[4], "yaw_deg": deltas[5],
                "sensor_x": final_pose[0], "sensor_y": final_pose[1], "sensor_z": final_pose[2],
                "sensor_rx": final_pose[3], "sensor_ry": final_pose[4], "sensor_rz": final_pose[5],
                "cmd_tcp_x": final_pose[0], "cmd_tcp_y": final_pose[1], "cmd_tcp_z": final_pose[2],
                "cmd_tcp_rx": final_pose[3], "cmd_tcp_ry": final_pose[4], "cmd_tcp_rz": final_pose[5],
                "act_tcp_x": actual[0], "act_tcp_y": actual[1], "act_tcp_z": actual[2],
                "act_tcp_rx": actual[3], "act_tcp_ry": actual[4], "act_tcp_rz": actual[5],
                "contact_detected": int(bool(contact)),
                "steps_taken": steps_taken,
                "contact_z": final_pose[2],
            }
            if LOG_EIT_RAW_STRINGS:
                row["eit_raw_before"] = eit_raw_before
                row["eit_raw_after"]  = eit_raw_after

            if eit_cols:
                # baseline snapshot → eitb_*
                if parse_vec(eit_raw_before, work_vec) >= eit_cols:
                    for i in range(eit_cols): row[f"eitb_{i}"] = work_vec[i]
                else:
                    if detector:
                        for i in range(eit_cols): row[f"eitb_{i}"] = detector.baseline[i]
                    else:
                        for i in range(eit_cols): row[f"eitb_{i}"] = ""
                # post-contact snapshot → eita_*
                if parse_vec(eit_raw_after, work_vec) >= eit_cols:
                    for i in range(eit_cols): row[f"eita_{i}"] = work_vec[i]
                else:
                    for i in range(eit_cols): row[f"eita_{i}"] = ""

            if LOG_WRENCH and wrench:
                Fx,Fy,Fz,Tx,Ty,Tz = wrench
                row.update({"Fx":Fx,"Fy":Fy,"Fz":Fz,"Tx":Tx,"Ty":Ty,"Tz":Tz})

            writer.writerow(row); csv_f.flush()

            # back to hover for next pose
            if not safe_moveL(rtde_c, rtde_r, target_hover, SPEED, ACCEL):
                return False, None
            return True, final_pose

        # Iteration logic with resume
        if stream_all:
            # Mixed-radix counter
            idxs = [0,0,0,0,0,0]
            limits = [a.count for a in axes]
            linear = 0

            # Fast-forward if resuming
            if AUTO_RESUME and start_index_in_log > 1:
                skip = start_index_in_log - 1
                linear = skip
                # decode the (skip)th index in lexicographic order
                # by iterating counters skip times (cheap enough), or compute directly:
                # We'll increment counters skip times (safe & simple).
                for _ in range(skip):
                    for d in range(5, -1, -1):
                        idxs[d] += 1
                        if idxs[d] < limits[d]:
                            break
                        idxs[d] = 0

            while True:
                deltas, target = pose_from_indices(idxs)
                ok, _ = do_one_pose(linear+1, target, deltas)
                if not ok: break
                linear += 1
                for d in range(5, -1, -1):
                    idxs[d] += 1
                    if idxs[d] < limits[d]: break
                    idxs[d] = 0
                else:
                    break
        else:
            # Random subset in deterministic order; resume by skipping done rows
            start_j = start_index_in_log
            total_sel = len(selected_indices)
            for j, lin in enumerate(selected_indices, start=1):
                if j < start_j:
                    continue  # skip done
                idxs = decode_linear_index(lin, axes)
                deltas, target = pose_from_indices(idxs)
                print(f"[{j}/{total_sel}] target: "
                      f"{[round(v,6) for v in target]} (dx={deltas[0]},dy={deltas[1]},dz={deltas[2]},r={deltas[3]},p={deltas[4]},y={deltas[5]})")
                ok, _ = do_one_pose(j, target, deltas)
                if not ok: break

        # Return to anchor
        safe_moveL(rtde_c, rtde_r, add_z(start_pose, HOVER_LIFT_Z), SPEED, ACCEL)
        safe_moveL(rtde_c, rtde_r, start_pose, SPEED, ACCEL)

    finally:
        try: rtde_c.speedStop()
        except Exception: pass
        try: rtde_c.stopScript()
        except Exception: pass
        try:
            if csv_f: csv_f.close()
        except Exception: pass
        try:
            if eit: eit.close()
        except Exception: pass
        print("Done.")

if __name__ == "__main__":
    main()
