#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 EIT localizer — robust contact detection (MAD + hysteresis + gating)

Key ideas:
- Robust baseline per channel (median) with MAD scale.
- Prefilter each EIT frame: EMA low-pass + rolling median.
- Score = median of the top-K% per-channel robust z-scores.
- Two-threshold hysteresis (WARN slows, CONFIRM stops) with consecutive-frame debounce.
- Gating: require min downward travel from baseline before evaluating (prevents midair triggers).
- Optional force corroboration to further reduce false positives.
- Micro-step fallback near contact so speedStop has time to bite.

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
ACCEL = 0.10
DWELL = 1.0
HOVER_LIFT_Z = 0.050
EIT_BEFORE_DWELL = 0.30

# Active descent (coarse)
DESCENT_SPEED   = 0.040
DESCENT_ACCEL   = 0.200
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

# EIT serial
EIT_PORT = "/dev/ttyACM0"
EIT_BAUD = 115200
EIT_TIMEOUT = 0.2
EIT_SNIFF_SECS = 2.0

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

# ------------------------- Robust EIT detection ------------------------ #
# Tunables — start with these; tighten/loosen if still false positives.

BASELINE_FRAMES          = 50     # build robust baseline at hover
EMA_ALPHA                = 0.35   # low-pass for incoming frames
PREFILT_MEDIAN_WINDOW    = 3      # rolling median window per channel
TOP_K_PERCENT            = 20     # score uses median of top-K% channel z-scores
WARN_SCORE               = 3.0    # warn threshold (robust z-score units)
CONFIRM_SCORE            = 4.5    # confirm threshold
DEBOUNCE_CONSEC          = 3      # consecutive frames >= confirm
DZ_GUARD_ENABLE          = 0.003  # must descend at least this much from baseline (3 mm) before evaluating
REFRACTORY_SEC           = 0.5    # lockout after contact

# Optional force corroboration
REQUIRE_FORCE_BUMP       = False
FORCE_BUMP_N             = 2      # consecutive frames
FORCE_BUMP_DELTA         = 3.0    # N in |Fz| increase (N)
FORCE_ABS_MIN            = 5.0    # absolute |Fz| must be at least this much (N)

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
    cx, sx = math.cos(roll),  math.sin(roll)
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
            self.cols = max(0, raw.count(",")+1)
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
                if self.cols==0 or (s.count(",")+1)==self.cols:
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

# ------------------------ EIT vector utilities ------------------------- #

def parse_vec(line, buf):
    """Parse CSV of floats into preallocated list 'buf'. Returns length (0 if fail)."""
    if not line: return 0
    i = 0; start = 0; L = len(line)
    while start <= L:
        j = line.find(",", start)
        if j == -1: j = L
        try:
            buf[i] = float(line[start:j].strip())
        except Exception:
            return 0
        i += 1
        if i >= len(buf): break
        start = j + 1
        if j >= L: break
    return i

# ------------------------ Robust detector class ------------------------ #

class RobustEITDetector:
    """
    - Build baseline vector as per-channel MEDIAN over N frames.
    - Scale per channel by MAD; z-score_i = |x_i - med_i| / (MAD_i * 1.4826 + eps)
    - Prefilter: EMA + rolling median per channel.
    - Score = median(top-K% z-scores).
    - Hysteresis: WARN slows down; CONFIRM requires consecutive frames, gating by min dz.
    """
    def __init__(self, cols):
        self.cols = cols
        self.baseline = [0.0]*cols
        self.scale = [1.0]*cols
        self.ema = [0.0]*cols
        self.med_bufs = [deque(maxlen=PREFILT_MEDIAN_WINDOW) for _ in range(cols)]
        self._ready = False
        self.last_confirm_t = 0.0

    def build_baseline(self, eit, timeout_s=3.0):
        # collect frames
        frames = []
        work = [0.0]*self.cols
        t0 = time.time()
        while len(frames) < BASELINE_FRAMES and (time.time()-t0) < timeout_s:
            line = eit.latest()
            if parse_vec(line, work) == self.cols:
                frames.append(list(work))
            time.sleep(0.005)
        if not frames:
            # fall back to one sniff read
            self._ready = False
            return False

        # median per channel
        for k in range(self.cols):
            col = [f[k] for f in frames]
            self.baseline[k] = statistics.median(col)
            abs_dev = [abs(v - self.baseline[k]) for v in col]
            mad = statistics.median(abs_dev) if abs_dev else 0.0
            self.scale[k] = max(1e-6, 1.4826*mad)  # 1.4826 makes MAD ~ std for Gaussian
            self.ema[k] = self.baseline[k]  # start EMA at baseline
            self.med_bufs[k].clear()
        self._ready = True
        return True

    def prefilter(self, vec):
        # Per-channel EMA + rolling median
        out = [0.0]*self.cols
        a = EMA_ALPHA
        for k in range(self.cols):
            self.ema[k] = (1-a)*self.ema[k] + a*vec[k]
            self.med_bufs[k].append(self.ema[k])
            out[k] = statistics.median(self.med_bufs[k]) if self.med_bufs[k] else self.ema[k]
        return out

    def score(self, vec):
        # robust z-scores by channel
        zs = [abs(vec[k] - self.baseline[k]) / self.scale[k] for k in range(self.cols)]
        # take top-K% then median
        K = max(1, int(len(zs) * TOP_K_PERCENT / 100.0))
        zs_sorted = sorted(zs, reverse=True)[:K]
        return statistics.median(zs_sorted)

    def can_eval(self, dz_from_baseline):
        # gating: only evaluate once we moved down meaningful distance
        if dz_from_baseline is None:  # unknown => be conservative: don't eval
            return False
        return dz_from_baseline >= DZ_GUARD_ENABLE

    def refractory_ok(self):
        return (time.time() - self.last_confirm_t) >= REFRACTORY_SEC

    def mark_confirm(self):
        self.last_confirm_t = time.time()

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
      1) coarse stepping until WARN threshold is hit (or we reach z_goal)
      2) slow/fine stepping with short settle until CONFIRM hit (debounced)
    Returns (contact_detected, final_pose, steps_taken)
    """
    zmin_allowed = MIN_BASE_Z + FLOOR_MARGIN
    z_goal = max(target_pose[2] - NO_CONTACT_EXTRA_Z, zmin_allowed)

    # Track z at baseline hover to gate evaluation
    baseline_pose = rtde_r.getActualTCPPose()
    z_baseline = baseline_pose[2]

    # Coarse stage
    z = hover_pose[2]
    steps = 0
    consec = 0
    warn_mode = False
    contact = False
    last_scores = deque(maxlen=10)

    p_step = list(target_pose)
    p_step[0:3] = [target_pose[0], target_pose[1], z]  # start at hover z

    # Helper to take one measurement + score
    def measure_score():
        line = eit.latest() if eit else ""
        if parse_vec(line, work_vec) != cols:
            return None
        v = detector.prefilter(work_vec)
        s = detector.score(v)
        last_line_holder[0] = line
        last_scores.append(s)
        return s

    # Coarse stepping down
    while z - STEP_Z_COARSE >= z_goal - 1e-12:
        z -= STEP_Z_COARSE
        p_step[2] = z
        if not safe_moveL(rtde_c, rtde_r, p_step, DESCENT_SPEED, DESCENT_ACCEL):
            print(f"[DESCENT] moveL failed at z={z:.4f}")
            break
        steps += 1
        time.sleep(STEP_SETTLE)

        # Only evaluate after we moved enough since baseline hover
        dz_from_baseline = z_baseline - z
        if detector.can_eval(dz_from_baseline) and detector.refractory_ok():
            s = measure_score()
            if s is not None and s >= WARN_SCORE:
                warn_mode = True
                print(f"[WARN] at z={z:.4f} score={s:.3f} -> switching to fine steps")
                break

    # Fine stage
    if warn_mode and z - STEP_Z_FINE >= z_goal - 1e-12:
        while z - STEP_Z_FINE >= z_goal - 1e-12:
            z -= STEP_Z_FINE
            p_step[2] = z
            # micro step with slower speed
            if not safe_moveL(rtde_c, rtde_r, p_step, DESCENT_SPEED*0.5, DESCENT_ACCEL*0.5):
                print(f"[DESCENT] fine moveL failed at z={z:.4f}")
                break
            steps += 1
            time.sleep(max(0.005, STEP_SETTLE*0.5))

            dz_from_baseline = z_baseline - z
            if detector.can_eval(dz_from_baseline) and detector.refractory_ok():
                s = measure_score()
                if s is None:
                    consec = 0
                    continue

                # Optional force corroboration
                force_ok = True
                if REQUIRE_FORCE_BUMP:
                    Fx,Fy,Fz,Tx,Ty,Tz = rtde_r.getActualTCPForce()
                    # crude bump check vs recent history of Fz
                    # keep last few Fz in the deque via last_scores length
                    if abs(Fz) < FORCE_ABS_MIN:
                        force_ok = False

                if s >= CONFIRM_SCORE and force_ok:
                    consec += 1
                else:
                    consec = 0

                if consec >= DEBOUNCE_CONSEC:
                    contact = True
                    detector.mark_confirm()
                    print(f"[CONTACT] at z={z:.4f} (score={s:.6f})")
                    break

    # Final pose = current command (safe; settle and read actual)
    final_pose = list(target_pose)
    final_pose[2] = z
    # hard stop to be safe (in case we’ll log & go hover)
    try:
        rtde_c.speedStop()
    except Exception:
        pass
    time.sleep(0.02)
    return contact, final_pose, steps

# ------------------------------- Main --------------------------------- #

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
            selected_indices = random.sample(range(total), k)
            stream_all = False
            print(f"[GRID] Randomly selected {k} poses (seed={RANDOM_SEED}).")

        # CSV
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
                # stash baseline snapshot for logging “eitb_*”
                if parse_vec(eit_raw_before, baseline) != eit_cols:
                    # if parse fails, keep detector.baseline
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

            # Rebuild baseline each pose (robust against slow drift)
            eit_raw_before = ""
            if detector:
                tbh = time.time() + EIT_BEFORE_DWELL
                while time.time() < tbh:
                    s = eit.latest()
                    if s: eit_raw_before = s
                    time.sleep(0.01)
                detector.build_baseline(eit, timeout_s=3.0)

            # Descent
            contact = False; steps_taken=0; eit_raw_after=""
            final_pose = target
            if detector:
                contact, final_pose, steps_taken = descend_until_contact(
                    rtde_c, rtde_r, target_hover, target,
                    eit, eit_cols, detector, work_vec, last_line_holder
                )
                eit_raw_after = last_line_holder[0]
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
                # store the *baseline* snapshot and post-contact vecs for offline plots
                if parse_vec(eit_raw_before, work_vec) == eit_cols:
                    for i in range(eit_cols): row[f"eitb_{i}"] = work_vec[i]
                else:
                    for i in range(eit_cols): row[f"eitb_{i}"] = ""
                if parse_vec(eit_raw_after, work_vec) == eit_cols:
                    for i in range(eit_cols): row[f"eita_{i}"] = work_vec[i]
                else:
                    for i in range(eit_cols): row[f"eita_{i}"] = ""

            if LOG_WRENCH and wrench:
                Fx,Fy,Fz,Tx,Ty,Tz = wrench
                row.update({"Fx":Fx,"Fy":Fy,"Fz":Fz,"Tx":Tx,"Ty":Ty,"Tz":Tz})

            writer.writerow(row); csv_f.flush()

            # back to hover
            if not safe_moveL(rtde_c, rtde_r, target_hover, SPEED, ACCEL):
                return False, None
            return True, final_pose

        if stream_all:
            idxs = [0,0,0,0,0,0]
            limits = [a.count for a in axes]
            linear = 0
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
            for j, lin in enumerate(selected_indices, start=1):
                idxs = decode_linear_index(lin, axes)
                deltas, target = pose_from_indices(idxs)
                print(f"[{j}/{len(selected_indices)}] target: "
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
