#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 EIT localizer: range-based discretization + random subset selection
Adds "rise → reposition at hover → lower" between every pose.
Adds Way B: software Z-floor guard (MIN_BASE_Z).
Requires: ur_rtde, pyserial
"""

import math
import time
import csv, os, random
from datetime import datetime
from itertools import product

import serial  # pip install pyserial

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# ---------------------------- USER SETTINGS ---------------------------- #

ROBOT_IP = "169.254.150.50"   # your UR5 IP

# TCP offset to the *sensor center* (meters, axis-angle radians)
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion & dwell
SPEED = 0.10          # m/s
ACCEL = 0.10          # m/s^2
DWELL = 1.30          # s to settle & read EIT per pose

# Hover / retract height (base Z)
HOVER_LIFT_Z = 0.100  # 100 mm

# ---------- Way B: software floor (base-frame Z) ----------
MIN_BASE_Z = -0.390    # <-- set this to your safe floor height (meters)
CLAMP_BELOW_FLOOR = False  # True: clamp Z to floor instead of aborting

# Logging
LOG_CSV_PATH = "eit_localisation_log.csv"
LOG_WRENCH   = True

# EIT serial
EIT_PORT = "COM5"
EIT_BAUD = 115200
EIT_TIMEOUT = 0.2     # seconds for .readline()
EIT_SNIFF_SECS = 2.0  # try to grab one line at start to size columns

# Anchor pose
USE_CURRENT_POSE_AS_START = True
EXPLICIT_START_POSE = [0.40, -0.20, 0.20, 3.1415, 0.0, 0.0]

# -------- RANGE MODE (min/max with fixed increments) --------
# Distances in meters; angles in degrees. Increments are constant.
X_MIN, X_MAX, X_STEP = -0.030, +0.030, 0.005
Y_MIN, Y_MAX, Y_STEP = -0.030, +0.030, 0.005
Z_MIN, Z_MAX, Z_STEP =  0.000, +0.010, 0.001

ROLL_MIN,  ROLL_MAX,  ROLL_STEP  = -10, +10, 1
PITCH_MIN, PITCH_MAX, PITCH_STEP = -10, +10, 1
YAW_MIN,   YAW_MAX,   YAW_STEP   = -20, +20, 1

# Selection: do ALL poses or RANDOM sample of N
SELECTION_MODE   = "RANDOM"   # "ALL" or "RANDOM"
RANDOM_SAMPLE_N  = 50         # used only if SELECTION_MODE == "RANDOM"
RANDOM_SEED      = 42         # for reproducibility

# ---------------------------------------------------------------------- #
# ---------- math helpers: rotations & pose composition (axis-angle) --- #

def rvec_to_rotmat(rvec):
    rx, ry, rz = rvec
    theta = math.sqrt(rx*rx + ry*ry + rz*rz)
    if theta < 1e-12:
        return [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
    ux, uy, uz = rx/theta, ry/theta, rz/theta
    c = math.cos(theta); s = math.sin(theta); C = 1.0 - c
    return [
        [c+ux*ux*C,      ux*uy*C - uz*s, ux*uz*C + uy*s],
        [uy*ux*C + uz*s, c+uy*uy*C,      uy*uz*C - ux*s],
        [uz*ux*C - uy*s, uz*uy*C + ux*s, c+uz*uz*C     ]
    ]

def rotmat_to_rvec(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    cos_theta = max(min((tr - 1.0)/2.0, 1.0), -1.0)
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return [0.0, 0.0, 0.0]
    denom = 2.0*math.sin(theta)
    rx = (R[2][1] - R[1][2]) / denom
    ry = (R[0][2] - R[2][0]) / denom
    rz = (R[1][0] - R[0][1]) / denom
    return [rx*theta, ry*theta, rz*theta]

def rpy_to_rotmat(roll, pitch, yaw, order="XYZ"):
    cx, sx = math.cos(roll),  math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)

    Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]]
    Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rz = [[cz,-sz,0],[sz,cz,0],[0,0,1]]

    def mm(A,B):
        return [
            [A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0],
             A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1],
             A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2]],
            [A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0],
             A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1],
             A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2]],
            [A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0],
             A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1],
             A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2]]
        ]
    if order.upper() == "XYZ":
        return mm(mm(Rx, Ry), Rz)
    elif order.upper() == "ZYX":
        return mm(mm(Rz, Ry), Rx)
    else:
        raise ValueError("Unsupported RPY order")

def rpy_to_rvec(roll, pitch, yaw, order="XYZ"):
    R = rpy_to_rotmat(roll, pitch, yaw, order=order)
    return rotmat_to_rvec(R)

def pose_to_tf(pose):
    x,y,z, rx,ry,rz = pose
    R = rvec_to_rotmat([rx,ry,rz])
    return [
        [R[0][0], R[0][1], R[0][2], x],
        [R[1][0], R[1][1], R[1][2], y],
        [R[2][0], R[2][1], R[2][2], z],
        [0,0,0,1]
    ]

def tf_to_pose(T):
    x,y,z = T[0][3], T[1][3], T[2][3]
    R = [[T[0][0],T[0][1],T[0][2]],
         [T[1][0],T[1][1],T[1][2]],
         [T[2][0],T[2][1],T[2][2]]]
    rx,ry,rz = rotmat_to_rvec(R)
    return [x,y,z, rx,ry,rz]

def tf_mul(A,B):
    out = [[0.0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(A[i][k]*B[k][j] for k in range(4))
    return out

def compose_pose(base_pose, delta_pose_toolframe):
    T_base  = pose_to_tf(base_pose)
    T_delta = pose_to_tf(delta_pose_toolframe)
    T_new   = tf_mul(T_base, T_delta)
    return tf_to_pose(T_new)

def deg2rad(d): return d * math.pi / 180.0

# ---------- Way B helpers: software floor ---------- #
def floor_guard(pose):
    """Return a pose that is safe wrt MIN_BASE_Z.
    - If CLAMP_BELOW_FLOOR is False: raise ValueError if below floor.
    - If True: clamp Z to MIN_BASE_Z and return adjusted pose.
    """
    if pose[2] >= MIN_BASE_Z - 1e-9:
        return pose
    if CLAMP_BELOW_FLOOR:
        p = list(pose)
        p[2] = MIN_BASE_Z
        print(f"[FLOOR] Clamped Z from {pose[2]:.3f} to {MIN_BASE_Z:.3f}")
        return p
    raise ValueError(f"Target Z={pose[2]:.3f} m is below floor ({MIN_BASE_Z:.3f} m).")

def safe_moveL(rtde_c, rtde_r, target, speed, accel):
    """Move with floor check. Returns False if rejected or move fails."""
    try:
        tgt = floor_guard(target)
    except ValueError as e:
        print(f"[FLOOR ABORT] {e}")
        return False
    ok = rtde_c.moveL(tgt, speed, accel)
    return bool(ok)

# ---------- EIT serial helpers ---------- #

def open_eit_serial(port, baud, timeout):
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser

def read_eit_line(ser, timeout_s):
    t0 = time.time()
    last = ""
    while time.time() - t0 < timeout_s:
        line = ser.readline()
        if not line:
            continue
        try:
            s = line.decode("utf-8", errors="ignore").strip()
        except Exception:
            s = ""
        if s:
            last = s
            break
    return last

def sniff_eit_columns(ser, sniff_secs=2.0):
    raw = read_eit_line(ser, sniff_secs)
    if not raw:
        return [], ""
    parts = [p.strip() for p in raw.split(",")]
    cols = [f"eit_{i}" for i in range(len(parts))]
    return cols, raw

# ---------- CSV helper ---------- #
def open_csv_logger(path, include_wrench=True, eit_cols=None):
    fields = [
        "timestamp","session_id","index",
        "dx_m","dy_m","dz_m","roll_deg","pitch_deg","yaw_deg",
        "sensor_x","sensor_y","sensor_z","sensor_rx","sensor_ry","sensor_rz",
        "cmd_tcp_x","cmd_tcp_y","cmd_tcp_z","cmd_tcp_rx","cmd_tcp_ry","cmd_tcp_rz",
        "act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz",
        "eit_raw"
    ]
    if eit_cols:
        fields += eit_cols
    if include_wrench:
        fields += ["Fx","Fy","Fz","Tx","Ty","Tz"]

    exists = os.path.exists(path)
    f = open(path, "a", newline="")
    w = csv.DictWriter(f, fieldnames=fields)
    if not exists:
        w.writeheader()
    return f, w, fields

# ---------- Range tools ---------- #
def frange_fixed_step(vmin, vmax, step):
    if step <= 0:
        raise ValueError("step must be > 0")
    vals = []
    x = vmin
    while x <= vmax + 1e-12:
        vals.append(round(x, 9))
        x += step
    return vals

def build_ranges():
    X = frange_fixed_step(X_MIN, X_MAX, X_STEP)
    Y = frange_fixed_step(Y_MIN, Y_MAX, Y_STEP)
    Z = frange_fixed_step(Z_MIN, Z_MAX, Z_STEP)
    R = frange_fixed_step(ROLL_MIN,  ROLL_MAX,  ROLL_STEP)
    P = frange_fixed_step(PITCH_MIN, PITCH_MAX, PITCH_STEP)
    Yaw = frange_fixed_step(YAW_MIN,  YAW_MAX,  YAW_STEP)
    return X, Y, Z, R, P, Yaw

# ---------- Hover helpers ---------- #
def add_z(pose, dz):
    """Return pose with z shifted by dz (keep x,y,rx,ry,rz)."""
    return [pose[0], pose[1], pose[2] + dz, pose[3], pose[4], pose[5]]

# ---------------------------- main routine ---------------------------- #

def main():
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)

    # --- EIT serial ---
    try:
        ser = open_eit_serial(EIT_PORT, EIT_BAUD, EIT_TIMEOUT)
        eit_cols, first_raw = sniff_eit_columns(ser, EIT_SNIFF_SECS)
        print(f"EIT serial on {EIT_PORT} @ {EIT_BAUD}. Columns: {eit_cols or '[unknown – raw only]'}")
    except Exception as e:
        print(f"WARNING: Could not open EIT serial on {EIT_PORT}: {e}")
        ser = None
        eit_cols, first_raw = [], ""

    try:
        # 1) Set TCP to the sensor center
        # NOTE: If this triggers a protective stop on your setup, comment it out.
        rtde_c.setTcp(TCP_OFFSET_AT_SENSOR_CENTER)

        # 2) Anchor pose
        start_pose = rtde_r.getActualTCPPose() if USE_CURRENT_POSE_AS_START else EXPLICIT_START_POSE
        start_hover = add_z(start_pose, HOVER_LIFT_Z)
        print("Start anchor pose:", [round(v,6) for v in start_pose])
        print("Start hover pose :", [round(v,6) for v in start_hover])

        # Move to hover before starting the sequence (with floor guard)
        if not safe_moveL(rtde_c, rtde_r, start_hover, SPEED, ACCEL):
            print("Cannot reach start hover (floor guard). Exiting.")
            return

        # 3) Build discretized ranges
        X_OFFSETS, Y_OFFSETS, Z_OFFSETS, ROLL_DEG_LIST, PITCH_DEG_LIST, YAW_DEG_LIST = build_ranges()

        # 4) Build full grid then select subset
        full_grid = [(dx,dy,dz,r,p,y) for dx,dy,dz,r,p,y in
                     product(X_OFFSETS, Y_OFFSETS, Z_OFFSETS, ROLL_DEG_LIST, PITCH_DEG_LIST, YAW_DEG_LIST)]
        total = len(full_grid)
        print(f"Total grid size: {total}")

        if SELECTION_MODE.upper() == "ALL":
            selected = full_grid
        else:
            k = min(RANDOM_SAMPLE_N, total)
            rng = random.Random(RANDOM_SEED)  # independent RNG
            selected = rng.sample(full_grid, k)
            print(f"Randomly selected {k} poses (seed={RANDOM_SEED}).")

        # open csv
        session_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        csv_f, csv_w, _ = open_csv_logger(LOG_CSV_PATH, include_wrench=LOG_WRENCH, eit_cols=eit_cols)

        # Helper: combine RPY (deg) to axis–angle rvec
        def rpy_combo_to_rvec(roll_deg, pitch_deg, yaw_deg, order="XYZ"):
            return rpy_to_rvec(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg), order=order)

        # 5) Iterate selected poses with rise → reposition at hover → lower
        for idx, (dx, dy, dz, rdeg, pdeg, ydeg) in enumerate(selected, start=1):
            drx, dry, drz = rpy_combo_to_rvec(rdeg, pdeg, ydeg, order="XYZ")
            delta_tool   = [dx, dy, dz, drx, dry, drz]        # tool-frame delta
            target       = compose_pose(start_pose, delta_tool)
            target_hover = add_z(target, HOVER_LIFT_Z)        # same x,y,rx,ry,rz, but z + 100mm

            print(f"[{idx}/{len(selected)}] target      : {[round(v,6) for v in target]}")
            print(f"                  target_hover: {[round(v,6) for v in target_hover]}")

            # 1) Reposition at hover height
            if not safe_moveL(rtde_c, rtde_r, target_hover, SPEED, ACCEL):
                print("moveL() to target_hover rejected/failed — stopping.")
                break

            # 2) Lower straight down to target Z
            if not safe_moveL(rtde_c, rtde_r, target, SPEED, ACCEL):
                print("moveL() to target rejected/failed — stopping.")
                break

            # Dwell & read EIT
            if ser:
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass

            t_end = time.time() + DWELL
            last_raw = ""
            while time.time() < t_end:
                if ser:
                    last_raw = read_eit_line(ser, timeout_s=EIT_TIMEOUT) or last_raw
                time.sleep(0.01)

            # Log row
            sensor_target_pose = target  # TCP is set to sensor center, so same
            cmd_tcp_pose       = target
            actual_tcp_pose    = rtde_r.getActualTCPPose()
            wrench = rtde_r.getActualTCPForce() if LOG_WRENCH else [None]*6

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "session_id": session_id,
                "index": idx,
                "dx_m": float(dx), "dy_m": float(dy), "dz_m": float(dz),
                "roll_deg": float(rdeg), "pitch_deg": float(pdeg), "yaw_deg": float(ydeg),
                "sensor_x": sensor_target_pose[0], "sensor_y": sensor_target_pose[1], "sensor_z": sensor_target_pose[2],
                "sensor_rx": sensor_target_pose[3], "sensor_ry": sensor_target_pose[4], "sensor_rz": sensor_target_pose[5],
                "cmd_tcp_x": cmd_tcp_pose[0], "cmd_tcp_y": cmd_tcp_pose[1], "cmd_tcp_z": cmd_tcp_pose[2],
                "cmd_tcp_rx": cmd_tcp_pose[3], "cmd_tcp_ry": cmd_tcp_pose[4], "cmd_tcp_rz": cmd_tcp_pose[5],
                "act_tcp_x": actual_tcp_pose[0], "act_tcp_y": actual_tcp_pose[1], "act_tcp_z": actual_tcp_pose[2],
                "act_tcp_rx": actual_tcp_pose[3], "act_tcp_ry": actual_tcp_pose[4], "act_tcp_rz": actual_tcp_pose[5],
                "eit_raw": last_raw
            }

            if eit_cols and last_raw:
                parts = [p.strip() for p in last_raw.split(",")]
                for i, name in enumerate(eit_cols):
                    row[name] = parts[i] if i < len(parts) else ""

            if LOG_WRENCH:
                Fx,Fy,Fz,Tx,Ty,Tz = wrench
                row.update({"Fx":Fx,"Fy":Fy,"Fz":Fz,"Tx":Tx,"Ty":Ty,"Tz":Tz})

            csv_w.writerow(row)
            csv_f.flush()

            # 3) Rise back up to hover ready for the next pose
            if not safe_moveL(rtde_c, rtde_r, target_hover, SPEED, ACCEL):
                print("moveL() back to hover rejected/failed — stopping.")
                break

        # Return to anchor (hover, then lower)
        print("Returning to anchor pose.")
        if not safe_moveL(rtde_c, rtde_r, add_z(start_pose, HOVER_LIFT_Z), SPEED, ACCEL):
            print("Could not move to start hover (floor guard).")
        safe_moveL(rtde_c, rtde_r, start_pose, SPEED, ACCEL)

    finally:
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        try:
            csv_f.close()
        except Exception:
            pass
        try:
            if ser:
                ser.close()
        except Exception:
            pass
        print("Done.")

if __name__ == "__main__":
    main()
