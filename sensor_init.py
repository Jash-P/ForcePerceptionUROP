#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 EIT localizer: range-based discretization + random subset selection
- Rise → hover → lower at each pose
- Way B: software Z-floor guard (applies to the controlled tool-point)
- NEW: Z targets now refer to a specific tool-point (x=0 on line TCP→head)

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

ROBOT_IP = "169.254.150.50"

# Active TCP set to the *sensor center* (meters, axis-angle radians)
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion & dwell
SPEED = 0.10          # m/s
ACCEL = 0.10          # m/s^2
DWELL = 1.30          # s to settle & read EIT per pose

# Hover / retract height (applied to the controlled tool-point Z)
HOVER_LIFT_Z = 0.100  # 100 mm

# ---------- Controlled tool-point (in TCP/tool frame) ----------
# Choose the point along the TCP→head line **with x=0** that should hit the Z setpoint.
# Example: 50 mm below TCP along tool -Z, centered in x and y.
TOUCH_POINT_TOOL = [0.0, 0.0, -0.050]  # [x=0, y_p, z_p]  <-- MEASURE & SET FOR YOUR TOOL

# ---------- Way B: software floor (applies to the controlled tool-point) ----------
MIN_BASE_Z = -0.390          # meters (base frame)
CLAMP_BELOW_FLOOR = False    # True: clamp the point-Z to floor instead of aborting

# Logging
LOG_CSV_PATH = "eit_localisation_log.csv"
LOG_WRENCH   = True

# EIT serial
EIT_PORT = "COM5"
EIT_BAUD = 115200
EIT_TIMEOUT = 0.2
EIT_SNIFF_SECS = 2.0

# Anchor pose
USE_CURRENT_POSE_AS_START = True
EXPLICIT_START_POSE = [0.40, -0.20, 0.20, 3.1415, 0.0, 0.0]

# -------- RANGE MODE (min/max with fixed increments) --------
# Distances in meters; angles in degrees. Increments are constant.
X_MIN, X_MAX, X_STEP = -0.030, +0.030, 0.005
Y_MIN, Y_MAX, Y_STEP = -0.030, +0.030, 0.005
Z_MIN, Z_MAX, Z_STEP =  0.000, +0.005, 0.001     # <-- now interpreted as the *tool-point* Z offsets
ROLL_MIN,  ROLL_MAX,  ROLL_STEP  = -10, +10, 1
PITCH_MIN, PITCH_MAX, PITCH_STEP = -10, +10, 1
YAW_MIN,   YAW_MAX,   YAW_STEP   = -20, +20, 1

# Selection: do ALL poses or RANDOM sample of N
SELECTION_MODE   = "RANDOM"   # "ALL" or "RANDOM"
RANDOM_SAMPLE_N  = 5
RANDOM_SEED      = 42

# ---------------------------------------------------------------------- #
# ------------------------ math / pose utilities ----------------------- #

def rvec_to_rotmat(rvec):
    rx, ry, rz = rvec
    th = math.sqrt(rx*rx + ry*ry + rz*rz)
    if th < 1e-12:
        return [[1,0,0],[0,1,0],[0,0,1]]
    ux, uy, uz = rx/th, ry/th, rz/th
    c = math.cos(th); s = math.sin(th); C = 1.0 - c
    return [
        [c+ux*ux*C,      ux*uy*C - uz*s, ux*uz*C + uy*s],
        [uy*ux*C + uz*s, c+uy*uy*C,      uy*uz*C - ux*s],
        [uz*ux*C - uy*s, uz*uy*C + ux*s, c+uz*uz*C     ]
    ]

def rotmat_to_rvec(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    ct = max(min((tr - 1.0)/2.0, 1.0), -1.0)
    th = math.acos(ct)
    if th < 1e-12:
        return [0.0, 0.0, 0.0]
    denom = 2.0*math.sin(th)
    rx = (R[2][1]-R[1][2]) / denom
    ry = (R[0][2]-R[2][0]) / denom
    rz = (R[1][0]-R[0][1]) / denom
    return [rx*th, ry*th, rz*th]

def rpy_to_rotmat(roll, pitch, yaw, order="XYZ"):
    cx, sx = math.cos(roll),  math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]]
    Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rz = [[cz,-sz,0],[sz,cz,0],[0,0,1]]

    def mm(A,B):
        return [[A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0],
                 A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1],
                 A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2]],
                [A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0],
                 A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1],
                 A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2]],
                [A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0],
                 A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1],
                 A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2]]]
    return mm(mm(Rx,Ry),Rz) if order.upper()=="XYZ" else mm(mm(Rz,Ry),Rx)

def rpy_to_rvec(roll, pitch, yaw, order="XYZ"):
    return rotmat_to_rvec(rpy_to_rotmat(roll, pitch, yaw, order=order))

def pose_to_tf(p):
    x,y,z, rx,ry,rz = p
    R = rvec_to_rotmat([rx,ry,rz])
    return [[R[0][0],R[0][1],R[0][2],x],
            [R[1][0],R[1][1],R[1][2],y],
            [R[2][0],R[2][1],R[2][2],z],
            [0,0,0,1]]

def tf_to_pose(T):
    x,y,z = T[0][3], T[1][3], T[2][3]
    R = [[T[0][0],T[0][1],T[0][2]],
         [T[1][0],T[1][1],T[1][2]],
         [T[2][0],T[2][1],T[2][2]]]
    rx,ry,rz = rotmat_to_rvec(R)
    return [x,y,z, rx,ry,rz]

def tf_mul(A,B):
    return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def compose_pose(base_pose, delta_tool):
    return tf_to_pose(tf_mul(pose_to_tf(base_pose), pose_to_tf(delta_tool)))

def deg2rad(d): return d * math.pi / 180.0

# ---- tool-point control: convert desired point-Z -> required TCP-Z ---- #

def world_point_z_for_tcp_pose(tcp_pose, point_tool):
    """World Z of a tool-fixed point given a TCP pose."""
    rx,ry,rz = tcp_pose[3], tcp_pose[4], tcp_pose[5]
    R = rvec_to_rotmat([rx,ry,rz])
    px,py,pz = point_tool
    # world offset z from TCP to point
    dz = R[2][0]*px + R[2][1]*py + R[2][2]*pz
    return tcp_pose[2] + dz

def tcp_pose_for_desired_point_z(pose_like_tcp, point_tool):
    """Given (x,y,desired_point_z, rx,ry,rz), compute TCP z so that the tool-point hits desired_point_z."""
    x,y,desired_point_z, rx,ry,rz = pose_like_tcp
    R = rvec_to_rotmat([rx,ry,rz])
    px,py,pz = point_tool
    dz = R[2][0]*px + R[2][1]*py + R[2][2]*pz  # world z offset of point from TCP
    tcp_z = desired_point_z - dz
    return [x,y,tcp_z, rx,ry,rz]

def add_z(pose, dz):
    """Add dz to the Z (used for point-Z poses & also TCP poses where appropriate)."""
    return [pose[0], pose[1], pose[2] + dz, pose[3], pose[4], pose[5]]

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
        s = line.decode("utf-8", errors="ignore").strip()
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
        "point_set_z",  # desired Z of controlled tool-point
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
    vals, x = [], vmin
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

# ---------- point-floor guard (applies to desired point-Z) ---------- #
def enforce_point_floor_z(desired_point_z):
    if desired_point_z >= MIN_BASE_Z - 1e-9:
        return desired_point_z
    if CLAMP_BELOW_FLOOR:
        print(f"[POINT-FLOOR] Clamped point-Z from {desired_point_z:.3f} to {MIN_BASE_Z:.3f}")
        return MIN_BASE_Z
    raise ValueError(f"Desired point-Z {desired_point_z:.3f} < floor {MIN_BASE_Z:.3f}")

def safe_move_pointZ(rtde_c, pose_pointZ, speed, accel):
    """pose_pointZ = [x,y, desired_point_z, rx,ry,rz]; converts to TCP pose and moveL."""
    # floor check on the *point*
    z_ok = enforce_point_floor_z(pose_pointZ[2])
    pose_pointZ = [pose_pointZ[0], pose_pointZ[1], z_ok, pose_pointZ[3], pose_pointZ[4], pose_pointZ[5]]
    # convert to TCP pose so that the tool-point hits the desired Z
    tcp_pose = tcp_pose_for_desired_point_z(pose_pointZ, TOUCH_POINT_TOOL)
    return bool(rtde_c.moveL(tcp_pose, speed, accel)), tcp_pose

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
        rtde_c.setTcp(TCP_OFFSET_AT_SENSOR_CENTER)

        # 2) Anchor pose (TCP pose)
        start_tcp = rtde_r.getActualTCPPose() if USE_CURRENT_POSE_AS_START else EXPLICIT_START_POSE
        print("Start TCP pose:", [round(v,6) for v in start_tcp])

        # Compute current controlled point's Z and rise to hover (point-Z)
        current_point_z = world_point_z_for_tcp_pose(start_tcp, TOUCH_POINT_TOOL)
        start_point_hover = [start_tcp[0], start_tcp[1], current_point_z + HOVER_LIFT_Z,
                             start_tcp[3], start_tcp[4], start_tcp[5]]
        ok, _ = safe_move_pointZ(rtde_c, start_point_hover, SPEED, ACCEL)
        if not ok:
            print("Cannot reach start hover (point-floor). Exiting.")
            return

        # 3) Build discretized ranges
        Xs, Ys, Zs, Rls, Pcs, Yws = build_ranges()

        # 4) Build full grid then select subset (dx,dy,dz are tool-frame deltas; dz maps to point-Z)
        full_grid = [(dx,dy,dz,r,p,y) for dx,dy,dz,r,p,y in product(Xs, Ys, Zs, Rls, Pcs, Yws)]
        total = len(full_grid)
        print(f"Total grid size: {total}")

        if SELECTION_MODE.upper() == "ALL":
            selected = full_grid
        else:
            k = min(RANDOM_SAMPLE_N, total)
            rng = random.Random(RANDOM_SEED)
            selected = rng.sample(full_grid, k)
            print(f"Randomly selected {k} poses (seed={RANDOM_SEED}).")

        # open csv
        session_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        csv_f, csv_w, _ = open_csv_logger(LOG_CSV_PATH, include_wrench=LOG_WRENCH, eit_cols=eit_cols)

        # Helper: combine RPY (deg) to axis–angle rvec
        def rpy_combo_to_rvec(roll_deg, pitch_deg, yaw_deg, order="XYZ"):
            return rpy_to_rvec(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg), order=order)

        # 5) Iterate poses (rise→hover, orient/XY at hover, lower until *point* hits desired Z)
        for idx, (dx, dy, dz, rdeg, pdeg, ydeg) in enumerate(selected, start=1):
            # Build a base pose by applying tool-frame deltas to the *start TCP pose* (xy/orientation),
            # but remember: the z we supply below is for the controlled tool-point.
            drx, dry, drz = rpy_combo_to_rvec(rdeg, pdeg, ydeg, order="XYZ")
            delta_tool = [dx, dy, 0.0, drx, dry, drz]               # z handled as point-Z separately
            base_tcp   = compose_pose(start_tcp, delta_tool)        # x,y,orientation for this sample

            # Desired point-Z at target and hover:
            # Take the *start* controlled point-Z as reference, then add desired dz.
            start_point_z = world_point_z_for_tcp_pose(start_tcp, TOUCH_POINT_TOOL)
            target_pointZ = [base_tcp[0], base_tcp[1], start_point_z + dz,
                             base_tcp[3], base_tcp[4], base_tcp[5]]
            hover_pointZ  = add_z(target_pointZ, HOVER_LIFT_Z)

            print(f"[{idx}/{len(selected)}] pointZ target z={target_pointZ[2]:.3f}  rpy=({rdeg},{pdeg},{ydeg})")

            # 1) Reposition at hover (XY + orientation at safe height)
            ok, tcp_hover = safe_move_pointZ(rtde_c, hover_pointZ, SPEED, ACCEL)
            if not ok:
                print("moveL() to hover (point-Z) rejected/failed — stopping.")
                break

            # 2) Lower so the controlled tool-point hits its desired Z
            ok, tcp_tgt = safe_move_pointZ(rtde_c, target_pointZ, SPEED, ACCEL)
            if not ok:
                print("moveL() to target (point-Z) rejected/failed — stopping.")
                break

            # ---- Dwell & EIT read ----
            if ser:
                try: ser.reset_input_buffer()
                except Exception: pass
            t_end = time.time() + DWELL
            last_raw = ""
            while time.time() < t_end:
                if ser:
                    last_raw = read_eit_line(ser, timeout_s=EIT_TIMEOUT) or last_raw
                time.sleep(0.01)

            # ---- Log ----
            actual_tcp = rtde_r.getActualTCPPose()
            wrench = rtde_r.getActualTCPForce() if LOG_WRENCH else [None]*6

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "session_id": session_id,
                "index": idx,
                "dx_m": float(dx), "dy_m": float(dy), "dz_m": float(dz),
                "roll_deg": float(rdeg), "pitch_deg": float(pdeg), "yaw_deg": float(ydeg),
                "point_set_z": target_pointZ[2],
                "cmd_tcp_x": tcp_tgt[0], "cmd_tcp_y": tcp_tgt[1], "cmd_tcp_z": tcp_tgt[2],
                "cmd_tcp_rx": tcp_tgt[3], "cmd_tcp_ry": tcp_tgt[4], "cmd_tcp_rz": tcp_tgt[5],
                "act_tcp_x": actual_tcp[0], "act_tcp_y": actual_tcp[1], "act_tcp_z": actual_tcp[2],
                "act_tcp_rx": actual_tcp[3], "act_tcp_ry": actual_tcp[4], "act_tcp_rz": actual_tcp[5],
                "eit_raw": last_raw
            }
            if eit_cols and last_raw:
                parts = [p.strip() for p in last_raw.split(",")]
                for i, name in enumerate(eit_cols):
                    row[name] = parts[i] if i < len(parts) else ""

            if LOG_WRENCH:
                Fx,Fy,Fz,Tx,Ty,Tz = wrench
                row.update({"Fx":Fx,"Fy":Fy,"Fz":Fz,"Tx":Tx,"Ty":Ty,"Tz":Tz})

            csv_w.writerow(row); csv_f.flush()

            # 3) Rise back to hover (point-Z)
            ok, _ = safe_move_pointZ(rtde_c, hover_pointZ, SPEED, ACCEL)
            if not ok:
                print("moveL() back to hover (point-Z) rejected/failed — stopping.")
                break

        # Return near start (rise then back toward start TCP at current height)
        print("Returning to start hover.")
        # keep current orientation/XY, lift the *point* by HOVER_LIFT_Z and then go near start
        current_tcp = rtde_r.getActualTCPPose()
        current_point_z = world_point_z_for_tcp_pose(current_tcp, TOUCH_POINT_TOOL)
        back_hover = [current_tcp[0], current_tcp[1], current_point_z + HOVER_LIFT_Z,
                      current_tcp[3], current_tcp[4], current_tcp[5]]
        safe_move_pointZ(rtde_c, back_hover, SPEED, ACCEL)

    finally:
        try: rtde_c.stopScript()
        except Exception: pass
        try: csv_f.close()
        except Exception: pass
        try:
            if ser: ser.close()
        except Exception: pass
        print("Done.")

if __name__ == "__main__":
    main()
