#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 EIT localizer — SAFE / GUARDED
- No setTcp (avoids safety-plane trips)
- Internal TCP->sensor transform
- Pre-approach -> slow approach -> retract for each sample
- Segmented moveL with force/torque guard

Requires: ur-rtde (pip install ur-rtde)
"""

import math
import time
from itertools import product

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


# ---------------------------- USER SETTINGS ---------------------------- #

ROBOT_IP = "169.254.150.50"

# Transform from CURRENT ACTIVE TCP -> SENSOR CENTER  [m, axis-angle rad]
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion parameters (conservative)
SPEED_TRAVEL = 0.05     # m/s  (between waypoints / pre-approach)
ACCEL_TRAVEL = 0.15     # m/s^2
SPEED_APPROACH = 0.012  # m/s  (toward the handle/surface)
ACCEL_APPROACH = 0.10   # m/s^2
DWELL_AT_TARGET = 0.30  # s for EIT read

# Guard thresholds (tune to your AR10 + sensor)
MAX_FORCE_N = 18.0      # stop if |F| exceeds this
MAX_TORQUE_NM = 1.8     # stop if |T| exceeds this
CHECK_PERIOD = 0.02     # s between guard checks
SEGMENT_LEN = 0.002     # m per cartesian segment ~2 mm

# Safety floor in BASE Z (to avoid table strikes) — set this!
MIN_BASE_Z = 0.05       # m (example). If unknown, increase.

# Approach geometry
APPROACH_DIST = 0.020   # m: pre-approach stand-off distance along sensor +Z/-Z
APPROACH_DIR = -1       # -1: approach along sensor -Z (toward tool), +1: along +Z

# Choose anchor
USE_CURRENT_POSE_AS_START = True
EXPLICIT_START_POSE = [0.45, -0.20, 0.22, 3.1415, 0.0, 0.0]

# Scan deltas (sensor-frame)
X_OFFSETS = [-0.010, 0.000, +0.010]
Y_OFFSETS = [-0.010, 0.000, +0.010]
Z_OFFSETS = [-0.005, 0.000, +0.005]   # ~5 mm steps

ROLL_DEG  = [-8, 0, +8]
PITCH_DEG = [-8, 0, +8]
YAW_DEG   = [-15, 0, +15]

# Dry-run to verify poses without moving
DRY_RUN = False

# ---------------------------------------------------------------------- #
# -------------------------- math / transforms ------------------------- #

def rvec_to_rotmat(rvec):
    rx, ry, rz = rvec
    th = math.sqrt(rx*rx + ry*ry + rz*rz)
    if th < 1e-12:
        return [[1,0,0],[0,1,0],[0,0,1]]
    ux, uy, uz = rx/th, ry/th, rz/th
    c, s, C = math.cos(th), math.sin(th), 1 - math.cos(th)
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
    rx = (R[2][1] - R[1][2]) / denom
    ry = (R[0][2] - R[2][0]) / denom
    rz = (R[1][0] - R[0][1]) / denom
    return [rx*th, ry*th, rz*th]

def rpy_to_rvec(roll, pitch, yaw, order="XYZ"):
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
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
    R = mm(mm(Rx,Ry),Rz) if order.upper()=="XYZ" else mm(mm(Rz,Ry),Rx)
    return rotmat_to_rvec(R)

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

def tf_inv(T):
    R = [[T[0][0],T[0][1],T[0][2]],
         [T[1][0],T[1][1],T[1][2]],
         [T[2][0],T[2][1],T[2][2]]]
    Rt = [[R[0][0],R[1][0],R[2][0]],
          [R[0][1],R[1][1],R[2][1]],
          [R[0][2],R[1][2],R[2][2]]]
    p = [T[0][3], T[1][3], T[2][3]]
    pinv = [- (Rt[0][0]*p[0] + Rt[0][1]*p[1] + Rt[0][2]*p[2]),
            - (Rt[1][0]*p[0] + Rt[1][1]*p[1] + Rt[1][2]*p[2]),
            - (Rt[2][0]*p[0] + Rt[2][1]*p[1] + Rt[2][2]*p[2])]
    return [[Rt[0][0],Rt[0][1],Rt[0][2],pinv[0]],
            [Rt[1][0],Rt[1][1],Rt[1][2],pinv[1]],
            [Rt[2][0],Rt[2][1],Rt[2][2],pinv[2]],
            [0,0,0,1]]

def deg2rad(d): return d * math.pi / 180.0

# ------------------------ safety / guarded helpers -------------------- #

def force_too_high(rtde_r):
    """Return True if measured TCP wrench exceeds limits."""
    wrench = rtde_r.getActualTCPForce()  # [Fx,Fy,Fz,Tx,Ty,Tz]
    Fx,Fy,Fz,Tx,Ty,Tz = wrench
    fmag = math.sqrt(Fx*Fx + Fy*Fy + Fz*Fz)
    tmag = math.sqrt(Tx*Tx + Ty*Ty + Tz*Tz)
    return (fmag > MAX_FORCE_N) or (tmag > MAX_TORQUE_NM)

def z_below_floor(pose):
    return pose[2] < MIN_BASE_Z

def linspace_pose(p0, p1, max_step):
    """Yield intermediate poses from p0->p1 with translational step <= max_step.
    Rotation vector is linearly interpolated (OK for small steps)."""
    dx = p1[0]-p0[0]; dy = p1[1]-p0[1]; dz = p1[2]-p0[2]
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    n = max(1, int(math.ceil(dist / max_step)))
    for i in range(1, n+1):
        a = i / float(n)
        yield [
            p0[0] + a*dx,
            p0[1] + a*dy,
            p0[2] + a*dz,
            p0[3] + a*(p1[3]-p0[3]),
            p0[4] + a*(p1[4]-p0[4]),
            p0[5] + a*(p1[5]-p0[5]),
        ]

def guarded_moveL(rtde_c, rtde_r, target_pose, speed, accel, backoff_vec=None):
    """Segmented moveL with force guard; optional backoff on abort."""
    # quick z-floor check (target)
    if z_below_floor(target_pose):
        print("ABORT: target below MIN_BASE_Z")
        return False

    current = rtde_r.getActualTCPPose()
    for wp in linspace_pose(current, target_pose, SEGMENT_LEN):
        if z_below_floor(wp):
            print("ABORT: path would go below MIN_BASE_Z")
            return False
        ok = rtde_c.moveL(wp, speed, accel)
        if not ok:
            print("ABORT: moveL returned False")
            return False
        time.sleep(CHECK_PERIOD)
        if force_too_high(rtde_r):
            print("ABORT: force/torque limit exceeded; backing off.")
            if backoff_vec is not None:
                # back off a little along provided vector (in TCP frame)
                bx,by,bz = backoff_vec
                backoff_pose = [wp[0]+bx, wp[1]+by, wp[2]+bz, wp[3], wp[4], wp[5]]
                rtde_c.moveL(backoff_pose, 0.02, 0.1)
            return False
    return True

# ---------------------------------------------------------------------- #

def main():
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)

    try:
        # DO NOT setTcp — we apply the offset internally.
        # rtde_c.setTcp(TCP_OFFSET_AT_SENSOR_CENTER)

        # Anchor (base->tcp)
        tcp_anchor = rtde_r.getActualTCPPose() if USE_CURRENT_POSE_AS_START else EXPLICIT_START_POSE
        print("TCP anchor:", [round(v,6) for v in tcp_anchor])

        T_tcp_anchor    = pose_to_tf(tcp_anchor)
        T_tcp2sensor    = pose_to_tf(TCP_OFFSET_AT_SENSOR_CENTER)
        T_sensor2tcp    = tf_inv(T_tcp2sensor)

        # Sensor anchor
        T_sensor_anchor = tf_mul(T_tcp_anchor, T_tcp2sensor)
        sensor_anchor_pose = tf_to_pose(T_sensor_anchor)
        if z_below_floor(sensor_anchor_pose):
            print("ABORT: sensor anchor below MIN_BASE_Z")
            return

        # Helper: RPYdeg -> rvec
        def rpy_combo_to_rvec(r_deg, p_deg, y_deg, order="XYZ"):
            return rpy_to_rvec(deg2rad(r_deg), deg2rad(p_deg), deg2rad(y_deg), order=order)

        # Pre-approach transform (in SENSOR frame)
        pre_app_offset = [0.0, 0.0, APPROACH_DIR * APPROACH_DIST, 0.0, 0.0, 0.0]

        idx = 0
        total = len(X_OFFSETS)*len(Y_OFFSETS)*len(Z_OFFSETS)*len(ROLL_DEG)*len(PITCH_DEG)*len(YAW_DEG)
        print(f"Total target samples: {total}")

        for dx, dy, dz, rdeg, pdeg, ydeg in product(
            X_OFFSETS, Y_OFFSETS, Z_OFFSETS, ROLL_DEG, PITCH_DEG, YAW_DEG
        ):
            # Sensor-frame delta
            drx, dry, drz = rpy_combo_to_rvec(rdeg, pdeg, ydeg, order="XYZ")
            T_delta_sensor = pose_to_tf([dx, dy, dz, drx, dry, drz])

            # Build pre-approach and target in SENSOR frame
            T_sensor_target   = tf_mul(T_sensor_anchor, T_delta_sensor)
            T_sensor_pre_approach = tf_mul(T_sensor_target, pose_to_tf(pre_app_offset))

            # Convert both to TCP frame
            T_tcp_pre = tf_mul(T_sensor_pre_approach, T_sensor2tcp)
            T_tcp_tgt = tf_mul(T_sensor_target,        T_sensor2tcp)
            tcp_pre   = tf_to_pose(T_tcp_pre)
            tcp_tgt   = tf_to_pose(T_tcp_tgt)

            idx += 1
            print(f"[{idx}/{total}] pre→tgt z={tcp_tgt[2]:.3f}  rpy=({rdeg},{pdeg},{ydeg})")

            if DRY_RUN:
                # Just print checks
                if z_below_floor(tcp_pre) or z_below_floor(tcp_tgt):
                    print("  (SKIP) below Z floor.")
                continue

            # 1) Travel to pre-approach (faster but still cautious)
            if not guarded_moveL(rtde_c, rtde_r, tcp_pre, SPEED_TRAVEL, ACCEL_TRAVEL):
                print("  Abort during travel to pre-approach.")
                break

            # 2) Slow approach to target; backoff along sensor +Z (opposite of approach direction)
            backoff_in_tcp = None
            # compute sensor +Z in TCP coords for backoff (~5 mm)
            # Take a small vector (0,0, +5mm in SENSOR), transform to TCP at current orientation
            T_small_back = pose_to_tf([0,0,0.005, 0,0,0])
            T_tcp_backvec = tf_mul(T_sensor_target, tf_mul(T_small_back, T_sensor2tcp))
            tcp_back = tf_to_pose(T_tcp_backvec)
            backoff_in_tcp = [tcp_back[0]-tcp_tgt[0], tcp_back[1]-tcp_tgt[1], tcp_back[2]-tcp_tgt[2]]

            if not guarded_moveL(rtde_c, rtde_r, tcp_tgt, SPEED_APPROACH, ACCEL_APPROACH, backoff_vec=backoff_in_tcp):
                print("  Abort during approach — backing out and skipping this sample.")
                # retract to pre-approach for safety
                guarded_moveL(rtde_c, rtde_r, tcp_pre, 0.02, 0.1)
                continue

            # 3) Dwell & read EIT
            time.sleep(DWELL_AT_TARGET)
            # ---- your EIT read + logging here ----
            # eit = read_eit()
            # log_sample(tf_to_pose(T_sensor_target), eit)

            # 4) Retract to pre-approach
            if not guarded_moveL(rtde_c, rtde_r, tcp_pre, SPEED_APPROACH, ACCEL_APPROACH):
                print("  Warning: retract encountered a guard event.")

        # Return to anchor (safe travel)
        print("Returning to TCP anchor.")
        guarded_moveL(rtde_c, rtde_r, tcp_anchor, SPEED_TRAVEL, ACCEL_TRAVEL)

    finally:
        rtde_c.stopScript()
        print("Done.")

if __name__ == "__main__":
    main()
