#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 EIT localizer: set TCP to sensor center and scan over x/y/z + roll/pitch/yaw.
Requires: ur_rtde (pip install ur-rtde)
"""

import math
import time
from itertools import product

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# ---------------------------- USER SETTINGS ---------------------------- #

ROBOT_IP = "169.254.150.50"          # <-- your UR5 IP

# TCP pose offset that places the TCP at the *middle of the sensor* w.r.t. the flange.
# Pose format is UR axis-angle: [x, y, z, rx, ry, rz]  (meters, radians)
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion parameters
SPEED = 0.10         # m/s
ACCEL = 0.10         # m/s^2
DWELL = 1.30         # s to settle & read EIT per pose

# Use the current TCP pose as the "start pose" anchor:
USE_CURRENT_POSE_AS_START = True
# Or, provide an explicit base-frame anchor pose below (meters, axis-angle):
EXPLICIT_START_POSE = [0.40, -0.20, 0.20, 3.1415, 0.0, 0.0]   # example; ignored if flag above is True

# Offsets in meters (relative to the tool/TCP frame)
X_OFFSETS = [-0.010, 0.000, +0.010]    # ±10 mm
Y_OFFSETS = [-0.010, 0.000, +0.010]
Z_OFFSETS = [0.000, +0.005, +0.010]    # z varies in ~5 mm steps

# Angle increments in DEGREES (roll about X, pitch about Y, yaw about Z)
ROLL_DEG  = [-10, 0, +10]
PITCH_DEG = [-10, 0, +10]
YAW_DEG   = [-20, 0, +20]

# ---------------------------------------------------------------------- #

# ---------- math helpers: rotations & pose composition (axis-angle) ---------- #

def rvec_to_rotmat(rvec):
    rx, ry, rz = rvec
    theta = math.sqrt(rx*rx + ry*ry + rz*rz)
    if theta < 1e-12:
        return [[1.0,0.0,0.0],
                [0.0,1.0,0.0],
                [0.0,0.0,1.0]]
    ux, uy, uz = rx/theta, ry/theta, rz/theta
    c = math.cos(theta)
    s = math.sin(theta)
    C = 1.0 - c
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

# ---------------------------- main routine ---------------------------- #

def main():
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)

    try:
        # 1) Set TCP to the sensor's center (relative to flange)
        rtde_c.setTcp(TCP_OFFSET_AT_SENSOR_CENTER)

        # 2) Establish the start/anchor pose in BASE frame
        if USE_CURRENT_POSE_AS_START:
            start_pose = rtde_r.getActualTCPPose()  # [x,y,z, rx,ry,rz]
        else:
            start_pose = EXPLICIT_START_POSE

        print("Start anchor pose (base frame):", [round(v,6) for v in start_pose])

        # Helper: combine RPY (deg) to a single axis–angle rvec
        def rpy_combo_to_rvec(roll_deg, pitch_deg, yaw_deg, order="XYZ"):
            return rpy_to_rvec(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg), order=order)

        # 3) Iterate the grid in tool frame
        idx = 0
        total = len(X_OFFSETS)*len(Y_OFFSETS)*len(Z_OFFSETS)*len(ROLL_DEG)*len(PITCH_DEG)*len(YAW_DEG)
        print(f"Total target poses: {total}")

        for dx, dy, dz, rdeg, pdeg, ydeg in product(X_OFFSETS, Y_OFFSETS, Z_OFFSETS,
                                                    ROLL_DEG, PITCH_DEG, YAW_DEG):
            drx, dry, drz = rpy_combo_to_rvec(rdeg, pdeg, ydeg, order="XYZ")
            delta_tool = [dx, dy, dz, drx, dry, drz]  # tool-frame delta
            target = compose_pose(start_pose, delta_tool)

            idx += 1
            print(f"[{idx}/{total}] -> target (m, rvec): {[round(v,6) for v in target]}")

            rtde_c.moveL(target, SPEED, ACCEL)
            time.sleep(DWELL)

            # ---- place your EIT reading + logging here ----
            # eit_value = read_eit()
            # log_sample(target, eit_value)
            # -----------------------------------------------

        # Optionally return to the start anchor pose
        print("Returning to anchor pose.")
        rtde_c.moveL(start_pose, SPEED, ACCEL)

    finally:
        rtde_c.stopScript()
        print("Done.")

if __name__ == "__main__":
    main()
