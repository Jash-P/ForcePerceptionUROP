#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5 EIT localizer with first-contact control along tool X-line
- Handle modeled as poly-cubic Bezier in the YZ plane (x = constant): z = z(y)
- Rise → hover → ACTIVE LOWER: step down until EIT change, then +2 mm, OR continue up to an extra limit if no change
- Software floor guard on: (1) chosen contact point, (2) tool x-line endpoints/TCP, (3) all robot links via FK+IK
- MEMORY-SAFE grid iteration: generator for ALL; fast random sampling for RANDOM

Requires: ur_rtde, pyserial
"""

import math
import time
import csv, os, random, statistics
from math import prod
from datetime import datetime
from itertools import product

import sys
import platform
import re
from pathlib import Path

import serial  # pip install pyserial

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# ---------------------------- USER SETTINGS ---------------------------- #

ROBOT_IP = "169.254.150.50"

# Active TCP set to the *sensor center* (meters, axis-angle radians)
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion & dwell
SPEED = 0.10          # m/s for XY/orientation + hover moves
ACCEL = 0.05          # m/s^2
DWELL = 1.30          # s to settle & read EIT at final depth

# Hover / retract height (applied to the controlled contact-point Z)
HOVER_LIFT_Z = 0.025       # 25 mm (base value)
HOVER_LIFT_Z_MIN = 0.050   # ensure a minimally higher hover when adapting

# ---------- Active descent parameters (point-Z stepping) ----------
DESCENT_SPEED     = 0.050   # m/s while stepping down
DESCENT_ACCEL     = 0.100   # m/s^2
STEP_Z            = 0.001  # m per step (0.5 mm)
CONTACT_EXTRA_Z   = 0.0020  # m after detection (2 mm)
STEP_SETTLE       = 0.01    # s pause after each step before sampling

# If no EIT change at the planned target depth, keep lowering by up to this extra:
NO_CONTACT_EXTRA_Z = 0.020  # 20mm beyond planned target (still obeys floor guard)

# Detection settings (baseline vs. live)
BASELINE_SAMPLES  = 20      # EIT lines at hover to build baseline
DETECT_METHOD     = "sigma" # "sigma" (z-score) or "abs"
K_SIGMA           = 6.0     # trigger when mean |zscore| ≥ K_SIGMA
ABS_THRESHOLD     = 0.05    # if DETECT_METHOD=="abs": trigger when mean |x-mu| ≥ ABS_THRESHOLD
MIN_CONSEC_TRIG   = 2       # require N consecutive triggers to confirm contact

# Runtime safety (live watchdog)
RUNTIME_FLOOR_EPS   = 0.003   # extra 3 mm buffer for live checks
SEG_MAX_DPOS        = 0.005   # max segment length (m) per guarded step
SEG_MIN_STEPS       = 6       # at least this many segments per long move
WATCHDOG_POLL_DT    = 0.008   # ~125 Hz polling during motion
WATCHDOG_TIMEOUT_S  = 6.0     # per-segment timeout (s)

# ---------- Tool line through x=0 (tool frame) ----------
TOUCH_POINT_YZ    = (0.0, -0.050)  # (y0, z0) at x=0; e.g., 50 mm below TCP along tool -Z
TOOL_LINE_X_MIN   = -0.020         # meters along tool ±X from x=0
TOOL_LINE_X_MAX   = +0.020
TOOL_LINE_SAMPLES = 21             # number of x samples for first-contact search

# ---------- Software floor ----------
MIN_BASE_Z = -0.390          # meters (Base/world frame)
FLOOR_MARGIN = 0.010         # keep everything at least 10 mm above the floor
CLAMP_BELOW_FLOOR = True     # True: lift to keep safe; False: abort moves that would violate
GUARD_XLINE  = True          # also guard the x-line endpoints besides the chosen contact point
GUARD_ARM_LINKS = True       # guard all UR5 links via FK+IK

# ---- Robust EIT/force detection knobs ----
EIT_MIN_COLS           = 8       # ignore frames that have fewer columns than this
EIT_PER_STEP_SAMPLES   = 4       # read N lines after each Z step, average the score
EIT_MEDIAN_FILTER_WIN  = 3       # median filter window (odd). Set 1 to disable.
K_SIGMA                = 3.5     # lowered from 6.0 → easier to trigger real contact
MIN_CONSEC_TRIG        = 2       # unchanged: consecutive step confirmations

# Optional backup trigger via TCP force (Newtons). Set FORCE_THR_N=None to disable.
FORCE_THR_N            = 8.0     # ~8 N normal force
FORCE_MIN_CONSEC       = 2       # consecutive polls over threshold to confirm


# Logging
LOG_CSV_PATH = "eit_localisation_log.csv"
LOG_WRENCH   = True

# ------------------ WSL/Windows-friendly EIT serial ------------------ #
# Windows COM ports map to /dev/ttyS<NUM> in WSL/Linux (COM5 -> /dev/ttyS5).
# You can override the port via env var: EIT_PORT=/dev/ttyS5 python script.py
EIT_PORT = "/dev/ttyACM0"    # mapped serial device in WSL
EIT_BAUD = 115200
EIT_TIMEOUT = 0.2
EIT_SNIFF_SECS = 2.0
# Some devices need DTR/RTS; many USB serial EIT boards do not:
EIT_DSRDTR = False
EIT_RTSCTS = False

def map_com_to_ttyS(port_str: str) -> str:
    """Map 'COM5' -> '/dev/ttyS5' on Linux/WSL. Return original if not 'COM<d>'."""
    m = re.match(r"^COM(\d+)$", port_str, re.IGNORECASE)
    if not m:
        return port_str
    n = int(m.group(1))
    return f"/dev/ttyS{n}"

def resolve_serial_candidates(port_str: str):
    """
    Produce a list of port names to try in order, depending on OS/WSL.
    - On Windows: ['COM5', '/dev/ttyS5']
    - On Linux/WSL: ['/dev/ttyS5', 'COM5']
    """
    mapped = map_com_to_ttyS(port_str)
    if sys.platform.startswith("win"):
        return [port_str, mapped]
    else:
        return [mapped, port_str]

# Anchor pose
USE_CURRENT_POSE_AS_START = True
EXPLICIT_START_POSE = [0.40, -0.20, 0.20, 3.1415, 0.0, 0.0]

# -------- RANGE MODE (min/max with fixed increments) --------
# Distances in meters; angles in degrees. Increments are constant.
X_MIN, X_MAX, X_STEP = -0.030, +0.030, 0.001
Y_MIN, Y_MAX, Y_STEP = -0.030, +0.030, 0.001
Z_MIN, Z_MAX, Z_STEP =  0.000, +0.005, 0.001     # interpreted as *contact-point* Z offsets (positive = up)
ROLL_MIN,  ROLL_MAX,  ROLL_STEP  = -10, +10, 1
PITCH_MIN, PITCH_MAX, PITCH_STEP = -10, +10, 1
YAW_MIN,   YAW_MAX,   YAW_STEP   = -20, +20, 1

# Selection: do ALL poses or RANDOM sample of N (memory-safe)
SELECTION_MODE   = "RANDOM"   # "ALL" or "RANDOM"
RANDOM_SAMPLE_N  = 5
RANDOM_SEED      = 42

# ---------- Handle surface: poly-cubic Bezier in YZ plane ----------
HANDLE_YZ_CUBICS_MM = [
    [(0.000000, 0.000000),(6.582317, 9.227972),(9.528611, 13.074924),(11.778850, 15.889835)],
    [(11.778850, 15.889835),(13.459081, 17.991701),(14.751230, 19.518157),(16.077065, 20.962935)],
    [(16.077065, 20.962935),(17.036667, 22.008626),(18.013916, 23.011530),(19.373441, 24.179558)],
    [(19.373441, 24.179558),(20.332920, 25.003889),(21.482802, 25.910464),(22.565909, 26.678084)],
    [(22.565909, 26.678084),(23.906658, 27.628301),(25.145083, 28.365591),(26.331285, 28.943223)],
    [(26.331285, 28.943223),(27.121687, 29.328117),(27.888901, 29.642124),(29.285988, 29.865607)],
    [(29.285988, 29.865607),(30.344195, 30.034882),(31.763768, 30.152223),(33.025479, 30.216231)],
    [(33.025479, 30.216231),(36.291950, 30.381943),(38.500350, 30.190191),(40.370327, 29.608173)],
    [(40.370327, 29.608173),(41.573502, 29.233693),(42.636575, 28.697648),(43.606840, 28.108089)],
    [(43.606840, 28.108089),(44.498611, 27.566225),(45.311983, 26.979155),(46.438863, 26.349910)],
    [(46.438863, 26.349910),(47.192431, 25.929121),(48.086195, 25.489472),(49.114878, 25.099189)],
    [(49.114878, 25.099189),(50.560010, 24.550905),(52.271412, 24.100047),(53.993565, 23.786435)],
    [(53.993565, 23.786435),(56.460413, 23.337210),(58.949319, 23.169592),(61.410806, 23.171903)],
    [(61.410806, 23.171903),(64.807060, 23.175092),(68.151116, 23.501781),(71.399994, 23.921130)],
    [(71.399994, 23.921130),(75.760380, 24.483947),(79.949325, 25.213670),(83.951839, 25.840267)],
    [(83.951839, 25.840267),(88.858687, 26.608440),(93.485342, 27.221621),(98.018588, 27.501152)],
    [(98.018588, 27.501152),(102.841085, 27.798519),(107.557872, 27.718299),(112.001103, 27.314028)],
    [(112.001103, 27.314028),(116.663648, 26.889803),(121.024966, 26.108749),(125.076468, 25.172545)],
    [(125.076468, 25.172545),(128.701272, 24.334941),(132.078082, 23.373147),(135.206867, 22.369624)],
    [(135.206867, 22.369624),(137.761699, 21.550189),(140.151157, 20.702932),(142.373984, 19.805666)],
    [(142.373984, 19.805666),(143.988015, 19.154147),(145.514192, 18.476260),(146.905701, 17.757248)],
    [(146.905701, 17.757248),(147.856329, 17.266045),(148.744105, 16.755648),(149.566633, 16.153559)],
    [(149.566633, 16.153559),(150.074447, 15.781840),(150.557390, 15.375172),(151.002451, 14.852806)],
    [(151.002451, 14.852806),(151.294119, 14.510478),(151.569518, 14.118462),(151.761686, 13.588162)],
    [(151.761686, 13.588162),(151.908065, 13.184221),(152.006152, 12.700044),(151.994145, 12.195908)],
    [(151.994145, 12.195908),(151.981456, 11.663087),(151.845788, 11.107969),(151.619597, 10.502067)],
    [(151.619597, 10.502067),(151.389491, 9.885677),(151.065700, 9.216729),(150.932007, 8.928562)],
    [(150.932007, 8.928562),(150.724673, 8.481669),(150.974527, 8.950562),(150.563562, 8.179317)],
]

HANDLE_PLANE_X   = 0.0
HANDLE_ORIGIN_YZ = (-0.55, 0.060)  # meters: where CAD (y=0,z=0) should land in Base
Y_SIGN = -1.0
Z_SIGN = +1.0
HANDLE_Y_RANGE = None  # or (min_y, max_y) in meters

# ============================== MATH UTILS ==============================

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

def rpy_to_rotmat(roll, pitch, yaw):
    cx, sx = math.cos(roll),  math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw),   math.sin(yaw)
    Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]]
    Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rz = [[cz,-sz,0],[sz,cz,0],[0,0,1]]
    def mm(A,B):
        # robust 3x3 multiply
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mm(mm(Rx,Ry),Rz)

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

# ==================== HANDLE SURFACE (poly-cubic YZ) ====================

class HandleSurface:
    def z_at_world(self, xw, yw):
        raise NotImplementedError

MM_TO_M = 1e-3

def bezier_cubic(a,b,c,d,t):
    mt = 1.0 - t
    return (mt*mt*mt)*a + 3*(mt*mt)*t*b + 3*mt*(t*t)*c + (t*t*t)*d

def bezier_cubic_deriv(a,b,c,d,t):
    mt = 1.0 - t
    return 3*((b-a)*(mt*mt) + 2*(c-b)*mt*t + (d-c)*(t*t))

def solve_t_for_y_cubic(y0,y1,y2,y3, y, iters=12):
    if y3 != y0:
        t = max(0.0, min(1.0, (y - y0)/(y3 - y0)))
    else:
        t = 0.5
    for _ in range(iters):
        yt  = bezier_cubic(y0,y1,y2,y3, t)
        dyt = bezier_cubic_deriv(y0,y1,y2,y3, t)
        err = yt - y
        if abs(err) < 1e-9: break
        if abs(dyt) < 1e-12:
            t = max(0.0, min(1.0, t - 0.25*err / ( (y3-y0) if (y3!=y0) else 1.0 )))
        else:
            t = max(0.0, min(1.0, t - err/dyt))
    return t

class HandleSurfaceYZPolyBezier(HandleSurface):
    def __init__(self, segments_yz_m, y_range=None):
        self.segs = []
        for seg in segments_yz_m:
            (y0,z0),(y1,z1),(y2,z2),(y3,z3) = seg
            if y3 < y0:
                seg = [(y3,z3),(y2,z2),(y1,z1),(y0,z0)]
            self.segs.append(seg)
        self.ranges = [(s[0][0], s[3][0]) for s in self.segs]
        self.ymin_all = min(a for a,b in self.ranges)
        self.ymax_all = max(b for a,b in self.ranges)
        self.ymin, self.ymax = (y_range if y_range else (self.ymin_all, self.ymax_all))

    def z_at_world(self, xw, yw):
        y = min(max(yw, self.ymin), self.ymax)
        for (a,b), seg in zip(self.ranges, self.segs):
            if y <= b or seg is self.segs[-1]:
                (y0,z0),(y1,z1),(y2,z2),(y3,z3) = seg
                t = solve_t_for_y_cubic(y0,y1,y2,y3, y)
                return bezier_cubic(z0,z1,z2,z3, t)
        return self.segs[-1][3][1]

def build_handle_from_mm_segments(mm_segments, origin_yz_m=(0.0,0.0), y_sign=+1.0, z_sign=+1.0, y_range_m=None):
    y0, z0 = origin_yz_m
    segs_m = []
    for seg in mm_segments:
        pts = []
        for (y_mm, z_mm) in seg:
            y = y0 + y_sign * (y_mm * MM_TO_M)
            z = z0 + z_sign * (z_mm * MM_TO_M)
            pts.append((y, z))
        segs_m.append(pts)
    return HandleSurfaceYZPolyBezier(segs_m, y_range=y_range_m)

HANDLE = build_handle_from_mm_segments(
    HANDLE_YZ_CUBICS_MM,
    origin_yz_m = HANDLE_ORIGIN_YZ,
    y_sign      = Y_SIGN,
    z_sign      = Z_SIGN,
    y_range_m   = HANDLE_Y_RANGE
)

# ==================== CONTACT GEOMETRY & MOTION ====================

def world_point_from_tool(tcp_pose, p_tool):
    xw, yw, zw = tcp_pose[0], tcp_pose[1], tcp_pose[2]
    R = rvec_to_rotmat(tcp_pose[3:6])
    px,py,pz = p_tool
    return (
        xw + R[0][0]*px + R[0][1]*py + R[0][2]*pz,
        yw + R[1][0]*px + R[1][1]*py + R[1][2]*pz,
        zw + R[2][0]*px + R[2][1]*py + R[2][2]*pz
    )

def world_point_z_for_tcp_pose(tcp_pose, point_tool):
    return world_point_from_tool(tcp_pose, point_tool)[2]

def tcp_pose_for_desired_point_z(pose_like_tcp, point_tool):
    x,y,desired_point_z, rx,ry,rz = pose_like_tcp
    R = rvec_to_rotmat([rx,ry,rz])
    px,py,pz = point_tool
    dz = R[2][0]*px + R[2][1]*py + R[2][2]*pz
    tcp_z = desired_point_z - dz
    return [x,y,tcp_z, rx,ry,rz]

# --- NEW: helpers to build/check the exact TCP for a given point-Z ---
def tcp_from_pointZ(base_tcp, point_tool, desired_point_z):
    """Compose a TCP pose that yields desired Z at the specified tool point."""
    return tcp_pose_for_desired_point_z(
        [base_tcp[0], base_tcp[1], desired_point_z, base_tcp[3], base_tcp[4], base_tcp[5]],
        point_tool
    )

def ik_ok_for_pointZ(rtde_c, rtde_r, base_tcp, point_tool, desired_point_z):
    """Return a safe IK solution (list of 6) for the TCP that realizes desired point-Z, or None."""
    tcp_pose = tcp_from_pointZ(base_tcp, point_tool, desired_point_z)
    return try_ik_with_seeds(rtde_c, rtde_r, tcp_pose)

def linspace(a,b,n):
    if n<=1: return [0.5*(a+b)]
    step = (b-a)/(n-1)
    return [a + i*step for i in range(n)]

def first_contact_x_along_tool_line(tcp_pose):
    y0, z0 = TOUCH_POINT_YZ
    xs = linspace(TOOL_LINE_X_MIN, TOOL_LINE_X_MAX, TOOL_LINE_SAMPLES)
    best_x = xs[0]
    best_clear = float('inf')
    for x in xs:
        xw, yw, zw = world_point_from_tool(tcp_pose, (x, y0, z0))
        z_handle = HANDLE.z_at_world(xw, yw)
        clear = zw - z_handle
        if clear < best_clear:
            best_clear = clear
            best_x = x
    return best_x

# ---------- FLOOR GUARDS ----------

def enforce_point_floor_z(desired_point_z):
    limit = MIN_BASE_Z + FLOOR_MARGIN
    if desired_point_z >= limit - 1e-9:
        return desired_point_z
    if CLAMP_BELOW_FLOOR:
        print(f"[POINT-FLOOR] Clamped desired point-Z from {desired_point_z:.3f} to {limit:.3f}")
        return limit
    raise ValueError(f"Desired point-Z {desired_point_z:.3f} < limit {limit:.3f}")

def build_guard_points(point_tool):
    pts = [point_tool]
    if GUARD_XLINE:
        y0, z0 = point_tool[1], point_tool[2]
        pts += [(TOOL_LINE_X_MIN, y0, z0), (TOOL_LINE_X_MAX, y0, z0), (0.0, y0, z0)]
    return pts

# ---- UR5 forward kinematics (approx; classic UR5/CB DH) ----
UR5_DH = {
    "a":   [0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0],
    "d":   [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823],
    "alf": [math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0]
}

# ---- Joint limit safety filter ----
# UR5 software/mech limits (radians)
UR5_LIMITS = [
    (-2*math.pi,  2*math.pi),         # Base (pan)
    (-math.pi,    math.pi/2),         # Shoulder  (-180° to +90°)
    (-2.618,      2.618),             # Elbow     (-150° to +150°)
    (-math.pi,    math.pi),           # Wrist1    (-180° to +180°)
    (-math.pi,    math.pi),           # Wrist2    (-180° to +180°)
    (-math.pi,    math.pi),           # Wrist3    (-180° to +180°)
]
SAFE_MARGIN = 0.10        # rad buffer (~6°)
COMFORT_MARGIN = 0.20     # rad buffer for deciding if we should go HOME

def within_joint_limits(q):
    if not q or len(q) != 6:
        return False
    for qi, (lo, hi) in zip(q, UR5_LIMITS):
        if qi < lo + SAFE_MARGIN or qi > hi - SAFE_MARGIN:
            return False
    return True

def comfortably_within_limits(q):
    if not q or len(q) != 6:
        return False
    for qi, (lo, hi) in zip(q, UR5_LIMITS):
        if qi < lo + COMFORT_MARGIN or qi > hi - COMFORT_MARGIN:
            return False
    return True

# A safe, neutral HOME posture (UR standard-ish)
HOME_Q = [0.0, -math.pi/2,  math.pi/2,  0.0,  math.pi/2,  0.0]
HOME_SPEED = 0.4
HOME_ACCEL = 0.8

def try_ik_with_seeds(rtde_c, rtde_r, tcp_pose, extra_seeds=None):
    """
    Try IK with multiple seeds; return the first solution that is within limits.
    Seeds order: q_current, HOME_Q, (optional) extra_seeds.
    """
    seeds = []
    try:
        qc = rtde_r.getActualQ()
        if qc and len(qc) == 6:
            seeds.append(qc)
    except Exception:
        pass
    seeds.append(HOME_Q)
    if extra_seeds:
        seeds.extend(extra_seeds)

    for seed in seeds:
        try:
            q = rtde_c.getInverseKinematics(tcp_pose, seed)
            if q and len(q) == 6 and within_joint_limits(q):
                return q
        except Exception:
            continue
    return None


def RzTxRx(theta, d, a, alpha):
    c,s = math.cos(theta), math.sin(theta)
    ca,sa = math.cos(alpha), math.sin(alpha)
    return [
        [ c, -s*ca,  s*sa, a*c],
        [ s,  c*ca, -c*sa, a*s],
        [ 0,    sa,    ca,   d],
        [ 0,     0,     0,   1]
    ]

def fk_ur5_points(q):
    a, d, alf = UR5_DH["a"], UR5_DH["d"], UR5_DH["alf"]
    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    origins = [ (0.0,0.0,0.0) ]
    for i in range(6):
        A = RzTxRx(q[i], d[i], a[i], alf[i])
        T = [[sum(T[r][k]*A[k][c] for k in range(4)) for c in range(4)] for r in range(4)]
        origins.append( (T[0][3], T[1][3], T[2][3]) )
    pts = []
    for i in range(6):
        p0 = origins[i]; p1 = origins[i+1]
        mid = (0.5*(p0[0]+p1[0]), 0.5*(p0[1]+p1[1]), 0.5*(p0[2]+p1[2]))
        pts.extend([p1, mid])
    return pts

def min_arm_z(q):
    pts = fk_ur5_points(q)
    return min(p[2] for p in pts)

# ---- Hardened move that guards point, x-line, TCP & arm links ----
def safe_move_pointZ(rtde_c, rtde_r, pose_pointZ, point_tool, speed, accel):
    """
    Move so *point_tool* reaches desired Z, enforcing floor limits for the chosen
    tool points/TCP AND live actual feedback during the move. The motion is split
    into small guarded segments; each segment is cancelled if live Z dips below
    the runtime floor.
    Returns: (ok: bool, final_tcp_pose: list[6])
    """
    limit_static = MIN_BASE_Z + FLOOR_MARGIN
    limit_runtime = limit_static + RUNTIME_FLOOR_EPS

    desired_point_z = enforce_point_floor_z(pose_pointZ[2])
    pose_pointZ = [pose_pointZ[0], pose_pointZ[1], desired_point_z,
                   pose_pointZ[3], pose_pointZ[4], pose_pointZ[5]]

    guard_pts = build_guard_points(point_tool)

    # --- Build final target TCP from desired point-Z
    tcp_target = tcp_pose_for_desired_point_z(pose_pointZ, point_tool)

    # --- Pre-check IK & arm links for the final pose
    q_final = try_ik_with_seeds(rtde_c, rtde_r, tcp_target)
    if not q_final:
        print("[JOINT LIMIT] Skipping pose: IK near joint limits (all seeds).")
        return False, tcp_target
    # Evaluate lowest Z among tool guard points, TCP, and arm links
    zs_tool = [world_point_from_tool(tcp_target, p)[2] for p in guard_pts] + [tcp_target[2]]
    min_z_all = min(min(zs_tool), min_arm_z(q_final))
    if min_z_all < limit_static - 1e-9:
        # Try clamping up in Z (raise desired point-Z) until safe or give up
        adjusted = 0.0
        for _ in range(20):
            deficit = (limit_static - min_z_all)
            pose_pointZ[2] += deficit
            adjusted += deficit
            tcp_target = tcp_pose_for_desired_point_z(pose_pointZ, point_tool)
            q_final = try_ik_with_seeds(rtde_c, rtde_r, tcp_target)
            if not q_final:
                continue
            zs_tool = [world_point_from_tool(tcp_target, p)[2] for p in guard_pts] + [tcp_target[2]]
            min_z_all = min(min(zs_tool), min_arm_z(q_final))
            if min_z_all >= limit_static - 1e-9:
                if adjusted > 0:
                    print(f"[FLOOR] Raised target by {adjusted*1000:.1f} mm to maintain margin.")
                break
        if min_z_all < limit_static - 1e-9:
            print("[FLOOR ABORT] Could not find safe target even after clamping.")
            return False, tcp_target

    # --- Segment the move from CURRENT to TARGET and watchdog each segment
    try:
        tcp_curr = rtde_r.getActualTCPPose()
    except Exception:
        tcp_curr = tcp_target[:]  # fallback

    def lerp(a, b, t):
        return [a[i] + (b[i] - a[i]) * t for i in range(6)]

    # Number of segments based on distance in TCP space (xyz only)
    dist = math.sqrt((tcp_target[0]-tcp_curr[0])**2 +
                     (tcp_target[1]-tcp_curr[1])**2 +
                     (tcp_target[2]-tcp_curr[2])**2)
    steps = max(SEG_MIN_STEPS, int(math.ceil(dist / SEG_MAX_DPOS)))

    for k in range(1, steps+1):
        t = k / steps
        seg_tcp = lerp(tcp_curr, tcp_target, t)

        # IK + predicted floor check for the segment endpoint
        q_seg = try_ik_with_seeds(rtde_c, rtde_r, seg_tcp)
        if not q_seg:
            print("[GUARD] Segment IK failed near joint limits; aborting.")
            return False, tcp_curr

        zs_tool = [world_point_from_tool(seg_tcp, p)[2] for p in guard_pts] + [seg_tcp[2]]
        min_z_pred = min(min(zs_tool), min_arm_z(q_seg))
        if min_z_pred < limit_static - 1e-9:
            print("[GUARD] Segment would violate static floor; aborting.")
            return False, tcp_curr

        # Command small moveL to seg_tcp
        ok_cmd = rtde_c.moveL(seg_tcp, speed, accel)
        if not ok_cmd:
            print("[MOVE] moveL command failed; aborting.")
            return False, tcp_curr

        # Live watchdog while we wait for the segment to complete
        t0 = time.time()
        while time.time() - t0 < WATCHDOG_TIMEOUT_S:
            try:
                atcp = rtde_r.getActualTCPPose()
                # Hard runtime floor on actual TCP Z
                if atcp[2] < limit_runtime:
                    print(f"[WATCHDOG] Actual TCP Z {atcp[2]:.3f} < runtime limit {limit_runtime:.3f}. Stopping.")
                    try:
                        rtde_c.speedStop()
                    except Exception:
                        pass
                    return False, atcp

                # Optional: quick arm link estimate (use current joint state)
                if GUARD_ARM_LINKS:
                    try:
                        q_now = rtde_r.getActualQ()
                        if q_now and min_arm_z(q_now) < limit_runtime:
                            print("[WATCHDOG] Arm link below runtime floor. Stopping.")
                            try:
                                rtde_c.speedStop()
                            except Exception:
                                pass
                            return False, atcp
                    except Exception:
                        pass
            except Exception:
                pass
            # crude completion check (close enough in xyz)
            if (abs(seg_tcp[0]-atcp[0]) < 1e-4 and
                abs(seg_tcp[1]-atcp[1]) < 1e-4 and
                abs(seg_tcp[2]-atcp[2]) < 1e-4):
                break
            time.sleep(WATCHDOG_POLL_DT)

        # proceed to next segment
        tcp_curr = seg_tcp[:]

    return True, tcp_target


# ============================ EIT & DETECTION ============================

def open_eit_serial(port, baud, timeout):
    """
    Try OS-appropriate candidates and print per-candidate errors so it’s obvious
    why each attempt failed (e.g., permission denied vs. port busy).
    """
    candidates = resolve_serial_candidates(port)
    errors = []
    for p in candidates:
        try:
            if p.startswith("/dev/") and not Path(p).exists():
                errors.append((p, "device node not found"))
                continue
            ser = serial.Serial(
                port=p,
                baudrate=baud,
                timeout=timeout,
                dsrdtr=EIT_DSRDTR,
                rtscts=EIT_RTSCTS,
            )
            time.sleep(0.2)
            try: ser.reset_input_buffer()
            except Exception: pass
            print(f"EIT serial on {p} @ {baud}.")
            return ser
        except Exception as e:
            errors.append((p, str(e)))
    print("[WARNING] Could not open EIT serial. Tried:")
    for p, err in errors:
        print(f"  - {p}: {err}")
    return None

def read_eit_line(ser, timeout_s):
    if not ser: return ""
    t0 = time.time(); last = ""
    while time.time() - t0 < timeout_s:
        line = ser.readline()
        if not line: continue
        s = line.decode("utf-8", errors="ignore").strip()
        if s:
            last = s; break
    return last

def parse_eit_csv(s):
    """
    Parse 1 CSV line of EIT values -> list[float] or None.
    Filters obviously short frames.
    """
    try:
        vals = [float(x.strip()) for x in s.split(",") if x.strip() != ""]
        if len(vals) < EIT_MIN_COLS:
            return None
        return vals
    except Exception:
        return None

def _robust_mean_sd(cols):
    """
    Given iterable of columns (list-of-lists by channel), return mean and a robust sd per channel.
    sd uses max(pop SD, 1.4826*MAD, eps) to avoid zero-variance stalls.
    """
    mu = [statistics.fmean(col) for col in cols]
    sd = []
    for col in cols:
        try:
            pop = statistics.pstdev(col)
        except Exception:
            pop = 0.0
        # MAD
        m = statistics.fmean(col)
        mad = statistics.fmedian([abs(v - m) for v in col]) if hasattr(statistics, 'fmedian') else statistics.median([abs(v - m) for v in col])
        robust = 1.4826 * mad
        s = max(pop, robust, 1e-9)
        sd.append(s)
    return mu, sd

def baseline_eit(ser, n=BASELINE_SAMPLES, per_line_timeout=EIT_TIMEOUT):
    """
    Read n frames at hover; build robust baseline (mu, sd).
    """
    if not ser: return None, None, ""
    vals = []
    last_raw = ""
    for _ in range(n):
        s = read_eit_line(ser, per_line_timeout)
        last_raw = s or last_raw
        v = parse_eit_csv(s) if s else None
        if v is not None:
            vals.append(v)
        time.sleep(0.005)
    if not vals:
        return None, None, last_raw

    m = min(len(v) for v in vals)
    vals = [v[:m] for v in vals]
    cols = list(zip(*vals))  # channel-wise lists
    mu, sd = _robust_mean_sd(cols)
    return mu, sd, last_raw

def _median_filter(x_list, win):
    if win <= 1 or len(x_list) < 2: return x_list[:]
    if win % 2 == 0: win += 1
    k = win // 2
    out = []
    for i in range(len(x_list)):
        a = max(0, i-k); b = min(len(x_list), i+k+1)
        out.append(sorted(x_list[a:b])[len(range(a,b))//2])
    return out

def change_score(curr, mu, sd, method="sigma"):
    """
    Mean absolute z-score across channels. Uses robust sd from baseline.
    """
    if curr is None or mu is None or sd is None: return None
    m = min(len(curr), len(mu), len(sd))
    if m == 0: return None
    eps = 1e-9
    zs = [abs(curr[i]-mu[i]) / (sd[i] if sd[i] > eps else 1.0) for i in range(m)]
    return sum(zs)/m

# ========================= ACTIVE DESCENT CORE =========================

def active_descend_to_contact(rtde_c, rtde_r, base_tcp, point_tool,
                              start_point_z, planned_target_point_z,
                              ser,
                              step_z=STEP_Z,
                              speed=DESCENT_SPEED,
                              accel=DESCENT_ACCEL,
                              post_extra=CONTACT_EXTRA_Z,
                              method=DETECT_METHOD,
                              k_sigma=K_SIGMA,
                              abs_thr=ABS_THRESHOLD,
                              min_consec=MIN_CONSEC_TRIG,
                              step_settle=STEP_SETTLE,
                              no_contact_extra=NO_CONTACT_EXTRA_Z):
    """
    Step the *controlled point-Z* from hover toward planned_target_point_z.
    Uses robust EIT detection with multiple frames per step and optional
    force-based backup trigger. Returns: (ok, tcp_final, info)
    """
    # Fresh baseline at hover
    mu, sd, _ = baseline_eit(ser) if ser else (None, None, "")
    if ser and mu is None:
        print("[EIT] Baseline failed; descending without detection gating.")

    # Direction: lowering means target < hover (UR Base Z up)
    direction = -1.0 if planned_target_point_z < start_point_z else +1.0

    # If target above hover: simple guarded move
    if direction > 0:
        pose_pointZ = [base_tcp[0], base_tcp[1], planned_target_point_z,
                       base_tcp[3], base_tcp[4], base_tcp[5]]
        ok, tcp_pose = safe_move_pointZ(rtde_c, rtde_r, pose_pointZ, point_tool, speed, accel)
        info = dict(triggered=False, score=None, z_contact=None, z_final=planned_target_point_z,
                    mode="upmove")
        return ok, tcp_pose, info

    deeper_limit_z = planned_target_point_z + direction * abs(no_contact_extra)

    # (Optional) running median filter and history for debug
    score_hist = []
    consec_eit = 0
    consec_force = 0
    z_contact = None
    current_z = start_point_z
    tcp_pose = None
    last_score = None

    # Ensure serial buffer is clean before starting the loop
    if ser:
        try: ser.reset_input_buffer()
        except Exception: pass

    while True:
        # Step toward deeper_limit_z
        next_z = current_z + direction * step_z
        if direction < 0 and next_z < deeper_limit_z:
            next_z = deeper_limit_z

        pose_pointZ = [base_tcp[0], base_tcp[1], next_z,
                       base_tcp[3], base_tcp[4], base_tcp[5]]
        ok, tcp_pose = safe_move_pointZ(rtde_c, rtde_r, pose_pointZ, point_tool, speed, accel)
        if not ok:
            print("[MOVE] Step move rejected/failed; aborting descent.")
            break
        current_z = next_z

        # small settle
        time.sleep(step_settle)

        # ----- collect multiple EIT frames and average their score -----
        scores = []
        if ser and mu is not None:
            for _ in range(max(1, EIT_PER_STEP_SAMPLES)):
                s = read_eit_line(ser, EIT_TIMEOUT)
                v = parse_eit_csv(s) if s else None
                sc = change_score(v, mu, sd, method=method) if v is not None else None
                if sc is not None:
                    scores.append(sc)
                # a tiny pause between frames to avoid spamming
                time.sleep(0.003)

        if scores:
            if EIT_MEDIAN_FILTER_WIN > 1:
                scores = _median_filter(scores, EIT_MEDIAN_FILTER_WIN)
            last_score = sum(scores)/len(scores)
            score_hist.append(last_score)
            print(f"[EIT] z={current_z:.4f} m, score≈{last_score:.2f}")
        else:
            last_score = None
            print(f"[EIT] z={current_z:.4f} m, no valid frame")

        # ----- force backup trigger (optional) -----
        force_trigger = False
        if FORCE_THR_N is not None:
            try:
                Fx,Fy,Fz,Tx,Ty,Tz = rtde_r.getActualTCPForce()
                # Consider only downward (negative Z) or absolute? Here absolute:
                if abs(Fz) >= FORCE_THR_N:
                    consec_force += 1
                else:
                    consec_force = 0
                if consec_force >= FORCE_MIN_CONSEC:
                    force_trigger = True
            except Exception:
                pass

        # ----- evaluate triggers -----
        eit_trigger = False
        if last_score is not None:
            thr = (k_sigma if method == "sigma" else abs_thr)
            eit_trigger = last_score >= thr
            consec_eit = consec_eit + 1 if eit_trigger else 0

        if (consec_eit >= min_consec) or force_trigger:
            z_contact = current_z
            print(f"[CONTACT] detected at z={z_contact:.4f} m "
                  f"({'force' if force_trigger else 'eit'}; score={last_score})")
            break

        # Stop if we reached allowed deeper limit with no trigger
        if abs(current_z - deeper_limit_z) < 1e-12:
            if z_contact is None:
                print("[CONTACT] No change detected — stopped at deeper limit.")
            break

    # If detected, go a bit deeper and stop
    if z_contact is not None:
        final_z = z_contact + direction * abs(post_extra)
        pose_pointZ = [base_tcp[0], base_tcp[1], final_z,
                       base_tcp[3], base_tcp[4], base_tcp[5]]
        ok2, tcp_pose2 = safe_move_pointZ(rtde_c, rtde_r, pose_pointZ, point_tool, speed, accel)
        if ok2:
            current_z = final_z
            tcp_pose = tcp_pose2
        else:
            print("[CONTACT] Post-extra move rejected/failed — holding at contact.")
        info = dict(triggered=True, score=last_score, z_contact=z_contact, z_final=current_z,
                    score_hist=score_hist[-20:])
        return True, tcp_pose, info

    # Otherwise we stopped at deeper limit (no trigger)
    info = dict(triggered=False, score=last_score, z_contact=None, z_final=current_z,
                score_hist=score_hist[-20:])
    return True, tcp_pose, info


# ============================== CSV LOG ==============================

def sniff_eit_columns(ser, sniff_secs=2.0):
    raw = read_eit_line(ser, sniff_secs)
    if not raw:
        return [], ""
    parts = [p.strip() for p in raw.split(",")]
    cols = [f"eit_{i}" for i in range(len(parts))]
    return cols, raw

def open_csv_logger(path, include_wrench=True, eit_cols=None):
    fields = [
        "timestamp","session_id","index",
        "dx_m","dy_m","dz_m","roll_deg","pitch_deg","yaw_deg",
        "contact_x_sel","point_set_z",
        "contact_triggered","contact_score","contact_z","final_point_z",
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

# ===================== MEMORY-SAFE SAMPLING HELPERS =====================

def frange(vmin, vmax, step):
    vals, x = [], vmin
    while x <= vmax + 1e-12:
        vals.append(round(x, 9)); x += step
    return vals

def fast_random_product_sample(axes_lists, k, seed=None, max_tries=1000000):
    rng = random.Random(seed)
    if k <= 0:
        return []
    total = 1
    for L in axes_lists:
        total *= len(L)
    k = min(k, total)
    chosen = set()
    tries = 0
    while len(chosen) < k and tries < max_tries:
        tup = tuple(rng.choice(L) for L in axes_lists)
        chosen.add(tup)
        tries += 1
    return list(chosen)

# ============================== MAIN ==============================
def main():
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    time.sleep(0.1)

    # --- EIT serial ---
    ser = open_eit_serial(EIT_PORT, EIT_BAUD, EIT_TIMEOUT)
    eit_cols, _ = sniff_eit_columns(ser, EIT_SNIFF_SECS) if ser else ([], "")

    csv_f = None
    try:
        # 1) Set TCP to the sensor center
        rtde_c.setTcp(TCP_OFFSET_AT_SENSOR_CENTER)

        # 2) Anchor pose (TCP)
        start_tcp = rtde_r.getActualTCPPose() if USE_CURRENT_POSE_AS_START else EXPLICIT_START_POSE
        print("Start TCP pose:", [round(v,6) for v in start_tcp])

        # Optional: if current joints are uncomfortable, go to HOME once
        try:
            q_now = rtde_r.getActualQ()
            if not comfortably_within_limits(q_now):
                print("[INFO] Current joints near limits; moving to HOME first.")
                try:
                    rtde_c.moveJ(HOME_Q, HOME_SPEED, HOME_ACCEL)
                    time.sleep(0.2)
                except Exception as e:
                    print(f"[WARN] moveJ to HOME failed ({e}); continuing without it.")
        except Exception:
            pass

        # Compute current contact-point Z (x=0) and rise to hover once
        y0, z0 = TOUCH_POINT_YZ
        current_point_z = world_point_z_for_tcp_pose(start_tcp, (0.0, y0, z0))
        start_point_hover = [start_tcp[0], start_tcp[1], current_point_z + HOVER_LIFT_Z,
                             start_tcp[3], start_tcp[4], start_tcp[5]]
        ok, _ = safe_move_pointZ(rtde_c, rtde_r, start_point_hover, (0.0, y0, z0), SPEED, ACCEL)
        if not ok:
            print("[WARN] Cannot reach global start hover (floor/joint guard). Continuing anyway.")

        # 3) Build discretized ranges
        Xs = frange(X_MIN, X_MAX, X_STEP)
        Ys = frange(Y_MIN, Y_MAX, Y_STEP)
        Zs = frange(Z_MIN, Z_MAX, Z_STEP)
        Rls = frange(ROLL_MIN,  ROLL_MAX,  ROLL_STEP)
        Pcs = frange(PITCH_MIN, PITCH_MAX, PITCH_STEP)
        Yws = frange(YAW_MIN,   YAW_MAX,   YAW_STEP)

        # open csv
        session_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        csv_f, csv_w, _ = open_csv_logger(LOG_CSV_PATH, include_wrench=LOG_WRENCH, eit_cols=eit_cols)

        # Helper: combine RPY (deg) to axis–angle rvec
        def rpy_combo_to_rvec(roll_deg, pitch_deg, yaw_deg):
            R = rpy_to_rotmat(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg))
            return rotmat_to_rvec(R)

        # 4) Random sampling until N successful poses, with adaptive shrinking
        rng = random.Random(RANDOM_SEED)
        successful = 0
        attempted = 0
        max_attempts = 10 * RANDOM_SAMPLE_N

        # Mutable ranges for adaptation
        dx_range = list(Xs)
        dy_range = list(Ys)
        dz_range = list(Zs)
        roll_range = list(Rls)
        pitch_range = list(Pcs)
        yaw_range = list(Yws)

        consecutive_skips = 0

        def halve_centered(vals):
            if len(vals) < 3:
                return vals
            mid = len(vals)//2
            span = max(1, len(vals)//4)
            lo = max(0, mid - span)
            hi = min(len(vals), mid + span + 1)
            return vals[lo:hi]

        def shrink_ranges():
            nonlocal dx_range, dy_range, roll_range, pitch_range, yaw_range
            dx_range[:] = halve_centered(dx_range)
            dy_range[:] = halve_centered(dy_range)
            roll_range[:] = halve_centered(roll_range)
            pitch_range[:] = halve_centered(pitch_range)
            yaw_range[:] = halve_centered(yaw_range)
            print("[ADAPT] Shrunk search ranges to improve feasibility.")

        while successful < RANDOM_SAMPLE_N and attempted < max_attempts:
            attempted += 1

            # pick a random pose from (possibly shrunk) ranges
            dx = rng.choice(dx_range)
            dy = rng.choice(dy_range)
            dz = rng.choice(dz_range)
            rdeg = rng.choice(roll_range)
            pdeg = rng.choice(pitch_range)
            ydeg = rng.choice(yaw_range)

            drx, dry, drz = rpy_combo_to_rvec(rdeg, pdeg, ydeg)
            base_tcp = compose_pose(start_tcp, [dx, dy, 0.0, drx, dry, drz])

            # Choose the contact point along the tool X-line that would touch first
            x_sel = first_contact_x_along_tool_line(base_tcp)
            point_tool = (x_sel, TOUCH_POINT_YZ[0], TOUCH_POINT_YZ[1])

            # Planned contact-point Z and hover Z
            start_point_z_ref = world_point_z_for_tcp_pose(start_tcp, (0.0, TOUCH_POINT_YZ[0], TOUCH_POINT_YZ[1]))
            planned_point_z = start_point_z_ref + dz
            hover_z = planned_point_z + max(HOVER_LIFT_Z, HOVER_LIFT_Z_MIN)

            # Early IK check specifically for the HOVER TCP (what we will actually command)
            q_hover = ik_ok_for_pointZ(rtde_c, rtde_r, base_tcp, point_tool, hover_z)
            if not q_hover:
                print("[SKIP] Cannot reach hover safely — skipping pose.")
                consecutive_skips += 1
                if consecutive_skips in (10, 20, 30):
                    shrink_ranges()
                continue

            # Try to hover above planned target (guards inside)
            hover_pointZ = [base_tcp[0], base_tcp[1], hover_z,
                            base_tcp[3], base_tcp[4], base_tcp[5]]
            ok, _ = safe_move_pointZ(rtde_c, rtde_r, hover_pointZ, point_tool, SPEED, ACCEL)
            if not ok:
                print("[SKIP] Cannot reach hover safely — skipping pose.")
                consecutive_skips += 1
                if consecutive_skips in (10, 20, 30):
                    shrink_ranges()
                continue

            # Active descent
            if ser:
                try: ser.reset_input_buffer()
                except Exception: pass

            ok, tcp_tgt, info = active_descend_to_contact(
                rtde_c, rtde_r, base_tcp, point_tool,
                start_point_z = hover_pointZ[2],
                planned_target_point_z = planned_point_z,
                ser = ser,
                step_z = STEP_Z,
                speed = DESCENT_SPEED,
                accel = DESCENT_ACCEL,
                post_extra = CONTACT_EXTRA_Z,
                method = DETECT_METHOD,
                k_sigma = K_SIGMA,
                abs_thr = ABS_THRESHOLD,
                min_consec = MIN_CONSEC_TRIG,
                step_settle = STEP_SETTLE,
                no_contact_extra = NO_CONTACT_EXTRA_Z
            )
            if not ok:
                print("[SKIP] Active descent failed — skipping pose.")
                consecutive_skips += 1
                if consecutive_skips in (10, 20, 30):
                    shrink_ranges()
                continue

            # ---- Dwell & EIT read ----
            if ser:
                try: ser.reset_input_buffer()
                except Exception: pass
            time.sleep(DWELL)
            last_raw = read_eit_line(ser, EIT_TIMEOUT) if ser else ""

            # ---- Log successful pose ----
            actual_tcp = rtde_r.getActualTCPPose()
            wrench = rtde_r.getActualTCPForce() if LOG_WRENCH else [None]*6

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "session_id": session_id,
                "index": successful + 1,
                "dx_m": float(dx), "dy_m": float(dy), "dz_m": float(dz),
                "roll_deg": float(rdeg), "pitch_deg": float(pdeg), "yaw_deg": float(ydeg),
                "contact_x_sel": x_sel,
                "point_set_z": planned_point_z,
                "contact_triggered": bool(info.get("triggered", False)),
                "contact_score": info.get("score", None),
                "contact_z": info.get("z_contact", None),
                "final_point_z": info.get("z_final", None),
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
            successful += 1
            consecutive_skips = 0
            print(f"[SUCCESS] Logged pose {successful}/{RANDOM_SAMPLE_N}")

            # Rise back to hover (guards) at current XY/ori
            back_hover = [base_tcp[0], base_tcp[1], info.get("z_final", planned_point_z) + max(HOVER_LIFT_Z, HOVER_LIFT_Z_MIN),
                          base_tcp[3], base_tcp[4], base_tcp[5]]
            safe_move_pointZ(rtde_c, rtde_r, back_hover, point_tool, SPEED, ACCEL)

        print(f"Sequence complete: {successful} successful poses out of {attempted} attempts.")

    finally:
        try: rtde_c.stopScript()
        except Exception: pass
        try:
            if csv_f: csv_f.close()
        except Exception: pass
        try:
            if ser: ser.close()
        except Exception: pass
        print("Done.")

if __name__ == "__main__":
    main()
