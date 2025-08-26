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

import serial  # pip install pyserial

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# ---------------------------- USER SETTINGS ---------------------------- #

ROBOT_IP = "169.254.150.50"

# Active TCP set to the *sensor center* (meters, axis-angle radians)
TCP_OFFSET_AT_SENSOR_CENTER = [-0.020, 0.000, 0.100, 0.0, 0.0, 0.0]

# Motion & dwell
SPEED = 0.10          # m/s for XY/orientation + hover moves
ACCEL = 0.10          # m/s^2
DWELL = 1.30          # s to settle & read EIT at final depth

# Hover / retract height (applied to the controlled contact-point Z)
HOVER_LIFT_Z = 0.100  # 100 mm

# ---------- Active descent parameters (point-Z stepping) ----------
DESCENT_SPEED     = 0.010   # m/s while stepping down
DESCENT_ACCEL     = 0.100   # m/s^2
STEP_Z            = 0.0005  # m per step (0.5 mm)
CONTACT_EXTRA_Z   = 0.0020  # m after detection (2 mm)
STEP_SETTLE       = 0.05    # s pause after each step before sampling

# If no EIT change at the planned target depth, keep lowering by up to this extra:
NO_CONTACT_EXTRA_Z = 0.004  # 4 mm beyond planned target (still obeys floor guard)

# Detection settings (baseline vs. live)
BASELINE_SAMPLES  = 20      # EIT lines at hover to build baseline
DETECT_METHOD     = "sigma" # "sigma" (z-score) or "abs"
K_SIGMA           = 6.0     # trigger when mean |zscore| ≥ K_SIGMA
ABS_THRESHOLD     = 0.05    # if DETECT_METHOD=="abs": trigger when mean |x-mu| ≥ ABS_THRESHOLD
MIN_CONSEC_TRIG   = 2       # require N consecutive triggers to confirm contact

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

# Logging
LOG_CSV_PATH = "eit_localisation_log.csv"
LOG_WRENCH   = True

# EIT serial
EIT_PORT = "COM5"            # use "/dev/ttyUSB0" or "/dev/ttyACM0" on Linux
EIT_BAUD = 115200
EIT_TIMEOUT = 0.2
EIT_SNIFF_SECS = 2.0

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
        return [[A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0],
                 A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1],
                 A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2]],
                [A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0],
                 A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1],
                 A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0],
                 ]]
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
    pose_pointZ=[x,y, desired_point_z, rx,ry,rz]; move so *point_tool* hits desired Z AND
    ensure TCP and arm links don't go below MIN_BASE_Z + FLOOR_MARGIN. Lifts the commanded
    Z if needed (when clamping enabled).
    """
    limit = MIN_BASE_Z + FLOOR_MARGIN
    desired_point_z = enforce_point_floor_z(pose_pointZ[2])
    pose_pointZ = [pose_pointZ[0], pose_pointZ[1], desired_point_z, pose_pointZ[3], pose_pointZ[4], pose_pointZ[5]]
    guard_pts = build_guard_points(point_tool)

    adjusted = 0.0
    for _ in range(20):  # up to 20 corrective lifts
        tcp_pose = tcp_pose_for_desired_point_z(pose_pointZ, point_tool)

        # Guard tool points + TCP height
        zs = []
        for p in guard_pts:
            zs.append( world_point_from_tool(tcp_pose, p)[2] )
        zs.append(tcp_pose[2])
        min_z_tool = min(zs)

        # Guard arm links (via IK + FK)
        min_z_arm = min_z_tool
        if GUARD_ARM_LINKS:
            try:
                q_current = rtde_r.getActualQ()
                q_tgt = rtde_c.getInverseKinematics(tcp_pose, q_current)
                if q_tgt and len(q_tgt) == 6:
                    min_z_arm = min_arm_z(q_tgt)
            except Exception:
                min_z_arm = min_z_tool

        min_z_all = min(min_z_tool, min_z_arm)

        if min_z_all >= limit - 1e-9:
            if adjusted > 0:
                print(f"[FLOOR] Raised target by {adjusted*1000:.1f} mm to maintain margin.")
            ok = rtde_c.moveL(tcp_pose, speed, accel)
            return bool(ok), tcp_pose

        # Need to lift
        deficit = (limit - min_z_all)
        if not CLAMP_BELOW_FLOOR:
            print(f"[FLOOR ABORT] minZ={min_z_all:.3f} < limit={limit:.3f}. "
                  f"Required lift {deficit*1000:.1f} mm.")
            return False, tcp_pose

        pose_pointZ[2] += deficit
        adjusted += deficit

    print("[FLOOR ABORT] Too many corrective lifts; giving up.")
    return False, tcp_pose

# ============================ EIT & DETECTION ============================

def open_eit_serial(port, baud, timeout):
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        time.sleep(0.2)
        ser.reset_input_buffer()
        print(f"EIT serial on {port} @ {baud}.")
        return ser
    except Exception as e:
        print(f"WARNING: Could not open EIT serial on {port}: {e}")
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
    try:
        return [float(x.strip()) for x in s.split(",") if x.strip() != ""]
    except Exception:
        return None

def baseline_eit(ser, n=BASELINE_SAMPLES, per_line_timeout=EIT_TIMEOUT):
    vals = []
    for _ in range(n):
        s = read_eit_line(ser, per_line_timeout)
        v = parse_eit_csv(s)
        if v is not None: vals.append(v)
        time.sleep(0.005)
    if not vals:
        return None, None, ""
    m = min(len(v) for v in vals)
    vals = [v[:m] for v in vals]
    mu = [statistics.fmean(col) for col in zip(*vals)]
    sd = []
    for col in zip(*vals):
        try:
            sd.append(statistics.pstdev(col))
        except Exception:
            sd.append(0.0)
    return mu, sd, ",".join(str(x) for x in vals[-1])

def change_score(curr, mu, sd, method="sigma"):
    if curr is None or mu is None: return None
    m = min(len(curr), len(mu))
    if m == 0: return None
    if method == "abs":
        diffs = [abs(curr[i] - mu[i]) for i in range(m)]
        return sum(diffs)/m
    # sigma
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
    - If EIT change is detected: go an additional `post_extra` deeper and stop.
    - If NOT detected at the planned target: continue up to `no_contact_extra` deeper, then stop.
    Always obeys MIN_BASE_Z via floor guard.
    Returns: (ok, tcp_final, contact_info_dict)
    """
    # Baseline at hover (if serial available)
    mu, sd, _ = baseline_eit(ser) if ser else (None, None, "")
    if ser and mu is None:
        print("[EIT] Baseline failed; descending without detection gating.")

    # Decide descent direction (UR Base Z up; lowering means target < hover)
    direction = -1.0 if planned_target_point_z < start_point_z else +1.0

    # If target is above hover, just move there and finish
    if direction > 0:
        pose_pointZ = [base_tcp[0], base_tcp[1], planned_target_point_z,
                       base_tcp[3], base_tcp[4], base_tcp[5]]
        ok, tcp_pose = safe_move_pointZ(rtde_c, rtde_r, pose_pointZ, point_tool, speed, accel)
        info = dict(triggered=False, score=None, z_contact=None, z_final=planned_target_point_z)
        return ok, tcp_pose, info

    # Going DOWN: allow continuing beyond planned target by `no_contact_extra`
    deeper_limit_z = planned_target_point_z + direction * abs(no_contact_extra)  # direction = -1 → planned - extra

    consec = 0
    z_contact = None
    current_z = start_point_z
    tcp_pose = None
    score = None

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
        time.sleep(step_settle)

        # Score latest line if we have a baseline
        if ser and mu is not None:
            s = read_eit_line(ser, EIT_TIMEOUT)
            curr = parse_eit_csv(s) if s else None
            score = change_score(curr, mu, sd, method=method)

            triggered = False
            if score is not None:
                triggered = (score >= (k_sigma if method == "sigma" else abs_thr))

            consec = consec + 1 if triggered else 0
            if consec >= min_consec:
                z_contact = current_z
                print(f"[CONTACT] change detected (score={score:.2f}) at z={z_contact:.4f} m")
                break

        # Stop if we reached allowed deeper limit with no trigger
        if abs(current_z - deeper_limit_z) < 1e-12:
            if z_contact is None:
                print("[CONTACT] No change detected — stopped at deeper limit.")
            break

    # If detected contact, go post-contact extra and stop
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
        info = dict(triggered=True, score=score, z_contact=z_contact, z_final=current_z)
        return True, tcp_pose, info

    # Otherwise we stopped at deeper_limit_z (no trigger)
    info = dict(triggered=False, score=score, z_contact=None, z_final=current_z)
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

        # Compute current contact-point Z (x=0) and rise to hover
        y0, z0 = TOUCH_POINT_YZ
        current_point_z = world_point_z_for_tcp_pose(start_tcp, (0.0, y0, z0))
        start_point_hover = [start_tcp[0], start_tcp[1], current_point_z + HOVER_LIFT_Z,
                             start_tcp[3], start_tcp[4], start_tcp[5]]
        ok, _ = safe_move_pointZ(rtde_c, rtde_r, start_point_hover, (0.0, y0, z0), SPEED, ACCEL)
        if not ok:
            print("Cannot reach start hover (floor guard). Exiting.")
            return

        # 3) Build discretized ranges
        Xs = frange(X_MIN, X_MAX, X_STEP)
        Ys = frange(Y_MIN, Y_MAX, Y_STEP)
        Zs = frange(Z_MIN, Z_MAX, Z_STEP)
        Rls = frange(ROLL_MIN,  ROLL_MAX,  ROLL_STEP)
        Pcs = frange(PITCH_MIN, PITCH_MAX, PITCH_STEP)
        Yws = frange(YAW_MIN,   YAW_MAX,   YAW_STEP)

        axes_lists = [Xs, Ys, Zs, Rls, Pcs, Yws]
        total_count = prod(len(L) for L in axes_lists)
        print(f"Grid cardinality (not materialized): {total_count:,}")

        # 4) Choose iteration WITHOUT building the full list
        if SELECTION_MODE.upper() == "ALL":
            selected_iter = product(*axes_lists)   # generator
            total_known = total_count
        else:
            selected_list = fast_random_product_sample(axes_lists, RANDOM_SAMPLE_N, seed=RANDOM_SEED)
            selected_iter = iter(selected_list)
            total_known = len(selected_list)
            print(f"Randomly selected {total_known} poses (seed={RANDOM_SEED}).")

        # open csv
        session_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        csv_f, csv_w, _ = open_csv_logger(LOG_CSV_PATH, include_wrench=LOG_WRENCH, eit_cols=eit_cols)

        # Helper: combine RPY (deg) to axis–angle rvec
        def rpy_combo_to_rvec(roll_deg, pitch_deg, yaw_deg):
            R = rpy_to_rotmat(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg))
            return rotmat_to_rvec(R)

        # 5) Iterate poses
        for idx, (dx, dy, dz, rdeg, pdeg, ydeg) in enumerate(selected_iter, start=1):
            drx, dry, drz = rpy_combo_to_rvec(rdeg, pdeg, ydeg)
            base_tcp = compose_pose(start_tcp, [dx, dy, 0.0, drx, dry, drz])

            # Choose the contact point along the tool X-line that would touch first
            x_sel = first_contact_x_along_tool_line(base_tcp)
            point_tool = (x_sel, TOUCH_POINT_YZ[0], TOUCH_POINT_YZ[1])

            # Planned contact-point Z (relative to the start point’s contact Z at x=0).
            # NOTE: If your dz is "depth-positive" (down), use start_point_z - dz instead.
            start_point_z_ref = world_point_z_for_tcp_pose(start_tcp, (0.0, TOUCH_POINT_YZ[0], TOUCH_POINT_YZ[1]))
            planned_point_z = start_point_z_ref + dz

            # Go to hover above planned target at current XY/orientation
            hover_pointZ = [base_tcp[0], base_tcp[1], planned_point_z + HOVER_LIFT_Z,
                            base_tcp[3], base_tcp[4], base_tcp[5]]
            ok, _ = safe_move_pointZ(rtde_c, rtde_r, hover_pointZ, point_tool, SPEED, ACCEL)
            if not ok:
                print("moveL() to hover (guards) rejected/failed — skipping pose.")
                continue

            # ACTIVE DESCENT to contact (or to deeper limit if no contact)
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
                print("Active descent failed — skipping logging for this pose.")
                continue

            # ---- Dwell & EIT read ----
            if ser:
                try: ser.reset_input_buffer()
                except Exception: pass
            time.sleep(DWELL)
            last_raw = read_eit_line(ser, EIT_TIMEOUT) if ser else ""

            # ---- Log ----
            actual_tcp = rtde_r.getActualTCPPose()
            wrench = rtde_r.getActualTCPForce() if LOG_WRENCH else [None]*6

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "session_id": session_id,
                "index": idx,
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

            if csv_f is None:
                csv_f, csv_w, _ = open_csv_logger(LOG_CSV_PATH, include_wrench=LOG_WRENCH, eit_cols=eit_cols)
            csv_w.writerow(row); csv_f.flush()

            # Rise back to hover (guards) at current XY/ori
            back_hover = [base_tcp[0], base_tcp[1], info.get("z_final", planned_point_z) + HOVER_LIFT_Z,
                          base_tcp[3], base_tcp[4], base_tcp[5]]
            safe_move_pointZ(rtde_c, rtde_r, back_hover, point_tool, SPEED, ACCEL)

        print("Sequence complete. Returning to safe hover near current position.")
        current_tcp = rtde_r.getActualTCPPose()
        current_point_z = world_point_z_for_tcp_pose(current_tcp, (0.0, TOUCH_POINT_YZ[0], TOUCH_POINT_YZ[1]))
        back_hover = [current_tcp[0], current_tcp[1], current_point_z + HOVER_LIFT_Z,
                      current_tcp[3], current_tcp[4], current_tcp[5]]
        safe_move_pointZ(rtde_c, rtde_r, back_hover, (0.0, TOUCH_POINT_YZ[0], TOUCH_POINT_YZ[1]), SPEED, ACCEL)

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
