# ------------------ Repeated random taps config ------------------ #
N_TAPS = 25                # <- number of taps
RANDOM_SEED = 123          # reproducible randomness

# Ranges (uniform) about the ORIGINAL start pose (meters / degrees)
RAND_X_RANGE   = 0.050     # ±50 mm
RAND_Y_RANGE   = 0.050     # ±50 mm
RAND_Z_RANGE   = 0.005     # ±5 mm
RAND_ROLL_DEG  = 9.0       # ±9°
RAND_PITCH_DEG = 9.0       # ±9°
RAND_YAW_DEG   = 18.0       # ±18°

# Tap motion parameters
HOVER_LIFT_Z   = 0.100     # 100 mm above each target
TAP_DWELL      = 0.15      # seconds at contact depth
TAP_SPEED      = SPEED     # reuse your SPEED / ACCEL
TAP_ACCEL      = ACCEL

# Optional: how far to "poke" past the target Z (positive number)
# If you want the target pose's Z to be the tap depth, set this to 0.
EXTRA_TAP_DEPTH = 0.000    # meters


# ------------- helper: build a pose delta around origin ------------- #
import random as _rnd
_rnd.seed(RANDOM_SEED)

def rpy_combo_to_rvec(roll_deg, pitch_deg, yaw_deg, order="XYZ"):
    return rpy_to_rvec(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg), order=order)

def sample_delta_toolframe(first=False):
    """Return a tool-frame delta [dx,dy,dz,drx,dry,drz].
       For the FIRST tap we return zeros (exact starting pose)."""
    if first:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    dx = _rnd.uniform(-RAND_X_RANGE,   +RAND_X_RANGE)
    dy = _rnd.uniform(-RAND_Y_RANGE,   +RAND_Y_RANGE)
    dz = _rnd.uniform(-RAND_Z_RANGE,   +RAND_Z_RANGE)
    r  = _rnd.uniform(-RAND_ROLL_DEG,  +RAND_ROLL_DEG)
    p  = _rnd.uniform(-RAND_PITCH_DEG, +RAND_PITCH_DEG)
    y  = _rnd.uniform(-RAND_YAW_DEG,   +RAND_YAW_DEG)
    drx,dry,drz = rpy_combo_to_rvec(r, p, y, order="XYZ")
    return [dx, dy, dz, drx, dry, drz]


# ------------------ main: N repeated taps around start ------------------ #
# Anchor pose (keep a copy to ensure every tap is relative to ORIGINAL)
start_pose = rtde_r.getActualTCPPose() if USE_CURRENT_POSE_AS_START else EXPLICIT_START_POSE
orig_pose  = list(start_pose)

# Move to a safe hover above the original start before beginning
orig_hover = add_z(orig_pose, HOVER_LIFT_Z)
if not safe_moveL(rtde_c, rtde_r, orig_hover, TAP_SPEED, TAP_ACCEL):
    print("[TAPS] Could not reach original hover (floor guard). Aborting.")
else:
    for i in range(1, N_TAPS+1):
        # Sample a random delta around the ORIGINAL pose (first tap = zero delta)
        delta_tool = sample_delta_toolframe(first=(i == 1))
        target     = compose_pose(orig_pose, delta_tool)

        # Define hover above this target and tap depth (optionally a bit deeper)
        target_hover   = add_z(target, HOVER_LIFT_Z)
        tap_pose       = list(target)
        tap_pose[2]   -= EXTRA_TAP_DEPTH  # push slightly further if desired

        print(f"[Tap {i}/{N_TAPS}] target={ [round(v,6) for v in target] }")

        # go to hover for this tap
        if not safe_moveL(rtde_c, rtde_r, target_hover, TAP_SPEED, TAP_ACCEL):
            print("[TAPS] Move to per-tap hover rejected/failed. Skipping tap.")
            continue

        # descend to tap (straight-line Cartesian). If you have contact logic,
        # you can replace this with your lower-until-contact routine.
        if not safe_moveL(rtde_c, rtde_r, tap_pose, TAP_SPEED, TAP_ACCEL):
            print("[TAPS] Move to tap depth rejected/failed. Skipping.")
            # try to retreat to hover to keep sequence alive
            safe_moveL(rtde_c, rtde_r, target_hover, TAP_SPEED, TAP_ACCEL)
            continue

        # dwell briefly at tap depth
        time.sleep(TAP_DWELL)

        # retreat back to hover
        if not safe_moveL(rtde_c, rtde_r, target_hover, TAP_SPEED, TAP_ACCEL):
            print("[TAPS] Retreat to hover rejected/failed. Stopping.")
            break

    # Return to original anchor pose (hover -> anchor)
    print("[TAPS] Returning to original anchor.")
    if not safe_moveL(rtde_c, rtde_r, orig_hover, TAP_SPEED, TAP_ACCEL):
        print("[TAPS] Could not return to original hover (floor guard).")
    safe_moveL(rtde_c, rtde_r, orig_pose, TAP_SPEED, TAP_ACCEL)
