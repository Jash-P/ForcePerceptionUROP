import time
import csv
import json
import serial
import rtde_control
import rtde_receive

# --- Robot & IO ---
ROBOT_IP = "169.254.150.50"
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# --- Motion ---
HOVER_OFFSET_Z = 0.10     # 10 cm up in Z
SPEED = 0.25              # m/s
ACCEL = 0.25              # m/s^2
CYCLES = 10

# --- Dwell/capture ---
DWELL_TOTAL_S = 3.0              # total pause at each pose
SETTLE_BEFORE_CAPTURE_S = 2.0    # capture at t = 2 s into dwell
SERIAL_READ_WINDOW_S = 1.5       # collect latest line within this window
CSV_PATH = "ten_repeated_taps.csv"

HEADER = ["hovered", "lowered", "lowered_actual_tcp", "lowered_target_tcp"]

# -------------------- Serial helpers (robust) --------------------
def open_serial(port, baudrate, timeout=0.25):
    """
    Short timeout so loops can sample multiple lines within the read window.
    """
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    # Clear any very old bytes on startup only
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def read_latest_line(ser, window_s=SERIAL_READ_WINDOW_S):
    """
    Read as many complete lines as arrive within window_s and return the last one.
    Returns "" if no full line arrived.
    """
    deadline = time.time() + window_s
    last = ""
    while time.time() < deadline:
        line = ser.readline()  # respects ser.timeout; returns b"" on timeout
        if line:
            last = line.strip().decode("utf-8", errors="replace")
    return last

def drain_pending_lines(ser, max_lines=200):
    """
    Gently drain pending lines (without nuking newly arriving ones).
    Stops when in_waiting is empty or after max_lines safeguards.
    """
    count = 0
    while ser.in_waiting and count < max_lines:
        _ = ser.readline()
        count += 1

def dwell_and_capture_serial(ser, dwell_total=DWELL_TOTAL_S, settle_s=SETTLE_BEFORE_CAPTURE_S):
    """
    For the hovered position: dwell, capture one serial line at settle time.
    Avoids hard flush at the moment of capture and returns the latest line seen.
    """
    t0 = time.time()

    # Approach the settle moment, then drain stale lines just before it
    if settle_s > 0.25:
        time.sleep(settle_s - 0.25)
        drain_pending_lines(ser)
        time.sleep(0.05)  # small guard for a fresh sample to arrive
        captured = read_latest_line(ser)
    else:
        time.sleep(settle_s)
        captured = read_latest_line(ser)

    # Finish the dwell precisely
    remain = max(0.0, dwell_total - (time.time() - t0))
    if remain > 0:
        time.sleep(remain)

    return captured

def dwell_and_capture_serial_and_poses(ser, rtde_r, dwell_total=DWELL_TOTAL_S, settle_s=SETTLE_BEFORE_CAPTURE_S):
    """
    For the lowered position: dwell, capture serial (latest line) and robot poses
    at the same time slice.
    """
    t0 = time.time()

    if settle_s > 0.25:
        time.sleep(settle_s - 0.25)
        drain_pending_lines(ser)
        time.sleep(0.05)
        lowered_csv = read_latest_line(ser)
    else:
        time.sleep(settle_s)
        lowered_csv = read_latest_line(ser)

    # Sample robot poses as close as possible to the serial capture
    actual_tcp = rtde_r.getActualTCPPose()
    try:
        target_tcp = rtde_r.getTargetTCPPose()
    except AttributeError:
        target_tcp = None  # fallback handled below

    remain = max(0.0, dwell_total - (time.time() - t0))
    if remain > 0:
        time.sleep(remain)

    return lowered_csv, actual_tcp, target_tcp

# -------------------- Main routine --------------------
def main():
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    ser = open_serial(SERIAL_PORT, BAUDRATE, timeout=0.25)

    # Prepare CSV header if new
    try:
        need_header = False
        try:
            with open(CSV_PATH, "r", newline="") as f:
                need_header = f.readline().strip() == ""
        except FileNotFoundError:
            need_header = True

        if need_header:
            with open(CSV_PATH, "w", newline="") as f:
                csv.writer(f).writerow(HEADER)

        # Compute poses
        pose_init = rtde_r.getActualTCPPose()  # [x,y,z,rx,ry,rz]
        pose_up = pose_init.copy()
        pose_up[2] += HOVER_OFFSET_Z

        for i in range(CYCLES):
            print(f"Cycle {i+1}/{CYCLES}")

            # Move up → dwell & capture (hovered)
            rtde_c.moveL(pose_up, SPEED, ACCEL)
            hovered_csv = dwell_and_capture_serial(ser)

            # Move down → dwell & capture (lowered + poses)
            rtde_c.moveL(pose_init, SPEED, ACCEL)
            lowered_csv, actual_tcp, target_tcp = dwell_and_capture_serial_and_poses(ser, rtde_r)

            actual_tcp_str = json.dumps(actual_tcp, separators=(",", ":")) if actual_tcp is not None else ""
            target_tcp_str = (
                json.dumps(target_tcp, separators=(",", ":"))
                if target_tcp is not None
                else json.dumps(pose_init, separators=(",", ":"))
            )

            # Append one row
            with open(CSV_PATH, "a", newline="") as f:
                csv.writer(f).writerow([hovered_csv, lowered_csv, actual_tcp_str, target_tcp_str])

        rtde_c.stopScript()
        print(f"Done. Saved to {CSV_PATH}")

    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            rtde_c.disconnect()
        except Exception:
            pass

if __name__ == "__main__":
    main()
