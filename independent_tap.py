import os
import time
import csv
import json
import glob
import serial
import rtde_control
import rtde_receive

ROBOT_IP = "169.254.150.50"
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# Motion
HOVER_OFFSET_Z = 0.10     # 10 cm
SPEED = 0.25              # m/s
ACCEL = 0.25              # m/s^2

# Dwell/capture
DWELL_TOTAL_S = 3.0
SETTLE_BEFORE_CAPTURE_S = 2.0
SERIAL_READ_WINDOW_S = 1.5  # try 2.0–3.0 if device is slower

HEADER = ["hovered", "lowered", "lowered_actual_tcp", "lowered_target_tcp"]

# -------------------- Serial helpers (robust) --------------------
def open_serial(port, baudrate, timeout=0.25):
    """
    Short timeout so we can loop and collect multiple lines within a window.
    """
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    # Clear very old bytes once at startup
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
        line = ser.readline()  # returns b"" on timeout
        if line:
            last = line.strip().decode("utf-8", errors="replace")
    return last

def drain_pending_lines(ser, max_lines=200):
    """
    Gently drain pending lines without nuking a just-arrived sample.
    """
    count = 0
    while getattr(ser, "in_waiting", 0) and count < max_lines:
        ser.readline()
        count += 1

def dwell_and_capture_serial(ser, dwell_total=DWELL_TOTAL_S, settle_s=SETTLE_BEFORE_CAPTURE_S):
    """
    Hover capture: dwell for dwell_total, capture one serial line at ~settle_s.
    Avoid hard flush at capture moment; return latest complete line seen.
    """
    t0 = time.time()
    if settle_s > 0.25:
        time.sleep(settle_s - 0.25)
        drain_pending_lines(ser)
        time.sleep(0.05)
        captured = read_latest_line(ser)
    else:
        time.sleep(settle_s)
        captured = read_latest_line(ser)

    remain = max(0.0, dwell_total - (time.time() - t0))
    if remain > 0:
        time.sleep(remain)
    return captured

def dwell_and_capture_serial_and_poses(ser, rtde_r, dwell_total=DWELL_TOTAL_S, settle_s=SETTLE_BEFORE_CAPTURE_S):
    """
    Lowered capture: same as above, plus snapshot actual & target TCP poses.
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

    actual_tcp = rtde_r.getActualTCPPose()
    try:
        target_tcp = rtde_r.getTargetTCPPose()
    except AttributeError:
        target_tcp = None

    remain = max(0.0, dwell_total - (time.time() - t0))
    if remain > 0:
        time.sleep(remain)

    return lowered_csv, actual_tcp, target_tcp

# -------------------- CSV helpers (append and rename) --------------------
def find_current_csv():
    """Return (filename, n_rows_ex_header). If none, return (None, 0)."""
    files = glob.glob("*_different_taps.csv")
    if not files:
        return None, 0

    # Choose the highest n in name like '12_different_taps.csv'
    best = None
    best_n = -1
    for f in files:
        try:
            n = int(os.path.basename(f).split("_")[0])
            if n > best_n:
                best = f
                best_n = n
        except ValueError:
            continue

    rows = 0
    try:
        with open(best, newline="") as fh:
            reader = csv.reader(fh)
            for i, row in enumerate(reader):
                if i == 0 and row == HEADER:
                    continue
                rows += 1
    except FileNotFoundError:
        return None, 0

    return best, rows

def ensure_file_and_get_target(rows_after_append):
    """Desired filename like 'N_different_taps.csv'."""
    return f"{rows_after_append}_different_taps.csv"

def append_row_and_rename(row_values):
    """
    Append one row to the current n_different_taps.csv (creating it if needed),
    then rename the file so n matches the new total row count.
    """
    current_file, current_rows = find_current_csv()

    # If no file, start a new one at n=0 then append → becomes 1
    if current_file is None:
        current_file = "0_different_taps.csv"
        with open(current_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(HEADER)

    # Append the row
    with open(current_file, mode="a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row_values)

    new_total_rows = current_rows + 1
    desired_name = ensure_file_and_get_target(new_total_rows)

    if os.path.basename(current_file) != desired_name:
        if os.path.exists(desired_name):
            os.remove(desired_name)
        os.rename(current_file, desired_name)

    return desired_name, new_total_rows

# -------------------- Main (single tap) --------------------
def main():
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    ser = open_serial(SERIAL_PORT, BAUDRATE, timeout=0.25)

    try:
        # Establish poses
        pose_init = rtde_r.getActualTCPPose()
        pose_up = pose_init.copy()
        pose_up[2] += HOVER_OFFSET_Z

        # Hover → capture serial at ~t=2 s
        rtde_c.moveL(pose_up, SPEED, ACCEL)
        hovered_csv = dwell_and_capture_serial(ser)

        # Lower → capture serial + poses at ~t=2 s
        rtde_c.moveL(pose_init, SPEED, ACCEL)
        lowered_csv, actual_tcp, target_tcp = dwell_and_capture_serial_and_poses(ser, rtde_r)

        actual_tcp_str = json.dumps(actual_tcp, separators=(",", ":")) if actual_tcp is not None else ""
        target_tcp_str = json.dumps(target_tcp, separators=(",", ":")) if target_tcp is not None else json.dumps(pose_init, separators=(",", ":"))

        # Append and rename so n reflects total row count
        out_file, total_rows = append_row_and_rename(
            [hovered_csv, lowered_csv, actual_tcp_str, target_tcp_str]
        )

        print(f"Appended 1 row. File is now '{out_file}' with {total_rows} data row(s).")
        rtde_c.stopScript()

    finally:
        try: ser.close()
        except Exception: pass
        try: rtde_c.disconnect()
        except Exception: pass

if __name__ == "__main__":
    main()
