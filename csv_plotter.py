#!/usr/bin/env python3
import argparse
import csv
import os
from typing import List, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

COL_HOVERED = "hovered"
COL_LOWERED = "lowered"

def is_single_tap_csv(path: str) -> bool:
    base = os.path.basename(path).lower()
    return base.endswith("_different_taps.csv")

def parse_series(cell: str) -> List[float]:
    if cell is None:
        return []
    cell = cell.strip()
    if not cell:
        return []
    out: List[float] = []
    for part in cell.split(","):
        p = part.strip()
        if p == "":
            continue
        try:
            out.append(float(p))
        except ValueError:
            continue
    return out

def read_rows(path: str) -> Tuple[List[List[float]], List[List[float]]]:
    hovered_list: List[List[float]] = []
    lowered_list: List[List[float]] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        headers = {h.lower().strip(): h for h in (reader.fieldnames or [])}
        def col(key): return headers.get(key, key)
        for row in reader:
            hovered = parse_series(row.get(col(COL_HOVERED), ""))
            lowered = parse_series(row.get(col(COL_LOWERED), ""))
            if not hovered and not lowered: 
                continue
            hovered_list.append(hovered)
            lowered_list.append(lowered)
    return hovered_list, lowered_list

def compute_global_limits(hovered_list, lowered_list):
    vals = []
    for series in hovered_list + lowered_list:
        vals.extend(series)
    if not vals:
        return (0, 1)
    vmin, vmax = min(vals), max(vals)
    pad = 0.05 * (vmax - vmin if vmax > vmin else 1.0)
    return vmin - pad, vmax + pad

def compute_max_len(hovered_list, lowered_list) -> int:
    max_h = max([len(s) for s in hovered_list], default=0)
    max_l = max([len(s) for s in lowered_list], default=0)
    return max(max_h, max_l, 1)

def pad_series(series: List[float], target_len: int) -> np.ndarray:
    arr = np.full(target_len, np.nan, dtype=float)
    n = min(len(series), target_len)
    if n > 0:
        arr[:n] = series[:n]
    return arr

def main():
    ap = argparse.ArgumentParser(description="Scroll through taps; one iteration/position per view with Hovered & Lowered side-by-side.")
    ap.add_argument("csv_path", help="Path to CSV (ten_repeated_taps.csv or <n>_different_taps.csv)")
    args = ap.parse_args()

    csv_path = args.csv_path
    if not os.path.exists(csv_path):
        raise SystemExit(f"File not found: {csv_path}")

    is_single = is_single_tap_csv(csv_path)
    label_prefix = "Position" if is_single else "Iteration"

    hovered_list, lowered_list = read_rows(csv_path)
    if not hovered_list and not lowered_list:
        raise SystemExit("No parsable hovered/lowered data found.")

    N = max(len(hovered_list), len(lowered_list))
    y_min, y_max = compute_global_limits(hovered_list, lowered_list)
    X_LEN = compute_max_len(hovered_list, lowered_list)
    X = np.arange(X_LEN)

    # --- Figure & Axes ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), dpi=120)
    ax_h, ax_l = axes
    plt.subplots_adjust(bottom=0.18)

    def plot_one(ax, series: List[float], title: str):
        ax.clear()
        y = pad_series(series, X_LEN)
        ax.plot(X, y, marker="o", linewidth=1.25)
        ax.set_title(title)
        ax.set_xlabel("Index")
        ax.set_ylabel("Value")
        ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
        ax.set_ylim(y_min, y_max)           # uniform Y across all graphs
        ax.set_xlim(0, X_LEN - 1)           # uniform X across all graphs

    def update_view(i: int):
        i = max(0, min(N - 1, i))
        h = hovered_list[i] if i < len(hovered_list) else []
        l = lowered_list[i] if i < len(lowered_list) else []
        plot_one(ax_h, h, "Hovered")
        plot_one(ax_l, l, "Lowered")
        fig.suptitle(f"{os.path.basename(csv_path)} — {label_prefix} {i+1}/{N}", fontsize=14, y=0.98)
        fig.canvas.draw_idle()

    # Initial draw
    idx = 0
    update_view(idx)

    # --- Slider ---
    ax_slider = plt.axes([0.12, 0.08, 0.70, 0.04])
    s_idx = Slider(ax=ax_slider, label=f"{label_prefix}", valmin=1, valmax=N, valinit=idx+1, valstep=1)
    s_idx.on_changed(lambda val: update_view(int(val) - 1))

    # --- Buttons ---
    ax_prev = plt.axes([0.12, 0.02, 0.08, 0.045])
    ax_next = plt.axes([0.22, 0.02, 0.08, 0.045])
    b_prev = Button(ax_prev, "Prev")
    b_next = Button(ax_next, "Next")

    def go(delta):
        new_i = int(s_idx.val) - 1 + delta
        new_i = max(0, min(N - 1, new_i))
        if new_i != int(s_idx.val) - 1:
            s_idx.set_val(new_i + 1)

    b_prev.on_clicked(lambda event: go(-1))
    b_next.on_clicked(lambda event: go(1))

    # --- Keyboard arrows ---
    def on_key(event):
        if event.key in ("left", "a"):
            go(-1)
        elif event.key in ("right", "d"):
            go(1)
    fig.canvas.mpl_connect("key_press_event", on_key)

    # --- Mouse wheel ---
    def on_scroll(event):
        if event.button == "up":
            go(-1)
        elif event.button == "down":
            go(1)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    plt.show()

if __name__ == "__main__":
    main()
