#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EIT Δ (pressed - hovered) → UR5 TCP Pose — Ridge (with Quaternion Orientation)

WHAT CHANGED (vs earlier linear script)
--------------------------------------
- Orientation target switched from axis–angle (rx,ry,rz) to **unit quaternion**
  q = (w,x,y,z). This removes angle wrapping and singularities.
- Cross-validation selects alpha using a **composite score**:
    score = pos_RMSE_m + ori_weight * mean_geodesic_deg
  where pos_RMSE_m is the RMSE over x,y,z (metres) and mean_geodesic_deg is
  the mean geodesic angle error (degrees) between predicted and true quats.
  You can set `--ori_weight` (default 0.01 m/deg).
- Reports & learning curves now include **position RMSE (m)** and **geodesic
  orientation error (deg)** for both TRAIN/TEST.
- The saved model still outputs a 7D vector [x,y,z,qw,qx,qy,qz]. Helpers are
  provided to get back axis–angle if needed.

USAGE
-----
python train_eit_pose_regression.py --csv /path/to/logs.csv \
  --min_Fz 50 --test_size 0.2 --cv_folds 5 \
  --alpha_grid 0.1,0.3,1,3,10,30,100 \
  --ori_weight 0.01 \
  --lc_points auto --outdir ./outputs
"""
import argparse, json, os
from datetime import datetime
import numpy as np, pandas as pd, joblib
from sklearn.model_selection import GroupKFold, GroupShuffleSplit
from sklearn.linear_model import Ridge
from sklearn.preprocessing import StandardScaler
from sklearn.compose import TransformedTargetRegressor
from sklearn.pipeline import Pipeline
from sklearn.metrics import mean_squared_error, mean_absolute_error

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from tqdm.auto import tqdm
    _TQDM_AVAILABLE = True
except Exception:
    _TQDM_AVAILABLE = False
    class tqdm:
        def __init__(self,*a,**k): pass
        def update(self,*a,**k): pass
        def close(self): pass
        def __enter__(self): return self
        def __exit__(self,*e): pass
        def __iter__(self): return iter([])

RANDOM_STATE = 42

# ---------- rotation utilities ----------
def axis_angle_to_quat(r):
    r = np.asarray(r, dtype=float)
    theta = np.linalg.norm(r, axis=-1, keepdims=True)
    small = theta < 1e-12
    half = 0.5 * theta
    w = np.cos(half)
    s = np.zeros_like(theta); s[~small] = np.sin(half[~small]) / theta[~small]
    v = r * s
    q = np.concatenate([w, v], axis=-1)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)

def quat_geodesic_deg(qp, qt):
    qp = quat_normalize(qp); qt = quat_normalize(qt)
    dot = np.sum(qp * qt, axis=-1)
    dot = np.clip(np.abs(dot), -1.0, 1.0)
    ang = 2.0 * np.arccos(dot)
    return ang * 180.0 / np.pi

def quat_to_axis_angle(q):
    q = quat_normalize(q)
    w,x,y,z = q[...,0],q[...,1],q[...,2],q[...,3]
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sqrt(1.0 - w*w)
    axis = np.zeros(q.shape[:-1] + (3,))
    small = s < 1e-12
    axis[~small,0] = x[~small]/s[~small]
    axis[~small,1] = y[~small]/s[~small]
    axis[~small,2] = z[~small]/s[~small]
    return axis * angle[...,None]

def quat_to_axis_angle_ur(q):
    """
    Quaternion (w,x,y,z) -> axis-angle vector r (UR convention).
    Canonicalizes q so w>=0 and maps angle to (−π, π].
    """
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True).clip(1e-12, None)

    # Canonicalize: q and -q are same rotation; pick w>=0
    flip = q[..., 0] < 0
    if np.any(flip): q[flip] *= -1.0

    w, x, y, z = q[...,0], q[...,1], q[...,2], q[...,3]
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))         # angle ∈ [0, π]
    s = np.sqrt(np.maximum(1.0 - w*w, 1e-12))
    axis = np.stack([x/s, y/s, z/s], axis=-1)

    r = axis * angle[..., None]                            # axis–angle vector
    # If angle is exactly π, choose the (−π) representative (UR-style uniqueness)
    near_pi = np.isclose(angle, np.pi)
    if np.any(near_pi): r[near_pi] *= -1.0
    return r

# ---------- metrics & plots (position only) ----------
def per_axis_pos_metrics(y_true_pos, y_pred_pos):
    names = ["act_tcp_x","act_tcp_y","act_tcp_z"]
    mse = np.mean((y_true_pos - y_pred_pos) ** 2, axis=0)
    rmse = np.sqrt(mse)
    mae = np.mean(np.abs(y_true_pos - y_pred_pos), axis=0)
    return {n: {"rmse": float(r), "mae": float(m)} for n, r, m in zip(names, rmse, mae)}

def format_pos_metrics(metrics):
    return "\n".join(f"{k:>11s}  RMSE={v['rmse']:.4f} m   MAE={v['mae']:.4f} m" for k,v in metrics.items())

def save_pred_vs_true_plots_pos(y_true_pos, y_pred_pos, outdir):
    names = ["act_tcp_x","act_tcp_y","act_tcp_z"]
    for i, name in enumerate(names):
        plt.figure(); plt.scatter(y_true_pos[:,i], y_pred_pos[:,i], s=10)
        lims = [min(y_true_pos[:,i].min(), y_pred_pos[:,i].min()),
                max(y_true_pos[:,i].max(), y_pred_pos[:,i].max())]
        plt.plot(lims, lims); plt.xlabel(f"True {name}"); plt.ylabel(f"Pred {name}")
        plt.title(f"Predicted vs True — {name}"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(outdir, f"pvstrue_{name}.png"), dpi=140)

        plt.figure(); resid = y_pred_pos[:,i] - y_true_pos[:,i]
        plt.hist(resid, bins=50); plt.xlabel(f"Residual ({name})"); plt.ylabel("Count")
        plt.title(f"Residuals — {name}"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(outdir, f"residuals_{name}.png"), dpi=140)

def binned_stats(x, err_vals, bins):
    x = np.asarray(x); err_vals = np.asarray(err_vals)
    idx = np.digitize(x, bins) - 1
    centers = 0.5*(bins[:-1] + bins[1:])
    out = []
    for b in range(len(centers)):
        m = idx == b
        out.append(float(np.mean(np.abs(err_vals[m]))) if np.any(m) else np.nan)
    return centers, np.array(out)

def save_error_vs_slice_plots(df_test, y_true_pos, y_pred_pos, geo_deg, outdir):
    pos_abs = np.abs(y_pred_pos - y_true_pos)
    pos_mae_sample = np.mean(pos_abs, axis=1)

    if "Fz" in df_test.columns:
        fz = df_test["Fz"].values
        bins = np.linspace(np.nanmin(fz), np.nanmax(fz), 8)
        c, mae_pos = binned_stats(fz, pos_mae_sample, bins)
        _, mae_geo = binned_stats(fz, geo_deg, bins)
        plt.figure(); plt.plot(c, mae_pos, "o-"); plt.xlabel("Fz"); plt.ylabel("Mean |pos error| (m)")
        plt.title("Position error vs Fz"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(outdir, "error_vs_Fz_position.png"), dpi=140)
        plt.figure(); plt.plot(c, mae_geo, "o-"); plt.xlabel("Fz"); plt.ylabel("Geodesic ori error (deg)")
        plt.title("Orientation error vs Fz"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(outdir, "error_vs_Fz_orientation.png"), dpi=140)

    yvals = df_test["act_tcp_y"].values if "act_tcp_y" in df_test.columns else y_true_pos[:,1]
    ybins = np.linspace(np.nanmin(yvals), np.nanmax(yvals), 8)
    c, mae_pos = binned_stats(yvals, pos_mae_sample, ybins)
    _, mae_geo = binned_stats(yvals, geo_deg, ybins)
    plt.figure(); plt.plot(c, mae_pos, "o-"); plt.xlabel("act_tcp_y"); plt.ylabel("Mean |pos error| (m)")
    plt.title("Position error vs act_tcp_y"); plt.grid(True, linestyle="--", lw=0.5)
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "error_vs_y_position.png"), dpi=140)
    plt.figure(); plt.plot(c, mae_geo, "o-"); plt.xlabel("act_tcp_y"); plt.ylabel("Geodesic ori error (deg)")
    plt.title("Orientation error vs act_tcp_y"); plt.grid(True, linestyle="--", lw=0.5)
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "error_vs_y_orientation.png"), dpi=140)

    if "act_tcp_ry" in df_test.columns:
        pitch_deg = df_test["act_tcp_ry"].values * 180/np.pi
        pbins = np.linspace(np.nanmin(pitch_deg), np.nanmax(pitch_deg), 8)
        c, mae_pos = binned_stats(pitch_deg, pos_mae_sample, pbins)
        _, mae_geo = binned_stats(pitch_deg, geo_deg, pbins)
        plt.figure(); plt.plot(c, mae_pos, "o-"); plt.xlabel("act_tcp_ry (deg)"); plt.ylabel("Mean |pos error| (m)")
        plt.title("Position error vs pitch"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(outdir, "error_vs_pitch_position.png"), dpi=140)
        plt.figure(); plt.plot(c, mae_geo, "o-"); plt.xlabel("act_tcp_ry (deg)"); plt.ylabel("Geodesic ori error (deg)")
        plt.title("Orientation error vs pitch"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(outdir, "error_vs_pitch_orientation.png"), dpi=140)

def save_axis_angle_plots(r_true_rad, r_pred_rad, outdir, use_degrees=True):
    """Make Pred vs True + residual hist for rx,ry,rz.
    Inputs are in radians; plots default to degrees for readability.
    """
    import os
    import numpy as np
    import matplotlib.pyplot as plt

    names = ["act_tcp_rx","act_tcp_ry","act_tcp_rz"]
    r_true = np.asarray(r_true_rad)
    r_pred = np.asarray(r_pred_rad)

    if use_degrees:
        r_true = r_true * 180.0/np.pi
        r_pred = r_pred * 180.0/np.pi
        unit = "deg"
    else:
        unit = "rad"

    for i, name in enumerate(names):
        # Predicted vs True
        plt.figure()
        plt.scatter(r_true[:, i], r_pred[:, i], s=10)
        lims = [min(r_true[:, i].min(), r_pred[:, i].min()),
                max(r_true[:, i].max(), r_pred[:, i].max())]
        plt.plot(lims, lims)
        plt.xlabel(f"True {name} ({unit})")
        plt.ylabel(f"Pred {name} ({unit})")
        plt.title(f"Predicted vs True — {name} ({unit})")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"pvstrue_{name}_{unit}.png"), dpi=140)

        # Residual histogram
        plt.figure()
        resid = r_pred[:, i] - r_true[:, i]
        plt.hist(resid, bins=50)
        plt.xlabel(f"Residual ({name}) [{unit}]")
        plt.ylabel("Count")
        plt.title(f"Residuals — {name} ({unit})")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"residuals_{name}_{unit}.png"), dpi=140)

def _vector_pos_err(y_true_xyz, y_pred_xyz):
    """Per-sample Euclidean position error [m] and per-axis abs errors [m]."""
    dif = y_pred_xyz - y_true_xyz
    err_vec = np.linalg.norm(dif, axis=1)
    err_abs = np.abs(dif)
    return err_vec, err_abs

def _bin_stats(x, mask, y_true_xyz, y_pred_xyz, nbins=12):
    lo, hi = np.nanpercentile(x[mask], 1), np.nanpercentile(x[mask], 99)
    bins = np.linspace(lo, hi, nbins+1)
    centers = 0.5*(bins[:-1] + bins[1:])
    rmse, mae, n = [], [], []
    per_axis_rmse = []
    idx = np.digitize(x, bins) - 1
    for b in range(nbins):
        m = mask & (idx == b)
        if not np.any(m):
            rmse.append(np.nan); mae.append(np.nan); n.append(0)
            per_axis_rmse.append([np.nan, np.nan, np.nan])
            continue
        d = y_pred_xyz[m] - y_true_xyz[m]
        rmse.append(float(np.sqrt(np.mean(np.sum(d**2, axis=1)))))  # vector RMSE
        mae.append(float(np.mean(np.linalg.norm(d, axis=1))))
        n.append(int(m.sum()))
        per_axis_rmse.append(np.sqrt(np.mean(d**2, axis=0)).tolist())
    return centers, np.array(rmse), np.array(mae), np.array(n), np.array(per_axis_rmse)

def _spearman_cheap(x, y):
    """Tie-agnostic Spearman approximation without SciPy."""
    rx = np.argsort(np.argsort(x))
    ry = np.argsort(np.argsort(y))
    if rx.std() == 0 or ry.std() == 0: return np.nan
    return float(np.corrcoef(rx, ry)[0,1])

def analyse_pos_robustness_vs_orientation(df_test, pos_true, pos_pred, outdir, nbins=12,
                                          fixed_angle_xlim=None, oneplot_stat="rmse"):  # NEW param
    """
    Creates:
      - robustness_xyz_vs_{roll,pitch,yaw}.png  (all share same y-scale)
      - robustness_xyz_vs_{roll,pitch,yaw}.csv
      - robustness_xyz_heatmap_pitch_yaw.png
      - robustness_xyz_vs_angles_stats.json
      - robustness_xyz_vs_rpy_oneplot.png         # NEW (all curves on one plot)
      - robustness_xyz_vs_rpy_oneplot.csv         # NEW (binned values used in the one-plot)

    fixed_angle_xlim: set to a tuple like (-180, 180) to use the SAME x-limits on all plots.
    oneplot_stat: "rmse" (default) or "mae" for the combined figure.
    """
    import os, json
    import numpy as np
    import pandas as pd
    import matplotlib.pyplot as plt

    os.makedirs(outdir, exist_ok=True)

    # True angles from CSV (radians) -> degrees
    roll = df_test["act_tcp_rx"].values * 180/np.pi
    pitch = df_test["act_tcp_ry"].values * 180/np.pi
    yaw = df_test["act_tcp_rz"].values * 180/np.pi

    # Position errors
    dif = pos_pred - pos_true
    err_vec = np.linalg.norm(dif, axis=1)

    # Keep finite
    mask = np.isfinite(roll) & np.isfinite(pitch) & np.isfinite(yaw) & np.isfinite(err_vec)
    roll, pitch, yaw = roll[mask], pitch[mask], yaw[mask]
    pos_true, pos_pred = pos_true[mask], pos_pred[mask]

    def _bin_stats(x, nbins):
        # Use robust percentiles to define bin edges (per-angle), then compute vector RMSE/MAE
        lo, hi = np.nanpercentile(x, 1), np.nanpercentile(x, 99)
        bins = np.linspace(lo, hi, nbins+1)
        centers = 0.5*(bins[:-1] + bins[1:])
        idx = np.digitize(x, bins) - 1
        rmse, mae, n, per_axis_rmse = [], [], [], []
        for b in range(nbins):
            m = (idx == b)
            if not np.any(m):
                rmse.append(np.nan); mae.append(np.nan); n.append(0)
                per_axis_rmse.append([np.nan, np.nan, np.nan]); continue
            d = pos_pred[m] - pos_true[m]
            vdist = np.linalg.norm(d, axis=1)
            rmse.append(float(np.sqrt(np.mean(vdist**2))))
            mae.append(float(np.mean(vdist)))
            n.append(int(m.sum()))
            per_axis_rmse.append(np.sqrt(np.mean(d**2, axis=0)).tolist())
        return centers, np.array(rmse), np.array(mae), np.array(n), np.array(per_axis_rmse)

    # Compute all three first
    results = {}
    for ang, name in [(roll,"roll"), (pitch,"pitch"), (yaw,"yaw")]:
        centers, rmse, mae, n, per_axis = _bin_stats(ang, nbins)
        results[name] = dict(centers=centers, rmse=rmse, mae=mae, n=n, per_axis=per_axis)

        # Save CSVs now
        rows = []
        for i, c in enumerate(centers):
            rows.append({
                f"{name}_center_deg": float(c),
                "rmse_m": float(rmse[i]),
                "mae_m": float(mae[i]),
                "n": int(n[i]),
                "rmse_x_m": float(per_axis[i,0]),
                "rmse_y_m": float(per_axis[i,1]),
                "rmse_z_m": float(per_axis[i,2]),
            })
        pd.DataFrame(rows).to_csv(os.path.join(outdir, f"robustness_xyz_vs_{name}.csv"), index=False)

    # ---- Enforce identical y-scale across the three figures ----
    y_vals = []
    for name in ["roll","pitch","yaw"]:
        y_vals.append(results[name]["rmse"])
        y_vals.append(results[name]["mae"])
    global_ymax = np.nanmax(np.concatenate(y_vals)) if len(y_vals) else np.nan
    if not np.isfinite(global_ymax):  # fallback
        global_ymax = 1.0
    y_lim = (0.0, float(global_ymax)*1.05)

    # Optional: same x-limits
    if fixed_angle_xlim is not None:
        x_lim = (float(fixed_angle_xlim[0]), float(fixed_angle_xlim[1]))
    else:
        x_lim = None

    # Plot with common y-limits (and optional common x-limits)
    for name in ["roll","pitch","yaw"]:
        centers = results[name]["centers"]
        rmse    = results[name]["rmse"]
        mae     = results[name]["mae"]

        plt.figure()
        plt.plot(centers, rmse, "o-", label="Vector RMSE (m)")
        plt.plot(centers, mae,  "o-", label="Vector MAE (m)")
        if x_lim is not None:
            plt.xlim(*x_lim)
        plt.ylim(*y_lim)
        plt.xlabel(f"{name.capitalize()} (deg)")
        plt.ylabel("Position error (m)")
        plt.title(f"Position error vs {name} (common y-scale)")
        plt.grid(True, linestyle="--", lw=0.5)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"robustness_xyz_vs_{name}.png"), dpi=160)

    # --- NEW: one combined plot (roll, pitch, yaw on same axes) ---
    stat_key = "rmse" if str(oneplot_stat).lower() != "mae" else "mae"
    ylab = "Position error (vector RMSE, m)" if stat_key == "rmse" else "Position error (vector MAE, m)"

    # CSV for combined plot
    pd.DataFrame({
        "roll_center_deg":  results["roll"]["centers"],
        f"roll_{stat_key}_m":  results["roll"][stat_key],
        "pitch_center_deg": results["pitch"]["centers"],
        f"pitch_{stat_key}_m": results["pitch"][stat_key],
        "yaw_center_deg":   results["yaw"]["centers"],
        f"yaw_{stat_key}_m":   results["yaw"][stat_key],
        "count_roll":  results["roll"]["n"],
        "count_pitch": results["pitch"]["n"],
        "count_yaw":   results["yaw"]["n"],
    }).to_csv(os.path.join(outdir, "robustness_xyz_vs_rpy_oneplot.csv"), index=False)

    # Plot (uses same y_lim as above; x_lim if provided)
    plt.figure()
    plt.plot(results["roll"]["centers"],  results["roll"][stat_key],  "o-", label="roll",  linewidth=1.6)
    plt.plot(results["pitch"]["centers"], results["pitch"][stat_key], "o-", label="pitch", linewidth=1.6)
    plt.plot(results["yaw"]["centers"],   results["yaw"][stat_key],   "o-", label="yaw",   linewidth=1.6)
    if x_lim is not None:
        plt.xlim(*x_lim)
    plt.ylim(*y_lim)
    plt.xlabel("Angle (deg)")
    plt.ylabel(ylab)
    plt.title("Positional error vs roll / pitch / yaw (common axes)")
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "robustness_xyz_vs_rpy_oneplot.png"), dpi=160)
    # ---------------------------------------------------------------

    # --- Keep the 2D heatmap as before (optional) ---
    nb = int(np.sqrt(max(9, nbins*nbins//3)))
    p_edges = np.linspace(np.nanpercentile(pitch,1), np.nanpercentile(pitch,99), nb+1)
    y_edges = np.linspace(np.nanpercentile(yaw,  1), np.nanpercentile(yaw,  99), nb+1)
    H = np.full((nb, nb), np.nan)
    pi = np.digitize(pitch, p_edges)-1
    yi = np.digitize(yaw,   y_edges)-1
    for i in range(nb):
        for j in range(nb):
            m = (pi==i) & (yi==j)
            if np.any(m):
                d = pos_pred[m] - pos_true[m]
                H[i,j] = np.sqrt(np.mean(np.sum(d**2, axis=1)))
    plt.figure()
    extent = [y_edges[0], y_edges[-1], p_edges[0], p_edges[-1]]
    plt.imshow(np.flipud(H), aspect="auto", extent=[extent[0],extent[1],extent[2],extent[3]])
    cbar = plt.colorbar(); cbar.set_label("Vector RMSE (m)")
    plt.xlabel("Yaw (deg)"); plt.ylabel("Pitch (deg)")
    plt.title("Position RMSE heatmap vs Pitch × Yaw")
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "robustness_xyz_heatmap_pitch_yaw.png"), dpi=160)

    # Summary stats file (unchanged)
    stats = {
        "global_ymax_used_m": float(y_lim[1]),
        "oneplot_stat": stat_key,  # NEW: record which stat was plotted
    }
    with open(os.path.join(outdir, "robustness_xyz_vs_angles_stats.json"), "w") as f:
        json.dump(stats, f, indent=2)
    print("[robustness] Saved xyz-vs-angle plots with COMMON y-scale, CSVs, heatmap, and the combined RPY plot.")

# ---------- main ----------
def main():
    parser = argparse.ArgumentParser(description="Train EIT→pose linear model (Ridge) with quaternion orientation and diagnostics.")
    parser.add_argument("--csv", type=str, required=True)
    parser.add_argument("--test_size", type=float, default=0.20)
    parser.add_argument("--cv_folds", type=int, default=5)
    parser.add_argument("--contact_only", action="store_true")
    parser.add_argument("--min_Fz", type=float, default=50.0)
    parser.add_argument("--alpha_grid", type=str, default="0.1,0.3,1,3,10,30,100")
    parser.add_argument("--lc_points", type=str, default="auto")
    parser.add_argument("--ori_weight", type=float, default=0.01, help="Composite CV weight in m/deg (pos_RMSE_m + ori_weight*geo_deg).")
    parser.add_argument("--outdir", type=str, default="/mnt/data")
    parser.add_argument("--no_progress", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    df = pd.read_csv(args.csv)
    if 'contact_detected' in df.columns and args.contact_only:
        df = df[df['contact_detected']==1].copy()
    if 'Fz' not in df.columns:
        raise ValueError("Column 'Fz' not found in CSV.")
    pre = len(df); df = df[df['Fz'] >= args.min_Fz].copy()
    print(f"[filter] Kept {len(df)}/{pre} rows (Fz >= {args.min_Fz}).")

    # ΔEIT features
    eitb = [f"eitb_{i}" for i in range(256)]
    eita = [f"eita_{i}" for i in range(256)]
    for cols in (eitb, eita):
        miss = [c for c in cols if c not in df.columns]
        if miss: raise ValueError(f"Missing EIT cols, e.g. {miss[:5]}")
    X = (df[eita].values - df[eitb].values).astype(float)

    # Targets: xyz + quaternion
    for t in ["act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz"]:
        if t not in df.columns: raise ValueError(f"Missing target col: {t}")
    pos = df[["act_tcp_x","act_tcp_y","act_tcp_z"]].values.astype(float)
    rvec = df[["act_tcp_rx","act_tcp_ry","act_tcp_rz"]].values.astype(float)
    quat = axis_angle_to_quat(rvec)
    y = np.hstack([pos, quat])  # [x,y,z,qw,qx,qy,qz]

    good = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
    if not good.all():
        dropped = int((~good).sum())
        df = df.loc[good].reset_index(drop=True); X = X[good]; y = y[good]
        print(f"[info] Dropped {dropped} NaN/Inf rows.")

    # Group & split
    def build_pose_id(df, pos_bin_m=0.001, ang_bin_rad=0.05):
        qx = np.round(df['act_tcp_x'].values/pos_bin_m).astype(np.int64)
        qy = np.round(df['act_tcp_y'].values/pos_bin_m).astype(np.int64)
        qz = np.round(df['act_tcp_z'].values/pos_bin_m).astype(np.int64)
        qrx = np.round(df['act_tcp_rx'].values/ang_bin_rad).astype(np.int64)
        qry = np.round(df['act_tcp_ry'].values/ang_bin_rad).astype(np.int64)
        qrz = np.round(df['act_tcp_rz'].values/ang_bin_rad).astype(np.int64)
        return pd.Series([f"{a}_{b}_{c}_{d}_{e}_{f}" for a,b,c,d,e,f in zip(qx,qy,qz,qrx,qry,qrz)], index=df.index)

    pose_id = build_pose_id(df)
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=RANDOM_STATE)
    trainval_idx, test_idx = next(gss.split(X, y, groups=pose_id))
    X_trainval, y_trainval = X[trainval_idx], y[trainval_idx]
    X_test, y_test = X[test_idx], y[test_idx]
    g_trainval = pose_id.iloc[trainval_idx]
    print(f"[split] Train+Val={X_trainval.shape[0]}  Test={X_test.shape[0]}  Total={X.shape[0]}")

    # CV with composite score
    alphas = [float(s) for s in args.alpha_grid.split(",")]
    cv = GroupKFold(n_splits=args.cv_folds)
    results, best_score, best_alpha = [], np.inf, None
    total_cv = len(alphas)*args.cv_folds
    with tqdm(total=total_cv, desc="CV (alphas × folds)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for alpha in alphas:
            fold_scores = []
            for tr, va in cv.split(X_trainval, y_trainval, groups=g_trainval):
                X_tr, Y_tr = X_trainval[tr], y_trainval[tr]
                X_va, Y_va = X_trainval[va], y_trainval[va]
                inner = Pipeline([("scaler", StandardScaler()), ("ridge", Ridge(alpha=alpha, fit_intercept=True))])
                model = TransformedTargetRegressor(regressor=inner, transformer=StandardScaler())
                model.fit(X_tr, Y_tr)
                Y_hat = model.predict(X_va)
                pos_true, pos_pred = Y_va[:,:3], Y_hat[:,:3]
                quat_true, quat_pred = Y_va[:,3:], quat_normalize(Y_hat[:,3:])
                pos_rmse = float(np.sqrt(mean_squared_error(pos_true, pos_pred)))
                geo_deg  = float(quat_geodesic_deg(quat_pred, quat_true).mean())
                score = pos_rmse + args.ori_weight * geo_deg
                fold_scores.append(score); pbar.update(1)
            mean_score = float(np.mean(fold_scores))
            results.append({"alpha": alpha, "cv_score": mean_score})
            if mean_score < best_score: best_score, best_alpha = mean_score, alpha
    print("[cv] Composite: pos_RMSE_m + ori_weight*mean_geo_deg (lower is better)")
    for r in results:
        print(f"   alpha={r['alpha']:>6g}  score={r['cv_score']:.6f}")
    print(f"[cv] Best alpha: {best_alpha} (score={best_score:.6f}, ori_weight={args.ori_weight})")

    # Fit final
    final_inner = Pipeline([("scaler", StandardScaler()), ("ridge", Ridge(alpha=best_alpha, fit_intercept=True))])
    final_model = TransformedTargetRegressor(regressor=final_inner, transformer=StandardScaler())
    final_model.fit(X_trainval, y_trainval)

    # Test eval
    Y_hat = final_model.predict(X_test)
    pos_true, pos_pred = y_test[:,:3], Y_hat[:,:3]
    quat_true, quat_pred = y_test[:,3:], quat_normalize(Y_hat[:,3:])
    pos_overall_rmse = float(np.sqrt(mean_squared_error(pos_true, pos_pred)))
    pos_overall_mae  = float(mean_absolute_error(pos_true, pos_pred))
    pos_per_axis = per_axis_pos_metrics(pos_true, pos_pred)
    geo_errors_deg = quat_geodesic_deg(quat_pred, quat_true)
    geo_summary = {
        "mean_deg": float(np.mean(geo_errors_deg)),
        "median_deg": float(np.median(geo_errors_deg)),
        "p90_deg": float(np.percentile(geo_errors_deg, 90)),
        "p95_deg": float(np.percentile(geo_errors_deg, 95)),
    }
    print("\n[test] Position (xyz):")
    print(f"   RMSE = {pos_overall_rmse:.6f} m   MAE = {pos_overall_mae:.6f} m")
    print("[test] Per-axis (m):"); print(format_pos_metrics(pos_per_axis))
    print("\n[test] Orientation (quaternion → geodesic):")
    print(f"   mean={geo_summary['mean_deg']:.2f}°, median={geo_summary['median_deg']:.2f}°, P90={geo_summary['p90_deg']:.2f}°, P95={geo_summary['p95_deg']:.2f}°")

    # Save
    model_path = os.path.join(args.outdir, "eit_pose_linear_quat.joblib")
    joblib.dump(final_model, model_path)
    meta = {
        "created_utc": datetime.utcnow().isoformat()+"Z",
        "random_state": RANDOM_STATE,
        "alpha_grid": alphas, "best_alpha": best_alpha,
        "ori_weight_m_per_deg": args.ori_weight,
        "n_trainval": int(X_trainval.shape[0]), "n_test": int(X_test.shape[0]), "n_total": int(X.shape[0]),
        "feature_names": eita, "target_format": "xyz (m) + quaternion (w,x,y,z)",
        "csv_path": os.path.abspath(args.csv),
        "notes": "Composite CV: pos_RMSE_m + ori_weight*mean_geodesic_deg. Quaternion normalized at inference."
    }
    with open(os.path.join(args.outdir, "eit_pose_linear_quat_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    report = []
    report += ["EIT → Pose Linear (Ridge) with Quaternion Orientation",
               f"Date (UTC): {meta['created_utc']}",
               f"CSV: {meta['csv_path']}",
               f"Samples: total={meta['n_total']} | train+val={meta['n_trainval']} | test={meta['n_test']}",
               f"Best alpha: {best_alpha}  (composite CV; ori_weight={args.ori_weight} m/deg)",
               "", "[Test] Position (xyz)",
               f"RMSE = {pos_overall_rmse:.6f} m   MAE = {pos_overall_mae:.6f} m",
               format_pos_metrics(pos_per_axis),
               "", "[Test] Orientation (quaternion → geodesic)",
               f"mean={geo_summary['mean_deg']:.2f}°, median={geo_summary['median_deg']:.2f}°, P90={geo_summary['p90_deg']:.2f}°, P95={geo_summary['p95_deg']:.2f}°"]
    with open(os.path.join(args.outdir, "eit_pose_linear_quat_report.txt"), "w") as f:
        f.write("\n".join(report))

    print(f"\n[save] Model  -> {model_path}")
    print(f"[save] Meta   -> {os.path.join(args.outdir, 'eit_pose_linear_quat_meta.json')}")
    print(f"[save] Report -> {os.path.join(args.outdir, 'eit_pose_linear_quat_report.txt')}")

    # Learning curve (grouped) — records pos RMSE (m) & geodesic deg
    rng = np.random.default_rng(RANDOM_STATE)
    def parse_points(s, n_trainval):
        s = s.strip().lower()
        if s == "auto":
            if n_trainval < 120:
                pts = [max(20, n_trainval//4), max(40, n_trainval//2), n_trainval]
            else:
                pts = list(np.unique(np.clip(np.round(np.linspace(150, n_trainval, 6)).astype(int), 50, n_trainval)))
                if pts[-1] != n_trainval: pts[-1] = n_trainval
            return pts
        pts = [int(x) for x in s.split(",") if x.strip()]
        pts = [p for p in pts if p > 10]
        pts = sorted(set([min(p, n_trainval) for p in pts]))
        if len(pts) == 0 or pts[-1] != n_trainval: pts.append(n_trainval)
        return pts
    def sample_groups_for_size(groups, desired_n, rng):
        groups = np.asarray(groups)
        uniq, cnt = np.unique(groups, return_counts=True)
        idx = np.arange(len(uniq)); rng.shuffle(idx)
        total, sel = 0, set()
        for i in idx:
            g = uniq[i]; sel.add(g); total += cnt[i]
            if total >= desired_n: break
        return np.isin(groups, list(sel))

    sizes = parse_points(args.lc_points, X_trainval.shape[0])
    lc_rows = []
    with tqdm(total=len(sizes), desc="Learning curve (train sizes)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for n in sizes:
            mask = sample_groups_for_size(g_trainval.values, desired_n=n, rng=rng)
            X_sub, Y_sub = X_trainval[mask], y_trainval[mask]
            inner = Pipeline([("scaler", StandardScaler()), ("ridge", Ridge(alpha=best_alpha, fit_intercept=True))])
            model_sub = TransformedTargetRegressor(regressor=inner, transformer=StandardScaler())
            model_sub.fit(X_sub, Y_sub)

            Y_hat_tr = model_sub.predict(X_sub)
            pos_tr, pos_ph = Y_sub[:,:3], Y_hat_tr[:,:3]
            q_tr, q_ph = Y_sub[:,3:], quat_normalize(Y_hat_tr[:,3:])
            tr_pos_rmse = float(np.sqrt(mean_squared_error(pos_tr, pos_ph)))
            tr_geo_deg  = float(quat_geodesic_deg(q_ph, q_tr).mean())

            Y_hat_te = model_sub.predict(X_test)
            pos_te, pos_ph_te = y_test[:,:3], Y_hat_te[:,:3]
            q_te, q_ph_te = y_test[:,3:], quat_normalize(Y_hat_te[:,3:])
            te_pos_rmse = float(np.sqrt(mean_squared_error(pos_te, pos_ph_te)))
            te_geo_deg  = float(quat_geodesic_deg(q_ph_te, q_te).mean())

            lc_rows.append({"n_train": int(X_sub.shape[0]),
                            "pos_rmse_train_m": tr_pos_rmse,
                            "pos_rmse_test_m":  te_pos_rmse,
                            "rot_geo_train_deg": tr_geo_deg,
                            "rot_geo_test_deg":  te_geo_deg})
            print(f"[lc] n_train={X_sub.shape[0]}  pos_RMSE_test={te_pos_rmse:.6f} m  rot_geo_test={te_geo_deg:.2f}°")
            pbar.update(1)

    lc_df = pd.DataFrame(lc_rows).sort_values("n_train")
    lc_csv = os.path.join(args.outdir, "eit_pose_linear_quat_learning_curve.csv")
    lc_df.to_csv(lc_csv, index=False)
    plt.figure()
    plt.plot(lc_df["n_train"].values, lc_df["pos_rmse_test_m"].values, "o-", label="Position RMSE (test) [m]")
    plt.plot(lc_df["n_train"].values, lc_df["rot_geo_test_deg"].values, "o-", label="Orientation geodesic (test) [deg]")
    plt.xlabel("Training samples (by grouped subset)"); plt.ylabel("Error")
    plt.title("Learning Curve: ΔEIT → Pose (Ridge + Quaternion)")
    plt.grid(True, linestyle="--", lw=0.5); plt.legend()
    lc_png = os.path.join(args.outdir, "eit_pose_linear_quat_learning_curve.png")
    plt.tight_layout(); plt.savefig(lc_png, dpi=160)
    print(f"[save] Learning-curve CSV -> {lc_csv}")
    print(f"[save] Learning-curve PNG -> {lc_png}")

    # Diagnostics
    df_test = df.iloc[test_idx].copy()
    # --- Orientation plots in axis–angle (rx,ry,rz) for convenience ---
    r_true_ax = df_test[["act_tcp_rx","act_tcp_ry","act_tcp_rz"]].values   # radians from CSV
    q_true    = axis_angle_to_quat(r_true_ax)
    r_true    = quat_to_axis_angle_ur(q_true)         # canonicalized truth
    r_pred    = quat_to_axis_angle_ur(quat_pred)      # canonicalized prediction
    save_axis_angle_plots(r_true, r_pred, args.outdir, use_degrees=True)
    print(f"[save] Axis–angle plots -> pvstrue_act_tcp_r*_deg.png & residuals_act_tcp_r*_deg.png in {args.outdir}")
    save_pred_vs_true_plots_pos(pos_true, pos_pred, args.outdir)
    save_error_vs_slice_plots(df_test, pos_true, pos_pred, geo_errors_deg, args.outdir)
    diag = {"position_overall":{"rmse_m":pos_overall_rmse,"mae_m":pos_overall_mae},
            "position_per_axis_m":pos_per_axis,
            "orientation_geodesic_deg":geo_summary,
            "cv":{"best_alpha":best_alpha,"ori_weight_m_per_deg":args.ori_weight}}
    analyse_pos_robustness_vs_orientation(
        df_test=df_test,
        pos_true=pos_true,
        pos_pred=pos_pred,
        outdir=args.outdir,
        nbins=12,
        fixed_angle_xlim=(-100, 100),   # <- same x-limits for all three plots
    )

    with open(os.path.join(args.outdir, "eit_pose_linear_quat_diagnostics.json"), "w") as f:
        json.dump(diag, f, indent=2)
    return 0

# ---------- inference helpers ----------
def load_model(model_path="./outputs/eit_pose_linear_quat.joblib"):
    return joblib.load(model_path)

def predict_pose_quat_delta(delta_eit_256, model=None, model_path="./outputs/eit_pose_linear_quat.joblib"):
    v = np.asarray(delta_eit_256, dtype=float).reshape(1,-1)
    if model is None: model = load_model(model_path)
    y = model.predict(v)[0]
    pos, quat = y[:3], quat_normalize(y[3:])
    return np.concatenate([pos, quat])

def predict_pose_axisangle_delta(delta_eit_256, model=None, model_path="./outputs/eit_pose_linear_quat.joblib"):
    y = predict_pose_quat_delta(delta_eit_256, model=model, model_path=model_path)
    pos, quat = y[:3], y[3:]
    rvec = quat_to_axis_angle(quat)
    return np.concatenate([pos, rvec])

if __name__ == "__main__":
    raise SystemExit(main())
