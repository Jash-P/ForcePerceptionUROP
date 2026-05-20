#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EIT Δ (pressed - hovered) → UR5 TCP Pose (x,y,z, rx,ry,rz) — Linear Baseline (Ridge)

GOAL
-----
Train a simple, interpretable *multi-output linear regression* that maps
a 256-dim Electrical Impedance Tomography (EIT) feature vector to the 6-DoF
TCP pose measured by the UR5 (actual pose from RTDE).
This gives you a fast baseline for pose prediction from the sensor.

WHY THIS DESIGN
---------------
- We use ΔEIT = eita - eitb for each channel (pressed minus hovered) to cancel
  slow drifts / lighting / temperature effects. This is your "contact-induced"
  signal per electrode.
- We use a *single* multi-output linear regressor (Ridge). This is equivalent
  to 6 separate linear regressions that share the same standardized features,
  but it is trained jointly and is easier to deploy as one pipeline.
- We standardize both X (features) and y (targets) to stabilize optimization
  and make the Ridge penalty well-conditioned.
- We avoid data leakage by splitting train/validation/test *by pose groups*
  (GroupShuffleSplit & GroupKFold). Repeats of the same nominal pose never
  land across splits.
- We include a *learning curve* to show how test error scales with more data.
- We generate diagnostic plots (pred vs true, residual histograms) and
  error slices (vs Fz, vs y, vs pitch) to reveal weak regions.
- We filter out low-force rows (Fz < min_Fz) because very light contacts tend
  to be noisy and hurt model fit.

INPUT CSV (required columns)
----------------------------
- EIT channels (baseline/hovered):  eitb_0 ... eitb_255
- EIT channels (pressed/lowered):   eita_0 ... eita_255
- Pose targets (actual TCP):        act_tcp_x, act_tcp_y, act_tcp_z,
                                    act_tcp_rx, act_tcp_ry, act_tcp_rz
  NOTE: rx/ry/rz are the UR *axis-angle* representation (radians).
- Force:                            Fz   (rows with Fz < min_Fz are dropped)
- Optional:                         contact_detected (1/0) for filtering

OUTPUT ARTIFACTS
----------------
- eit_pose_linear_model.joblib            : trained pipeline (ready for predict)
- eit_pose_linear_model_meta.json         : metadata (settings, shapes, paths)
- eit_pose_training_report.txt            : human-readable summary (metrics)
- eit_pose_diagnostics.json               : machine-readable diagnostics
- eit_pose_learning_curve.csv / .png      : train vs test RMSE across sizes
- pvstrue_*.png, residuals_*.png          : per-axis diagnostic figures
- error_vs_*.png                          : error slices vs Fz / y / pitch

USAGE
-----
python train_eit_pose_regression.py --csv /path/to/logs.csv \
  --min_Fz 50 --test_size 0.2 --cv_folds 5 --lc_points auto \
  --alpha_grid 0.1,0.3,1,3,10,30,100

TIP: Run with fewer folds and fewer alphas for a quick pass:
  --cv_folds 3 --alpha_grid 1,3,10

"""

import argparse
import json
import os
from datetime import datetime

import numpy as np
import pandas as pd
from sklearn.model_selection import GroupKFold, GroupShuffleSplit
from sklearn.linear_model import Ridge
from sklearn.preprocessing import StandardScaler
from sklearn.compose import TransformedTargetRegressor
from sklearn.pipeline import Pipeline
from sklearn.metrics import mean_squared_error, mean_absolute_error
import joblib

# Matplotlib is only used for saving plots (no interactive UI required).
import matplotlib
matplotlib.use("Agg")  # headless backend for servers or scripts
import matplotlib.pyplot as plt

# Progress bars (tqdm) — optional; falls back to no-op if not installed.
try:
    from tqdm.auto import tqdm  # pretty progress bars in notebooks/terminals
    _TQDM_AVAILABLE = True
except Exception:  # pragma: no cover
    _TQDM_AVAILABLE = False
    class tqdm:  # minimal no-op replacement
        def __init__(self, *args, **kwargs): pass
        def update(self, *args, **kwargs): pass
        def close(self): pass
        def __enter__(self): return self
        def __exit__(self, *exc): pass
        def __iter__(self): return iter([])


# -----------------------------------------------------------------------------
# Misc configuration
# -----------------------------------------------------------------------------

RANDOM_STATE = 42  # seed for reproducibility of shuffles / sampling


# -----------------------------------------------------------------------------
# Geometry helpers (orientation math) and plotting utilities
# -----------------------------------------------------------------------------

def axis_angle_to_rotmat(v):
    """
    Convert an axis-angle vector (rx,ry,rz), where the vector encodes
    axis * angle (radians), to a 3x3 rotation matrix using Rodrigues' formula.
    If the angle is extremely small, return identity.
    Parameters
    ----------
    v : array-like, shape (3,)
        UR axis-angle vector in radians.
    Returns
    -------
    R : ndarray, shape (3,3)
        Rotation matrix.
    """
    v = np.asarray(v, dtype=float)
    theta = np.linalg.norm(v)
    if theta < 1e-12:
        return np.eye(3)
    k = v / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    R = np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)
    return R


def geodesic_angle_deg(r_pred, r_true):
    """
    Geodesic rotation error *in degrees* between two axis-angle vectors.
    We compute R_err = R_pred * R_true^T, and convert its angle to degrees.
    """
    R_p = axis_angle_to_rotmat(r_pred)
    R_t = axis_angle_to_rotmat(r_true)
    R_e = R_p @ R_t.T
    tr = np.trace(R_e)
    val = (tr - 1.0) / 2.0
    val = np.clip(val, -1.0, 1.0)  # numerical safety
    ang = np.arccos(val)           # [0, pi]
    return float(ang * 180.0 / np.pi)


def per_axis_metrics(y_true, y_pred, target_names):
    """
    Compute RMSE and MAE *per output axis*.
    Returns a dict keyed by target name.
    """
    mse = np.mean((y_true - y_pred) ** 2, axis=0)
    rmse = np.sqrt(mse)
    mae = np.mean(np.abs(y_true - y_pred), axis=0)
    metrics = {}
    for i, name in enumerate(target_names):
        entry = {"rmse": float(rmse[i]), "mae": float(mae[i])}
        # For angular axes, also report degrees for interpretability.
        if name in ("act_tcp_rx", "act_tcp_ry", "act_tcp_rz"):
            entry["rmse_deg"] = float(rmse[i] * 180.0 / np.pi)
            entry["mae_deg"]  = float(mae[i] * 180.0 / np.pi)
        metrics[name] = entry
    return metrics


def format_metrics(metrics):
    """
    Nicely format the per-axis metrics for the human-readable report.
    """
    lines = []
    for k, v in metrics.items():
        if "rx" in k or "ry" in k or "rz" in k:
            lines.append(f"{k:>11s}  RMSE={v['rmse']:.4f} rad ({v['rmse_deg']:.2f}°)   "
                         f"MAE={v['mae']:.4f} rad ({v['mae_deg']:.2f}°)")
        else:
            lines.append(f"{k:>11s}  RMSE={v['rmse']:.4f} m           MAE={v['mae']:.4f} m")
    return "\n".join(lines)


def compute_bias_percentiles(y_true, y_pred, target_names):
    """
    Compute mean error (bias) and |error| percentiles P50/P90/P95 per axis.
    Bias helps spot systematic offsets; percentiles show typical & tail errors.
    """
    err = y_pred - y_true
    abs_err = np.abs(err)
    out = {}
    for i, name in enumerate(target_names):
        bias = float(np.mean(err[:, i]))
        p50  = float(np.percentile(abs_err[:, i], 50))
        p90  = float(np.percentile(abs_err[:, i], 90))
        p95  = float(np.percentile(abs_err[:, i], 95))
        out[name] = {"bias": bias, "p50": p50, "p90": p90, "p95": p95}
    return out


def format_bias_percentiles(stats):
    """
    Human-readable formatting for bias and percentile stats.
    """
    lines = []
    for k, v in stats.items():
        if "rx" in k or "ry" in k or "rz" in k:
            lines.append(f"{k:>11s}  bias={v['bias']:.4f} rad | "
                         f"|err| P50={v['p50']:.4f} rad, P90={v['p90']:.4f}, P95={v['p95']:.4f}")
        else:
            lines.append(f"{k:>11s}  bias={v['bias']:.4f} m   | "
                         f"|err| P50={v['p50']:.4f} m, P90={v['p90']:.4f}, P95={v['p95']:.4f}")
    return "\n".join(lines)


def save_pred_vs_true_plots(y_true, y_pred, target_names, outdir):
    """
    For each axis, save:
      - a scatter plot of predicted vs true (with identity line)
      - a residual histogram
    These quickly reveal bias and spread.
    """
    for i, name in enumerate(target_names):
        # Scatter: predicted vs true
        plt.figure()
        plt.scatter(y_true[:, i], y_pred[:, i], s=10)
        # Identity y=x line for reference
        lims = [min(y_true[:, i].min(), y_pred[:, i].min()),
                max(y_true[:, i].max(), y_pred[:, i].max())]
        plt.plot(lims, lims)
        plt.xlabel(f"True {name}")
        plt.ylabel(f"Pred {name}")
        plt.title(f"Predicted vs True — {name}")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"pvstrue_{name}.png"), dpi=140)

        # Residual histogram
        plt.figure()
        resid = y_pred[:, i] - y_true[:, i]
        plt.hist(resid, bins=50)
        plt.xlabel(f"Residual ({name})")
        plt.ylabel("Count")
        plt.title(f"Residuals — {name}")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"residuals_{name}.png"), dpi=140)


def binned_stats(x, err_vals, bins):
    """
    Compute mean absolute error within bins of x.
    Returns (bin_centers, mean_abs_error_per_bin).
    """
    x = np.asarray(x); err_vals = np.asarray(err_vals)
    idx = np.digitize(x, bins) - 1
    centers = 0.5*(bins[:-1] + bins[1:])
    mae = []
    for b in range(len(centers)):
        mask = idx == b
        if np.any(mask):
            mae.append(float(np.mean(np.abs(err_vals[mask]))))
        else:
            mae.append(np.nan)  # keep placeholder to preserve bin positions
    return centers, np.array(mae)


def save_error_vs_slice_plots(df_test, y_true, y_pred, target_names, outdir):
    """
    Make simple one-line plots to show how error changes across:
      - normal force Fz
      - workspace y (act_tcp_y)
      - pitch (act_tcp_ry, shown in degrees)
    These help you decide where to collect more data.
    """
    # Position error per sample: mean absolute error across x,y,z (in metres)
    pos_abs = np.abs(y_pred[:, :3] - y_true[:, :3])
    pos_mae_sample = np.mean(pos_abs, axis=1)

    # Orientation error per sample: geodesic angle (in degrees)
    geo = []
    for i in range(y_true.shape[0]):
        r_pred = y_pred[i, 3:6]
        r_true = y_true[i, 3:6]
        geo.append(geodesic_angle_deg(r_pred, r_true))
    geo = np.array(geo)

    # Fz bins (if available)
    if "Fz" in df_test.columns:
        fz = df_test["Fz"].values
        bins = np.linspace(np.nanmin(fz), np.nanmax(fz), 8)
        c, mae_pos = binned_stats(fz, pos_mae_sample, bins)
        _, mae_geo = binned_stats(fz, geo, bins)
        # Plot position
        plt.figure()
        plt.plot(c, mae_pos, marker="o")
        plt.xlabel("Fz")
        plt.ylabel("Mean |pos error| (m)")
        plt.title("Position error vs Fz")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, "error_vs_Fz_position.png"), dpi=140)
        # Plot orientation
        plt.figure()
        plt.plot(c, mae_geo, marker="o")
        plt.xlabel("Fz")
        plt.ylabel("Geodesic ori error (deg)")
        plt.title("Orientation error vs Fz")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, "error_vs_Fz_orientation.png"), dpi=140)

    # act_tcp_y bins
    yvals = df_test["act_tcp_y"].values if "act_tcp_y" in df_test.columns else y_true[:,1]
    ybins = np.linspace(np.nanmin(yvals), np.nanmax(yvals), 8)
    c, mae_pos = binned_stats(yvals, pos_mae_sample, ybins)
    _, mae_geo = binned_stats(yvals, geo, ybins)
    plt.figure()
    plt.plot(c, mae_pos, marker="o")
    plt.xlabel("act_tcp_y")
    plt.ylabel("Mean |pos error| (m)")
    plt.title("Position error vs act_tcp_y")
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "error_vs_y_position.png"), dpi=140)
    plt.figure()
    plt.plot(c, mae_geo, marker="o")
    plt.xlabel("act_tcp_y")
    plt.ylabel("Geodesic ori error (deg)")
    plt.title("Orientation error vs act_tcp_y")
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "error_vs_y_orientation.png"), dpi=140)

    # pitch bins (act_tcp_ry), x-axis in degrees for readability
    pitch = df_test["act_tcp_ry"].values if "act_tcp_ry" in df_test.columns else y_true[:,4]
    pitch_deg = pitch * 180.0 / np.pi
    pbins = np.linspace(np.nanmin(pitch_deg), np.nanmax(pitch_deg), 8)
    c, mae_pos = binned_stats(pitch_deg, pos_mae_sample, pbins)
    _, mae_geo = binned_stats(pitch_deg, geo, pbins)
    plt.figure()
    plt.plot(c, mae_pos, marker="o")
    plt.xlabel("act_tcp_ry (deg)")
    plt.ylabel("Mean |pos error| (m)")
    plt.title("Position error vs pitch")
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "error_vs_pitch_position.png"), dpi=140)
    plt.figure()
    plt.plot(c, mae_geo, marker="o")
    plt.xlabel("act_tcp_ry (deg)")
    plt.ylabel("Geodesic ori error (deg)")
    plt.title("Orientation error vs pitch")
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "error_vs_pitch_orientation.png"), dpi=140)


# -----------------------------------------------------------------------------
# Data grouping & learning curve utilities
# -----------------------------------------------------------------------------

def build_pose_id(df, pos_bin_m=0.001, ang_bin_rad=0.05):
    """
    Build a *group ID* for each sample so repeated measurements at the
    same nominal pose are grouped together for splitting.
    We quantize (actual) xyz by 'pos_bin_m' and rx/ry/rz by 'ang_bin_rad'.
    Returns a pandas Series of strings (one group id per row).
    """
    xq  = np.round(df['act_tcp_x'].values  / pos_bin_m).astype(np.int64)
    yq  = np.round(df['act_tcp_y'].values  / pos_bin_m).astype(np.int64)
    zq  = np.round(df['act_tcp_z'].values  / pos_bin_m).astype(np.int64)
    rxq = np.round(df['act_tcp_rx'].values / ang_bin_rad).astype(np.int64)
    ryq = np.round(df['act_tcp_ry'].values / ang_bin_rad).astype(np.int64)
    rzq = np.round(df['act_tcp_rz'].values / ang_bin_rad).astype(np.int64)
    groups = [f"{a}_{b}_{c}_{d}_{e}_{f}" for a,b,c,d,e,f in zip(xq,yq,zq,rxq,ryq,rzq)]
    return pd.Series(groups, index=df.index)


def parse_points(s, n_trainval):
    """
    Parse '--lc_points' which controls the training sizes for the learning curve.
    - 'auto': pick ~6 sizes up to the full train+val set.
    - comma list: e.g., '200,400,800'.
    Always ensures the last point equals n_trainval.
    """
    s = s.strip().lower()
    if s == "auto":
        if n_trainval < 120:
            pts = [max(20, n_trainval//4), max(40, n_trainval//2), n_trainval]
        else:
            pts = list(np.unique(np.clip(np.round(np.linspace(150, n_trainval, 6)).astype(int), 50, n_trainval)))
            if pts[-1] != n_trainval:
                pts[-1] = n_trainval
        return pts
    pts = [int(x) for x in s.split(",") if x.strip()]
    pts = [p for p in pts if p > 10]
    pts = sorted(set([min(p, n_trainval) for p in pts]))
    if len(pts) == 0 or pts[-1] != n_trainval:
        pts.append(n_trainval)
    return pts


def sample_groups_for_size(groups, desired_n, rng):
    """
    Randomly pick a minimal set of *entire groups* whose total number of samples
    is >= desired_n. This preserves group integrity when building smaller
    training sets for the learning curve.
    Returns a boolean mask over samples (True = selected).

    NOTE: Because groups vary in size, the returned count may be slightly above
          'desired_n' — that's expected and fine.
    """
    groups = np.asarray(groups)
    unique, counts = np.unique(groups, return_counts=True)
    idx = np.arange(len(unique))
    rng.shuffle(idx)
    selected_groups = set()
    total = 0
    for i in idx:
        g = unique[i]
        selected_groups.add(g)
        total += counts[i]
        if total >= desired_n:
            break
    mask = np.isin(groups, list(selected_groups))
    return mask


# -----------------------------------------------------------------------------
# Main training / evaluation routine
# -----------------------------------------------------------------------------

def main():
    # 1) Parse command-line arguments
    parser = argparse.ArgumentParser(description="Train EIT→pose linear model (Ridge) with grouped CV and diagnostics.")
    parser.add_argument("--csv", type=str, required=True,
                        help="Path to CSV with eitb_*, eita_* (256 each), act_tcp_* targets, and Fz.")
    parser.add_argument("--test_size", type=float, default=0.20,
                        help="Fraction of *groups* for the final hold-out test split.")
    parser.add_argument("--cv_folds", type=int, default=5,
                        help="Number of GroupKFold folds within train+val for α tuning.")
    parser.add_argument("--contact_only", action="store_true",
                        help="If set, only use rows with contact_detected == 1.")
    parser.add_argument("--min_Fz", type=float, default=50.0,
                        help="Drop rows where Fz < min_Fz (filters low-force/noisy contacts).")
    parser.add_argument("--alpha_grid", type=str, default="0.1,0.3,1,3,10,30,100",
                        help="Comma-separated Ridge alphas to try during CV (e.g., '1,3,10').")
    parser.add_argument("--lc_points", type=str, default="auto",
                        help="Learning-curve train sizes: 'auto' or '200,400,800'.")
    parser.add_argument("--outdir", type=str, default="/mnt/data",
                        help="Output directory for model, reports, curves, and figures.")
    parser.add_argument("--no_progress", action="store_true",
                        help="Disable progress bars (tqdm). Useful in very quiet logs.")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # 2) Load the CSV into memory
    df = pd.read_csv(args.csv)

    # Optional: restrict to contact rows (if you want pure contact mapping)
    if 'contact_detected' in df.columns and args.contact_only:
        df = df[df['contact_detected'] == 1].copy()

    # Enforce force threshold (drop low-force rows)
    if 'Fz' not in df.columns:
        raise ValueError("Column 'Fz' not found in CSV; required for filtering.")
    pre_filter_n = len(df)
    df = df[df['Fz'] >= args.min_Fz].copy()
    post_filter_n = len(df)
    print(f"[filter] Kept {post_filter_n}/{pre_filter_n} rows after Fz >= {args.min_Fz} filter.")

    # 3) Build the 256-dim ΔEIT feature matrix X = eita - eitb
    eitb_cols = [f"eitb_{i}" for i in range(256)]
    eita_cols = [f"eita_{i}" for i in range(256)]
    missing_b = [c for c in eitb_cols if c not in df.columns]
    missing_a = [c for c in eita_cols if c not in df.columns]
    if missing_b or missing_a:
        raise ValueError(f"Missing EIT columns. Missing eitb: {missing_b[:5]}... Missing eita: {missing_a[:5]}...")
    # (N,256) array of channel differences: contact-induced signal
    X = df[eita_cols].values - df[eitb_cols].values

    # 4) Targets y = actual TCP pose (6 dims). rx,ry,rz are in radians.
    target_names = ["act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz"]
    for t in target_names:
        if t not in df.columns:
            raise ValueError(f"Missing target column: {t}")
    y = df[target_names].values  # shape (N,6)

    # 5) Drop any rows containing NaN/Inf (from parsing or logging anomalies)
    good = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
    if not good.all():
        dropped = int((~good).sum())
        df = df.loc[good].reset_index(drop=True)
        X = X[good]
        y = y[good]
        print(f"[info] Dropped {dropped} rows containing NaN/Inf in features/targets.")

    # 6) Build *pose groups* so repeats of the same nominal pose stay together
    #    in splitting (prevents train/test leakage).
    pose_id = build_pose_id(df, pos_bin_m=0.001, ang_bin_rad=0.05)  # 1 mm, ~2.9° bins

    # 7) Create a final *held-out* test split by group (never touched during tuning)
    #    The remainder will be used for train+validation.
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=RANDOM_STATE)
    trainval_idx, test_idx = next(gss.split(X, y, groups=pose_id))
    X_trainval, y_trainval = X[trainval_idx], y[trainval_idx]
    X_test,     y_test     = X[test_idx],     y[test_idx]
    g_trainval = pose_id.iloc[trainval_idx]
    print(f"[split] Train+Val: {X_trainval.shape[0]} samples; Test: {X_test.shape[0]} samples; Total: {X.shape[0]} samples.")

    # 8) Hyperparameter search: choose Ridge alpha by GroupKFold CV
    #    We use a pipeline that standardizes X and applies Ridge,
    #    wrapped by TransformedTargetRegressor to standardize y as well.
    alphas = [float(s) for s in args.alpha_grid.split(",")]
    cv = GroupKFold(n_splits=args.cv_folds)
    results = []
    best_score = np.inf
    best_alpha = None

    for alpha in alphas:
        fold_rmses = []
        for fold, (tr, va) in enumerate(cv.split(X_trainval, y_trainval, groups=g_trainval), 1):
            X_tr, y_tr = X_trainval[tr], y_trainval[tr]
            X_va, y_va = X_trainval[va], y_trainval[va]

            # Inner pipeline: standardize features, then Ridge regression
            inner = Pipeline([
                ("scaler", StandardScaler()),
                ("ridge", Ridge(alpha=alpha, fit_intercept=True))
            ])
            # Wrap to standardize targets as well (multi-output safe)
            model = TransformedTargetRegressor(regressor=inner, transformer=StandardScaler())

            # Fit on training split, evaluate on validation split
            model.fit(X_tr, y_tr)
            y_hat = model.predict(X_va)

            # Single overall RMSE for model selection (averages across outputs)
            rmse = np.sqrt(mean_squared_error(y_va, y_hat))
            fold_rmses.append(rmse)

        mean_rmse = float(np.mean(fold_rmses))
        results.append({"alpha": alpha, "cv_rmse": mean_rmse})
        if mean_rmse < best_score:
            best_score = mean_rmse
            best_alpha = alpha

    print("[cv] Results (alpha -> mean RMSE):")
    for r in results:
        print(f"   alpha={r['alpha']:>6g}  cv_rmse={r['cv_rmse']:.6f}")
    print(f"[cv] Best alpha: {best_alpha} (mean RMSE={best_score:.6f})")

    # 9) Refit the final model on *all* train+val samples using the best alpha
    final_inner = Pipeline([
        ("scaler", StandardScaler()),
        ("ridge", Ridge(alpha=best_alpha, fit_intercept=True))
    ])
    final_model = TransformedTargetRegressor(regressor=final_inner, transformer=StandardScaler())
    final_model.fit(X_trainval, y_trainval)

    # 10) Evaluate once on the untouched TEST set (this is your deployment proxy)
    y_pred_test = final_model.predict(X_test)

    # Overall errors (aggregate across all 6 outputs)
    overall_rmse = float(np.sqrt(mean_squared_error(y_test, y_pred_test)))
    overall_mae  = float(mean_absolute_error(y_test, y_pred_test))

    # Per-axis errors (x,y,z in m; rx/ry/rz in rad + deg for readability)
    per_axis = per_axis_metrics(y_test, y_pred_test, target_names)

    print("\n[test] Overall:")
    print(f"   RMSE = {overall_rmse:.6f}   MAE = {overall_mae:.6f}")
    print("\n[test] Per-axis:")
    print(format_metrics(per_axis))

    # ---- Additional diagnostics on TEST ----
    # Bias and percentiles (P50/P90/P95) per axis
    bias_pct = compute_bias_percentiles(y_test, y_pred_test, target_names)

    # Geodesic orientation error (deg) summary
    geo_errors_deg = []
    for i in range(y_test.shape[0]):
        geo_errors_deg.append(geodesic_angle_deg(y_pred_test[i, 3:6], y_test[i, 3:6]))
    geo_errors_deg = np.array(geo_errors_deg)
    geo_summary = {
        "mean_deg":   float(np.mean(geo_errors_deg)),
        "median_deg": float(np.median(geo_errors_deg)),
        "p90_deg":    float(np.percentile(geo_errors_deg, 90)),
        "p95_deg":    float(np.percentile(geo_errors_deg, 95)),
    }

    # Save per-axis diagnostic figures (pred vs true + residual histograms)
    df_test = df.iloc[test_idx].copy()  # bring along Fz/pose for slicing plots
    save_pred_vs_true_plots(y_test, y_pred_test, target_names, args.outdir)

    # Error vs force / workspace slices
    save_error_vs_slice_plots(df_test, y_test, y_pred_test, target_names, args.outdir)

    # Save diagnostics JSON for programmatic consumption
    diag = {
        "overall": {"rmse": overall_rmse, "mae": overall_mae},
        "per_axis_bias_percentiles": bias_pct,
        "geodesic_orientation_error_deg": geo_summary
    }
    with open(os.path.join(args.outdir, "eit_pose_diagnostics.json"), "w") as f:
        json.dump(diag, f, indent=2)

    # 11) Persist the trained model and metadata
    #     The model includes scalers and Ridge stage; pass ΔEIT (1x256) to predict.
    model_path = os.path.join(args.outdir, "eit_pose_linear_model.joblib")
    joblib.dump(final_model, model_path)

    meta = {
        "created_utc": datetime.utcnow().isoformat() + "Z",
        "random_state": RANDOM_STATE,
        "alpha_grid": alphas,
        "best_alpha": best_alpha,
        "n_trainval": int(X_trainval.shape[0]),
        "n_test":     int(X_test.shape[0]),
        "n_total":    int(X.shape[0]),
        "feature_names": eita_cols,  # Δ computed from eita - eitb (same indexing)
        "target_names": target_names,
        "pos_bin_m": 0.001,
        "ang_bin_rad": 0.05,
        "contact_only": bool(args.contact_only),
        "min_Fz": float(args.min_Fz),
        "csv_path": os.path.abspath(args.csv),
        "notes": (
            "Model maps ΔEIT (eita - eitb) to actual TCP pose (act_tcp_*). "
            "Ridge with X and y standardized. Rows with Fz < min_Fz are excluded."
        )
    }
    meta_path = os.path.join(args.outdir, "eit_pose_linear_model_meta.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)

    # 12) Human-readable training report (TXT)
    report = []
    report.append("EIT → Pose Linear Regression (Ridge)")
    report.append(f"Date (UTC): {meta['created_utc']}")
    report.append(f"CSV: {meta['csv_path']}")
    report.append(f"Rows after Fz filter (>= {args.min_Fz}): {len(df)}")
    report.append(f"Total samples: {meta['n_total']} | Train+Val: {meta['n_trainval']} | Test: {meta['n_test']}")
    report.append(f"Best alpha: {best_alpha}")
    report.append("")
    report.append("[Test] Overall")
    report.append(f"RMSE = {overall_rmse:.6f}   MAE = {overall_mae:.6f}")
    report.append("")
    report.append("[Test] Per-axis")
    report.append(format_metrics(per_axis))
    report.append("")
    report.append("[Test] Bias & |error| percentiles")
    report.append(format_bias_percentiles(bias_pct))
    report.append("")
    report.append("[Test] Geodesic orientation error (deg)")
    report.append(f"mean={geo_summary['mean_deg']:.2f}°, median={geo_summary['median_deg']:.2f}°, "
                  f"P90={geo_summary['p90_deg']:.2f}°, P95={geo_summary['p95_deg']:.2f}°")
    report.append("")
    report.append("Saved figures:")
    report.append(" - pvstrue_*.png (predicted vs true per axis)")
    report.append(" - residuals_*.png (residual histograms per axis)")
    report.append(" - error_vs_Fz_position.png / error_vs_Fz_orientation.png")
    report.append(" - error_vs_y_position.png / error_vs_y_orientation.png")
    report.append(" - error_vs_pitch_position.png / error_vs_pitch_orientation.png")
    report_txt = os.path.join(args.outdir, "eit_pose_training_report.txt")
    with open(report_txt, "w") as f:
        f.write("\n".join(report))

    print(f"\n[save] Model -> {model_path}")
    print(f"[save] Meta  -> {meta_path}")
    print(f"[save] Report-> {report_txt}")

    # 13) Learning curve: fit on *increasing grouped subsets* of train+val
    #     and evaluate both TRAIN and TEST errors for each subset size.
    rng = np.random.default_rng(RANDOM_STATE)
    n_trainval = X_trainval.shape[0]
    sizes = parse_points(args.lc_points, n_trainval)

    lc_rows = []
    for n in sizes:
        # Choose enough *groups* to reach at least 'n' samples
        mask = sample_groups_for_size(g_trainval.values, desired_n=n, rng=rng)
        X_sub, y_sub = X_trainval[mask], y_trainval[mask]

        # Fit on the subset
        inner = Pipeline([
            ("scaler", StandardScaler()),
            ("ridge", Ridge(alpha=best_alpha, fit_intercept=True))
        ])
        model_sub = TransformedTargetRegressor(regressor=inner, transformer=StandardScaler())
        model_sub.fit(X_sub, y_sub)

        # Compute TRAIN errors on the same subset (for bias/variance insight)
        y_pred_tr = model_sub.predict(X_sub)
        rmse_tr = float(np.sqrt(mean_squared_error(y_sub, y_pred_tr)))
        mae_tr  = float(mean_absolute_error(y_sub, y_pred_tr))
        per_axis_tr = per_axis_metrics(y_sub, y_pred_tr, target_names)

        # Compute TEST errors on the *fixed* held-out test set
        y_pred_te = model_sub.predict(X_test)
        rmse_te = float(np.sqrt(mean_squared_error(y_test, y_pred_te)))
        mae_te  = float(mean_absolute_error(y_test, y_pred_te))
        per_axis_te = per_axis_metrics(y_test, y_pred_te, target_names)

        # Record both overall and per-axis metrics for CSV
        row = {
            "n_train": int(X_sub.shape[0]),
            "rmse_overall_train": rmse_tr,
            "mae_overall_train":  mae_tr,
            "rmse_overall_test":  rmse_te,
            "mae_overall_test":   mae_te,
        }
        for k, v in per_axis_tr.items():
            row[f"rmse_train_{k}"] = v["rmse"]
            row[f"mae_train_{k}"]  = v["mae"]
        for k, v in per_axis_te.items():
            row[f"rmse_test_{k}"] = v["rmse"]
            row[f"mae_test_{k}"]  = v["mae"]

        lc_rows.append(row)
        print(f"[lc] n_train={X_sub.shape[0]}  train_RMSE={rmse_tr:.6f}  test_RMSE={rmse_te:.6f}")

    # Save learning-curve table
    lc_df = pd.DataFrame(lc_rows).sort_values("n_train")
    lc_csv = os.path.join(args.outdir, "eit_pose_learning_curve.csv")
    lc_df.to_csv(lc_csv, index=False)
    print(f"[save] Learning-curve CSV -> {lc_csv}")

    # Plot: overall TRAIN vs TEST RMSE across sizes (single figure)
    plt.figure()
    plt.plot(lc_df["n_train"].values, lc_df["rmse_overall_test"].values, marker="o", label="Test RMSE")
    plt.plot(lc_df["n_train"].values, lc_df["rmse_overall_train"].values, marker="o", label="Train RMSE")
    plt.xlabel("Training samples (by grouped subset)")
    plt.ylabel("RMSE (overall)")
    plt.title("Learning Curve: ΔEIT → Pose (Ridge)")
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.legend()
    lc_png = os.path.join(args.outdir, "eit_pose_learning_curve.png")
    plt.tight_layout()
    plt.savefig(lc_png, dpi=160)
    print(f"[save] Learning-curve PNG -> {lc_png}")

    return 0


# -----------------------------------------------------------------------------
# Lightweight API for reusing the trained model in other scripts
# -----------------------------------------------------------------------------

def load_model(model_path="/mnt/data/eit_pose_linear_model.joblib"):
    """
    Load the saved pipeline. The pipeline includes scalers + Ridge
    and expects a 256-dim ΔEIT row vector as input to predict().
    """
    return joblib.load(model_path)


def predict_pose_delta(delta_eit_256, model=None, model_path="/mnt/data/eit_pose_linear_model.joblib"):
    """
    Convenience wrapper for one-off predictions.
    Parameters
    ----------
    delta_eit_256 : array-like, shape (256,)
        The ΔEIT vector (eita - eitb) ordered the same way as training.
    model : Optional fitted pipeline (if None, loads from model_path).
    Returns
    -------
    pose : ndarray, shape (6,)
        [x, y, z, rx, ry, rz] with angles in radians (axis-angle).
    """
    if model is None:
        model = load_model(model_path)
    delta_eit_256 = np.asarray(delta_eit_256, dtype=float).reshape(1, -1)
    if delta_eit_256.shape[1] != 256:
        raise ValueError("delta_eit_256 must have 256 elements.")
    return model.predict(delta_eit_256)[0]


# -----------------------------------------------------------------------------

if __name__ == "__main__":
    raise SystemExit(main())
