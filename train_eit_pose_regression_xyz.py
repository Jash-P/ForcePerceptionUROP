#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EIT Δ -> UR5 TCP Position (x,y,z) — Linear Ridge Regression (XYZ-only)

This script trains a *pure position* model to maximize xyz accuracy
from EIT differentials (pressed - hovered). Orientation is *not*
modeled or used for selection any more.

Outputs (in --outdir):
  - eit_pose_linear_xyz.joblib                  (scikit model via joblib)
  - eit_pose_linear_xyz_meta.json               (metadata / CV results)
  - eit_pose_linear_xyz_report.txt              (test metrics summary)
  - pred_vs_true_act_tcp_{x,y,z}.png            (scatter)
  - residuals_act_tcp_{x,y,z}.png               (histogram)
  - learning_curve_xyz.csv/.png                 (train size vs vector RMSE/MAE)
  - robustness_xyz_vs_{roll,pitch,yaw}.png/.csv (common y-scale)
  - robustness_xyz_vs_rpy_oneplot.png/.csv      (all R/P/Y on one plot)
  - robustness_xyz_heatmap_pitch_yaw.png        (optional heatmap)

Install:
  python3 -m pip install numpy pandas scikit-learn matplotlib tqdm joblib
"""

import argparse, json, os, warnings
from datetime import datetime

import numpy as np
import pandas as pd

from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GroupKFold, GroupShuffleSplit
from sklearn.pipeline import Pipeline
from sklearn.compose import TransformedTargetRegressor
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_squared_error, mean_absolute_error
from joblib import dump

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from tqdm.auto import tqdm
    _TQDM = True
except Exception:
    _TQDM = False
    class tqdm:  # fallback no-op
        def __init__(self, *a, **k): pass
        def update(self, *a, **k): pass
        def close(self): pass
        def __enter__(self): return self
        def __exit__(self, *e): pass

RANDOM_STATE = 42

# ------------------------------- Helpers -------------------------------------

def ensure_cols(df, cols, name):
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise ValueError(f"Missing {name} columns, e.g.: {missing[:8]}")

def vector_metrics(y_true, y_pred):
    """Return vector RMSE/MAE (3D distance) and per-axis RMSE/MAE."""
    dif = y_pred - y_true
    d = np.linalg.norm(dif, axis=1)  # per-sample 3D distance [m]
    vec_rmse = float(np.sqrt(np.mean(d**2)))
    vec_mae  = float(np.mean(d))
    rmse_axes = np.sqrt(np.mean(dif**2, axis=0)).tolist()
    mae_axes  = np.mean(np.abs(dif), axis=0).tolist()
    return vec_rmse, vec_mae, rmse_axes, mae_axes

def build_pose_groups_xyz(df, pos_bin_m=0.001):
    """Quantize xyz to mm bins to form groups for leakage-safe CV/test splits."""
    xq = np.round(df['act_tcp_x'].values/pos_bin_m).astype(np.int64)
    yq = np.round(df['act_tcp_y'].values/pos_bin_m).astype(np.int64)
    zq = np.round(df['act_tcp_z'].values/pos_bin_m).astype(np.int64)
    return pd.Series([f"{a}_{b}_{c}" for a,b,c in zip(xq,yq,zq)], index=df.index)

def auto_learning_curve_points(n_trainval):
    """Sensible train sizes for learning curve (grouped subsets)."""
    if n_trainval < 120:
        pts = [max(20, n_trainval//4), max(40, n_trainval//2), n_trainval]
    else:
        pts = np.unique(np.clip(np.round(np.linspace(150, n_trainval, 6)).astype(int),
                                 50, n_trainval)).tolist()
        if pts[-1] != n_trainval: pts[-1] = n_trainval
    return list(pts)

def sample_groups_for_size(groups, desired_n, rng):
    """Pick whole groups until reaching ~desired_n rows."""
    groups = np.asarray(groups)
    unique, counts = np.unique(groups, return_counts=True)
    order = rng.permutation(len(unique))
    selected, total = [], 0
    for i in order:
        g = unique[i]; selected.append(g); total += counts[i]
        if total >= desired_n: break
    return np.isin(groups, selected)

# ------------------------------ Plotting -------------------------------------

def save_pred_vs_true_plots_pos(y_true_pos, y_pred_pos, outdir, prefix=""):
    names = ["act_tcp_x","act_tcp_y","act_tcp_z"]
    for i, name in enumerate(names):
        # Scatter
        plt.figure()
        plt.scatter(y_true_pos[:, i], y_pred_pos[:, i], s=12)
        lo = min(y_true_pos[:, i].min(), y_pred_pos[:, i].min())
        hi = max(y_true_pos[:, i].max(), y_pred_pos[:, i].max())
        plt.plot([lo,hi],[lo,hi], 'k-', linewidth=1)
        plt.xlabel(f"True {name} [m]"); plt.ylabel(f"Pred {name} [m]")
        plt.title(f"Predicted vs True — {name}")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"{prefix}pred_vs_true_{name}.png"), dpi=150)

        # Residual hist
        plt.figure()
        resid = y_pred_pos[:, i] - y_true_pos[:, i]
        plt.hist(resid, bins=60)
        plt.xlabel(f"Residual ({name}) [m]"); plt.ylabel("Count")
        plt.title(f"Residuals — {name}")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"{prefix}residuals_{name}.png"), dpi=150)

def analyse_pos_robustness_vs_orientation(
    df_test, pos_true, pos_pred, outdir, nbins=12, fixed_angle_xlim=None, oneplot_stat="rmse"
):
    """
    Makes:
      - robustness_xyz_vs_{roll,pitch,yaw}.png + .csv  (common y-scale)
      - robustness_xyz_vs_rpy_oneplot.png + .csv       (roll/pitch/yaw together)
      - robustness_xyz_heatmap_pitch_yaw.png
      - robustness_xyz_vs_angles_stats.json
    """
    import json

    os.makedirs(outdir, exist_ok=True)

    roll = df_test["act_tcp_rx"].values * 180/np.pi
    pitch= df_test["act_tcp_ry"].values * 180/np.pi
    yaw  = df_test["act_tcp_rz"].values * 180/np.pi

    dif = pos_pred - pos_true
    err_vec = np.linalg.norm(dif, axis=1)

    m = np.isfinite(roll) & np.isfinite(pitch) & np.isfinite(yaw) & np.isfinite(err_vec)
    roll, pitch, yaw = roll[m], pitch[m], yaw[m]
    pos_true, pos_pred = pos_true[m], pos_pred[m]

    def _bin_stats(angle, nb):
        lo, hi = np.nanpercentile(angle, 1), np.nanpercentile(angle, 99)
        bins = np.linspace(lo, hi, nb+1)
        centers = 0.5*(bins[:-1] + bins[1:])
        idx = np.digitize(angle, bins) - 1
        rmse = np.full(nb, np.nan); mae = np.full(nb, np.nan); n = np.zeros(nb, int)
        per_axis = np.full((nb,3), np.nan, dtype=float)
        for b in range(nb):
            sel = (idx == b)
            if not np.any(sel): continue
            d = pos_pred[sel] - pos_true[sel]
            dist = np.linalg.norm(d, axis=1)
            rmse[b] = np.sqrt(np.mean(dist**2))
            mae[b]  = np.mean(dist)
            n[b]    = int(sel.sum())
            per_axis[b] = np.sqrt(np.mean(d**2, axis=0))
        return centers, rmse, mae, n, per_axis

    results = {}
    for ang, name in [(roll,"roll"), (pitch,"pitch"), (yaw,"yaw")]:
        c, r, a, n, pax = _bin_stats(ang, nbins)
        results[name] = dict(centers=c, rmse=r, mae=a, n=n, per_axis=pax)
        # CSV
        rows = []
        for i in range(len(c)):
            rows.append({
                f"{name}_center_deg": float(c[i]),
                "rmse_m": float(r[i]) if np.isfinite(r[i]) else np.nan,
                "mae_m":  float(a[i]) if np.isfinite(a[i]) else np.nan,
                "n":      int(n[i]),
                "rmse_x_m": float(pax[i,0]) if np.isfinite(pax[i,0]) else np.nan,
                "rmse_y_m": float(pax[i,1]) if np.isfinite(pax[i,1]) else np.nan,
                "rmse_z_m": float(pax[i,2]) if np.isfinite(pax[i,2]) else np.nan,
            })
        pd.DataFrame(rows).to_csv(os.path.join(outdir, f"robustness_xyz_vs_{name}.csv"), index=False)

    # Common y-scale across three separate plots
    y_all = np.concatenate([results["roll"]["rmse"], results["roll"]["mae"],
                            results["pitch"]["rmse"],results["pitch"]["mae"],
                            results["yaw"]["rmse"],  results["yaw"]["mae"]], axis=0)
    ymax = float(np.nanmax(y_all)) if np.isfinite(y_all).any() else 1.0
    y_lim = (0.0, max(1e-6, 1.05*ymax))
    x_lim = (float(fixed_angle_xlim[0]), float(fixed_angle_xlim[1])) if fixed_angle_xlim else None

    for name in ["roll","pitch","yaw"]:
        c, r, a = results[name]["centers"], results[name]["rmse"], results[name]["mae"]
        plt.figure()
        plt.plot(c, r, "o-", label="Vector RMSE (m)")
        plt.plot(c, a, "o-", label="Vector MAE (m)")
        if x_lim: plt.xlim(*x_lim)
        plt.ylim(*y_lim)
        plt.xlabel(f"{name.capitalize()} (deg)"); plt.ylabel("Position error (m)")
        plt.title(f"Position error vs {name} (common y-scale)")
        plt.grid(True, linestyle="--", lw=0.5); plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"robustness_xyz_vs_{name}.png"), dpi=160)

    # Combined one-plot (roll/pitch/yaw together) with same axes
    stat_key = "rmse" if str(oneplot_stat).lower() != "mae" else "mae"
    ylab = "Position error (vector RMSE, m)" if stat_key=="rmse" else "Position error (vector MAE, m)"
    # CSV (combined)
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

    plt.figure()
    plt.plot(results["roll"]["centers"],  results["roll"][stat_key],  "o-", label="roll",  linewidth=1.6)
    plt.plot(results["pitch"]["centers"], results["pitch"][stat_key], "o-", label="pitch", linewidth=1.6)
    plt.plot(results["yaw"]["centers"],   results["yaw"][stat_key],   "o-", label="yaw",   linewidth=1.6)
    if x_lim: plt.xlim(*x_lim)
    plt.ylim(*y_lim)
    plt.xlabel("Angle (deg)"); plt.ylabel(ylab)
    plt.title("Positional error vs roll / pitch / yaw (common axes)")
    plt.grid(True, linestyle="--", lw=0.5); plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "robustness_xyz_vs_rpy_oneplot.png"), dpi=160)

    # Optional 2D heatmap pitch×yaw (vector RMSE)
    nb = int(np.sqrt(max(9, nbins*nbins//3)))
    p_edges = np.linspace(np.nanpercentile(pitch,1), np.nanpercentile(pitch,99), nb+1)
    y_edges = np.linspace(np.nanpercentile(yaw,  1), np.nanpercentile(yaw,  99), nb+1)
    H = np.full((nb, nb), np.nan)
    pi = np.digitize(pitch, p_edges)-1
    yi = np.digitize(yaw,   y_edges)-1
    for i in range(nb):
        for j in range(nb):
            sel = (pi==i) & (yi==j)
            if np.any(sel):
                d = pos_pred[sel] - pos_true[sel]
                H[i,j] = np.sqrt(np.mean(np.sum(d**2, axis=1)))
    plt.figure()
    extent = [y_edges[0], y_edges[-1], p_edges[0], p_edges[-1]]
    plt.imshow(np.flipud(H), aspect="auto", extent=[extent[0],extent[1],extent[2],extent[3]])
    cb = plt.colorbar(); cb.set_label("Vector RMSE (m)")
    plt.xlabel("Yaw (deg)"); plt.ylabel("Pitch (deg)")
    plt.title("Position RMSE heatmap vs Pitch × Yaw")
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "robustness_xyz_heatmap_pitch_yaw.png"), dpi=160)

    # Summary stats
    stats = {"global_ymax_used_m": float(y_lim[1]), "oneplot_stat": stat_key}
    with open(os.path.join(outdir, "robustness_xyz_vs_angles_stats.json"), "w") as f:
        json.dump(stats, f, indent=2)

    print("[robustness] Saved xyz-vs-angle (3x), combined-RPY plot, heatmap, CSVs & stats.")

# ------------------------------- Main ----------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Train linear Ridge regression for XYZ from ΔEIT (pressed-hovered).")
    ap.add_argument("--csv", type=str, required=True, help="Path to dataset CSV.")
    ap.add_argument("--outdir", type=str, default="./outputs_xyz")
    ap.add_argument("--min_Fz", type=float, default=50.0, help="Keep rows with Fz >= min_Fz.")
    ap.add_argument("--contact_only", action="store_true", help="If present and column exists, keep contact_detected==1.")
    ap.add_argument("--test_size", type=float, default=0.2)
    ap.add_argument("--cv_folds", type=int, default=5)
    ap.add_argument("--alpha_grid", type=str, default="0.03,0.1,0.3,1,3,10,30,100",
                    help="Comma-separated Ridge alphas to try.")
    ap.add_argument("--lc_points", type=str, default="auto",
                    help="'auto' or comma-separated train sizes (integers).")
    ap.add_argument("--no_progress", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    df = pd.read_csv(args.csv)

    # Optional contact filter
    if 'contact_detected' in df.columns and args.contact_only:
        df = df[df['contact_detected']==1].copy()

    # Force threshold
    if 'Fz' not in df.columns:
        raise ValueError("CSV missing 'Fz' column.")
    pre = len(df)
    df = df[df['Fz'] >= args.min_Fz].copy()
    print(f"[filter] Kept {len(df)}/{pre} rows (Fz >= {args.min_Fz}).")

    # Build ΔEIT features: eita - eitb (pressed minus hovered)
    eitb = [f"eitb_{i}" for i in range(256)]
    eita = [f"eita_{i}" for i in range(256)]
    ensure_cols(df, eitb, "eitb_*"); ensure_cols(df, eita, "eita_*")
    X = (df[eita].values - df[eitb].values).astype(np.float32)

    # Targets: xyz in meters
    for c in ["act_tcp_x","act_tcp_y","act_tcp_z"]:
        if c not in df.columns:
            raise ValueError(f"CSV missing required target '{c}'")
    Y = df[["act_tcp_x","act_tcp_y","act_tcp_z"]].values.astype(np.float32)

    # Drop any NaN/Inf
    good = np.isfinite(X).all(axis=1) & np.isfinite(Y).all(axis=1)
    if not good.all():
        dropped = int((~good).sum())
        X = X[good]; Y = Y[good]; df = df.loc[good].reset_index(drop=True)
        print(f"[clean] Dropped {dropped} rows with NaN/Inf.")

    # Grouped split (by position bins) to avoid leakage
    groups_all = build_pose_groups_xyz(df)
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=RANDOM_STATE)
    trval_idx, te_idx = next(gss.split(X, Y, groups=groups_all))
    X_trval, Y_trval = X[trval_idx], Y[trval_idx]
    X_test, Y_test   = X[te_idx],   Y[te_idx]
    groups_trval = groups_all.iloc[trval_idx]
    df_test = df.iloc[te_idx].copy()
    print(f"[split] Train+Val={X_trval.shape[0]}  Test={X_test.shape[0]}  Total={X.shape[0]}")

    # Pipeline: Standardize X, Ridge; scale Y via TransformedTargetRegressor
    def make_model(alpha):
        base = Pipeline([
            ("x_scaler", StandardScaler()),
            ("ridge", Ridge(alpha=float(alpha), fit_intercept=True))
        ])
        return TransformedTargetRegressor(regressor=base, transformer=StandardScaler())

    # Grid search (manual) with GroupKFold based on *vector RMSE (3D distance)*
    alphas = [float(s) for s in args.alpha_grid.split(",")]
    cv = GroupKFold(n_splits=args.cv_folds)
    best_alpha, best_score = None, np.inf
    cv_rows = []
    total = len(alphas) * args.cv_folds

    with tqdm(total=total, desc="CV (alphas × folds)", disable=(args.no_progress or not _TQDM)) as pbar:
        for a in alphas:
            fold_scores = []
            for tr_idx, va_idx in cv.split(X_trval, Y_trval, groups=groups_trval):
                model = make_model(a)
                model.fit(X_trval[tr_idx], Y_trval[tr_idx])
                pred = model.predict(X_trval[va_idx])
                true = Y_trval[va_idx]
                # vector RMSE (3D distance)
                dist = np.linalg.norm(pred - true, axis=1)
                score = float(np.sqrt(np.mean(dist**2)))
                fold_scores.append(score)
                pbar.update(1)
            mean_score = float(np.mean(fold_scores))
            cv_rows.append({"alpha": a, "mean_vector_RMSE_m": mean_score})
            if mean_score < best_score:
                best_score, best_alpha = mean_score, a

    print(f"[cv] Best alpha={best_alpha:g}  (vector RMSE={best_score:.6f} m)")
    pd.DataFrame(cv_rows).to_csv(os.path.join(args.outdir, "cv_results_xyz.csv"), index=False)

    # Train final model on all train+val
    final = make_model(best_alpha)
    final.fit(X_trval, Y_trval)

    # Test evaluation
    Y_pred = final.predict(X_test)
    vec_rmse, vec_mae, rmse_axes, mae_axes = vector_metrics(Y_test, Y_pred)
    print("\n[test] Position (xyz only):")
    print(f"   Vector RMSE = {vec_rmse:.6f} m   Vector MAE = {vec_mae:.6f} m")
    for name, r, m in zip(["act_tcp_x","act_tcp_y","act_tcp_z"], rmse_axes, mae_axes):
        print(f"{name:>11s}  RMSE={r:.6f} m   MAE={m:.6f} m")

    # Save artifacts
    dump(final, os.path.join(args.outdir, "eit_pose_linear_xyz.joblib"))
    meta = {
        "created_utc": datetime.utcnow().isoformat()+"Z",
        "csv_path": os.path.abspath(args.csv),
        "rows_after_Fz": int(len(df)),
        "test_size_groups": args.test_size,
        "cv_folds": args.cv_folds,
        "best_alpha": float(best_alpha),
        "vector_RMSE_cv_m": float(best_score),
        "notes": "Linear Ridge model optimized for XYZ only (ΔEIT features)."
    }
    with open(os.path.join(args.outdir, "eit_pose_linear_xyz_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # Report
    rep = []
    rep.append("EIT Δ -> Position (XYZ) — Linear Ridge")
    rep.append(f"Date (UTC): {meta['created_utc']}")
    rep.append(f"CSV: {meta['csv_path']}")
    rep.append(f"Samples: train+val={X_trval.shape[0]} | test={X_test.shape[0]} | total={X.shape[0]}")
    rep.append(f"Best alpha: {best_alpha:g}  (CV vector RMSE={best_score:.6f} m)")
    rep.append("")
    rep.append("[Test] XYZ metrics")
    rep.append(f"Vector RMSE = {vec_rmse:.6f} m   Vector MAE = {vec_mae:.6f} m")
    for name, r, m in zip(["act_tcp_x","act_tcp_y","act_tcp_z"], rmse_axes, mae_axes):
        rep.append(f"{name:>11s}  RMSE={r:.6f} m   MAE={m:.6f} m")
    with open(os.path.join(args.outdir, "eit_pose_linear_xyz_report.txt"), "w") as f:
        f.write("\n".join(rep))

    # Diagnostics plots
    save_pred_vs_true_plots_pos(Y_test, Y_pred, args.outdir)

    # Learning curve (train sizes vs vector RMSE/MAE), using best_alpha fixed
    def parse_points(s, n):
        if s.strip().lower() == "auto": return auto_learning_curve_points(n)
        pts = sorted(set(int(x) for x in s.split(",") if x.strip()))
        pts = [p for p in pts if p > 10]
        if not pts or pts[-1] != n: pts.append(n)
        return pts

    sizes = parse_points(args.lc_points, X_trval.shape[0])
    rng = np.random.default_rng(RANDOM_STATE)
    rows = []
    with tqdm(total=len(sizes), desc="Learning curve (train sizes)", disable=(args.no_progress or not _TQDM)) as pbar:
        for n in sizes:
            mask = sample_groups_for_size(groups_trval.values, desired_n=n, rng=rng)
            X_sub, Y_sub = X_trval[mask], Y_trval[mask]
            # Internal tiny val split for early sanity (not strictly needed for Ridge)
            model = make_model(best_alpha)
            model.fit(X_sub, Y_sub)
            # Train subset metrics
            Y_hat_tr = model.predict(X_sub)
            vrmse_tr, vmae_tr, _, _ = vector_metrics(Y_sub, Y_hat_tr)
            # Test metrics (fixed test set)
            Y_hat_te = model.predict(X_test)
            vrmse_te, vmae_te, _, _ = vector_metrics(Y_test, Y_hat_te)
            rows.append({
                "n_train": int(X_sub.shape[0]),
                "vector_rmse_train_m": vrmse_tr,
                "vector_mae_train_m": vmae_tr,
                "vector_rmse_test_m": vrmse_te,
                "vector_mae_test_m": vmae_te,
            })
            pbar.update(1)

    # --- Learning curve: finalize, save CSV, and plot (safe for newer pandas) ---
    lc_df = pd.DataFrame(rows).sort_values("n_train").reset_index(drop=True)

    lc_csv = os.path.join(args.outdir, "learning_curve_xyz.csv")
    lc_df.to_csv(lc_csv, index=False)

    # Cast Series -> NumPy to avoid matplotlib doing series[:, None]
    x = lc_df["n_train"].to_numpy()
    y_rmse = lc_df["vector_rmse_test_m"].to_numpy()
    y_mae  = lc_df["vector_mae_test_m"].to_numpy()

    plt.figure()
    plt.plot(x, y_rmse, "o-", label="Vector RMSE (test)")
    plt.plot(x, y_mae,  "o-", label="Vector MAE (test)")
    plt.xlabel("Training samples (grouped subset count)")
    plt.ylabel("Position error (m)")
    plt.title("Learning Curve: ΔEIT → XYZ (Ridge)")
    plt.grid(True, linestyle="--", lw=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(args.outdir, "learning_curve_xyz.png"), dpi=160)



    # Robustness vs orientation (common y-scale) + combined RPY plot
    analyse_pos_robustness_vs_orientation(
        df_test=df_test,
        pos_true=Y_test,
        pos_pred=Y_pred,
        outdir=args.outdir,
        nbins=12,
        fixed_angle_xlim=(-180, 180),
        oneplot_stat="rmse"
    )

    print(f"\n[save] Artifacts written to: {os.path.abspath(args.outdir)}")
    return 0

if __name__ == "__main__":
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")  # quiet down any benign sklearn warnings
        raise SystemExit(main())