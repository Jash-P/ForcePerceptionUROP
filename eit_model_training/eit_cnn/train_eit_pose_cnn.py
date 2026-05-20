#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EIT Δ (pressed - hovered) → UR5 TCP Pose (x,y,z, rx,ry,rz) — CNN Regressor (PyTorch)

This script mirrors the linear baseline pipeline but uses a small 1D CNN
to map 256-dim ΔEIT features to the 6-DoF pose.

Key features:
- Filters rows with Fz < --min_Fz (default: 50)
- ΔEIT = eita_* - eitb_* as input (256 channels)
- Targets = actual TCP (act_tcp_x/y/z/rx/ry/rz), rx/ry/rz in radians
- Group-aware final test split (GroupShuffleSplit by quantised actual TCP)
- GroupKFold cross-validated hyperparameter search (lr, weight decay, channels)
- Early stopping on validation loss
- Learning curve over increasing train sizes (by groups), with train vs test metrics
- Diagnostics: per-axis RMSE/MAE, bias & percentiles, geodesic orientation error,
  predicted-vs-true plots, residual histograms, error vs Fz/y/pitch plots
- Progress bars via tqdm (disable with --no_progress)
- Saves model (.pt), scalers (.npz), meta (.json), report (.txt), learning-curve (.csv/.png)

Install dependencies:
  pip install numpy pandas scikit-learn joblib matplotlib tqdm torch

On Apple Silicon (M2): PyTorch supports 'mps' device if installed from pytorch.org.

Usage:
  python train_eit_pose_cnn.py --csv /path/to/your.csv --outdir ./outputs
"""

import argparse
import json
import os
from datetime import datetime

import numpy as np
import pandas as pd
from sklearn.model_selection import GroupKFold, GroupShuffleSplit
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error

# Matplotlib for saving plots (no interactive backend)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# tqdm progress bars (safe fallback if not available)
try:
    from tqdm.auto import tqdm
    _TQDM_AVAILABLE = True
except Exception:
    _TQDM_AVAILABLE = False
    class tqdm:
        def __init__(self, *a, **k): pass
        def update(self, *a, **k): pass
        def close(self): pass
        def __enter__(self): return self
        def __exit__(self, *exc): pass
        def __iter__(self): return iter([])

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader

# ---------------------- Geometry & plotting helpers (shared) ------------------

def axis_angle_to_rotmat(v):
    v = np.asarray(v, dtype=float)
    theta = np.linalg.norm(v)
    if theta < 1e-12:
        return np.eye(3)
    k = v / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)

def geodesic_angle_deg(r_pred, r_true):
    R_p = axis_angle_to_rotmat(r_pred)
    R_t = axis_angle_to_rotmat(r_true)
    R_e = R_p @ R_t.T
    tr = np.trace(R_e)
    val = (tr - 1.0) / 2.0
    val = np.clip(val, -1.0, 1.0)
    ang = np.arccos(val)
    return float(ang * 180.0 / np.pi)

def per_axis_metrics(y_true, y_pred, target_names):
    mse = np.mean((y_true - y_pred) ** 2, axis=0)
    rmse = np.sqrt(mse)
    mae = np.mean(np.abs(y_true - y_pred), axis=0)
    metrics = {}
    for i, name in enumerate(target_names):
        entry = {"rmse": float(rmse[i]), "mae": float(mae[i])}
        if name in ("act_tcp_rx", "act_tcp_ry", "act_tcp_rz"):
            entry["rmse_deg"] = float(rmse[i] * 180.0 / np.pi)
            entry["mae_deg"]  = float(mae[i] * 180.0 / np.pi)
        metrics[name] = entry
    return metrics

def format_metrics(metrics):
    lines = []
    for k, v in metrics.items():
        if "rx" in k or "ry" in k or "rz" in k:
            lines.append(f"{k:>11s}  RMSE={v['rmse']:.4f} rad ({v['rmse_deg']:.2f}°)   MAE={v['mae']:.4f} rad ({v['mae_deg']:.2f}°)")
        else:
            lines.append(f"{k:>11s}  RMSE={v['rmse']:.4f} m           MAE={v['mae']:.4f} m")
    return "\n".join(lines)

def compute_bias_percentiles(y_true, y_pred, target_names):
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
    lines = []
    for k, v in stats.items():
        if "rx" in k or "ry" in k or "rz" in k:
            lines.append(f"{k:>11s}  bias={v['bias']:.4f} rad | |err| P50={v['p50']:.4f} rad, P90={v['p90']:.4f}, P95={v['p95']:.4f}")
        else:
            lines.append(f"{k:>11s}  bias={v['bias']:.4f} m   | |err| P50={v['p50']:.4f} m, P90={v['p90']:.4f}, P95={v['p95']:.4f}")
    return "\n".join(lines)

def save_pred_vs_true_plots(y_true, y_pred, target_names, outdir):
    for i, name in enumerate(target_names):
        plt.figure()
        plt.scatter(y_true[:, i], y_pred[:, i], s=10)
        lims = [min(y_true[:, i].min(), y_pred[:, i].min()), max(y_true[:, i].max(), y_pred[:, i].max())]
        plt.plot(lims, lims)
        plt.xlabel(f"True {name}")
        plt.ylabel(f"Pred {name}")
        plt.title(f"Predicted vs True — {name}")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"cnn_pvstrue_{name}.png"), dpi=140)

        plt.figure()
        resid = y_pred[:, i] - y_true[:, i]
        plt.hist(resid, bins=50)
        plt.xlabel(f"Residual ({name})")
        plt.ylabel("Count")
        plt.title(f"Residuals — {name}")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"cnn_residuals_{name}.png"), dpi=140)

def binned_stats(x, err_vals, bins):
    x = np.asarray(x); err_vals = np.asarray(err_vals)
    idx = np.digitize(x, bins) - 1
    centers = 0.5*(bins[:-1] + bins[1:])
    mae = []
    for b in range(len(centers)):
        mask = idx == b
        if np.any(mask):
            mae.append(float(np.mean(np.abs(err_vals[mask]))))
        else:
            mae.append(np.nan)
    return centers, np.array(mae)

def save_error_vs_slice_plots(df_test, y_true, y_pred, outdir):
    pos_abs = np.abs(y_pred[:, :3] - y_true[:, :3])
    pos_mae_sample = np.mean(pos_abs, axis=1)

    geo = []
    for i in range(y_true.shape[0]):
        geo.append(geodesic_angle_deg(y_pred[i, 3:6], y_true[i, 3:6]))
    geo = np.array(geo)

    if "Fz" in df_test.columns:
        fz = df_test["Fz"].values
        bins = np.linspace(np.nanmin(fz), np.nanmax(fz), 8)
        c, mae_pos = binned_stats(fz, pos_mae_sample, bins)
        _, mae_geo = binned_stats(fz, geo, bins)
        plt.figure(); plt.plot(c, mae_pos, marker="o"); plt.xlabel("Fz"); plt.ylabel("Mean |pos error| (m)"); plt.title("Position error vs Fz"); plt.grid(True, linestyle="--", linewidth=0.5); plt.tight_layout(); plt.savefig(os.path.join(outdir, "cnn_error_vs_Fz_position.png"), dpi=140)
        plt.figure(); plt.plot(c, mae_geo, marker="o"); plt.xlabel("Fz"); plt.ylabel("Geodesic ori error (deg)"); plt.title("Orientation error vs Fz"); plt.grid(True, linestyle="--", linewidth=0.5); plt.tight_layout(); plt.savefig(os.path.join(outdir, "cnn_error_vs_Fz_orientation.png"), dpi=140)

    yvals = df_test["act_tcp_y"].values if "act_tcp_y" in df_test.columns else y_true[:,1]
    ybins = np.linspace(np.nanmin(yvals), np.nanmax(yvals), 8)
    c, mae_pos = binned_stats(yvals, pos_mae_sample, ybins)
    _, mae_geo = binned_stats(yvals, geo, ybins)
    plt.figure(); plt.plot(c, mae_pos, marker="o"); plt.xlabel("act_tcp_y"); plt.ylabel("Mean |pos error| (m)"); plt.title("Position error vs act_tcp_y"); plt.grid(True, linestyle="--", linewidth=0.5); plt.tight_layout(); plt.savefig(os.path.join(outdir, "cnn_error_vs_y_position.png"), dpi=140)
    plt.figure(); plt.plot(c, mae_geo, marker="o"); plt.xlabel("act_tcp_y"); plt.ylabel("Geodesic ori error (deg)"); plt.title("Orientation error vs act_tcp_y"); plt.grid(True, linestyle="--", linewidth=0.5); plt.tight_layout(); plt.savefig(os.path.join(outdir, "cnn_error_vs_y_orientation.png"), dpi=140)

    pitch = df_test["act_tcp_ry"].values if "act_tcp_ry" in df_test.columns else y_true[:,4]
    pitch_deg = pitch * 180.0 / np.pi
    pbins = np.linspace(np.nanmin(pitch_deg), np.nanmax(pitch_deg), 8)
    c, mae_pos = binned_stats(pitch_deg, pos_mae_sample, pbins)
    _, mae_geo = binned_stats(pitch_deg, geo, pbins)
    plt.figure(); plt.plot(c, mae_pos, marker="o"); plt.xlabel("act_tcp_ry (deg)"); plt.ylabel("Mean |pos error| (m)"); plt.title("Position error vs pitch"); plt.grid(True, linestyle="--", linewidth=0.5); plt.tight_layout(); plt.savefig(os.path.join(outdir, "cnn_error_vs_pitch_position.png"), dpi=140)
    plt.figure(); plt.plot(c, mae_geo, marker="o"); plt.xlabel("act_tcp_ry (deg)"); plt.ylabel("Geodesic ori error (deg)"); plt.title("Orientation error vs pitch"); plt.grid(True, linestyle="--", linewidth=0.5); plt.tight_layout(); plt.savefig(os.path.join(outdir, "cnn_error_vs_pitch_orientation.png"), dpi=140)

# --------------------------- Grouping & utils --------------------------------

def build_pose_id(df, pos_bin_m=0.001, ang_bin_rad=0.05):
    xq = np.round(df['act_tcp_x'].values  / pos_bin_m).astype(np.int64)
    yq = np.round(df['act_tcp_y'].values  / pos_bin_m).astype(np.int64)
    zq = np.round(df['act_tcp_z'].values  / pos_bin_m).astype(np.int64)
    rxq = np.round(df['act_tcp_rx'].values / ang_bin_rad).astype(np.int64)
    ryq = np.round(df['act_tcp_ry'].values / ang_bin_rad).astype(np.int64)
    rzq = np.round(df['act_tcp_rz'].values / ang_bin_rad).astype(np.int64)
    return pd.Series([f"{a}_{b}_{c}_{d}_{e}_{f}" for a,b,c,d,e,f in zip(xq,yq,zq,rxq,ryq,rzq)], index=df.index)

def parse_points(s, n_trainval):
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

# ------------------------------ CNN model ------------------------------------

class EIT1DCNN(nn.Module):
    def __init__(self, in_len=256, ch1=64, ch2=128, dropout=0.1):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv1d(1, ch1, kernel_size=5, padding=2),
            nn.BatchNorm1d(ch1),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Conv1d(ch1, ch2, kernel_size=5, padding=2),
            nn.BatchNorm1d(ch2),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool1d(1),  # -> (B, ch2, 1)
            nn.Flatten(),             # -> (B, ch2)
        )
        self.head = nn.Sequential(
            nn.Linear(ch2, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(128, 6)  # x,y,z,rx,ry,rz (standardized during training)
        )

    def forward(self, x):  # x: (B, 1, 256)
        features = self.net(x)
        return self.head(features)

# ----------------------------- Training utils --------------------------------

def get_device():
    if torch.cuda.is_available():
        return torch.device("cuda")
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")

def train_model(model, X_tr, y_tr, X_va, y_va, lr=1e-3, weight_decay=1e-4,
                batch_size=64, max_epochs=150, patience=20, device=None, progress=True):
    device = device or get_device()
    model = model.to(device)
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    criterion = nn.MSELoss()

    ds_tr = TensorDataset(torch.from_numpy(X_tr).float().unsqueeze(1), torch.from_numpy(y_tr).float())
    ds_va = TensorDataset(torch.from_numpy(X_va).float().unsqueeze(1), torch.from_numpy(y_va).float())
    dl_tr = DataLoader(ds_tr, batch_size=batch_size, shuffle=True)
    dl_va = DataLoader(ds_va, batch_size=batch_size, shuffle=False)

    best_state = None
    best_val = np.inf
    epochs_no_improve = 0

    pbar = tqdm(total=max_epochs, desc="epochs", disable=not(progress and _TQDM_AVAILABLE))
    for epoch in range(max_epochs):
        model.train()
        train_loss = 0.0
        for xb, yb in dl_tr:
            xb = xb.to(device)
            yb = yb.to(device)
            optimizer.zero_grad()
            pred = model(xb)
            loss = criterion(pred, yb)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * xb.size(0)
        train_loss /= len(ds_tr)

        # validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for xb, yb in dl_va:
                xb = xb.to(device); yb = yb.to(device)
                pred = model(xb)
                loss = criterion(pred, yb)
                val_loss += loss.item() * xb.size(0)
        val_loss /= len(ds_va)

        if val_loss + 1e-8 < best_val:
            best_val = val_loss
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            epochs_no_improve = 0
        else:
            epochs_no_improve += 1
            if epochs_no_improve >= patience:
                pbar.update(max_epochs - epoch)
                break
        pbar.update(1)
    pbar.close()

    if best_state is not None:
        model.load_state_dict(best_state)

    # final val rmse (in standardized target space)
    model.eval()
    with torch.no_grad():
        xb = torch.from_numpy(X_va).float().unsqueeze(1).to(device)
        yb = torch.from_numpy(y_va).float().to(device)
        pred = model(xb).cpu().numpy()
        val_rmse = float(np.sqrt(np.mean((pred - y_va)**2)))
    return model, val_rmse

# ------------------------------- Main ----------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Train EIT→pose CNN (PyTorch) with grouped CV, early stopping, and diagnostics.")
    parser.add_argument("--csv", type=str, required=True, help="Path to CSV with eitb_*, eita_* and act_tcp_* columns.")
    parser.add_argument("--outdir", type=str, default="./outputs", help="Directory to save model and reports.")
    parser.add_argument("--min_Fz", type=float, default=50.0, help="Omit rows where Fz < min_Fz.")
    parser.add_argument("--contact_only", action="store_true", help="If set, use rows with contact_detected==1 only.")
    parser.add_argument("--test_size", type=float, default=0.2, help="Fraction of groups for final test split.")
    parser.add_argument("--cv_folds", type=int, default=5, help="GroupKFold folds for hyperparam search.")
    parser.add_argument("--lr_grid", type=str, default="0.001,0.0003", help="Comma-separated learning rates to try.")
    parser.add_argument("--wd_grid", type=str, default="0.0,0.0001", help="Comma-separated weight decays to try.")
    parser.add_argument("--ch_grid", type=str, default="64,128", help="Comma-separated first conv channel sizes to try (second is 2x).")
    parser.add_argument("--dropout", type=float, default=0.1, help="Dropout rate.")
    parser.add_argument("--batch_size", type=int, default=64, help="Batch size.")
    parser.add_argument("--epochs", type=int, default=150, help="Max training epochs per fold.")
    parser.add_argument("--patience", type=int, default=20, help="Early stopping patience (epochs).")
    parser.add_argument("--lc_points", type=str, default="auto", help="Learning-curve train sizes (e.g., '200,400,800' or 'auto').")
    parser.add_argument("--lc_epochs", type=int, default=80, help="Epochs for each learning-curve refit (smaller to save time).")
    parser.add_argument("--no_progress", action="store_true", help="Disable tqdm progress bars.")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # 1) Load CSV
    df = pd.read_csv(args.csv)

    # Optional filters
    if 'contact_detected' in df.columns and args.contact_only:
        df = df[df['contact_detected'] == 1].copy()
    if 'Fz' not in df.columns:
        raise ValueError("Column 'Fz' not found in CSV; required for filtering.")
    pre_filter_n = len(df)
    df = df[df['Fz'] >= args.min_Fz].copy()
    print(f"[filter] Kept {len(df)}/{pre_filter_n} rows after Fz >= {args.min_Fz} filter.")

    # 2) Build ΔEIT features
    eitb_cols = [f"eitb_{i}" for i in range(256)]
    eita_cols = [f"eita_{i}" for i in range(256)]
    for cols in (eitb_cols, eita_cols):
        missing = [c for c in cols if c not in df.columns]
        if missing:
            raise ValueError(f"Missing columns: {missing[:5]} ...")
    X = (df[eita_cols].values - df[eitb_cols].values).astype(np.float32)

    # 3) Targets
    target_names = ["act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz"]
    for t in target_names:
        if t not in df.columns:
            raise ValueError(f"Missing target column: {t}")
    y = df[target_names].values.astype(np.float32)

    # 4) Clean NaNs/Infs
    good = np.isfinite(X).all(axis=1) & np.isfinite(y).all(axis=1)
    if not good.all():
        dropped = int((~good).sum())
        df = df.loc[good].reset_index(drop=True); X = X[good]; y = y[good]
        print(f"[info] Dropped {dropped} rows containing NaN/Inf.")

    # 5) Group ids and split
    pose_id = build_pose_id(df, pos_bin_m=0.001, ang_bin_rad=0.05)
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=42)
    trainval_idx, test_idx = next(gss.split(X, y, groups=pose_id))
    X_trainval, y_trainval = X[trainval_idx], y[trainval_idx]
    X_test,     y_test     = X[test_idx],     y[test_idx]
    g_trainval = pose_id.iloc[trainval_idx]
    print(f"[split] Train+Val: {X_trainval.shape[0]} | Test: {X_test.shape[0]} | Total: {X.shape[0]}")

    # 6) Standardize features and targets based on train+val only
    x_scaler = StandardScaler().fit(X_trainval)
    y_scaler = StandardScaler().fit(y_trainval)
    X_trainval_std = x_scaler.transform(X_trainval).astype(np.float32)
    y_trainval_std = y_scaler.transform(y_trainval).astype(np.float32)
    X_test_std = x_scaler.transform(X_test).astype(np.float32)
    y_test_std = y_scaler.transform(y_test).astype(np.float32)

    # 7) Cross-validated hyperparameter search
    lrs = [float(s) for s in args.lr_grid.split(",")]
    wds = [float(s) for s in args.wd_grid.split(",")]
    chs = [int(s) for s in args.ch_grid.split(",")]

    combos = [(lr, wd, ch) for lr in lrs for wd in wds for ch in chs]
    cv = GroupKFold(n_splits=args.cv_folds)
    best_combo = None
    best_score = np.inf
    results = []

    total_steps = len(combos) * args.cv_folds
    with tqdm(total=total_steps, desc="CV (combos × folds)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for (lr, wd, ch) in combos:
            fold_scores = []
            for fold, (tr, va) in enumerate(cv.split(X_trainval_std, y_trainval_std, groups=g_trainval), 1):
                X_tr, y_tr = X_trainval_std[tr], y_trainval_std[tr]
                X_va, y_va = X_trainval_std[va], y_trainval_std[va]

                model = EIT1DCNN(in_len=256, ch1=ch, ch2=ch*2, dropout=args.dropout)
                _model, val_rmse = train_model(
                    model, X_tr, y_tr, X_va, y_va,
                    lr=lr, weight_decay=wd, batch_size=args.batch_size,
                    max_epochs=args.epochs, patience=args.patience,
                    device=None, progress=not args.no_progress
                )
                fold_scores.append(val_rmse)
                pbar.update(1)
            mean_rmse = float(np.mean(fold_scores))
            results.append({"lr": lr, "wd": wd, "ch1": ch, "cv_rmse": mean_rmse})
            if mean_rmse < best_score:
                best_score = mean_rmse
                best_combo = (lr, wd, ch)

    print("[cv] Results (mean val RMSE in standardized space):")
    for r in results:
        print(f"   lr={r['lr']:g} wd={r['wd']:g} ch1={r['ch1']} -> cv_rmse={r['cv_rmse']:.6f}")
    print(f"[cv] Best combo: lr={best_combo[0]:g} wd={best_combo[1]:g} ch1={best_combo[2]} (mean RMSE={best_score:.6f})")

    # 8) Train final model on all train+val (use a small internal val for early stopping)
    #    Here we use a 90/10 split by groups for early stopping only.
    inner_gss = GroupShuffleSplit(n_splits=1, test_size=0.1, random_state=123)
    tr_idx, va_idx = next(inner_gss.split(X_trainval_std, y_trainval_std, groups=g_trainval))
    X_tr, y_tr = X_trainval_std[tr_idx], y_trainval_std[tr_idx]
    X_va, y_va = X_trainval_std[va_idx], y_trainval_std[va_idx]

    lr, wd, ch = best_combo
    final_model = EIT1DCNN(in_len=256, ch1=ch, ch2=ch*2, dropout=args.dropout)
    final_model, _ = train_model(
        final_model, X_tr, y_tr, X_va, y_va,
        lr=lr, weight_decay=wd, batch_size=args.batch_size,
        max_epochs=args.epochs, patience=args.patience,
        device=None, progress=not args.no_progress
    )

    # 9) Evaluate on TEST (convert back from standardized space)
    device = get_device()
    final_model.eval().to(device)
    with torch.no_grad():
        xb = torch.from_numpy(X_test_std).float().unsqueeze(1).to(device)
        pred_std = final_model(xb).cpu().numpy()
    y_pred = y_scaler.inverse_transform(pred_std)

    overall_rmse = float(np.sqrt(mean_squared_error(y_test, y_pred)))
    overall_mae  = float(mean_absolute_error(y_test, y_pred))
    per_axis = per_axis_metrics(y_test, y_pred, target_names)

    print("\n[test] Overall:")
    print(f"   RMSE = {overall_rmse:.6f}   MAE = {overall_mae:.6f}")
    print("\n[test] Per-axis:")
    print(format_metrics(per_axis))

    # Additional diagnostics
    bias_pct = compute_bias_percentiles(y_test, y_pred, target_names)
    geo_errors_deg = np.array([geodesic_angle_deg(y_pred[i,3:6], y_test[i,3:6]) for i in range(y_test.shape[0])])
    geo_summary = {
        "mean_deg": float(np.mean(geo_errors_deg)),
        "median_deg": float(np.median(geo_errors_deg)),
        "p90_deg": float(np.percentile(geo_errors_deg, 90)),
        "p95_deg": float(np.percentile(geo_errors_deg, 95)),
    }

    # Plots
    df_test = df.iloc[test_idx].copy()
    save_pred_vs_true_plots(y_test, y_pred, target_names, args.outdir)
    save_error_vs_slice_plots(df_test, y_test, y_pred, args.outdir)

    # 10) Save model & scalers
    model_path = os.path.join(args.outdir, "eit_pose_cnn.pt")
    torch.save({
        "state_dict": final_model.state_dict(),
        "arch": {"in_len": 256, "ch1": ch, "ch2": ch*2, "dropout": args.dropout},
    }, model_path)
    np.savez(os.path.join(args.outdir, "eit_pose_cnn_scalers.npz"),
             x_mean=x_scaler.mean_, x_scale=x_scaler.scale_,
             y_mean=y_scaler.mean_, y_scale=y_scaler.scale_)

    meta = {
        "created_utc": datetime.utcnow().isoformat() + "Z",
        "random_state": 42,
        "csv_path": os.path.abspath(args.csv),
        "rows_after_Fz": int(len(df)),
        "test_size_groups": args.test_size,
        "cv_folds": args.cv_folds,
        "lr_grid": lrs,
        "wd_grid": wds,
        "ch_grid": chs,
        "best_combo": {"lr": best_combo[0], "wd": best_combo[1], "ch1": best_combo[2]},
        "batch_size": args.batch_size,
        "epochs": args.epochs,
        "patience": args.patience,
        "target_names": target_names,
        "notes": "CNN trained on standardized ΔEIT and targets; outputs inverse-transformed to original units."
    }
    with open(os.path.join(args.outdir, "eit_pose_cnn_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # 11) Text report
    report = []
    report.append("EIT → Pose CNN (PyTorch)")
    report.append(f"Date (UTC): {meta['created_utc']}")
    report.append(f"CSV: {meta['csv_path']}")
    report.append(f"Rows after Fz filter (>= {args.min_Fz}): {len(df)}")
    report.append(f"Train+Val: {X_trainval.shape[0]} | Test: {X_test.shape[0]} | Total: {X.shape[0]}")
    report.append(f"Best combo: lr={best_combo[0]:g}, wd={best_combo[1]:g}, ch1={best_combo[2]}")
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
    report.append(f"mean={geo_summary['mean_deg']:.2f}°, median={geo_summary['median_deg']:.2f}°, P90={geo_summary['p90_deg']:.2f}°, P95={geo_summary['p95_deg']:.2f}°")
    report.append("")
    report.append("Saved figures:")
    report.append(" - cnn_pvstrue_*.png (predicted vs true per axis)")
    report.append(" - cnn_residuals_*.png (residual histograms per axis)")
    report.append(" - cnn_error_vs_Fz_position.png / cnn_error_vs_Fz_orientation.png")
    report.append(" - cnn_error_vs_y_position.png / cnn_error_vs_y_orientation.png")
    report.append(" - cnn_error_vs_pitch_position.png / cnn_error_vs_pitch_orientation.png")
    with open(os.path.join(args.outdir, "eit_pose_cnn_report.txt"), "w") as f:
        f.write("\n".join(report))

    print(f"\n[save] Model   -> {model_path}")
    print(f"[save] Scalers -> {os.path.join(args.outdir, 'eit_pose_cnn_scalers.npz')}")
    print(f"[save] Meta    -> {os.path.join(args.outdir, 'eit_pose_cnn_meta.json')}")
    print(f"[save] Report  -> {os.path.join(args.outdir, 'eit_pose_cnn_report.txt')}")

    # 12) Learning curve: train on increasing group-sized subsets and evaluate
    rng = np.random.default_rng(42)
    sizes = parse_points(args.lc_points, X_trainval_std.shape[0])
    lc_rows = []
    with tqdm(total=len(sizes), desc="Learning curve (train sizes)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for n in sizes:
            mask = sample_groups_for_size(g_trainval.values, desired_n=n, rng=rng)
            X_sub, y_sub = X_trainval_std[mask], y_trainval_std[mask]

            # 90/10 split inside subset for early stopping
            inner = GroupShuffleSplit(n_splits=1, test_size=0.1, random_state=123)
            g_sub = pose_id.iloc[trainval_idx][mask]
            tr_i, va_i = next(inner.split(X_sub, y_sub, groups=g_sub))
            X_tr, y_tr = X_sub[tr_i], y_sub[tr_i]
            X_va, y_va = X_sub[va_i], y_sub[va_i]

            m = EIT1DCNN(in_len=256, ch1=best_combo[2], ch2=best_combo[2]*2, dropout=args.dropout)
            m, _ = train_model(
                m, X_tr, y_tr, X_va, y_va,
                lr=best_combo[0], weight_decay=best_combo[1], batch_size=args.batch_size,
                max_epochs=args.lc_epochs, patience=max(10, args.lc_epochs//4),
                device=None, progress=False
            )

            # train metrics on subset
            with torch.no_grad():
                device = get_device(); m.eval().to(device)
                xb_tr = torch.from_numpy(X_sub).float().unsqueeze(1).to(device)
                pred_tr_std = m(xb_tr).cpu().numpy()
            pred_tr = y_scaler.inverse_transform(pred_tr_std)
            tr_rmse = float(np.sqrt(mean_squared_error(y_trainval[mask], pred_tr)))
            tr_mae  = float(mean_absolute_error(y_trainval[mask], pred_tr))
            per_axis_tr = per_axis_metrics(y_trainval[mask], pred_tr, target_names)

            # test metrics (fixed test set)
            with torch.no_grad():
                xb_te = torch.from_numpy(X_test_std).float().unsqueeze(1).to(device)
                pred_te_std = m(xb_te).cpu().numpy()
            pred_te = y_scaler.inverse_transform(pred_te_std)
            te_rmse = float(np.sqrt(mean_squared_error(y_test, pred_te)))
            te_mae  = float(mean_absolute_error(y_test, pred_te))
            per_axis_te = per_axis_metrics(y_test, pred_te, target_names)

            row = {
                "n_train": int(X_sub.shape[0]),
                "rmse_overall_train": tr_rmse,
                "mae_overall_train":  tr_mae,
                "rmse_overall_test":  te_rmse,
                "mae_overall_test":   te_mae,
            }
            for k, v in per_axis_tr.items():
                row[f"rmse_train_{k}"] = v["rmse"]; row[f"mae_train_{k}"] = v["mae"]
            for k, v in per_axis_te.items():
                row[f"rmse_test_{k}"]  = v["rmse"]; row[f"mae_test_{k}"]  = v["mae"]
            lc_rows.append(row)

            print(f"[lc] n_train={X_sub.shape[0]}  train_RMSE={tr_rmse:.6f}  test_RMSE={te_rmse:.6f}")
            pbar.update(1)

    lc_df = pd.DataFrame(lc_rows).sort_values("n_train")
    lc_csv = os.path.join(args.outdir, "eit_pose_cnn_learning_curve.csv")
    lc_df.to_csv(lc_csv, index=False)

    # Plot learning curve
    plt.figure()
    plt.plot(lc_df["n_train"].values, lc_df["rmse_overall_test"].values, marker="o", label="Test RMSE")
    plt.plot(lc_df["n_train"].values, lc_df["rmse_overall_train"].values, marker="o", label="Train RMSE")
    plt.xlabel("Training samples (by grouped subset)")
    plt.ylabel("RMSE (overall)")
    plt.title("Learning Curve: ΔEIT → Pose (CNN)")
    plt.grid(True, linestyle="--", linewidth=0.5)
    plt.legend()
    lc_png = os.path.join(args.outdir, "eit_pose_cnn_learning_curve.png")
    plt.tight_layout(); plt.savefig(lc_png, dpi=160)

    print(f"[save] Learning-curve CSV -> {lc_csv}")
    print(f"[save] Learning-curve PNG -> {lc_png}")
    return 0

# --------------------- Inference helper (reuse in runtime) -------------------

def load_cnn(model_path, scalers_path, device=None):
    device = device or get_device()
    ckpt = torch.load(model_path, map_location=device)
    arch = ckpt["arch"]
    model = EIT1DCNN(in_len=arch["in_len"], ch1=arch["ch1"], ch2=arch["ch2"], dropout=arch["dropout"])
    model.load_state_dict(ckpt["state_dict"])
    model.eval().to(device)
    scal = np.load(scalers_path)
    x_mean, x_scale = scal["x_mean"], scal["x_scale"]
    y_mean, y_scale = scal["y_mean"], scal["y_scale"]
    return model, (x_mean, x_scale, y_mean, y_scale), device

def predict_pose_cnn(delta_eit_256, model=None, scalers=None, device=None):
    """
    Predict [x,y,z,rx,ry,rz] from a 256-dim ΔEIT vector.
    """
    device = device or get_device()
    delta = np.asarray(delta_eit_256, dtype=np.float32).reshape(1, -1)
    if delta.shape[1] != 256:
        raise ValueError("delta_eit_256 must have 256 elements.")
    x_mean, x_scale, y_mean, y_scale = scalers
    delta_std = (delta - x_mean) / x_scale
    with torch.no_grad():
        xb = torch.from_numpy(delta_std).float().unsqueeze(1).to(device)
        pred_std = model(xb).cpu().numpy()
    pred = pred_std * y_scale + y_mean
    return pred[0]

if __name__ == "__main__":
    raise SystemExit(main())
