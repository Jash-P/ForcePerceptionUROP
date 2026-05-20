#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EIT Δ → UR5 TCP Pose — MLP (Quaternion Orientation + Geodesic Loss)

This trains a Multi-Layer Perceptron to predict:
  [x, y, z, qw, qx, qy, qz]

Loss per batch:
  L = MSE(standardized position) + lambda_rot * (geodesic(q_pred, q_true) [rad])^2

Model selection (CV):
  Uses a composite validation score:
    pos_RMSE_m + ori_weight * mean_geodesic_deg

Outputs (in --outdir):
  - eit_pose_mlp_quat.pt                (model state dict + arch)
  - eit_pose_mlp_quat_scalers.npz       (X and position scalers)
  - eit_pose_mlp_quat_meta.json         (metadata / chosen hyperparams)
  - eit_pose_mlp_quat_report.txt        (test metrics summary)
  - Pred-vs-True and residual plots for xyz
  - Axis–angle (rx,ry,rz) plots derived from predicted quaternions
  - Learning curve CSV/PNG for position RMSE and geodesic orientation error

Install:
  python3 -m pip install numpy pandas scikit-learn matplotlib tqdm torch
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

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

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
        def __exit__(self, *e): pass
        def __iter__(self): return iter([])

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader

RANDOM_STATE = 42

# ------------------------- Rotation helpers ----------------------------------

def axis_angle_to_quat(r):
    r = np.asarray(r, dtype=float)
    theta = np.linalg.norm(r, axis=-1, keepdims=True)
    small = theta < 1e-12
    half = 0.5 * theta
    w = np.cos(half)
    s = np.zeros_like(theta); s[~small] = np.sin(half[~small]) / theta[~small]
    v = r * s
    q = np.concatenate([w, v], axis=-1)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    return q

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)

def quat_geodesic_deg(qp, qt):
    qp = quat_normalize(qp); qt = quat_normalize(qt)
    dot = np.sum(qp * qt, axis=-1)
    dot = np.clip(np.abs(dot), -1.0, 1.0)
    ang = 2.0 * np.arccos(dot)  # radians
    return ang * 180.0 / np.pi

def quat_geodesic_rad_torch(qp, qt):
    # qp, qt: (B,4) torch tensors (not necessarily normalized)
    qp = qp / qp.norm(dim=1, keepdim=True).clamp_min(1e-12)
    qt = qt / qt.norm(dim=1, keepdim=True).clamp_min(1e-12)
    dot = torch.sum(qp * qt, dim=1).abs().clamp(-1.0, 1.0)
    ang = 2.0 * torch.arccos(dot)  # radians in [0, pi]
    return ang

def quat_to_axis_angle(q):
    """Return axis–angle vector (rx,ry,rz) from quaternion (w,x,y,z)."""
    q = quat_normalize(q)
    w, x, y, z = q[...,0], q[...,1], q[...,2], q[...,3]
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sqrt(1.0 - w*w)
    axis = np.zeros(q.shape[:-1] + (3,))
    small = s < 1e-12
    axis[~small, 0] = x[~small] / s[~small]
    axis[~small, 1] = y[~small] / s[~small]
    axis[~small, 2] = z[~small] / s[~small]
    return axis * angle[...,None]

def quat_to_axis_angle_ur(q):
    """
    Quaternion (w,x,y,z) -> axis-angle vector r in UR-style convention:
    angle in (−π, π], r = axis * angle. Canonicalizes q and flips axis if needed.
    """
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)                    # unit
    # Canonicalize sign so w >= 0 (q and -q represent the same rotation)
    flip = q[...,0] < 0
    q[flip] *= -1.0

    w, x, y, z = q[...,0], q[...,1], q[...,2], q[...,3]
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))                       # [0, π]
    s = np.sqrt(np.maximum(1.0 - w*w, 1e-12))
    axis = np.stack([x/s, y/s, z/s], axis=-1)
    r = axis * angle[..., None]                                          # [0, π] * axis

    # Map to (−π, π]: if angle == π, flip axis to make angle = −π equivalent
    big = angle > np.pi
    if np.any(big):
        r[big] = -axis[big] * (2*np.pi - angle[big])[..., None]
    # Handle any rare +π exactly -> use −π
    near_pi = np.isclose(angle, np.pi)
    r[near_pi] *= -1.0
    return r

# --------------------------- Model (MLP) -------------------------------------

class MLPQuat(nn.Module):
    def __init__(self, in_dim=256, hidden="512,256,128", dropout=0.1):
        super().__init__()
        sizes = [int(s) for s in hidden.split(",") if s.strip()]
        layers = []
        last = in_dim
        for h in sizes:
            layers += [nn.Linear(last, h), nn.ReLU(inplace=True), nn.Dropout(dropout)]
            last = h
        # Output 7: [pos_std(3), quat_raw(4)]
        layers += [nn.Linear(last, 7)]
        self.net = nn.Sequential(*layers)

    def forward(self, x):  # x: (B,256) standardized input
        y = self.net(x)    # (B,7)
        return y[:, :3], y[:, 3:]

# --------------------------- Train / Eval utils ------------------------------

def get_device():
    if torch.cuda.is_available(): return torch.device("cuda")
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available(): return torch.device("mps")
    return torch.device("cpu")

def train_model(model, X_tr, y_tr_pos_std, y_tr_quat, X_va, y_va_pos_std, y_va_quat,
                lr=1e-3, weight_decay=1e-4, batch_size=64, max_epochs=150, patience=20,
                lambda_rot=5.0, min_delta=0.0, device=None, progress=True):
    device = device or get_device()
    model = model.to(device)
    opt = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    mse = nn.MSELoss(reduction="mean")

    ds_tr = TensorDataset(torch.from_numpy(X_tr).float(),
                          torch.from_numpy(y_tr_pos_std).float(),
                          torch.from_numpy(y_tr_quat).float())
    ds_va = TensorDataset(torch.from_numpy(X_va).float(),
                          torch.from_numpy(y_va_pos_std).float(),
                          torch.from_numpy(y_va_quat).float())
    dl_tr = DataLoader(ds_tr, batch_size=batch_size, shuffle=True)
    dl_va = DataLoader(ds_va, batch_size=batch_size, shuffle=False)

    best_state = None
    best_val = np.inf
    no_improve = 0
    pbar = tqdm(total=max_epochs, desc="epochs", disable=not(progress and _TQDM_AVAILABLE))
    for epoch in range(max_epochs):
        model.train()
        for xb, yb_pos, yb_q in dl_tr:
            xb = xb.to(device); yb_pos = yb_pos.to(device); yb_q = yb_q.to(device)
            opt.zero_grad()
            pred_pos_std, pred_q_raw = model(xb)
            loss_pos = mse(pred_pos_std, yb_pos)
            ang = quat_geodesic_rad_torch(pred_q_raw, yb_q)  # (B,)
            loss_rot = torch.mean(ang * ang)
            loss = loss_pos + lambda_rot * loss_rot
            loss.backward(); opt.step()

        # validation
        model.eval(); val_loss = 0.0; n = 0
        with torch.no_grad():
            for xb, yb_pos, yb_q in dl_va:
                xb = xb.to(device); yb_pos = yb_pos.to(device); yb_q = yb_q.to(device)
                pred_pos_std, pred_q_raw = model(xb)
                ang = quat_geodesic_rad_torch(pred_q_raw, yb_q)
                loss = mse(pred_pos_std, yb_pos) + lambda_rot * torch.mean(ang * ang)
                val_loss += float(loss.item()) * xb.size(0); n += xb.size(0)
        val_loss /= max(1, n)

        if (best_val - val_loss) > max(1e-8, min_delta):
            best_val = val_loss
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            no_improve = 0
        else:
            no_improve += 1
            if no_improve >= patience:
                pbar.update(max_epochs - epoch); break
        pbar.update(1)
    pbar.close()
    if best_state is not None:
        model.load_state_dict(best_state)
    return model, best_val

# ------------------------------- Main ----------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Train EIT→pose MLP with quaternion orientation and geodesic loss.")
    parser.add_argument("--csv", type=str, required=True)
    parser.add_argument("--outdir", type=str, default="./outputs")
    parser.add_argument("--min_Fz", type=float, default=50.0)
    parser.add_argument("--contact_only", action="store_true")
    parser.add_argument("--test_size", type=float, default=0.2)
    parser.add_argument("--cv_folds", type=int, default=5)

    # Architecture / training
    parser.add_argument("--hidden", type=str, default="512,256,128", help="Comma-separated MLP layer sizes.")
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--min_delta", type=float, default=0.0, help="Min improvement in val loss to reset patience.")
    parser.add_argument("--lambda_rot", type=float, default=5.0, help="Weight for rotation geodesic loss term.")
    parser.add_argument("--lr_grid", type=str, default="0.001,0.0003")
    parser.add_argument("--wd_grid", type=str, default="0.0,0.0001")
    parser.add_argument("--hidden_grid", type=str, default="512,256,128|512,256|256,128", help="Grid over hidden specs; '|' separated list.")

    # Learning curve
    parser.add_argument("--lc_points", type=str, default="auto")
    parser.add_argument("--lc_epochs", type=int, default=80)

    # CV selection composite weight
    parser.add_argument("--ori_weight", type=float, default=0.01, help="Composite CV weight in m/deg for model selection.")

    parser.add_argument("--no_progress", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # Load & filter
    df = pd.read_csv(args.csv)
    if 'contact_detected' in df.columns and args.contact_only:
        df = df[df['contact_detected']==1].copy()
    if 'Fz' not in df.columns: raise ValueError("Missing 'Fz' column.")
    pre = len(df); df = df[df['Fz'] >= args.min_Fz].copy()
    print(f"[filter] Kept {len(df)}/{pre} rows (Fz >= {args.min_Fz}).")

    # ΔEIT
    eitb_cols = [f"eitb_{i}" for i in range(256)]
    eita_cols = [f"eita_{i}" for i in range(256)]
    for cols in (eitb_cols, eita_cols):
        miss = [c for c in cols if c not in df.columns]
        if miss: raise ValueError(f"Missing EIT cols, e.g. {miss[:5]}")
    X = (df[eita_cols].values - df[eitb_cols].values).astype(np.float32)

    # Targets: xyz + quat
    for t in ["act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz"]:
        if t not in df.columns: raise ValueError(f"Missing target col: {t}")
    pos = df[["act_tcp_x","act_tcp_y","act_tcp_z"]].values.astype(np.float32)
    rvec = df[["act_tcp_rx","act_tcp_ry","act_tcp_rz"]].values.astype(np.float32)
    quat = axis_angle_to_quat(rvec).astype(np.float32)

    # Clean
    good = np.isfinite(X).all(axis=1) & np.isfinite(pos).all(axis=1) & np.isfinite(quat).all(axis=1)
    if not good.all():
        dropped = int((~good).sum())
        df = df.loc[good].reset_index(drop=True); X = X[good]; pos = pos[good]; quat = quat[good]
        print(f"[info] Dropped {dropped} NaN/Inf rows.")

    # Groups & split
    def build_pose_id(df, pos_bin_m=0.001, ang_bin_rad=0.05):
        xq  = np.round(df['act_tcp_x'].values  / pos_bin_m).astype(np.int64)
        yq  = np.round(df['act_tcp_y'].values  / pos_bin_m).astype(np.int64)
        zq  = np.round(df['act_tcp_z'].values  / pos_bin_m).astype(np.int64)
        rxq = np.round(df['act_tcp_rx'].values / ang_bin_rad).astype(np.int64)
        ryq = np.round(df['act_tcp_ry'].values / ang_bin_rad).astype(np.int64)
        rzq = np.round(df['act_tcp_rz'].values / ang_bin_rad).astype(np.int64)
        return pd.Series([f"{a}_{b}_{c}_{d}_{e}_{f}" for a,b,c,d,e,f in zip(xq,yq,zq,rxq,ryq,rzq)], index=df.index)

    pose_id = build_pose_id(df)
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=RANDOM_STATE)
    trainval_idx, test_idx = next(gss.split(X, pos, groups=pose_id))
    X_trainval, pos_trainval, quat_trainval = X[trainval_idx], pos[trainval_idx], quat[trainval_idx]
    X_test, pos_test, quat_test = X[test_idx], pos[test_idx], quat[test_idx]
    g_trainval = pose_id.iloc[trainval_idx]
    print(f"[split] Train+Val={X_trainval.shape[0]}  Test={X_test.shape[0]}  Total={X.shape[0]}")

    # Standardize X and position only; quaternions remain unit-normalized.
    x_scaler = StandardScaler().fit(X_trainval)
    pos_scaler = StandardScaler().fit(pos_trainval)
    X_trainval_std = x_scaler.transform(X_trainval).astype(np.float32)
    X_test_std     = x_scaler.transform(X_test).astype(np.float32)
    pos_trainval_std = pos_scaler.transform(pos_trainval).astype(np.float32)
    pos_test_std     = pos_scaler.transform(pos_test).astype(np.float32)

    # Hyperparameter grid
    lrs  = [float(s) for s in args.lr_grid.split(",")]
    wds  = [float(s) for s in args.wd_grid.split(",")]
    hiddens = [s.strip() for s in args.hidden_grid.split("|") if s.strip()]
    combos = [(lr, wd, hid) for lr in lrs for wd in wds for hid in hiddens]

    # GroupKFold CV with composite score
    cv = GroupKFold(n_splits=args.cv_folds)
    best_combo = None
    best_score = np.inf
    results = []

    total = len(combos) * args.cv_folds
    with tqdm(total=total, desc="CV (combos × folds)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for (lr, wd, hid) in combos:
            fold_scores = []
            for tr, va in cv.split(X_trainval_std, pos_trainval_std, groups=g_trainval):
                X_tr, X_va = X_trainval_std[tr], X_trainval_std[va]
                pos_tr, pos_va = pos_trainval_std[tr], pos_trainval_std[va]
                q_tr, q_va = quat_trainval[tr], quat_trainval[va]

                model = MLPQuat(in_dim=256, hidden=hid, dropout=args.dropout)
                model, _ = train_model(model, X_tr, pos_tr, q_tr, X_va, pos_va, q_va,
                                       lr=lr, weight_decay=wd, batch_size=args.batch_size,
                                       max_epochs=args.epochs, patience=args.patience,
                                       lambda_rot=args.lambda_rot, min_delta=args.min_delta,
                                       device=None, progress=False)

                # Evaluate composite score on validation set (physical units)
                model.eval(); device = get_device(); model.to(device)
                with torch.no_grad():
                    xb = torch.from_numpy(X_va).float().to(device)
                    pos_std_pred, quat_raw_pred = model(xb)
                    pos_pred = pos_scaler.inverse_transform(pos_std_pred.cpu().numpy())
                    quat_pred = quat_raw_pred.cpu().numpy()
                    quat_pred = quat_pred / np.linalg.norm(quat_pred, axis=1, keepdims=True).clip(1e-12, None)
                pos_true = pos_scaler.inverse_transform(pos_va)
                quat_true = q_va
                pos_rmse = float(np.sqrt(mean_squared_error(pos_true, pos_pred)))
                geo_deg = float(quat_geodesic_deg(quat_pred, quat_true).mean())
                score = pos_rmse + args.ori_weight * geo_deg
                fold_scores.append(score)
                pbar.update(1)
            mean_score = float(np.mean(fold_scores))
            results.append({"lr": lr, "wd": wd, "hidden": hid, "cv_score": mean_score})
            if mean_score < best_score:
                best_score = mean_score; best_combo = (lr, wd, hid)

    print("[cv] Composite score (lower is better): pos_RMSE_m + ori_weight * mean_geo_deg")
    for r in results:
        print(f"   lr={r['lr']:g} wd={r['wd']:g} hidden=[{r['hidden']}] -> score={r['cv_score']:.6f}")
    print(f"[cv] Best combo: lr={best_combo[0]:g} wd={best_combo[1]:g} hidden=[{best_combo[2]}] (score={best_score:.6f}, ori_weight={args.ori_weight})")

    # Final model (train+val with small internal val for early stopping)
    inner_gss = GroupShuffleSplit(n_splits=1, test_size=0.1, random_state=123)
    tr_idx, va_idx = next(inner_gss.split(X_trainval_std, pos_trainval_std, groups=g_trainval))
    X_tr, X_va = X_trainval_std[tr_idx], X_trainval_std[va_idx]
    pos_tr, pos_va = pos_trainval_std[tr_idx], pos_trainval_std[va_idx]
    quat_tr, quat_va = quat_trainval[tr_idx], quat_trainval[va_idx]

    lr, wd, hid = best_combo
    final = MLPQuat(in_dim=256, hidden=hid, dropout=args.dropout)
    final, _ = train_model(final, X_tr, pos_tr, quat_tr, X_va, pos_va, quat_va,
                           lr=lr, weight_decay=wd, batch_size=args.batch_size,
                           max_epochs=args.epochs, patience=args.patience,
                           lambda_rot=args.lambda_rot, min_delta=args.min_delta,
                           device=None, progress=not args.no_progress)

    # Test evaluation
    device = get_device(); final.eval().to(device)
    with torch.no_grad():
        xb = torch.from_numpy(X_test_std).float().to(device)
        pos_std_pred, quat_raw_pred = final(xb)
        pos_pred = pos_scaler.inverse_transform(pos_std_pred.cpu().numpy())
        quat_pred = quat_raw_pred.cpu().numpy()
        quat_pred = quat_pred / np.linalg.norm(quat_pred, axis=1, keepdims=True).clip(1e-12, None)

    pos_overall_rmse = float(np.sqrt(mean_squared_error(pos_test, pos_pred)))
    pos_overall_mae  = float(mean_absolute_error(pos_test, pos_pred))
    pos_names = ["act_tcp_x","act_tcp_y","act_tcp_z"]
    pos_rmse_axes = dict(zip(pos_names, np.sqrt(np.mean((pos_test - pos_pred)**2, axis=0)).tolist()))
    pos_mae_axes  = dict(zip(pos_names, np.mean(np.abs(pos_test - pos_pred), axis=0).tolist()))

    geo_errors_deg = quat_geodesic_deg(quat_pred, quat_test)
    geo_summary = {
        "mean_deg": float(np.mean(geo_errors_deg)),
        "median_deg": float(np.median(geo_errors_deg)),
        "p90_deg": float(np.percentile(geo_errors_deg, 90)),
        "p95_deg": float(np.percentile(geo_errors_deg, 95)),
    }

    print("\n[test] Position (xyz):")
    print(f"   RMSE = {pos_overall_rmse:.6f} m   MAE = {pos_overall_mae:.6f} m")
    print("[test] Per-axis (m):")
    for k in pos_names:
        print(f"{k:>11s}  RMSE={pos_rmse_axes[k]:.4f} m   MAE={pos_mae_axes[k]:.4f} m")
    print("\n[test] Orientation (quaternion → geodesic):")
    print(f"   mean={geo_summary['mean_deg']:.2f}°, median={geo_summary['median_deg']:.2f}°, P90={geo_summary['p90_deg']:.2f}°, P95={geo_summary['p95_deg']:.2f}°")

    # Save artifacts
    torch.save({
        "state_dict": final.state_dict(),
        "arch": {"in_dim":256, "hidden":hid, "dropout":args.dropout},
    }, os.path.join(args.outdir, "eit_pose_mlp_quat.pt"))
    np.savez(os.path.join(args.outdir, "eit_pose_mlp_quat_scalers.npz"),
             x_mean=x_scaler.mean_, x_scale=x_scaler.scale_,
             pos_mean=pos_scaler.mean_, pos_scale=pos_scaler.scale_)

    meta = {
        "created_utc": datetime.utcnow().isoformat()+"Z",
        "csv_path": os.path.abspath(args.csv),
        "rows_after_Fz": int(len(df)),
        "test_size_groups": args.test_size,
        "cv_folds": args.cv_folds,
        "best_combo": {"lr": lr, "wd": wd, "hidden": hid},
        "lambda_rot": args.lambda_rot,
        "ori_weight_m_per_deg": args.ori_weight,
        "notes": "MLP with quaternion orientation and geodesic loss. Composite CV score: pos_RMSE_m + ori_weight * mean_geodesic_deg."
    }
    with open(os.path.join(args.outdir, "eit_pose_mlp_quat_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # Report
    rep = []
    rep.append("EIT → Pose MLP (Quaternion + Geodesic Loss)")
    rep.append(f"Date (UTC): {meta['created_utc']}")
    rep.append(f"CSV: {meta['csv_path']}")
    rep.append(f"Samples: train+val={X_trainval.shape[0]} | test={X_test.shape[0]} | total={X.shape[0]}")
    rep.append(f"Best combo: lr={lr:g}, wd={wd:g}, hidden=[{hid}] | lambda_rot={args.lambda_rot} | ori_weight={args.ori_weight}")
    rep.append("")
    rep.append("[Test] Position (xyz)")
    rep.append(f"RMSE = {pos_overall_rmse:.6f} m   MAE = {pos_overall_mae:.6f} m")
    for k in pos_names:
        rep.append(f"{k:>11s}  RMSE={pos_rmse_axes[k]:.4f} m   MAE={pos_mae_axes[k]:.4f} m")
    rep.append("")
    rep.append("[Test] Orientation (quaternion → geodesic)")
    rep.append(f"mean={geo_summary['mean_deg']:.2f}°, median={geo_summary['median_deg']:.2f}°, P90={geo_summary['p90_deg']:.2f}°, P95={geo_summary['p95_deg']:.2f}°")
    with open(os.path.join(args.outdir, "eit_pose_mlp_quat_report.txt"), "w") as f:
        f.write("\n".join(rep))

    # Diagnostics plots (position) + axis–angle orientation plots
    def save_pred_vs_true_plots_pos(y_true_pos, y_pred_pos, outdir):
        for i, name in enumerate(pos_names):
            plt.figure()
            plt.scatter(y_true_pos[:, i], y_pred_pos[:, i], s=10)
            lims = [min(y_true_pos[:, i].min(), y_pred_pos[:, i].min()),
                    max(y_true_pos[:, i].max(), y_pred_pos[:, i].max())]
            plt.plot(lims, lims)
            plt.xlabel(f"True {name}"); plt.ylabel(f"Pred {name}")
            plt.title(f"MLP Pred vs True — {name}")
            plt.grid(True, linestyle="--", linewidth=0.5)
            plt.tight_layout(); plt.savefig(os.path.join(outdir, f"mlp_pvstrue_{name}.png"), dpi=140)

            plt.figure()
            resid = y_pred_pos[:, i] - y_true_pos[:, i]
            plt.hist(resid, bins=50)
            plt.xlabel(f"Residual ({name})"); plt.ylabel("Count")
            plt.title(f"MLP Residuals — {name}")
            plt.grid(True, linestyle="--", linewidth=0.5)
            plt.tight_layout(); plt.savefig(os.path.join(outdir, f"mlp_residuals_{name}.png"), dpi=140)

    def save_axis_angle_plots(r_true_rad, r_pred_rad, outdir, use_degrees=True):
        names = ["act_tcp_rx","act_tcp_ry","act_tcp_rz"]
        r_true = np.asarray(r_true_rad); r_pred = np.asarray(r_pred_rad)
        unit = "deg" if use_degrees else "rad"
        if use_degrees:
            r_true = r_true * 180.0/np.pi
            r_pred = r_pred * 180.0/np.pi
        for i, name in enumerate(names):
            plt.figure()
            plt.scatter(r_true[:, i], r_pred[:, i], s=10)
            lims = [min(r_true[:, i].min(), r_pred[:, i].min()),
                    max(r_true[:, i].max(), r_pred[:, i].max())]
            plt.plot(lims, lims)
            plt.xlabel(f"True {name} ({unit})"); plt.ylabel(f"Pred {name} ({unit})")
            plt.title(f"MLP Pred vs True — {name} ({unit})")
            plt.grid(True, linestyle="--", linewidth=0.5)
            plt.tight_layout(); plt.savefig(os.path.join(outdir, f"mlp_pvstrue_{name}_{unit}.png"), dpi=140)

            plt.figure()
            resid = r_pred[:, i] - r_true[:, i]
            plt.hist(resid, bins=50)
            plt.xlabel(f"Residual ({name}) [{unit}]"); plt.ylabel("Count")
            plt.title(f"MLP Residuals — {name} ({unit})")
            plt.grid(True, linestyle="--", linewidth=0.5)
            plt.tight_layout(); plt.savefig(os.path.join(outdir, f"mlp_residuals_{name}_{unit}.png"), dpi=140)

    df_test = df.iloc[test_idx].copy()
    save_pred_vs_true_plots_pos(pos_test, pos_pred, args.outdir)

    # Axis–angle (rx,ry,rz) plots derived from predicted quaternions
    r_true = df_test[["act_tcp_rx","act_tcp_ry","act_tcp_rz"]].values  # radians
    r_pred = quat_to_axis_angle_ur(quat_pred)
    save_axis_angle_plots(r_true, r_pred, args.outdir, use_degrees=True)
    print(f"[save] MLP axis–angle plots -> mlp_pvstrue_act_tcp_r*_deg.png & mlp_residuals_act_tcp_r*_deg.png")

    # Learning curve: grouped subsets with fewer epochs (lc_epochs)
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
        if len(pts) == 0 or pts[-1] != n_trainval:
            pts.append(n_trainval)
        return pts

    def sample_groups_for_size(groups, desired_n, rng):
        groups = np.asarray(groups)
        unique, counts = np.unique(groups, return_counts=True)
        idx = np.arange(len(unique)); rng.shuffle(idx)
        selected = set(); total = 0
        for i in idx:
            g = unique[i]; selected.add(g); total += counts[i]
            if total >= desired_n: break
        return np.isin(groups, list(selected))

    rng = np.random.default_rng(RANDOM_STATE)
    sizes = parse_points(args.lc_points, X_trainval_std.shape[0])
    rows = []

    with tqdm(total=len(sizes), desc="Learning curve (train sizes)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for n in sizes:
            mask = sample_groups_for_size(g_trainval.values, desired_n=n, rng=rng)
            X_sub, pos_sub, quat_sub = X_trainval_std[mask], pos_trainval_std[mask], quat_trainval[mask]

            inner = GroupShuffleSplit(n_splits=1, test_size=0.1, random_state=123)
            g_sub = pose_id.iloc[trainval_idx][mask]
            tr_i, va_i = next(inner.split(X_sub, pos_sub, groups=g_sub))
            X_tr, X_va = X_sub[tr_i], X_sub[va_i]
            pos_tr, pos_va = pos_sub[tr_i], pos_sub[va_i]
            q_tr, q_va = quat_sub[tr_i], quat_sub[va_i]

            m = MLPQuat(in_dim=256, hidden=hid, dropout=args.dropout)
            m, _ = train_model(m, X_tr, pos_tr, q_tr, X_va, pos_va, q_va,
                               lr=lr, weight_decay=wd, batch_size=args.batch_size,
                               max_epochs=args.lc_epochs, patience=max(10, args.lc_epochs//4),
                               lambda_rot=args.lambda_rot, min_delta=args.min_delta,
                               device=None, progress=False)

            # Train metrics (subset)
            device = get_device(); m.eval().to(device)
            with torch.no_grad():
                xb_tr = torch.from_numpy(X_sub).float().to(device)
                pos_std_ph, quat_raw_ph = m(xb_tr)
                pos_ph = pos_scaler.inverse_transform(pos_std_ph.cpu().numpy())
                quat_ph = quat_raw_ph.cpu().numpy()
                quat_ph = quat_ph / np.linalg.norm(quat_ph, axis=1, keepdims=True).clip(1e-12, None)
            tr_pos_rmse = float(np.sqrt(mean_squared_error(pos[trainval_idx][mask], pos_ph)))
            tr_geo_deg = float(np.mean(quat_geodesic_deg(quat_ph, quat_trainval[mask])))

            # Test metrics
            with torch.no_grad():
                xb_te = torch.from_numpy(X_test_std).float().to(device)
                pos_std_te, quat_raw_te = m(xb_te)
                pos_te_ph = pos_scaler.inverse_transform(pos_std_te.cpu().numpy())
                quat_te_ph = quat_raw_te.cpu().numpy()
                quat_te_ph = quat_te_ph / np.linalg.norm(quat_te_ph, axis=1, keepdims=True).clip(1e-12, None)
            te_pos_rmse = float(np.sqrt(mean_squared_error(pos_test, pos_te_ph)))
            te_geo_deg  = float(np.mean(quat_geodesic_deg(quat_te_ph, quat_test)))

            rows.append({
                "n_train": int(X_sub.shape[0]),
                "pos_rmse_train_m": tr_pos_rmse,
                "pos_rmse_test_m": te_pos_rmse,
                "rot_geo_train_deg": tr_geo_deg,
                "rot_geo_test_deg": te_geo_deg,
            })
            print(f"[lc] n_train={X_sub.shape[0]}  pos_RMSE_test={te_pos_rmse:.6f} m  rot_geo_test={te_geo_deg:.2f}°")
            pbar.update(1)

    lc_df = pd.DataFrame(rows).sort_values("n_train")
    lc_csv = os.path.join(args.outdir, "eit_pose_mlp_quat_learning_curve.csv")
    lc_df.to_csv(lc_csv, index=False)
    plt.figure()
    plt.plot(lc_df["n_train"].values, lc_df["pos_rmse_test_m"].values, marker="o", label="Position RMSE (test) [m]")
    plt.plot(lc_df["n_train"].values, lc_df["rot_geo_test_deg"].values, marker="o", label="Orientation geodesic (test) [deg]")
    plt.xlabel("Training samples (by grouped subset)"); plt.ylabel("Error")
    plt.title("Learning Curve: ΔEIT → Pose (MLP + Quaternion)")
    plt.grid(True, linestyle="--", linewidth=0.5); plt.legend()
    lc_png = os.path.join(args.outdir, "eit_pose_mlp_quat_learning_curve.png")
    plt.tight_layout(); plt.savefig(lc_png, dpi=160)
    print(f"[save] Learning-curve CSV -> {lc_csv}")
    print(f"[save] Learning-curve PNG -> {lc_png}")
    return 0

# ---------------------------- Inference helpers ------------------------------

def load_mlp(model_path, scalers_path, device=None):
    device = device or get_device()
    ckpt = torch.load(model_path, map_location=device)
    arch = ckpt["arch"]
    model = MLPQuat(in_dim=arch["in_dim"], hidden=arch["hidden"], dropout=arch["dropout"])
    model.load_state_dict(ckpt["state_dict"]); model.eval().to(device)
    s = np.load(scalers_path)
    x_mean, x_scale = s["x_mean"], s["x_scale"]
    pos_mean, pos_scale = s["pos_mean"], s["pos_scale"]
    return model, (x_mean, x_scale, pos_mean, pos_scale), device

def predict_pose_quat(delta_eit_256, model=None, scalers=None, device=None):
    """Return [x,y,z,qw,qx,qy,qz] given ΔEIT (256,)."""
    device = device or get_device()
    delta = np.asarray(delta_eit_256, dtype=np.float32).reshape(1,-1)
    x_mean, x_scale, pos_mean, pos_scale = scalers
    delta_std = (delta - x_mean) / x_scale
    with torch.no_grad():
        xb = torch.from_numpy(delta_std).float().to(device)
        pos_std, quat_raw = model(xb)
        pos = (pos_std.cpu().numpy() * pos_scale + pos_mean)[0]
        quat = quat_raw.cpu().numpy()[0]
        quat = quat / np.linalg.norm(quat).clip(1e-12, None)
    return np.concatenate([pos, quat])

if __name__ == "__main__":
    raise SystemExit(main())
