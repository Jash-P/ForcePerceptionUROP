#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EIT Δ → UR5 TCP Pose — CNN (Quaternion Orientation + Geodesic Loss)

- Outputs 7 dims: [x,y,z,qw,qx,qy,qz]
- Loss = MSE on standardized xyz + λ * (geodesic quaternion error in radians)^2
- Model selection (CV) uses composite score: pos_RMSE_m + ori_weight * mean_geodesic_deg
"""
import argparse, json, os
from datetime import datetime
import numpy as np, pandas as pd
from sklearn.model_selection import GroupKFold, GroupShuffleSplit
from sklearn.preprocessing import StandardScaler
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

import torch, torch.nn as nn, torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader

# ---- rotations ----
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

def quat_geodesic_rad_torch(qp, qt):
    qp = qp / qp.norm(dim=1, keepdim=True).clamp_min(1e-12)
    qt = qt / qt.norm(dim=1, keepdim=True).clamp_min(1e-12)
    dot = torch.sum(qp * qt, dim=1).abs().clamp(-1.0, 1.0)
    return 2.0 * torch.arccos(dot)  # radians

def quat_to_axis_angle(q):
    """Return axis–angle vector (rx,ry,rz) from quaternion (w,x,y,z)."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q[...,0], q[...,1], q[...,2], q[...,3]
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sqrt(1.0 - w*w)
    axis = np.zeros(q.shape[:-1] + (3,))
    small = s < 1e-12
    axis[~small, 0] = x[~small] / s[~small]
    axis[~small, 1] = y[~small] / s[~small]
    axis[~small, 2] = z[~small] / s[~small]
    return axis * angle[...,None]

def save_axis_angle_plots(r_true_rad, r_pred_rad, outdir, use_degrees=True):
    """Make Pred vs True + residual hist for rx, ry, rz.
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
        plt.savefig(os.path.join(outdir, f"cnn_pvstrue_{name}_{unit}.png"), dpi=140)

        # Residual histogram
        plt.figure()
        resid = r_pred[:, i] - r_true[:, i]
        plt.hist(resid, bins=50)
        plt.xlabel(f"Residual ({name}) [{unit}]")
        plt.ylabel("Count")
        plt.title(f"CNN Residuals — {name} ({unit})")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"cnn_residuals_{name}_{unit}.png"), dpi=140)

import numpy as np

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

# ---- model ----
class EIT1DCNNQuat(nn.Module):
    def __init__(self, in_len=256, ch1=64, ch2=128, dropout=0.1):
        super().__init__()
        self.fe = nn.Sequential(
            nn.Conv1d(1, ch1, 5, padding=2), nn.BatchNorm1d(ch1), nn.ReLU(True), nn.Dropout(dropout),
            nn.Conv1d(ch1, ch2, 5, padding=2), nn.BatchNorm1d(ch2), nn.ReLU(True),
            nn.AdaptiveAvgPool1d(1), nn.Flatten()
        )
        self.head = nn.Sequential(nn.Linear(ch2,128), nn.ReLU(True), nn.Dropout(dropout),
                                  nn.Linear(128,7))  # xyz_std (3) + quat_raw (4)
    def forward(self, x):  # x: (B,1,256)
        z = self.fe(x); y = self.head(z)
        return y[:,:3], y[:,3:]

# ---- train utils ----
def get_device():
    if torch.cuda.is_available(): return torch.device("cuda")
    if hasattr(torch.backends,"mps") and torch.backends.mps.is_available(): return torch.device("mps")
    return torch.device("cpu")

def train_model(model, X_tr, y_tr_pos_std, y_tr_quat, X_va, y_va_pos_std, y_va_quat,
                lr=1e-3, weight_decay=1e-4, batch_size=64, max_epochs=150, patience=20,
                lambda_rot=5.0, device=None, progress=True):
    device = device or get_device()
    model = model.to(device)
    opt = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    mse = nn.MSELoss()

    ds_tr = TensorDataset(torch.from_numpy(X_tr).float().unsqueeze(1),
                          torch.from_numpy(y_tr_pos_std).float(),
                          torch.from_numpy(y_tr_quat).float())
    ds_va = TensorDataset(torch.from_numpy(X_va).float().unsqueeze(1),
                          torch.from_numpy(y_va_pos_std).float(),
                          torch.from_numpy(y_va_quat).float())
    dl_tr = DataLoader(ds_tr, batch_size=batch_size, shuffle=True)
    dl_va = DataLoader(ds_va, batch_size=batch_size, shuffle=False)

    best, best_val, wait = None, np.inf, 0
    pbar = tqdm(total=max_epochs, desc="epochs", disable=not(progress and _TQDM_AVAILABLE))
    for ep in range(max_epochs):
        model.train()
        for xb, yb_pos, yb_q in dl_tr:
            xb, yb_pos, yb_q = xb.to(device), yb_pos.to(device), yb_q.to(device)
            opt.zero_grad()
            pos_std, q_raw = model(xb)
            loss_pos = mse(pos_std, yb_pos)
            ang = quat_geodesic_rad_torch(q_raw, yb_q)
            loss_rot = torch.mean(ang*ang)
            loss = loss_pos + lambda_rot*loss_rot
            loss.backward(); opt.step()
        # val
        model.eval(); val, n = 0.0, 0
        with torch.no_grad():
            for xb, yb_pos, yb_q in dl_va:
                xb, yb_pos, yb_q = xb.to(device), yb_pos.to(device), yb_q.to(device)
                pos_std, q_raw = model(xb)
                ang = quat_geodesic_rad_torch(q_raw, yb_q)
                loss = mse(pos_std, yb_pos) + lambda_rot*torch.mean(ang*ang)
                val += float(loss.item())*xb.size(0); n += xb.size(0)
        val /= max(1,n)
        if val + 1e-8 < best_val:
            best_val, wait = val, 0
            best = {k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        else:
            wait += 1
            if wait >= patience: pbar.update(max_epochs-ep); break
        pbar.update(1)
    pbar.close()
    if best is not None: model.load_state_dict(best)
    return model, best_val

# ---- main ----
def main():
    p = argparse.ArgumentParser(description="Train EIT→pose CNN with quaternion orientation and geodesic loss.")
    p.add_argument("--csv", type=str, required=True)
    p.add_argument("--outdir", type=str, default="./outputs")
    p.add_argument("--min_Fz", type=float, default=50.0)
    p.add_argument("--contact_only", action="store_true")
    p.add_argument("--test_size", type=float, default=0.2)
    p.add_argument("--cv_folds", type=int, default=5)
    p.add_argument("--lr_grid", type=str, default="0.001,0.0003")
    p.add_argument("--wd_grid", type=str, default="0.0,0.0001")
    p.add_argument("--ch_grid", type=str, default="64,128")
    p.add_argument("--dropout", type=float, default=0.1)
    p.add_argument("--batch_size", type=int, default=64)
    p.add_argument("--epochs", type=int, default=150)
    p.add_argument("--patience", type=int, default=20)
    p.add_argument("--lambda_rot", type=float, default=5.0, help="Weight for rotation geodesic loss term.")
    p.add_argument("--lc_points", type=str, default="auto")
    p.add_argument("--lc_epochs", type=int, default=80)
    p.add_argument("--ori_weight", type=float, default=0.01, help="Composite CV weight in m/deg.")
    p.add_argument("--no_progress", action="store_true")
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    df = pd.read_csv(args.csv)
    if 'contact_detected' in df.columns and args.contact_only:
        df = df[df['contact_detected']==1].copy()
    if 'Fz' not in df.columns: raise ValueError("Missing 'Fz' column.")
    pre = len(df); df = df[df['Fz'] >= args.min_Fz].copy()
    print(f"[filter] Kept {len(df)}/{pre} rows (Fz >= {args.min_Fz}).")

    eitb = [f"eitb_{i}" for i in range(256)]
    eita = [f"eita_{i}" for i in range(256)]
    for cols in (eitb, eita):
        miss = [c for c in cols if c not in df.columns]
        if miss: raise ValueError(f"Missing EIT cols, e.g. {miss[:5]}")
    X = (df[eita].values - df[eitb].values).astype(np.float32)

    for t in ["act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz"]:
        if t not in df.columns: raise ValueError(f"Missing target col: {t}")
    pos = df[["act_tcp_x","act_tcp_y","act_tcp_z"]].values.astype(np.float32)
    rvec = df[["act_tcp_rx","act_tcp_ry","act_tcp_rz"]].values.astype(np.float32)
    quat = axis_angle_to_quat(rvec).astype(np.float32)

    good = np.isfinite(X).all(axis=1) & np.isfinite(pos).all(axis=1) & np.isfinite(quat).all(axis=1)
    if not good.all():
        d = int((~good).sum())
        df = df.loc[good].reset_index(drop=True); X = X[good]; pos = pos[good]; quat = quat[good]
        print(f"[info] Dropped {d} NaN/Inf rows.")

    def build_pose_id(df, pos_bin_m=0.001, ang_bin_rad=0.05):
        return pd.Series([
            f"{int(round(df['act_tcp_x'].iat[i]/pos_bin_m))}_"
            f"{int(round(df['act_tcp_y'].iat[i]/pos_bin_m))}_"
            f"{int(round(df['act_tcp_z'].iat[i]/pos_bin_m))}_"
            f"{int(round(df['act_tcp_rx'].iat[i]/ang_bin_rad))}_"
            f"{int(round(df['act_tcp_ry'].iat[i]/ang_bin_rad))}_"
            f"{int(round(df['act_tcp_rz'].iat[i]/ang_bin_rad))}"
        for i in range(len(df))], index=df.index)

    pose_id = build_pose_id(df)
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=42)
    trval_idx, te_idx = next(gss.split(X, pos, groups=pose_id))
    X_trval, pos_trval, quat_trval = X[trval_idx], pos[trval_idx], quat[trval_idx]
    X_te, pos_te, quat_te = X[te_idx], pos[te_idx], quat[te_idx]
    g_trval = pose_id.iloc[trval_idx]
    print(f"[split] Train+Val={X_trval.shape[0]}  Test={X_te.shape[0]}  Total={X.shape[0]}")

    x_scaler = StandardScaler().fit(X_trval)
    pos_scaler = StandardScaler().fit(pos_trval)
    X_trval_std = x_scaler.transform(X_trval).astype(np.float32)
    X_te_std     = x_scaler.transform(X_te).astype(np.float32)
    pos_trval_std = pos_scaler.transform(pos_trval).astype(np.float32)

    lrs = [float(s) for s in args.lr_grid.split(",")]
    wds = [float(s) for s in args.wd_grid.split(",")]
    chs = [int(s) for s in args.ch_grid.split(",")]
    combos = [(lr, wd, ch) for lr in lrs for wd in wds for ch in chs]

    cv = GroupKFold(n_splits=args.cv_folds)
    best_combo, best_score, results = None, np.inf, []
    total = len(combos) * args.cv_folds
    with tqdm(total=total, desc="CV (combos × folds)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for (lr, wd, ch) in combos:
            scores = []
            for tr, va in cv.split(X_trval_std, pos_trval_std, groups=g_trval):
                X_tr, X_va = X_trval_std[tr], X_trval_std[va]
                pos_tr, pos_va = pos_trval_std[tr], pos_trval_std[va]
                q_tr, q_va = quat_trval[tr], quat_trval[va]
                m = EIT1DCNNQuat(in_len=256, ch1=ch, ch2=ch*2, dropout=args.dropout)
                m, _ = train_model(m, X_tr, pos_tr, q_tr, X_va, pos_va, q_va,
                                   lr=lr, weight_decay=wd, batch_size=args.batch_size,
                                   max_epochs=args.epochs, patience=args.patience,
                                   lambda_rot=args.lambda_rot, device=None, progress=False)
                m.eval().to(get_device())
                with torch.no_grad():
                    xb = torch.from_numpy(X_va).float().unsqueeze(1).to(get_device())
                    pos_std_pred, q_raw_pred = m(xb)
                    pos_pred = pos_scaler.inverse_transform(pos_std_pred.cpu().numpy())
                    q_pred = q_raw_pred.cpu().numpy()
                    q_pred = q_pred / np.linalg.norm(q_pred, axis=1, keepdims=True).clip(1e-12, None)
                pos_true = pos_scaler.inverse_transform(pos_va)
                pos_rmse = float(np.sqrt(mean_squared_error(pos_true, pos_pred)))
                geo_deg  = float(quat_geodesic_deg(q_pred, q_va).mean())
                scores.append(pos_rmse + args.ori_weight * geo_deg); pbar.update(1)
            mean_score = float(np.mean(scores))
            results.append({"lr":lr,"wd":wd,"ch1":ch,"cv_score":mean_score})
            if mean_score < best_score: best_score, best_combo = mean_score, (lr,wd,ch)
    print("[cv] Composite: pos_RMSE_m + ori_weight*mean_geo_deg (lower is better)")
    for r in results:
        print(f"   lr={r['lr']:g} wd={r['wd']:g} ch1={r['ch1']} -> score={r['cv_score']:.6f}")
    lr,wd,ch = best_combo
    print(f"[cv] Best combo: lr={lr:g} wd={wd:g} ch1={ch} (score={best_score:.6f}, ori_weight={args.ori_weight})")

    inner = GroupShuffleSplit(n_splits=1, test_size=0.1, random_state=123)
    tr_i, va_i = next(inner.split(X_trval_std, pos_trval_std, groups=g_trval))
    X_tr, X_va = X_trval_std[tr_i], X_trval_std[va_i]
    pos_tr, pos_va = pos_trval_std[tr_i], pos_trval_std[va_i]
    q_tr, q_va = quat_trval[tr_i], quat_trval[va_i]
    final = EIT1DCNNQuat(in_len=256, ch1=ch, ch2=ch*2, dropout=args.dropout)
    final, _ = train_model(final, X_tr, pos_tr, q_tr, X_va, pos_va, q_va,
                           lr=lr, weight_decay=wd, batch_size=args.batch_size,
                           max_epochs=args.epochs, patience=args.patience,
                           lambda_rot=args.lambda_rot, device=None, progress=not args.no_progress)

    device = get_device(); final.eval().to(device)
    with torch.no_grad():
        xb = torch.from_numpy(X_te_std).float().unsqueeze(1).to(device)
        pos_std_pred, q_raw_pred = final(xb)
        pos_pred = pos_scaler.inverse_transform(pos_std_pred.cpu().numpy())
        q_pred = q_raw_pred.cpu().numpy()
        q_pred = q_pred / np.linalg.norm(q_pred, axis=1, keepdims=True).clip(1e-12, None)

    pos_rmse = float(np.sqrt(mean_squared_error(pos_te, pos_pred)))
    pos_mae  = float(mean_absolute_error(pos_te, pos_pred))
    names = ["act_tcp_x","act_tcp_y","act_tcp_z"]
    rmse_axes = dict(zip(names, np.sqrt(np.mean((pos_te - pos_pred)**2, axis=0)).tolist()))
    mae_axes  = dict(zip(names, np.mean(np.abs(pos_te - pos_pred), axis=0).tolist()))
    geo_err_deg = quat_geodesic_deg(q_pred, quat_te)
    geo_summary = {"mean_deg":float(np.mean(geo_err_deg)),
                   "median_deg":float(np.median(geo_err_deg)),
                   "p90_deg":float(np.percentile(geo_err_deg, 90)),
                   "p95_deg":float(np.percentile(geo_err_deg, 95))}
    print("\n[test] Position (xyz):")
    print(f"   RMSE = {pos_rmse:.6f} m   MAE = {pos_mae:.6f} m")
    print("[test] Per-axis (m):")
    for k in names: print(f"{k:>11s}  RMSE={rmse_axes[k]:.4f} m   MAE={mae_axes[k]:.4f} m")
    print("\n[test] Orientation (quaternion → geodesic):")
    print(f"   mean={geo_summary['mean_deg']:.2f}°, median={geo_summary['median_deg']:.2f}°, P90={geo_summary['p90_deg']:.2f}°, P95={geo_summary['p95_deg']:.2f}°")

    # Save
    torch.save({"state_dict": final.state_dict(), "arch":{"in_len":256,"ch1":ch,"ch2":ch*2,"dropout":args.dropout}},
               os.path.join(args.outdir, "eit_pose_cnn_quat.pt"))
    np.savez(os.path.join(args.outdir, "eit_pose_cnn_quat_scalers.npz"),
             x_mean= x_scaler.mean_, x_scale= x_scaler.scale_,
             pos_mean= pos_scaler.mean_, pos_scale= pos_scaler.scale_)
    with open(os.path.join(args.outdir, "eit_pose_cnn_quat_meta.json"), "w") as f:
        json.dump({"created_utc": datetime.utcnow().isoformat()+"Z",
                   "csv_path": os.path.abspath(args.csv),
                   "rows_after_Fz": int(len(df)),
                   "test_size_groups": args.test_size, "cv_folds": args.cv_folds,
                   "best_combo": {"lr":lr,"wd":wd,"ch1":ch},
                   "lambda_rot": args.lambda_rot, "ori_weight_m_per_deg": args.ori_weight,
                   "notes":"Quaternion orientation with geodesic loss. Composite CV used."}, f, indent=2)

    # Position diagnostics plots + error slices
    def save_pred_vs_true_plots_pos(y_true_pos, y_pred_pos, outdir):
        for i, name in enumerate(names):
            plt.figure(); plt.scatter(y_true_pos[:,i], y_pred_pos[:,i], s=10)
            lims = [min(y_true_pos[:,i].min(), y_pred_pos[:,i].min()),
                    max(y_true_pos[:,i].max(), y_pred_pos[:,i].max())]
            plt.plot(lims, lims); plt.xlabel(f"True {name}"); plt.ylabel(f"Pred {name}")
            plt.title(f"CNN Pred vs True — {name}"); plt.grid(True, linestyle="--", lw=0.5)
            plt.tight_layout(); plt.savefig(os.path.join(outdir, f"cnn_pvstrue_{name}.png"), dpi=140)
            plt.figure(); resid = y_pred_pos[:,i]-y_true_pos[:,i]
            plt.hist(resid, bins=50); plt.xlabel(f"Residual ({name})"); plt.ylabel("Count")
            plt.title(f"CNN Residuals — {name}"); plt.grid(True, linestyle="--", lw=0.5)
            plt.tight_layout(); plt.savefig(os.path.join(outdir, f"cnn_residuals_{name}.png"), dpi=140)

    def binned_stats(x, err_vals, bins):
        x = np.asarray(x); err_vals = np.asarray(err_vals)
        idx = np.digitize(x, bins) - 1
        centers = 0.5*(bins[:-1]+bins[1:]); out=[]
        for b in range(len(centers)):
            m = idx==b; out.append(float(np.mean(np.abs(err_vals[m]))) if np.any(m) else np.nan)
        return centers, np.array(out)

    df_test = df.iloc[te_idx].copy()  # make sure this line exists before plotting

    # --- Orientation plots in axis–angle (rx,ry,rz) for convenience ---
    df_test = df.iloc[te_idx].copy()
    r_true_ax = df_test[["act_tcp_rx","act_tcp_ry","act_tcp_rz"]].values
    q_true    = axis_angle_to_quat(r_true_ax)
    r_true    = quat_to_axis_angle_ur(q_true)
    r_pred    = quat_to_axis_angle_ur(q_pred)   # or quat_pred, whichever name you use
    save_axis_angle_plots(r_true, r_pred, args.outdir, use_degrees=True)

    print(f"[save] CNN axis–angle plots -> cnn_pvstrue_act_tcp_r*_deg.png & cnn_residuals_act_tcp_r*_deg.png in {args.outdir}")

    save_pred_vs_true_plots_pos(pos_te, pos_pred, args.outdir)
    pos_mae_sample = np.mean(np.abs(pos_pred - pos_te), axis=1)
    if "Fz" in df_test.columns:
        fz = df_test["Fz"].values; bins = np.linspace(np.nanmin(fz), np.nanmax(fz), 8)
        c, mae_pos = binned_stats(fz, pos_mae_sample, bins)
        _, mae_geo = binned_stats(fz, geo_err_deg, bins)
        plt.figure(); plt.plot(c, mae_pos, "o-"); plt.xlabel("Fz"); plt.ylabel("Mean |pos error| (m)")
        plt.title("CNN Position error vs Fz"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(args.outdir, "cnn_error_vs_Fz_position.png"), dpi=140)
        plt.figure(); plt.plot(c, mae_geo, "o-"); plt.xlabel("Fz"); plt.ylabel("Geodesic ori error (deg)")
        plt.title("CNN Orientation error vs Fz"); plt.grid(True, linestyle="--", lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(args.outdir, "cnn_error_vs_Fz_orientation.png"), dpi=140)

    yvals = df_test["act_tcp_y"].values if "act_tcp_y" in df_test.columns else pos_te[:,1]
    ybins = np.linspace(np.nanmin(yvals), np.nanmax(yvals), 8)
    c, mae_pos = binned_stats(yvals, pos_mae_sample, ybins)
    _, mae_geo = binned_stats(yvals, geo_err_deg, ybins)
    plt.figure(); plt.plot(c, mae_pos, "o-"); plt.xlabel("act_tcp_y"); plt.ylabel("Mean |pos error| (m)")
    plt.title("CNN Position error vs act_tcp_y"); plt.grid(True, linestyle="--", lw=0.5)
    plt.tight_layout(); plt.savefig(os.path.join(args.outdir, "cnn_error_vs_y_position.png"), dpi=140)
    plt.figure(); plt.plot(c, mae_geo, "o-"); plt.xlabel("act_tcp_y"); plt.ylabel("Geodesic ori error (deg)")
    plt.title("CNN Orientation error vs act_tcp_y"); plt.grid(True, linestyle="--", lw=0.5)
    plt.tight_layout(); plt.savefig(os.path.join(args.outdir, "cnn_error_vs_y_orientation.png"), dpi=140)

    # Learning curve — records pos RMSE & geodesic deg
    def parse_points(s, n):
        s=s.strip().lower()
        if s=="auto":
            if n<120: pts=[max(20,n//4),max(40,n//2),n]
            else:
                pts=list(np.unique(np.clip(np.round(np.linspace(150,n,6)).astype(int),50,n)))
                if pts[-1]!=n: pts[-1]=n
            return pts
        pts=[int(x) for x in s.split(",") if x.strip()]
        pts=[p for p in pts if p>10]; pts=sorted(set([min(p,n) for p in pts]))
        if len(pts)==0 or pts[-1]!=n: pts.append(n)
        return pts
    def sample_groups_for_size(groups, desired_n, rng):
        groups=np.asarray(groups); uniq,cnt=np.unique(groups,return_counts=True)
        idx=np.arange(len(uniq)); rng.shuffle(idx)
        total,sel=0,set()
        for i in idx:
            g=uniq[i]; sel.add(g); total+=cnt[i]
            if total>=desired_n: break
        return np.isin(groups, list(sel))

    rng=np.random.default_rng(42)
    sizes=parse_points(args.lc_points, X_trval_std.shape[0])
    rows=[]
    with tqdm(total=len(sizes), desc="Learning curve (train sizes)", disable=(args.no_progress or not _TQDM_AVAILABLE)) as pbar:
        for n in sizes:
            mask=sample_groups_for_size(g_trval.values, desired_n=n, rng=rng)
            X_sub, pos_sub, quat_sub = X_trval_std[mask], pos_trval_std[mask], quat_trval[mask]
            inner=GroupShuffleSplit(n_splits=1, test_size=0.1, random_state=123)
            g_sub=pose_id.iloc[trval_idx][mask]
            tr_i,va_i=next(inner.split(X_sub, pos_sub, groups=g_sub))
            X_tr, X_va = X_sub[tr_i], X_sub[va_i]
            pos_tr, pos_va = pos_sub[tr_i], pos_sub[va_i]
            q_tr, q_va = quat_sub[tr_i], quat_sub[va_i]
            m=EIT1DCNNQuat(in_len=256, ch1=ch, ch2=ch*2, dropout=args.dropout)
            m,_=train_model(m, X_tr, pos_tr, q_tr, X_va, pos_va, q_va,
                            lr=lr, weight_decay=wd, batch_size=args.batch_size,
                            max_epochs=args.lc_epochs, patience=max(10,args.lc_epochs//4),
                            lambda_rot=args.lambda_rot, device=None, progress=False)
            m.eval().to(get_device())
            with torch.no_grad():
                xb_tr=torch.from_numpy(X_sub).float().unsqueeze(1).to(get_device())
                pos_std_ph, q_raw_ph = m(xb_tr)
                pos_ph=pos_scaler.inverse_transform(pos_std_ph.cpu().numpy())
                q_ph=q_raw_ph.cpu().numpy()
                q_ph=q_ph/np.linalg.norm(q_ph,axis=1,keepdims=True).clip(1e-12,None)
            tr_pos_rmse=float(np.sqrt(mean_squared_error(pos[trval_idx][mask], pos_ph)))
            tr_geo_deg=float(quat_geodesic_deg(quat_trval[mask], q_ph).mean())
            with torch.no_grad():
                xb_te=torch.from_numpy(X_te_std).float().unsqueeze(1).to(get_device())
                pos_std_te, q_raw_te = m(xb_te)
                pos_te_ph=pos_scaler.inverse_transform(pos_std_te.cpu().numpy())
                q_te_ph=q_raw_te.cpu().numpy()
                q_te_ph=q_te_ph/np.linalg.norm(q_te_ph,axis=1,keepdims=True).clip(1e-12,None)
            te_pos_rmse=float(np.sqrt(mean_squared_error(pos_te, pos_te_ph)))
            te_geo_deg=float(quat_geodesic_deg(q_te_ph, quat_te).mean())
            rows.append({"n_train":int(X_sub.shape[0]),
                         "pos_rmse_train_m":tr_pos_rmse, "pos_rmse_test_m":te_pos_rmse,
                         "rot_geo_train_deg":tr_geo_deg, "rot_geo_test_deg":te_geo_deg})
            print(f"[lc] n_train={X_sub.shape[0]}  pos_RMSE_test={te_pos_rmse:.6f} m  rot_geo_test={te_geo_deg:.2f}°")
            pbar.update(1)

    lc_df=pd.DataFrame(rows).sort_values("n_train")
    lc_csv=os.path.join(args.outdir,"eit_pose_cnn_quat_learning_curve.csv")
    lc_df.to_csv(lc_csv,index=False)
    plt.figure()
    plt.plot(lc_df["n_train"].values, lc_df["pos_rmse_test_m"].values, "o-", label="Position RMSE (test) [m]")
    plt.plot(lc_df["n_train"].values, lc_df["rot_geo_test_deg"].values, "o-", label="Orientation geodesic (test) [deg]")
    plt.xlabel("Training samples (by grouped subset)"); plt.ylabel("Error"); plt.grid(True, linestyle="--", lw=0.5)
    plt.title("Learning Curve: ΔEIT → Pose (CNN + Quaternion)"); plt.legend()
    lc_png=os.path.join(args.outdir,"eit_pose_cnn_quat_learning_curve.png")
    plt.tight_layout(); plt.savefig(lc_png,dpi=160)
    print(f"[save] Learning-curve CSV -> {lc_csv}")
    print(f"[save] Learning-curve PNG -> {lc_png}")
    return 0

# ---- inference helpers ----
def load_cnn(model_path, scalers_path, device=None):
    device = device or get_device()
    ckpt=torch.load(model_path, map_location=device)
    arch=ckpt["arch"]
    m=EIT1DCNNQuat(in_len=arch["in_len"], ch1=arch["ch1"], ch2=arch["ch2"], dropout=arch["dropout"])
    m.load_state_dict(ckpt["state_dict"]); m.eval().to(device)
    s=np.load(scalers_path)
    return m, (s["x_mean"], s["x_scale"], s["pos_mean"], s["pos_scale"]), device

def predict_pose_quat(delta_eit_256, model=None, scalers=None, device=None):
    device=device or get_device()
    delta=np.asarray(delta_eit_256, dtype=np.float32).reshape(1,-1)
    x_mean,x_scale,pos_mean,pos_scale = scalers
    delta_std=(delta - x_mean) / x_scale
    with torch.no_grad():
        xb=torch.from_numpy(delta_std).float().unsqueeze(1).to(device)
        pos_std, q_raw = model(xb)
        pos=(pos_std.cpu().numpy()*pos_scale + pos_mean)[0]
        q=q_raw.cpu().numpy()[0]; q=q/np.linalg.norm(q).clip(1e-12,None)
    return np.concatenate([pos, q])

if __name__ == "__main__":
    raise SystemExit(main())
