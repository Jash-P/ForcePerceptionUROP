#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage-2: Orientation (rpy) via Geometry Prior + Residual Quaternion MLP.
Inputs: ΔEIT = eita_* - eitb_*  (256 electrodes)  + Stage-1 predicted xyz.
No force/torque channels are used.

New in this version
-------------------
- Adds a correct cylinder prior for handles whose axis is +X:  --prior cylinder_x
  (keeps cylinder_z and plane options).
- Saves the appropriate center for the cylinder cross-section in the bundle.

Pipeline
--------
1) Load CSV, build ΔEIT, (optional) filters: Fz cutoff, ΔEIT percentile, winsorization,
   per-session ΔEIT z-scoring (same concepts as Stage-1).
2) Load Stage-1 ensemble (joblib). Predict xyz for ALL kept rows -> xyz_pred.
3) Split into Train/Val/Test with GroupShuffleSplit (grouping by xyz mm-bins or session).
4) Fit geometry prior R_prior(x,y,z):
   - cylinder_x: fit (y0,z0) center on TRAIN (y,z), build local frame with axis +X.
   - cylinder_z: fit (x0,y0) center on TRAIN (x,y), build local frame with axis +Z.
   - plane: fit plane normal via PCA on TRAIN xyz; frame lies on plane.
   Then learn a global quaternion offset q_off on TRAIN so q_prior = q_off ⊗ q_geo.
5) Compute residual targets: Δq_true = q_true ⊗ q_prior^{-1}.
6) MLP predicts Δq; outputs are normalized to unit quats; loss = mean( geodesic(Δq_pred, Δq_true)^2 ).
7) Compose final orientation: q_pred = Δq_pred ⊗ q_prior. Report geodesic stats + rx/ry/rz plots.

Install
-------
python3 -m pip install numpy pandas scikit-learn matplotlib joblib torch tqdm
"""

import argparse, json, os, warnings
from datetime import datetime

import numpy as np
import pandas as pd

from sklearn.model_selection import GroupShuffleSplit
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA

from joblib import load, dump

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
from tqdm.auto import tqdm

RANDOM_STATE = 42
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# ----------------------------- Helpers: I/O & filters ------------------------

def ensure_cols(df, cols, name):
    miss = [c for c in cols if c not in df.columns]
    if miss:
        raise ValueError(f"Missing {name} columns, e.g. {miss[:6]}")

def winsorize_cols(X, p=99.5):
    lo = np.percentile(X, 100-p, axis=0)
    hi = np.percentile(X, p, axis=0)
    return np.clip(X, lo, hi)

def per_session_zscore(X, sessions):
    Xn = X.copy()
    for sid in np.unique(sessions):
        m = (sessions == sid)
        if not np.any(m): continue
        mu = Xn[m].mean(axis=0)
        sd = Xn[m].std(axis=0); sd[sd < 1e-8] = 1.0
        Xn[m] = (Xn[m] - mu) / sd
    return Xn

def build_groups(df, mode="xyz", pos_bin_m=0.001):
    if mode == "session":
        if "session_id" not in df.columns:
            raise ValueError("--grouping session requested but no session_id column present.")
        return df["session_id"].astype(str).reset_index(drop=True)
    xq = np.round(df["act_tcp_x"].values/pos_bin_m).astype(np.int64)
    yq = np.round(df["act_tcp_y"].values/pos_bin_m).astype(np.int64)
    zq = np.round(df["act_tcp_z"].values/pos_bin_m).astype(np.int64)
    return pd.Series([f"{a}_{b}_{c}" for a,b,c in zip(xq,yq,zq)], index=df.index)

# ----------------------------- Geometry: rotations & quats -------------------

def euler_rpy_to_matrix(rx, ry, rz):
    """Roll-Pitch-Yaw about x,y,z (radians). Returns 3x3 rotation matrix. R = Rz * Ry * Rx."""
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,   0, 1]])
    Ry = np.array([[cy, 0, sy],
                   [ 0, 1,  0],
                   [-sy,0, cy]])
    Rx = np.array([[1,  0,   0],
                   [0, cx, -sx],
                   [0, sx,  cx]])
    return Rz @ Ry @ Rx

def matrix_to_quat(R):
    """Quaternion (w,x,y,z) from 3x3 R (right-handed)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0)*2.0
        w = 0.25*s
        x = (R[2,1]-R[1,2]) / s
        y = (R[0,2]-R[2,0]) / s
        z = (R[1,0]-R[0,1]) / s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = np.sqrt(1.0 + R[0,0]-R[1,1]-R[2,2])*2.0
            w = (R[2,1]-R[1,2]) / s
            x = 0.25*s
            y = (R[0,1]+R[1,0]) / s
            z = (R[0,2]+R[2,0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1,1]-R[0,0]-R[2,2])*2.0
            w = (R[0,2]-R[2,0]) / s
            x = (R[0,1]+R[1,0]) / s
            y = 0.25*s
            z = (R[1,2]+R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2]-R[0,0]-R[1,1])*2.0
            w = (R[1,0]-R[0,1]) / s
            x = (R[0,2]+R[2,0]) / s
            y = (R[1,2]+R[2,1]) / s
            z = 0.25*s
    q = np.array([w,x,y,z], dtype=float)
    return q / np.linalg.norm(q)

def quat_to_matrix(q):
    """(w,x,y,z) → 3x3."""
    q = q / (np.linalg.norm(q) + 1e-12)
    w,x,y,z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ])

def quat_mul(q1, q2):
    """Hamilton product (w,x,y,z)."""
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_inv(q):
    w,x,y,z = q
    return np.array([w,-x,-y,-z], dtype=float) / (np.dot(q,q) + 1e-12)

def quat_geodesic_angle(q1, q2):
    """Geodesic angle between unit quaternions (radians)."""
    q1 = q1 / (np.linalg.norm(q1) + 1e-12)
    q2 = q2 / (np.linalg.norm(q2) + 1e-12)
    d = abs(np.dot(q1, q2))
    d = np.clip(d, -1.0, 1.0)
    return 2.0*np.arccos(d)

def rpy_from_matrix(R):
    """Inverse of R = Rz*Ry*Rx. Returns roll, pitch, yaw (rx,ry,rz)."""
    sy = -R[2,0]
    sy = np.clip(sy, -1.0, 1.0)
    ry = np.arcsin(sy)
    cy = np.cos(ry)
    if abs(cy) < 1e-6:
        rx = np.arctan2(-R[0,1], R[1,1])
        rz = 0.0
    else:
        rx = np.arctan2(R[2,1], R[2,2])
        rz = np.arctan2(R[1,0], R[0,0])
    return rx, ry, rz

# ----------------------------- Geometry prior builders -----------------------

def fit_cylinder_z_center(xy):
    """Least-squares circle fit on (x,y) for a cylinder whose axis is +Z."""
    x = xy[:,0]; y = xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    b = x**2 + y**2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    x0, y0, _ = sol
    return float(x0), float(y0)

def R_geo_cylinder_z(x, y, x0, y0):
    """Local frame for a cylinder whose axis is +Z."""
    a = np.array([0.0, 0.0, 1.0])                # axis = +Z
    n = np.array([x - x0, y - y0, 0.0])          # radial in XY
    n_norm = np.linalg.norm(n)
    if n_norm < 1e-9:
        n = np.array([1.0, 0.0, 0.0]); n_norm = 1.0
    n = n / n_norm
    t = np.cross(a, n); t = t / (np.linalg.norm(t)+1e-12)
    b = np.cross(n, t)
    R = np.stack([t, b, n], axis=1)
    return R

def fit_cylinder_x_center(yz):
    """Least-squares circle fit on (y,z) for a cylinder whose axis is +X."""
    y = yz[:,0]; z = yz[:,1]
    A = np.c_[2*y, 2*z, np.ones_like(y)]
    b = y**2 + z**2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    y0, z0, _ = sol
    return float(y0), float(z0)

def R_geo_cylinder_x(y, z, y0, z0):
    """
    Local frame for a cylinder whose axis is +X.
      - â = +x̂ is the axis
      - n̂ is radial in the YZ plane from (y0,z0) to (y,z)
      - Orthonormal right-handed basis columns = [t̂, b̂, n̂]
        where t̂ = â × n̂, b̂ = n̂ × t̂
    """
    a = np.array([1.0, 0.0, 0.0])                # axis = +X
    n = np.array([0.0, y - y0, z - z0])          # radial in YZ
    n_norm = np.linalg.norm(n)
    if n_norm < 1e-9:
        n = np.array([0.0, 1.0, 0.0]); n_norm = 1.0
    n = n / n_norm
    t = np.cross(a, n); t = t / (np.linalg.norm(t)+1e-12)
    b = np.cross(n, t)
    R = np.stack([t, b, n], axis=1)
    return R

def fit_plane_normal(XYZ):
    """Plane via PCA; normal is smallest singular vector."""
    P = XYZ - XYZ.mean(axis=0, keepdims=True)
    _,_,Vt = np.linalg.svd(P, full_matrices=False)
    n = Vt[-1]
    n = n / (np.linalg.norm(n) + 1e-12)
    return n

def R_geo_plane(p, n):
    """Orthonormal frame on plane with normal n; t points along projection of ẑ."""
    n = n / (np.linalg.norm(n) + 1e-12)
    z = np.array([0.0,0.0,1.0])
    t = z - np.dot(z,n)*n
    if np.linalg.norm(t) < 1e-9:
        t = np.array([1.0,0.0,0.0])
    t = t / (np.linalg.norm(t)+1e-12)
    b = np.cross(n, t); b = b / (np.linalg.norm(b)+1e-12)
    R = np.stack([t, b, n], axis=1)
    return R

def markley_quat_mean(quats):
    """Average quaternions (Nx4) with Markley method."""
    A = np.zeros((4,4))
    for q in quats:
        q = q / (np.linalg.norm(q)+1e-12)
        A += np.outer(q, q)
    A /= len(quats)
    w, v = np.linalg.eigh(A)
    q = v[:, np.argmax(w)]
    if q[0] < 0: q = -q
    return q / (np.linalg.norm(q)+1e-12)

# ----------------------------- Torch bits ------------------------------------

class ResidualQuatMLP(nn.Module):
    def __init__(self, in_dim, hidden=(256,128), dropout=0.15):
        super().__init__()
        layers = []
        last = in_dim
        for h in hidden:
            layers += [nn.Linear(last, h), nn.ReLU(), nn.Dropout(dropout)]
            last = h
        layers += [nn.Linear(last, 4)]  # outputs raw quaternion; we normalize in forward
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        q = self.net(x)
        # normalize to unit quaternion
        q = q / (torch.norm(q, dim=1, keepdim=True) + 1e-12)
        # enforce positive scalar to reduce sign flips (optional)
        sign = torch.sign(q[:, :1] + 1e-12)
        q = q * sign
        return q

def geodesic_quat_loss(q_pred, q_true):
    # both [B,4], unit quats; loss = mean(angle^2) where angle = 2*arccos(|dot|)
    d = torch.clamp(torch.abs(torch.sum(q_pred*q_true, dim=1)), 0.0, 1.0)
    ang = 2.0*torch.arccos(d)
    return torch.mean(ang*ang)

# ----------------------------- Main -----------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Stage-2: Geometry prior + residual quaternion MLP (ΔEIT + xyz_pred)")
    ap.add_argument("--csv", required=True, type=str)
    ap.add_argument("--stage1_model", required=True, type=str, help="Path to Stage-1 ensemble .joblib")
    ap.add_argument("--outdir", default="./outputs_stage2", type=str)

    # Filtering / normalization (mirror Stage-1)
    ap.add_argument("--min_Fz", type=float, default=None)
    ap.add_argument("--min_deit_pct", type=float, default=None)
    ap.add_argument("--winsor_pct", type=float, default=None)
    ap.add_argument("--per_session_norm", action="store_true")
    ap.add_argument("--grouping", choices=["xyz","session"], default="xyz")
    ap.add_argument("--test_size", type=float, default=0.2)

    # Geometry prior options (default to cylinder_x per your setup)
    ap.add_argument("--prior", choices=["cylinder_z","cylinder_x","plane"], default="cylinder_x")

    # Feature shaping
    ap.add_argument("--pca_var", type=float, default=0.98, help="PCA on ΔEIT before MLP inputs; set <=0 to disable.")
    ap.add_argument("--use_true_xyz_for_training", action="store_true",
                    help="If set, use gt xyz for TRAIN inputs; test always uses stage1 xyz_pred.")

    # Model / training
    ap.add_argument("--hidden", type=str, default="256,128")
    ap.add_argument("--dropout", type=float, default=0.15)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--weight_decay", type=float, default=1e-4)
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument("--patience", type=int, default=20)
    ap.add_argument("--batch_size", type=int, default=64)
    ap.add_argument("--no_progress", action="store_true")

    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    # ---- Load data
    df = pd.read_csv(args.csv)
    eitb = [f"eitb_{i}" for i in range(256)]
    eita = [f"eita_{i}" for i in range(256)]
    ensure_cols(df, eitb, "eitb_*"); ensure_cols(df, eita, "eita_*")
    ensure_cols(df, ["act_tcp_x","act_tcp_y","act_tcp_z","act_tcp_rx","act_tcp_ry","act_tcp_rz"], "targets")

    # Filters
    if args.min_Fz is not None:
        if "Fz" not in df.columns:
            raise ValueError("CSV missing 'Fz' column but --min_Fz was provided.")
        before = len(df)
        df = df[df["Fz"] >= args.min_Fz].copy()
        print(f"[filter] Fz >= {args.min_Fz}: kept {len(df)}/{before} rows.")

    X_raw = (df[eita].values - df[eitb].values).astype(np.float32)
    Y_pos = df[["act_tcp_x","act_tcp_y","act_tcp_z"]].values.astype(np.float32)
    Y_rpy = df[["act_tcp_rx","act_tcp_ry","act_tcp_rz"]].values.astype(np.float64)  # radians

    if args.min_deit_pct is not None:
        norms = np.linalg.norm(X_raw, axis=1)
        thr = np.percentile(norms, args.min_deit_pct)
        keep = norms >= thr
        before = len(df)
        X_raw, Y_pos, Y_rpy, df = X_raw[keep], Y_pos[keep], Y_rpy[keep], df.loc[keep].reset_index(drop=True)
        print(f"[filter] ||ΔEIT|| ≥ P{args.min_deit_pct:g}: kept {len(df)}/{before} rows.")

    if args.winsor_pct is not None:
        X_raw = winsorize_cols(X_raw, p=float(args.winsor_pct))
        print(f"[clean] Winsorized ΔEIT at p={args.winsor_pct:g}.")

    if args.per_session_norm and "session_id" in df.columns:
        X_raw = per_session_zscore(X_raw, df["session_id"].astype(str).values)
        print("[norm] Per-session ΔEIT z-scoring applied.")
    elif args.per_session_norm:
        print("[warn] --per_session_norm set but no session_id column found; skipping.")

    # ---- Stage-1 predictions for xyz (bagged + stacked)
    ensembles = load(args.stage1_model)  # list of dicts with "models"
    if not isinstance(ensembles, list) or not len(ensembles):
        raise RuntimeError("Stage-1 model file doesn't contain an ensemble list.")
    preds_all = []
    for ens in ensembles:
        models = ens["models"]
        if not models: continue
        combo_preds = [m.predict(X_raw) for m in models]
        preds_all.append(np.mean(combo_preds, axis=0))
    weights = np.array([1.0 / (ens["cv_score"]**2 + 1e-12) for ens in ensembles], dtype=float)
    weights = weights / weights.sum()
    xyz_pred_all = np.tensordot(weights, np.stack(preds_all, axis=0), axes=(0,0)).astype(np.float64)

    # ---- Train/val/test split
    groups_all = build_groups(df, mode=args.grouping)
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=RANDOM_STATE)
    trval_idx, te_idx = next(gss.split(X_raw, Y_pos, groups=groups_all))
    df_train = df.iloc[trval_idx].reset_index(drop=True)
    df_test  = df.iloc[te_idx].reset_index(drop=True)

    X_raw_trval, X_raw_test = X_raw[trval_idx], X_raw[te_idx]
    Y_pos_trval, Y_pos_test = Y_pos[trval_idx], Y_pos[te_idx]
    Y_rpy_trval, Y_rpy_test = Y_rpy[trval_idx], Y_rpy[te_idx]
    xyz_pred_trval, xyz_pred_test = xyz_pred_all[trval_idx], xyz_pred_all[te_idx]

    # ---- Build geometry prior from TRAIN only
    x0 = y0 = z0 = None
    n = None
    if args.prior == "cylinder_z":
        x0, y0 = fit_cylinder_z_center(Y_pos_trval[:, :2])
        print(f"[prior] Cylinder-Z center: (x0={x0:.4f}, y0={y0:.4f})")
        def R_geo_from_pos(pos):
            # pos = [x,y,z]
            return R_geo_cylinder_z(pos[0], pos[1], x0, y0)
    elif args.prior == "cylinder_x":
        y0, z0 = fit_cylinder_x_center(Y_pos_trval[:, 1:3])
        print(f"[prior] Cylinder-X center: (y0={y0:.4f}, z0={z0:.4f})")
        def R_geo_from_pos(pos):
            # pos = [x,y,z]
            return R_geo_cylinder_x(pos[1], pos[2], y0, z0)
    else:
        n = fit_plane_normal(Y_pos_trval)
        print(f"[prior] Plane normal: {n}")
        def R_geo_from_pos(pos):
            return R_geo_plane(pos, n)

    # True quats & geo quats on TRAIN; learn global offset
    q_true_tr = []
    q_geo_tr  = []
    for p, rpy in zip(Y_pos_trval, Y_rpy_trval):
        R_true = euler_rpy_to_matrix(rpy[0], rpy[1], rpy[2])
        q_true_tr.append(matrix_to_quat(R_true))
        R_geo = R_geo_from_pos(p)
        q_geo_tr.append(matrix_to_quat(R_geo))
    q_true_tr = np.stack(q_true_tr, axis=0)
    q_geo_tr  = np.stack(q_geo_tr,  axis=0)
    q_delta   = np.array([quat_mul(qt, quat_inv(qg)) for qt,qg in zip(q_true_tr, q_geo_tr)])
    q_off     = markley_quat_mean(q_delta)
    print(f"[prior] Learned global offset quaternion: {q_off}")

    def q_prior_from_pos(pos):
        """Prior quaternion for any position: q_prior = q_off ⊗ q_geo(pos)."""
        Rg = R_geo_from_pos(pos)
        qg = matrix_to_quat(Rg)
        qp = quat_mul(q_off, qg)
        return qp / (np.linalg.norm(qp)+1e-12)

    # ---- Build residual targets on TRAIN/TEST
    def build_targets(Y_pos_arr, Y_rpy_arr):
        q_true = []
        q_prior = []
        for p, rpy in zip(Y_pos_arr, Y_rpy_arr):
            R_true = euler_rpy_to_matrix(rpy[0], rpy[1], rpy[2])
            q_true.append(matrix_to_quat(R_true))
            q_prior.append(q_prior_from_pos(p))
        q_true  = np.stack(q_true, 0)
        q_prior = np.stack(q_prior,0)
        q_res   = np.array([quat_mul(qt, quat_inv(qp)) for qt,qp in zip(q_true,q_prior)])
        q_res[q_res[:,0]<0] *= -1.0  # ensure positive scalar to reduce sign flip ambiguity
        return q_res, q_prior, q_true

    # Inputs to MLP: [ΔEIT (optionally PCA), xyz_input]
    xyz_input_tr = Y_pos_trval if args.use_true_xyz_for_training else xyz_pred_trval
    xyz_input_te = xyz_pred_test

    # ΔEIT scaler + PCA fit on TRAIN ONLY
    x_scaler = StandardScaler().fit(X_raw_trval)
    Xs_tr = x_scaler.transform(X_raw_trval)
    Xs_te = x_scaler.transform(X_raw_test)

    if args.pca_var is not None and args.pca_var > 0:
        pca = PCA(n_components=float(args.pca_var), svd_solver="full", random_state=RANDOM_STATE).fit(Xs_tr)
        Xf_tr = pca.transform(Xs_tr)
        Xf_te = pca.transform(Xs_te)
        print(f"[feat] PCA retained {pca.n_components_} comps (var={args.pca_var})")
    else:
        pca = None
        Xf_tr, Xf_te = Xs_tr, Xs_te
        print("[feat] PCA disabled.")

    # residual targets
    q_res_tr, q_prior_tr, q_true_tr2 = build_targets(xyz_input_tr, Y_rpy_trval)
    q_res_te, q_prior_te, q_true_te  = build_targets(xyz_input_te, Y_rpy_test)

    # Final input features
    X_mlp_tr = np.concatenate([Xf_tr, xyz_input_tr], axis=1).astype(np.float32)
    X_mlp_te = np.concatenate([Xf_te, xyz_input_te], axis=1).astype(np.float32)

    # Final input scaler
    in_scaler = StandardScaler().fit(X_mlp_tr)
    X_mlp_tr = in_scaler.transform(X_mlp_tr).astype(np.float32)
    X_mlp_te = in_scaler.transform(X_mlp_te).astype(np.float32)

    # Torch data
    y_tr = q_res_tr.astype(np.float32)
    y_te = q_res_te.astype(np.float32)
    dtrain = TensorDataset(torch.from_numpy(X_mlp_tr), torch.from_numpy(y_tr))
    dtest  = TensorDataset(torch.from_numpy(X_mlp_te), torch.from_numpy(y_te))
    LTe = DataLoader(dtest,  batch_size=args.batch_size, shuffle=False, drop_last=False)

    # Train/val split inside TRAIN
    rng = np.random.default_rng(RANDOM_STATE)
    n = X_mlp_tr.shape[0]
    idx = np.arange(n); rng.shuffle(idx)
    n_val = max(1, int(0.15*n))
    val_idx = idx[:n_val]; tr_idx = idx[n_val:]

    Xtr = torch.from_numpy(X_mlp_tr[tr_idx]).to(DEVICE)
    Ytr = torch.from_numpy(y_tr[tr_idx]).to(DEVICE)
    Xva = torch.from_numpy(X_mlp_tr[val_idx]).to(DEVICE)
    Yva = torch.from_numpy(y_tr[val_idx]).to(DEVICE)

    # Model
    hidden = tuple(int(h) for h in args.hidden.split(",") if h.strip())
    model = ResidualQuatMLP(in_dim=X_mlp_tr.shape[1], hidden=hidden, dropout=args.dropout).to(DEVICE)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)

    # Train w/ early stopping
    best = {"epoch": -1, "val": 1e9, "state": None}
    pbar = tqdm(range(args.epochs), disable=args.no_progress, desc="epochs")
    for ep in pbar:
        model.train()
        # mini-batch SGD over train subset
        perm = rng.permutation(len(Xtr))
        bsz = args.batch_size
        losses = []
        for i in range(0, len(Xtr), bsz):
            j = perm[i:i+bsz]
            xb = Xtr[j]; yb = Ytr[j]
            opt.zero_grad()
            qp = model(xb)
            loss = geodesic_quat_loss(qp, yb)
            loss.backward()
            opt.step()
            losses.append(float(loss.item()))
        # val
        model.eval()
        with torch.no_grad():
            qv = model(Xva)
            vloss = float(geodesic_quat_loss(qv, Yva).item())
        pbar.set_postfix(train=np.mean(losses), val=vloss)
        if vloss < best["val"] - 1e-6:
            best = {"epoch": ep, "val": vloss, "state": {k: v.cpu().clone() for k,v in model.state_dict().items()}}
            no_improve = 0
        else:
            no_improve = (ep - best["epoch"])
        if no_improve >= args.patience:
            break

    # Restore best
    if best["state"] is not None:
        model.load_state_dict(best["state"])
    print(f"[train] best val loss {best['val']:.6f} @ epoch {best['epoch']}")

    # Evaluate on TEST: build final quats and stats
    model.eval()
    with torch.no_grad():
        q_res_pred = model(torch.from_numpy(X_mlp_te).to(DEVICE)).cpu().numpy()

    # Compose final orientation: q_pred = Δq_pred ⊗ q_prior
    q_pred = np.array([quat_mul(qp, qr) for qp, qr in zip(q_res_pred, q_prior_te)])
    # Geodesic error
    geo = np.array([quat_geodesic_angle(qp, qt) for qp, qt in zip(q_pred, q_true_te)])
    geo_deg = np.degrees(geo)
    mean_deg = float(np.mean(geo_deg))
    med_deg  = float(np.median(geo_deg))
    p90_deg  = float(np.percentile(geo_deg, 90))
    p95_deg  = float(np.percentile(geo_deg, 95))
    print("\n[test] Orientation (quaternion → geodesic):")
    print(f"   mean={mean_deg:.2f}°, median={med_deg:.2f}°, P90={p90_deg:.2f}°, P95={p95_deg:.2f}°")

    # Also compute rx,ry,rz from q_pred (for intuition / plots)
    rpy_true = []
    rpy_pred = []
    for q_t, q_p in zip(q_true_te, q_pred):
        Rt = quat_to_matrix(q_t)
        Rp = quat_to_matrix(q_p)
        rpy_true.append(rpy_from_matrix(Rt))
        rpy_pred.append(rpy_from_matrix(Rp))
    rpy_true = np.array(rpy_true)
    rpy_pred = np.array(rpy_pred)
    names = ["act_tcp_rx_deg", "act_tcp_ry_deg", "act_tcp_rz_deg"]
    for i, nm in enumerate(names):
        plt.figure()
        plt.scatter(np.degrees(rpy_true[:,i]), np.degrees(rpy_pred[:,i]), s=10)
        lo = float(min(np.degrees(rpy_true[:,i]).min(), np.degrees(rpy_pred[:,i]).min()))
        hi = float(max(np.degrees(rpy_true[:,i]).max(), np.degrees(rpy_pred[:,i]).max()))
        plt.plot([lo,hi],[lo,hi],'k-',lw=1)
        plt.xlabel(f"True {nm}"); plt.ylabel(f"Pred {nm}")
        plt.title(f"Pred vs True — {nm}")
        plt.grid(True, ls="--", lw=0.5); plt.tight_layout()
        plt.savefig(os.path.join(args.outdir, f"mlp_pvstrue_{nm}.png"), dpi=160)

    # Hist of geodesic errors
    plt.figure()
    plt.hist(geo_deg, bins=60)
    plt.xlabel("Geodesic error (deg)"); plt.ylabel("Count")
    plt.title("Orientation geodesic errors (test)")
    plt.grid(True, ls="--", lw=0.5); plt.tight_layout()
    plt.savefig(os.path.join(args.outdir, "geodesic_errors_deg_hist.png"), dpi=160)

    # Save artifacts
    meta = {
        "created_utc": datetime.utcnow().isoformat()+"Z",
        "csv_path": os.path.abspath(args.csv),
        "stage1_model": os.path.abspath(args.stage1_model),
        "rows_final": int(len(df)),
        "test_size_groups": args.test_size,
        "grouping": args.grouping,
        "prior": args.prior,
        "pca_var": None if (args.pca_var is None or args.pca_var<=0) else float(args.pca_var),
        "hidden": [int(h) for h in hidden],
        "dropout": float(args.dropout),
        "lr": float(args.lr),
        "weight_decay": float(args.weight_decay),
        "epochs": int(args.epochs),
        "patience": int(args.patience),
        "batch_size": int(args.batch_size),
        "per_session_norm": bool(args.per_session_norm),
        "min_deit_pct": None if args.min_deit_pct is None else float(args.min_deit_pct),
        "winsor_pct": None if args.winsor_pct is None else float(args.winsor_pct),
        "min_Fz": None if args.min_Fz is None else float(args.min_Fz),
        "use_true_xyz_for_training": bool(args.use_true_xyz_for_training),
        "geo_stats_deg": {"mean": mean_deg, "median": med_deg, "p90": p90_deg, "p95": p95_deg}
    }
    with open(os.path.join(args.outdir, "stage2_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # Save model + preprocessors + prior params
    bundle = {
        "in_scaler_mean": in_scaler.mean_.astype(np.float32),
        "in_scaler_scale": in_scaler.scale_.astype(np.float32),
        "x_scaler_mean": x_scaler.mean_.astype(np.float32),
        "x_scaler_scale": x_scaler.scale_.astype(np.float32),
        "pca_components": None if pca is None else pca.components_.astype(np.float32),
        "pca_mean": None if pca is None else pca.mean_.astype(np.float32),
        "pca_var": None if pca is None else float(args.pca_var),
        "prior_type": args.prior,
        "cyl_axis": ("z" if args.prior=="cylinder_z" else ("x" if args.prior=="cylinder_x" else None)),
        # Center of the circular cross-section (depends on axis):
        #  - cylinder_z: (x0, y0) in the XY plane
        #  - cylinder_x: (y0, z0) in the YZ plane
        "cyl_center": (float(x0), float(y0)) if args.prior=="cylinder_z"
                       else ((float(y0), float(z0)) if args.prior=="cylinder_x" else None),
        "plane_normal": None if args.prior!="plane" else n.tolist(),
        "q_off": q_off.tolist(),
        "state_dict": {k: v.cpu().numpy() for k,v in model.state_dict().items()},
        "in_dim": int(X_mlp_tr.shape[1]),
        "hidden": [int(h) for h in hidden],
        "dropout": float(args.dropout)
    }
    dump(bundle, os.path.join(args.outdir, "stage2_quat_mlp.joblib"))
    print(f"\n[save] Wrote artifacts to: {os.path.abspath(args.outdir)}")
    return 0

if __name__ == "__main__":
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        raise SystemExit(main())
