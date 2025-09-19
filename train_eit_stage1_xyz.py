#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage-1 (XYZ) training script for EIT → TCP position.
- Inputs: ΔEIT = eita_* - eitb_* (256 electrodes)
- Targets: (act_tcp_x, act_tcp_y, act_tcp_z)

Features
--------
• Filtering:
  --min_Fz, --min_deit_pct (by ||ΔEIT||), --winsor_pct, --contact_only
• Normalization:
  --per_session_norm (z-score ΔEIT per-session)
• Group-aware split:
  --grouping {xyz, session}, --test_size
• Models:
  --model {ridge, pls, krr, both, all}
  Ridge:       --alpha_grid
  PLS:         --pls_comp_grid, --pca_var_grid
  KRR (RBF):   --krr_alpha_grid, --krr_gamma_grid, --pca_var_grid
• Cross-validation:
  --cv_folds
• Ensembling:
  --bag_models (per best combo), --stack_top_n (across families)
• Learning curve:
  --lc_points {auto|N}
• Progress control:
  --no_progress

Outputs
-------
• Console metrics on test:
  - Vector RMSE/MAE (3D)
  - Per-axis RMSE/MAE
  - XY-plane Vector RMSE/MAE  (NEW)
• Plots: pred-vs-true (x,y,z), learning curve
• Files:
  - {outdir}/eit_stage1_xyz_ensemble.joblib  (models + preprocessors + CV scores)
  - {outdir}/stage1_meta.json
  - {outdir}/learning_curve.csv / .png
"""

import argparse, json, os, warnings
from datetime import datetime

import numpy as np
import pandas as pd

from joblib import dump
from tqdm.auto import tqdm

from sklearn.model_selection import GroupShuffleSplit, GroupKFold
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.linear_model import Ridge
from sklearn.cross_decomposition import PLSRegression
from sklearn.kernel_ridge import KernelRidge

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RANDOM_STATE = 42


# ----------------------------- Utilities -------------------------------------

def ensure_cols(df, cols, name):
    miss = [c for c in cols if c not in df.columns]
    if miss:
        raise ValueError(f"Missing {name} columns (e.g. {miss[:5]})")

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
    # group by 1 mm xyz bins to reduce leakage
    xq = np.round(df["act_tcp_x"].values/pos_bin_m).astype(np.int64)
    yq = np.round(df["act_tcp_y"].values/pos_bin_m).astype(np.int64)
    zq = np.round(df["act_tcp_z"].values/pos_bin_m).astype(np.int64)
    return pd.Series([f"{a}_{b}_{c}" for a,b,c in zip(xq,yq,zq)], index=df.index)

def vec_rmse(y_true, y_pred):  # 3D vector RMSE (meters)
    d = y_pred - y_true
    v = np.linalg.norm(d, axis=1)
    return float(np.sqrt(np.mean(v**2)))

def vec_mae(y_true, y_pred):   # 3D vector MAE (meters)
    d = y_pred - y_true
    v = np.linalg.norm(d, axis=1)
    return float(np.mean(v))

def compute_xy_plane_metrics(pos_true: np.ndarray, pos_pred: np.ndarray):
    """
    NEW: XY-plane 'vector' errors treat per-sample error as ||[dx, dy]|| (m),
    ignoring z. Returns (RMSE_xy, MAE_xy).
    """
    d = pos_pred - pos_true                 # [N,3]
    vxy = np.linalg.norm(d[:, :2], axis=1)  # per-sample XY magnitude
    rmse_xy = float(np.sqrt(np.mean(vxy**2)))
    mae_xy  = float(np.mean(vxy))
    return rmse_xy, mae_xy

def per_axis_metrics(y_true, y_pred):
    d = y_pred - y_true
    rmse = np.sqrt(np.mean(d**2, axis=0))
    mae  = np.mean(np.abs(d), axis=0)
    return rmse.astype(float), mae.astype(float)

def parse_grid_floats(s):
    return [float(x) for x in str(s).split(",") if str(x).strip()]

def parse_grid_ints(s):
    return [int(x) for x in str(s).split(",") if str(x).strip()]

def parse_pca_grid(s):
    """
    Accepts floats (variance) or 'none' (disable PCA).
    Returns list of values; 'none' -> 0.0 (disabled).
    """
    items = [x.strip() for x in str(s).split(",")]
    vals = []
    for it in items:
        if it.lower() in ("none","0","0.0"):
            vals.append(0.0)
        else:
            vals.append(float(it))
    return vals

def cap_pls_components(n_samples, n_features, n_targets=3):
    # scikit constraint: n_components <= min(n_samples, n_features, n_targets)
    return max(1, int(min(n_samples, n_features, n_targets)))


# ----------------------------- Model Builders --------------------------------

def build_ridge(alpha):
    # Standardize features before Ridge
    return ("ridge", StandardScaler(with_mean=True, with_std=True), Ridge(alpha=alpha, random_state=RANDOM_STATE))

def build_pls(n_components):
    return ("pls", None, PLSRegression(n_components=n_components, scale=True))

def build_krr(alpha, gamma):
    # KRR benefits from standardized inputs
    return ("krr_rbf", StandardScaler(with_mean=True, with_std=True),
            KernelRidge(alpha=alpha, kernel="rbf", gamma=gamma))

def apply_pca_fit(X_tr, pca_var):
    if pca_var and pca_var > 0:
        pca = PCA(n_components=float(pca_var), svd_solver="full", random_state=RANDOM_STATE).fit(X_tr)
        return pca, pca.transform(X_tr)
    return None, X_tr

def apply_pca_transform(pca, X):
    return X if pca is None else pca.transform(X)


# ----------------------------- Main ------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Stage-1: ΔEIT → XYZ regression with CV, bagging, stacking, learning curve.")
    ap.add_argument("--csv", required=True, type=str)
    ap.add_argument("--outdir", default="./outputs_stage1", type=str)

    # Filtering / normalization
    ap.add_argument("--contact_only", action="store_true", help="Use only rows where contact_detected==1")
    ap.add_argument("--min_Fz", type=float, default=None, help="Keep rows with Fz >= this value")
    ap.add_argument("--min_deit_pct", type=float, default=None, help="Keep rows with ||ΔEIT|| >= p-th percentile (e.g., 12)")
    ap.add_argument("--winsor_pct", type=float, default=None, help="Winsorize ΔEIT columns at this percentile (e.g., 99.5)")
    ap.add_argument("--per_session_norm", action="store_true", help="Z-score ΔEIT per session_id")

    # Grouping / split
    ap.add_argument("--grouping", choices=["xyz","session"], default="xyz")
    ap.add_argument("--test_size", type=float, default=0.2)
    ap.add_argument("--cv_folds", type=int, default=5)

    # Model control
    ap.add_argument("--model", choices=["ridge","pls","krr","both","all"], default="both")
    ap.add_argument("--alpha_grid", type=str, default="0.03,0.1,0.3,1,3,10,30,100")
    ap.add_argument("--pls_comp_grid", type=str, default="8,16,32,48")
    ap.add_argument("--krr_alpha_grid", type=str, default="0.1,1,3,10")
    ap.add_argument("--krr_gamma_grid", type=str, default="1e-4,3e-4,1e-3,3e-3")
    ap.add_argument("--pca_var_grid", type=str, default="0.98,0.99,none")

    # Ensembling
    ap.add_argument("--bag_models", type=int, default=5, help="Per best combo")
    ap.add_argument("--stack_top_n", type=int, default=3, help="Top combos to stack across families")

    # Learning curve
    ap.add_argument("--lc_points", type=str, default="auto")

    # Progress
    ap.add_argument("--no_progress", action="store_true")

    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    # ---------------- Data load ----------------
    df = pd.read_csv(args.csv)
    eitb = [f"eitb_{i}" for i in range(256)]
    eita = [f"eita_{i}" for i in range(256)]
    ensure_cols(df, eitb, "eitb_*"); ensure_cols(df, eita, "eita_*")
    ensure_cols(df, ["act_tcp_x","act_tcp_y","act_tcp_z"], "targets")

    # ΔEIT
    X_raw = (df[eita].values - df[eitb].values).astype(np.float32)
    Y = df[["act_tcp_x","act_tcp_y","act_tcp_z"]].values.astype(np.float32)

    # Filters
    if args.contact_only:
        if "contact_detected" not in df.columns:
            raise ValueError("contact_only set but 'contact_detected' not in CSV.")
        before = len(df)
        m = (df["contact_detected"].astype(int) == 1)
        X_raw, Y, df = X_raw[m], Y[m], df.loc[m].reset_index(drop=True)
        print(f"[filter] contact_detected==1: kept {len(df)}/{before} rows.")

    if args.min_Fz is not None:
        if "Fz" not in df.columns:
            raise ValueError("CSV missing 'Fz' column but --min_Fz was provided.")
        before = len(df)
        m = (df["Fz"].values >= args.min_Fz)
        X_raw, Y, df = X_raw[m], Y[m], df.loc[m].reset_index(drop=True)
        print(f"[filter] Fz >= {args.min_Fz:.1f}: kept {len(df)}/{before} rows.")

    if args.min_deit_pct is not None:
        norms = np.linalg.norm(X_raw, axis=1)
        thr = np.percentile(norms, float(args.min_deit_pct))
        m = norms >= thr
        before = len(df)
        X_raw, Y, df = X_raw[m], Y[m], df.loc[m].reset_index(drop=True)
        print(f"[filter] ||ΔEIT|| ≥ P{args.min_deit_pct:g}: kept {len(df)}/{before} rows.")

    if args.winsor_pct is not None:
        X_raw = winsorize_cols(X_raw, p=float(args.winsor_pct))
        print(f"[clean] Winsorized ΔEIT at p={args.winsor_pct:g}.")

    if args.per_session_norm and "session_id" in df.columns:
        X_raw = per_session_zscore(X_raw, df["session_id"].astype(str).values)
        print("[norm] Per-session ΔEIT z-scoring applied.")
    elif args.per_session_norm:
        print("[warn] --per_session_norm set but no session_id column found; skipping.")

    # ---------------- Split ----------------
    groups_all = build_groups(df, mode=args.grouping)
    gss = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=RANDOM_STATE)
    trval_idx, te_idx = next(gss.split(X_raw, Y, groups=groups_all))

    X_trval, X_te = X_raw[trval_idx], X_raw[te_idx]
    Y_trval, Y_te = Y[trval_idx], Y[te_idx]
    df_trval = df.iloc[trval_idx].reset_index(drop=True)
    df_test  = df.iloc[te_idx].reset_index(drop=True)

    print(f"[split] Train+Val={len(X_trval)}  Test={len(X_te)}  Total={len(df)} (from original {len(df)})")

    # ---------------- CV setup ----------------
    families = []
    if args.model in ("ridge","both","all"):
        for a in parse_grid_floats(args.alpha_grid):
            families.append(("ridge", {"alpha": a, "pca_var": 0.0}))
    if args.model in ("pls","both","all"):
        for nc in parse_grid_ints(args.pls_comp_grid):
            for pv in parse_pca_grid(args.pca_var_grid):
                families.append(("pls", {"n_components": nc, "pca_var": pv}))
    if args.model in ("krr","both","all"):
        for a in parse_grid_floats(args.krr_alpha_grid):
            for g in parse_grid_floats(args.krr_gamma_grid):
                for pv in parse_pca_grid(args.pca_var_grid):
                    families.append(("krr", {"alpha": a, "gamma": g, "pca_var": pv}))

    # group K-fold on train/val portion
    groups_trval = build_groups(df_trval, mode=args.grouping)
    gkf = GroupKFold(n_splits=args.cv_folds)

    # Metric: lower is better (vector RMSE)
    results = []
    combos = []
    pbar = tqdm(families, disable=args.no_progress, desc=f"CV (combos × folds)")
    for fam, params in pbar:
        # Prepare features (scaler + optional PCA fit on each fold's train)
        fold_scores = []
        for fold, (tr_idx, va_idx) in enumerate(gkf.split(X_trval, Y_trval, groups=groups_trval)):
            X_tr, X_va = X_trval[tr_idx], X_trval[va_idx]
            Y_tr, Y_va = Y_trval[tr_idx], Y_trval[va_idx]

            # Optional PCA (fit on fold-train)
            if fam in ("pls","krr"):
                pca_var = float(params.get("pca_var", 0.0))
            else:
                pca_var = 0.0
            pca, X_tr_f = apply_pca_fit(X_tr, pca_var)
            X_va_f = apply_pca_transform(pca, X_va)

            # Build model
            if fam == "ridge":
                name, scaler, mdl = build_ridge(params["alpha"])
                # For ridge, we typically don't PCA (it can, but not needed)
                pca = None
                X_tr_f, X_va_f = X_tr, X_va
            elif fam == "pls":
                # cap components safely
                max_nc = cap_pls_components(len(X_tr), X_tr_f.shape[1], n_targets=3)
                n_comp = min(int(params["n_components"]), max_nc)
                name, scaler, mdl = build_pls(n_comp)
            elif fam == "krr":
                name, scaler, mdl = build_krr(params["alpha"], params["gamma"])
            else:
                raise ValueError(fam)

            # Scale if requested
            if scaler is not None:
                scaler = scaler.fit(X_tr_f)
                X_tr_f = scaler.transform(X_tr_f)
                X_va_f = scaler.transform(X_va_f)

            # Fit and score
            mdl.fit(X_tr_f, Y_tr)
            Y_hat = mdl.predict(X_va_f)
            score = vec_rmse(Y_va, Y_hat)
            fold_scores.append(score)

        cv = float(np.mean(fold_scores))
        combos.append((fam, params, cv))
        results.append(cv)

    # Summaries
    fam_params_sorted = sorted(combos, key=lambda x: x[2])  # lower = better
    print("[cv] Composite metric: vector RMSE (lower is better)")
    for fam, params, cv in fam_params_sorted[:min(12, len(fam_params_sorted))]:
        print(f"   {fam} {params} -> score={cv:.6f}")

    # Choose top-N combos across families for stacking
    topN = min(args.stack_top_n, len(fam_params_sorted))
    top_combos = fam_params_sorted[:topN]

    # ---------------- Fit bagged models per top combo ----------------
    ensembles = []  # list of {"family","params","cv_score","pca","scaler","models":[mdl1,...]}
    rng = np.random.default_rng(RANDOM_STATE)
    for fam, params, cv in tqdm(top_combos, disable=args.no_progress, desc="Bagging models"):
        # Prepare global PCA on all trval for this combo (like a refit on full trval)
        if fam in ("pls","krr"):
            pca_var = float(params.get("pca_var", 0.0))
        else:
            pca_var = 0.0
        pca, X_trval_f = apply_pca_fit(X_trval, pca_var)

        # Build scaler template and family-specific estimator template
        if fam == "ridge":
            name, scaler, tmpl = build_ridge(params["alpha"])
            pca = None
            X_trval_f = X_trval
        elif fam == "pls":
            max_nc = cap_pls_components(len(X_trval), X_trval_f.shape[1], n_targets=3)
            n_comp = min(int(params["n_components"]), max_nc)
            name, scaler, tmpl = build_pls(n_comp)
        elif fam == "krr":
            name, scaler, tmpl = build_krr(params["alpha"], params["gamma"])
        else:
            raise ValueError(fam)

        if scaler is not None:
            scaler = scaler.fit(X_trval_f)
            X_trval_f = scaler.transform(X_trval_f)

        # Bagging: bootstrap 80% with replacement
        models = []
        n = len(X_trval_f)
        bsz = max(64, int(0.8 * n))
        for b in range(args.bag_models):
            idx = rng.integers(0, n, size=bsz)
            Xb, Yb = X_trval_f[idx], Y_trval[idx]
            mdl = tmpl.__class__(**tmpl.get_params())
            mdl.fit(Xb, Yb)
            models.append(mdl)

        ensembles.append({
            "family": fam,
            "params": params,
            "cv_score": cv,
            "pca_components": None if pca is None else pca.components_.astype(np.float32),
            "pca_mean": None if pca is None else pca.mean_.astype(np.float32),
            "pca_var": None if pca is None else pca.n_components_,
            "scaler_mean": None if (scaler is None or not hasattr(scaler, "mean_")) else scaler.mean_.astype(np.float32),
            "scaler_scale": None if (scaler is None or not hasattr(scaler, "scale_")) else scaler.scale_.astype(np.float32),
            "models": models
        })

    # ---------------- Ensemble inference on TEST ----------------
    preds_per_combo = []
    weights = []
    for ens in ensembles:
        # rebuild PCA/scaler transforms
        pca = None
        if ens["pca_components"] is not None:
            pca = PCA(n_components=ens["pca_var"])
            pca.components_ = ens["pca_components"]
            pca.mean_ = ens["pca_mean"]
            pca.n_features_in_ = X_trval.shape[1]
            X_te_f = pca.transform(X_te)
            X_trval_f = pca.transform(X_trval)
        else:
            X_te_f = X_te
            X_trval_f = X_trval

        if ens["scaler_mean"] is not None:
            scaler = StandardScaler(with_mean=True, with_std=True)
            scaler.mean_ = ens["scaler_mean"]
            scaler.scale_ = ens["scaler_scale"]
            scaler.n_features_in_ = X_trval_f.shape[1]
            X_te_f = scaler.transform(X_te_f)
        else:
            scaler = None

        # average bag models
        bag_preds = [mdl.predict(X_te_f) for mdl in ens["models"]]
        preds_per_combo.append(np.mean(bag_preds, axis=0))
        # weight = inverse squared CV score (stabler than inverse)
        weights.append(1.0 / (ens["cv_score"]**2 + 1e-12))

    weights = np.array(weights, dtype=float)
    weights = weights / weights.sum()
    Y_hat_te = np.tensordot(weights, np.stack(preds_per_combo, axis=0), axes=(0,0))

    # ---------------- Metrics (TEST) ----------------
    v_rmse = vec_rmse(Y_te, Y_hat_te)
    v_mae  = vec_mae(Y_te, Y_hat_te)
    rmse_axis, mae_axis = per_axis_metrics(Y_te, Y_hat_te)
    rmse_xy, mae_xy = compute_xy_plane_metrics(Y_te, Y_hat_te)  # NEW

    print("\n[test] Position (xyz):")
    print(f"   Vector RMSE = {v_rmse:.6f} m   Vector MAE = {v_mae:.6f} m")
    print("  Per-axis (m):")
    for nm, r, m in zip(["act_tcp_x","act_tcp_y","act_tcp_z"], rmse_axis, mae_axis):
        print(f"  {nm:9s}  RMSE={r:.6f} m   MAE={m:.6f} m")
    print(f"  XY-plane   Vector RMSE = {rmse_xy:.6f} m")
    print(f"  XY-plane   Vector MAE  = {mae_xy:.6f} m")

    # ---------------- Plots: Pred vs True ----------------
    for i, nm in enumerate(["act_tcp_x","act_tcp_y","act_tcp_z"]):
        plt.figure()
        x = Y_te[:, i]; y = Y_hat_te[:, i]
        plt.scatter(x, y, s=10)
        lo, hi = float(min(x.min(), y.min())), float(max(x.max(), y.max()))
        plt.plot([lo,hi],[lo,hi],'k-',lw=1)
        plt.xlabel(f"True {nm} (m)"); plt.ylabel(f"Pred {nm} (m)")
        plt.title(f"Pred vs True — {nm}")
        plt.grid(True, ls="--", lw=0.5); plt.tight_layout()
        plt.savefig(os.path.join(args.outdir, f"pvstrue_{nm}.png"), dpi=160)

    # ---------------- Learning Curve ----------------
    if str(args.lc_points).lower() == "auto":
        n_points = 6 if len(X_trval) >= 200 else max(3, int(np.clip(len(X_trval)//100, 3, 6)))
    else:
        n_points = int(args.lc_points)

    sizes = np.linspace(0.15, 1.0, n_points)
    lc_rows = []
    pbar = tqdm(sizes, disable=args.no_progress, desc="Learning curve (train sizes)")
    best_fam, best_params, _ = top_combos[0]

    # Prepare a refit pipeline for the best combo
    def fit_best_on_subset(Xsub, Ysub):
        if best_fam in ("pls","krr"):
            pca_var = float(best_params.get("pca_var", 0.0))
        else:
            pca_var = 0.0
        pca, Xsub_f = apply_pca_fit(Xsub, pca_var)
        if best_fam == "ridge":
            name, scaler, mdl = build_ridge(best_params["alpha"])
            pca = None; Xsub_f = Xsub
        elif best_fam == "pls":
            max_nc = cap_pls_components(len(Xsub_f), Xsub_f.shape[1], n_targets=3)
            n_comp = min(int(best_params["n_components"]), max_nc)
            name, scaler, mdl = build_pls(n_comp)
        else:
            name, scaler, mdl = build_krr(best_params["alpha"], best_params["gamma"])

        if scaler is not None:
            scaler = scaler.fit(Xsub_f)
            Xsub_f = scaler.transform(Xsub_f)

        mdl.fit(Xsub_f, Ysub)
        # Return a closure that transforms & predicts
        def predict_fn(X):
            Z = apply_pca_transform(pca, X) if pca is not None else X
            if scaler is not None:
                Z = scaler.transform(Z)
            return mdl.predict(Z)
        return predict_fn

    for s in pbar:
        n = max(32, int(np.floor(len(X_trval)*float(s))))
        idx = np.random.default_rng(RANDOM_STATE).choice(len(X_trval), size=n, replace=False)
        pred_fn = fit_best_on_subset(X_trval[idx], Y_trval[idx])
        Y_hat = pred_fn(X_te)

        row = {
            "n_train": int(n),
            "vector_rmse_test_m": vec_rmse(Y_te, Y_hat),
            "vector_mae_test_m":  vec_mae(Y_te, Y_hat)
        }
        lc_rows.append(row)

    lc_df = pd.DataFrame(lc_rows)
    lc_csv = os.path.join(args.outdir, "learning_curve.csv")
    lc_df.to_csv(lc_csv, index=False)

    plt.figure()
    plt.plot(lc_df["n_train"].to_numpy(), lc_df["vector_rmse_test_m"].to_numpy(), "o-", label="Vector RMSE (test)")
    plt.plot(lc_df["n_train"].to_numpy(), lc_df["vector_mae_test_m"].to_numpy(),  "o-", label="Vector MAE (test)")
    plt.xlabel("Training set size"); plt.ylabel("Error (m)")
    plt.title("Learning curve (test set)")
    plt.grid(True, ls="--", lw=0.5); plt.legend(); plt.tight_layout()
    plt.savefig(os.path.join(args.outdir, "learning_curve.png"), dpi=160)

    # ---------------- Save artifacts ----------------
    meta = {
        "created_utc": datetime.utcnow().isoformat()+"Z",
        "csv_path": os.path.abspath(args.csv),
        "rows_final": int(len(df)),
        "test_size_groups": float(args.test_size),
        "grouping": args.grouping,
        "filters": {
            "contact_only": bool(args.contact_only),
            "min_Fz": None if args.min_Fz is None else float(args.min_Fz),
            "min_deit_pct": None if args.min_deit_pct is None else float(args.min_deit_pct),
            "winsor_pct": None if args.winsor_pct is None else float(args.winsor_pct),
            "per_session_norm": bool(args.per_session_norm),
        },
        "cv_folds": int(args.cv_folds),
        "model_choice": args.model,
        "stack_top_n": int(args.stack_top_n),
        "bag_models": int(args.bag_models),
        "top_combos": [
            {"family": fam, "params": params, "cv_score": float(cv)}
            for fam, params, cv in top_combos
        ],
        "test_metrics": {
            "vector_rmse_m": v_rmse,
            "vector_mae_m":  v_mae,
            "per_axis_rmse_m": rmse_axis.tolist(),
            "per_axis_mae_m":  mae_axis.tolist(),
            "vector_rmse_xy_m": rmse_xy,      # NEW
            "vector_mae_xy_m":  mae_xy        # NEW
        }
    }
    with open(os.path.join(args.outdir, "stage1_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # Save ensemble bundle (for Stage-2 use)
    dump(ensembles, os.path.join(args.outdir, "eit_stage1_xyz_ensemble.joblib"))
    print(f"\n[save] Artifacts written to: {os.path.abspath(args.outdir)}")
    return 0


if __name__ == "__main__":
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        raise SystemExit(main())