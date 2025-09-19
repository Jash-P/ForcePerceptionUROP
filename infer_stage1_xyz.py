# infer_stage1_xyz.py
import numpy as np

def _flatten_predictors(obj):
    """
    Recursively collect any objects that expose .predict(...)
    from dicts/lists/tuples/nested sklearn ensembles.
    """
    preds = []

    # direct predictor
    if hasattr(obj, "predict") and callable(getattr(obj, "predict")):
        preds.append(obj)

    # scikit-style attributes
    if hasattr(obj, "estimators_"):  # e.g., Bagging/Boosting/Stacking, etc.
        preds.extend(_flatten_predictors(obj.estimators_))
    if hasattr(obj, "models_"):
        preds.extend(_flatten_predictors(obj.models_))
    if hasattr(obj, "model_"):
        preds.extend(_flatten_predictors(obj.model_))
    if hasattr(obj, "base_estimator_"):
        preds.extend(_flatten_predictors(obj.base_estimator_))
    if hasattr(obj, "estimator"):    # some wrappers store a single estimator
        preds.extend(_flatten_predictors(obj.estimator))

    # containers
    if isinstance(obj, dict):
        for v in obj.values():
            preds.extend(_flatten_predictors(v))
    elif isinstance(obj, (list, tuple, set)):
        for v in obj:
            preds.extend(_flatten_predictors(v))

    # dedupe while preserving order
    uniq = []
    seen = set()
    for p in preds:
        if id(p) not in seen:
            uniq.append(p)
            seen.add(id(p))
    return uniq

def predict_xyz_delta_eit(delta_eit: np.ndarray, ensembles) -> np.ndarray:
    """
    delta_eit: np.ndarray [N, D] (e.g., [1,256])
    ensembles: joblib-loaded container (dict/list/tuple/sklearn object/single model)
    returns:   np.ndarray [N, 3]
    """
    # ensure 2D
    delta_eit = np.asarray(delta_eit, dtype=np.float32)
    if delta_eit.ndim == 1:
        delta_eit = delta_eit[None, :]

    predictors = _flatten_predictors(ensembles)

    if not predictors:
        # Helpful debug to understand unexpected containers
        desc = f"type={type(ensembles)}"
        if isinstance(ensembles, dict):
            desc += f", keys={list(ensembles.keys())[:10]}"
        raise TypeError(f"No predictors with .predict() found in ensembles ({desc}).")

    preds = []
    for m in predictors:
        p = m.predict(delta_eit)            # should be [N, 3]
        p = np.asarray(p, dtype=np.float32)
        if p.ndim == 1:
            p = p.reshape(-1, 3)
        if p.shape[-1] != 3:
            raise ValueError(f"Model predicted shape {p.shape}; expected last dim=3.")
        preds.append(p)

    if len(preds) == 1:
        return preds[0]
    return np.mean(np.stack(preds, axis=0), axis=0)
