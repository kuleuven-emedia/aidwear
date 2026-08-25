"""
Filename: hermes/revalexo/ai_fatigue/utils/config.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-06-25
Version: 1.0
Description: YAML config loader for live fatigue inference.
"""

import json
import os
import numpy as np
import torch
import yaml

from .feature_extraction import all_feature_names
from .types import Config, FatigueModalityType, ModalityConfig, OutputMode, PFConfig


def _abspath(path: str | None, base_dir: str) -> str | None:
    if not path:
        return None
    return os.path.abspath(
        path if os.path.isabs(path) else os.path.join(base_dir, path)
    )


def _validate_scaler(mean: np.ndarray, scale: np.ndarray, feature_names=None):
    live = all_feature_names()
    if feature_names is not None and list(feature_names) != live:
        raise ValueError(
            "Scaler feature_names disagree with live feature_extraction.all_feature_names(); "
            "refusing a scaler whose column order does not match the runtime vector."
        )
    if mean.shape != scale.shape or mean.shape[0] != len(live):
        raise ValueError(
            f"Scaler shape mismatch: mean={mean.shape}, scale={scale.shape}, expected ({len(live)},)"
        )
    return mean.astype(np.float32), scale.astype(np.float32)


def _load_scaler(pf_ckpt_path: str | None):
    """Return (mean, scale) from the PF checkpoint bundle or a sibling scaler.npz."""
    if not pf_ckpt_path or not os.path.exists(pf_ckpt_path):
        print(
            f"Warning: PF checkpoint not found ({pf_ckpt_path}); running unscaled.",
            flush=True,
        )
        return None, None

    bundle = torch.load(pf_ckpt_path, map_location="cpu", weights_only=False)
    if (
        isinstance(bundle, dict)
        and "scaler_mean" in bundle
        and "scaler_scale" in bundle
    ):
        return _validate_scaler(
            np.asarray(bundle["scaler_mean"]),
            np.asarray(bundle["scaler_scale"]),
            bundle.get("feature_names"),
        )

    weights_dir = os.path.dirname(pf_ckpt_path)
    npz_path = os.path.join(weights_dir, "scaler.npz")
    if os.path.exists(npz_path):
        arr = np.load(npz_path)
        feature_names = None
        meta_path = os.path.join(weights_dir, "meta.json")
        if os.path.exists(meta_path):
            feature_names = json.loads(open(meta_path).read()).get("feature_names")
        return _validate_scaler(arr["mean"], arr["scale"], feature_names)

    print(
        "Warning: no scaler found (bundle or scaler.npz); running unscaled.", flush=True
    )
    return None, None


def load_config(config_path: str, device: torch.device) -> Config:
    """Load fatigue inference config from a YAML file."""
    config_path = os.path.abspath(config_path)
    config_dir = os.path.dirname(config_path)

    with open(config_path, "r") as f:
        raw: dict = yaml.safe_load(f)

    pf_ckpt = _abspath(raw.get("pf_checkpoint"), config_dir)
    sysid_ckpt = _abspath(raw.get("sysid_checkpoint"), config_dir)

    modalities: dict[FatigueModalityType, ModalityConfig] = {}
    for key, mod_raw in raw["modalities"].items():
        mt = FatigueModalityType(key)
        modalities[mt] = ModalityConfig(type=mt, **(mod_raw or {}))

    pf = PFConfig(**raw["pf"])

    scaler_mean, scaler_scale = _load_scaler(pf_ckpt)
    kwargs = {}
    if "scaler_clip" in raw:
        kwargs["scaler_clip"] = raw["scaler_clip"]

    return Config(
        device=str(device),
        inference_period_s=float(raw["inference_period_s"]),
        modalities=modalities,
        pf=pf,
        pf_checkpoint_path=pf_ckpt,
        sysid_checkpoint_path=sysid_ckpt,
        sysid_epoch=int(raw["sysid_epoch"]),
        output_mode=OutputMode(raw["output_mode"]),
        scaler_mean=scaler_mean,
        scaler_scale=scaler_scale,
        **kwargs,
    )
