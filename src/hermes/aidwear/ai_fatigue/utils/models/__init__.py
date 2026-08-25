"""
Filename: hermes/revalexo/ai_fatigue/utils/models/__init__.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-06-25
Version: 1.0
Description: Model registry for live fatigue inference.

    - PFBase  : Projection-Fusion + 2-layer LSTM base.
    - SysId   : report-conditioned dense personalization adapter.
"""

from torch.nn import Module as _Module
from .pf_lstm import PFBase
from .sysid import SysIdUpdater, SysIdStreamer, rebuild as sysid_rebuild

MODEL_REGISTRY = {
    "PF_BASE": PFBase,
}


def get_model_class(name: str) -> type[_Module]:
    if name not in MODEL_REGISTRY:
        raise ValueError(
            f"Unknown fatigue model: {name}. Available: {list(MODEL_REGISTRY.keys())}"
        )
    return MODEL_REGISTRY[name]
