"""
Filename: hermes/revalexo/ai/utils/models/__init__.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-03-06
Version: 1.0
Description: Model registry for live inference.
"""

from torch.nn import Module as _Module
from .deepconvlstm import DeepConvLSTM
from .resnet_models import ResNet_Image, ResNet18_Image
from .video_models import X3D_Video
from .fusion import FusionModel
from .base_models import BaseEncoder, MultiHorizonClassifier

MODEL_REGISTRY = {
    "DeepConvLSTM": DeepConvLSTM,
    "ResNet_Image": ResNet_Image,
    "ResNet18_Image": ResNet18_Image,
    "X3D_Video": X3D_Video,
    "FusionModel": FusionModel,
}


def get_model_class(name: str) -> type[_Module]:
    """Get model class by name string."""
    if name not in MODEL_REGISTRY:
        raise ValueError(
            f"Unknown model: {name}. Available: {list(MODEL_REGISTRY.keys())}"
        )
    return MODEL_REGISTRY[name]
