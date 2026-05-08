"""
Filename: hermes/revalexo/ai/utils/types.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-11
Version: 1.0
Description: Multimodal AI specific data types.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional
import numpy as np


class ModalityType(Enum):
    UNKNOWN = "unknown"
    MULTIMODAL = "multimodal"
    RAW_IMU = "raw_imu"
    IMAGE = "image"
    VIDEO = "video"


class ModelType(Enum):
    RAW_IMU = ModalityType.RAW_IMU.value
    IMAGE = ModalityType.IMAGE.value
    VIDEO = ModalityType.VIDEO.value
    FUSION = "fusion"


@dataclass
class ModalityConfig:
    type: ModalityType
    receptive_field: Optional[int] = 120
    stride: Optional[int] = 1
    column_patterns: List[str] = field(default_factory=lambda: ["*acc_*", "*gyro_*"])
    eval_transforms: List = field(default_factory=[])


@dataclass
class InferenceResult:
    logits: np.ndarray
    predictions: np.ndarray
    start_time_s: float
    end_time_s: float
    window_start_s: np.ndarray
    window_end_s: np.ndarray
    counter: int


@dataclass
class ModelConfig:
    type: ModelType
    name: str
    params: Dict


@dataclass
class Config:
    prediction_horizons: List[float]
    num_classes: int
    device: str
    checkpoint_path: str
    label_mapping: Dict[str, int]
    predictions_bit_width: int
    modalities: Dict[ModalityType, ModalityConfig]
    models: Dict[ModelType, ModelConfig]
