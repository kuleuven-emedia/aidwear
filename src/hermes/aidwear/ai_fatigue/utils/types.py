"""
Filename: hermes/aidwear/ai_fatigue/utils/types.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-06-25
Version: 1.0
Description: Data types for live fatigue inference (PF base + SysID).
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple
import numpy as np


class FatigueModalityType(Enum):
    """Input modalities for the fatigue model (GSR is dropped: not a model input)."""

    UNKNOWN = "unknown"
    TMSI = "tmsi"  # from TMSi BIP-01
    IMU = "imu"  # from Xsens MVN pelvis tracker (acc xyz + gyro xyz)


class OutputMode(Enum):
    """Which estimate the pipeline emits as the primary RPE."""

    SYSID = "sysid"  # report-conditioned learned personalization
    BASE = "base"  # frozen PF base, no personalization


@dataclass
class ModalityConfig:
    type: FatigueModalityType
    receptive_field_s: float = 60.0  # window length in seconds


@dataclass
class PFConfig:
    """Frozen PF base architecture (built via the model registry by `name`)."""

    name: str = "PF_BASE"
    proj_dim: int = 8
    arch: Tuple[int, ...] = (64, 32)
    head_hidden: int = 32
    fusion_dropout: float = 0.3
    inter_cell_dropout: float = 0.3
    head_layernorm: bool = False
    head_dropout: float = 0.0


@dataclass
class FatigueResult:
    rpe: float  # primary predicted RPE (per output_mode), 0-10
    rpe_base: float  # frozen base prediction
    rpe_sysid: float  # SysID prediction
    features: np.ndarray  # raw 39-D feature vector (pre-scaler) for this window
    num_reports: int  # number of true RPE reports ingested so far
    start_time_s: float  # inference compute start
    end_time_s: float  # inference compute end
    window_start_s: np.ndarray  # oldest sample timestamp per modality
    window_end_s: np.ndarray  # newest sample timestamp per modality
    counter: int


@dataclass
class Config:
    device: str
    modalities: Dict[FatigueModalityType, ModalityConfig]
    pf: PFConfig
    inference_period_s: Optional[float] = 5.0  # how often to run inference
    pf_checkpoint_path: Optional[str] = None
    sysid_checkpoint_path: Optional[str] = None
    sysid_epoch: Optional[int] = 80
    output_mode: OutputMode = OutputMode.SYSID
    scaler_mean: Optional[np.ndarray] = None  # (39,) StandardScaler mean
    scaler_scale: Optional[np.ndarray] = None  # (39,) StandardScaler scale
    scaler_clip: Optional[float] = 10.0  # z-score clip
