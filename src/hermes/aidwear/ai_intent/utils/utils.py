"""
Filename: hermes/aidwear/ai_intent/utils/utils.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-16
Version: 1.0
Description: Preprocessing functions piping data from pinned shared memory
    circular buffers into target CPU/GPU contiguous memory through transformations.
"""

from typing import Callable, List, Optional
import numpy as np
import torch
from torch import Tensor

from hermes.nicla_sense_me.utils.types import NiclaLocation

from .transforms import Compose
from .datastructures import BufferSlice, SharedTensorCircularBuffer


def preprocess_sync_imu(
    staging_tensor: Tensor,
    imu_buffer: SharedTensorCircularBuffer,
    num_samples: int,
    transforms: Optional[Compose],
) -> tuple[BufferSlice, list[Callable[[], None]]]:
    """Preprocess a window of pinned tensor slice of grouped IMU data (MVN).

    In-place asynchronous copy of the latest batch of IMU data directly into the correct contiguous memory.

    Args:
        staging_tensor (Tensor): Contiguous memory tensor on the target device for staging model input data.
        imu_buffer (SharedTensorCircularBuffer): View of the latest window of IMU data, per-device, in pinned shared memory.
        num_samples (int): Number of newest samples to preprocess.
        transforms (Compose, optional): A set of transformation functions to preprocess the data with. Defaults to `None`.

    Returns:
        tuple[BufferSlice, list[Callable]]: Original staging tensor with preprocessed IMU data on the CPU/GPU,
            and a list of release callbacks to unlock the shared buffer.
    """

    res: tuple[list[Tensor], float, float] = imu_buffer.reserve(num_samples)
    windows, window_start_s, window_end_s = res

    # From windows of contiguous [T', C] to contiguous [T, C] target memory.
    staging_tensor[: windows[0].shape[0]].copy_(windows[0], non_blocking=True)
    if len(windows) > 1:
        staging_tensor[windows[0].shape[0] :].copy_(windows[1], non_blocking=True)

    return BufferSlice(
        transforms(staging_tensor) if transforms else staging_tensor,
        np.array([window_start_s], dtype=np.float64),
        np.array([window_end_s], dtype=np.float64),
    ), [imu_buffer.release]


def interp1d(t_in: Tensor, y_in: Tensor, t_out: Tensor) -> Tensor:
    """1D piecewise-linear interpolation on PyTorch Tensors.

    Args:
        t_in (Tensor): 1D monotonically increasing input coordinates (M,).
        y_in (Tensor): 2D input values (M, C).
        t_out (Tensor): 1D query coordinates (K,).

    Returns:
        Tensor: Interpolated values at t_out (K, C).
    """
    if t_in.numel() == 0:
        return torch.zeros(
            t_out.shape[0], y_in.shape[-1], device=y_in.device, dtype=y_in.dtype
        )
    if t_in.numel() == 1:
        return y_in.repeat(t_out.shape[0], 1)

    t_out_clamped = t_out.clamp(min=t_in[0], max=t_in[-1])
    idx = torch.searchsorted(t_in, t_out_clamped, right=False)
    i1 = idx.clamp(min=1, max=len(t_in) - 1)
    i0 = i1 - 1
    t0 = t_in[i0]
    t1 = t_in[i1]
    denom = t1 - t0
    w = torch.where(denom > 0, (t_out_clamped - t0) / denom, torch.zeros_like(denom))
    return y_in[i0] + w.unsqueeze(-1) * (y_in[i1] - y_in[i0])


def resample_async_imu(
    raw_imu: Tensor,
    toa_s: Tensor,
    current_time_s: float,
    window_duration_s: float = 2.0,
    target_samples: int = 120,
    num_dev: int = 5,
) -> Tensor:
    """Resample asynchronous IMU data to uniform, time-aligned samples over a fixed duration window.

    Masks out samples older than `current_time_s - window_duration_s` per sensor, and interpolates
    the remaining valid samples to `target_samples` aligned to `[current_time_s - window_duration_s, current_time_s]`.

    Args:
        raw_imu (Tensor): Staged IMU data tensor of shape (T, C) or (B, T, C).
        toa_s (Tensor): Matching times of arrival tensor of shape (T, num_dev).
        current_time_s (float): Reference current timestamp (window end).
        window_duration_s (float, optional): Window length in seconds. Defaults to 2.0.
        target_samples (int, optional): Desired number of resampled time steps. Defaults to 120.
        num_dev (int, optional): Number of IMU devices. Defaults to 5.

    Returns:
        Tensor: Resampled and time-aligned IMU tensor.
    """
    orig_ndim = raw_imu.ndim
    if orig_ndim == 3:
        raw_imu_2d = raw_imu.squeeze(0)
    else:
        raw_imu_2d = raw_imu

    _, total_channels = raw_imu_2d.shape
    t_start = current_time_s - window_duration_s
    target_t = torch.linspace(
        t_start,
        current_time_s,
        target_samples,
        device=raw_imu.device,
        dtype=torch.float64,
    )
    interp_imu = torch.zeros(
        target_samples, total_channels, device=raw_imu.device, dtype=raw_imu.dtype
    )

    for dev_id in range(num_dev):
        if total_channels == 3 * num_dev:
            col_indices = list(range(3 * dev_id, 3 * (dev_id + 1)))
        elif total_channels == 6 * num_dev:
            col_indices = list(range(3 * dev_id, 3 * (dev_id + 1))) + list(
                range(3 * num_dev + 3 * dev_id, 3 * num_dev + 3 * (dev_id + 1))
            )
        else:
            cols_per_dev = total_channels // num_dev
            col_indices = list(
                range(cols_per_dev * dev_id, cols_per_dev * (dev_id + 1))
            )

        t_sensor = toa_s[:, dev_id]
        y_sensor = raw_imu_2d[:, col_indices]

        # Mask out elements that exceed the 2-s window
        valid_mask = (
            (t_sensor >= t_start)
            & (t_sensor > 0.0)
            & (t_sensor <= current_time_s + 0.1)
        )
        if valid_mask.any():
            t_valid = t_sensor[valid_mask]
            y_valid = y_sensor[valid_mask]
        else:
            non_zero = t_sensor > 0.0
            t_valid = t_sensor[non_zero] if non_zero.any() else t_sensor
            y_valid = y_sensor[non_zero] if non_zero.any() else y_sensor

        interp_sensor = interp1d(t_valid, y_valid, target_t)
        interp_imu[:, col_indices] = interp_sensor.to(raw_imu.dtype)

    if orig_ndim == 3:
        interp_imu = interp_imu.unsqueeze(0)
    return interp_imu


def preprocess_async_imu(
    staging_tensor: Tensor,
    imu_buffers: dict[NiclaLocation, SharedTensorCircularBuffer],
    num_samples: int,
    feature_mapping: dict[NiclaLocation, int],
    transforms: Optional[Compose],
    staging_toa_s: Optional[Tensor] = None,
) -> tuple[BufferSlice, list[Callable[[], None]]]:
    """Preprocess a window of individual pinned tensor slices of IMU data.

    In-place asynchronous copy of the latest per-IMU data directly into the correct contiguous memory.

    Args:
        staging_tensor (Tensor): Contiguous memory tensor on the target device for staging model input data.
        imu_buffers (dict[NiclaLocation, SharedTensorCircularBuffer]): View of the latest window of IMU data, per-device, in pinned shared memory.
        num_samples (int): Number of newest samples to preprocess.
        feature_mapping (dict[NiclaLocation, int]): Mapping between IMU identifier and the corresponding column in the Tensor.
        transforms (Compose, optional): A set of transformation functions to preprocess the data with. Defaults to `None`.
        staging_toa_s (Tensor, optional): Pre-allocated contiguous memory tensor for staging matching arrival timestamps. Defaults to `None`.

    Returns:
        tuple[BufferSlice, list[Callable]]: Original staging tensor with preprocessed IMU data and matching toa_s on CPU/GPU,
            and a list of release callbacks to unlock the shared buffers.
    """

    num_dev = len(feature_mapping)
    if staging_toa_s is None:
        staging_toa_s = torch.zeros(
            num_samples,
            num_dev,
            dtype=torch.float64,
            device=staging_tensor.device,
        )

    def _copy_logic(
        windows: List[Tensor],
        toa_windows: List[Tensor],
        dev_id: int,
        num_dev: int,
    ):
        w0_len = windows[0].shape[0]
        num_cols = windows[0].shape[1]

        if num_cols > 3:
            start_acc = 3 * dev_id
            end_acc = 3 * (dev_id + 1)
            start_gyr = start_acc + 3 * num_dev
            end_gyr = end_acc + 3 * num_dev

            staging_tensor[:w0_len, start_acc:end_acc].copy_(
                windows[0][:, :3], non_blocking=True
            )
            staging_tensor[:w0_len, start_gyr:end_gyr].copy_(
                windows[0][:, 3:], non_blocking=True
            )
            if len(windows) > 1:
                staging_tensor[w0_len:, start_acc:end_acc].copy_(
                    windows[1][:, :3], non_blocking=True
                )
                staging_tensor[w0_len:, start_gyr:end_gyr].copy_(
                    windows[1][:, 3:], non_blocking=True
                )
        else:
            start_col = num_cols * dev_id
            end_col = num_cols * (dev_id + 1)
            staging_tensor[:w0_len, start_col:end_col].copy_(
                windows[0], non_blocking=True
            )
            if len(windows) > 1:
                staging_tensor[w0_len:, start_col:end_col].copy_(
                    windows[1], non_blocking=True
                )

        # Copy matching times of arrival
        staging_toa_s[:w0_len, dev_id].copy_(
            toa_windows[0].squeeze(-1), non_blocking=True
        )
        if len(toa_windows) > 1:
            staging_toa_s[w0_len:, dev_id].copy_(
                toa_windows[1].squeeze(-1), non_blocking=True
            )

    releasers = []
    windows_start: list[float] = []
    windows_end: list[float] = []
    for dev, buf in imu_buffers.items():
        res: tuple[list[Tensor], list[Tensor], float, float] = buf.reserve(
            num_samples, return_toa=True
        )
        windows, toa_windows, window_start_s, window_end_s = res
        _copy_logic(windows, toa_windows, feature_mapping[dev], num_dev)
        releasers.append(buf.release)
        windows_start.append(window_start_s)
        windows_end.append(window_end_s)

    return BufferSlice(
        transforms(staging_tensor) if transforms else staging_tensor,
        np.array(windows_start, dtype=np.float64),
        np.array(windows_end, dtype=np.float64),
        toa_s=staging_toa_s,
    ), releasers


def preprocess_video(
    staging_tensor: Tensor,
    frame_buffer: SharedTensorCircularBuffer,
    num_frames: int,
    stride: int,
    transforms: Optional[Compose],
) -> tuple[BufferSlice, list[Callable[[], None]]]:
    """Preprocess N sampled frames from the pinned circular buffer.

    TODO: optimize by avoiding copy of all frames into contiguous staging tensor before transforms,
        and instead applying transforms on the non-contiguous pinned memory slices directly,
        then copying only the final preprocessed frames into the contiguous staging tensor.

    Args:
        staging_tensor (Tensor): Contiguous memory tensor on the target device for staging model input data.
        frame_buffer (SharedTensorCircularBuffer): Pinned shared circular buffer of video frames.
        num_frames (int): Number of frames to sample.
        stride (int): Stride between frames to sample by (i.e. every N-th frame).
        transforms (Compose, optional): A set of transformation functions to preprocess the data with. Defaults to `None`.

    Returns:
        tuple[BufferSlice, list[Callable]]: Original staging tensor with preprocessed video data, and release callback.
    """

    read_range = num_frames * stride
    res: tuple[list[Tensor], float, float] = frame_buffer.reserve(read_range)
    windows, window_start_s, window_end_s = res

    # Generator of frames from non-contiguous [T, C, H, W] data in the pinned shared CPU memory.
    frame_generator = map(
        lambda id: (
            windows[0][id]
            if id < windows[0].shape[0]
            else windows[1][id - windows[0].shape[0]]
        ),
        range(0, read_range, stride),
    )

    # Contiguous [C, T, H, W] target memory.
    for id, frame in enumerate(frame_generator):
        staging_tensor[:, id].copy_(frame, non_blocking=True)

    return BufferSlice(
        transforms(staging_tensor) if transforms else staging_tensor,
        np.array([window_start_s], dtype=np.float64),
        np.array([window_end_s], dtype=np.float64),
    ), [frame_buffer.release]


def preprocess_image(
    staging_tensor: Tensor,
    frame_buffer: SharedTensorCircularBuffer,
    transforms: Optional[Compose],
) -> tuple[BufferSlice, list[Callable[[], None]]]:
    """Preprocess the latest image from the pinned circular buffer.

    Args:
        staging_tensor (Tensor): Contiguous memory tensor on the target device for staging model input data.
        frame_buffer (SharedTensorCircularBuffer): Pinned shared circular buffer of video frames.
        transforms (Compose, optional): A set of transformation functions to preprocess the data with. Defaults to `None`.

    Returns:
        tuple[BufferSlice, list[Callable]]: Original staging tensor with preprocessed image data, and release callback.
    """

    res: tuple[list[Tensor], float, float] = frame_buffer.reserve(1)
    windows, window_start_s, window_end_s = res

    # View over single contiguous [1, C, H, W] data in the pinned shared CPU memory.
    staging_tensor.copy_(windows[0], non_blocking=True)

    return BufferSlice(
        transforms(staging_tensor) if transforms else staging_tensor,
        np.array([window_start_s], dtype=np.float64),
        np.array([window_end_s], dtype=np.float64),
    ), [frame_buffer.release]
