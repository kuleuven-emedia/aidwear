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

    windows, window_start_s, window_end_s = imu_buffer.reserve(num_samples)

    # From windows of contiguous [T', C] to contiguous [T, C] target memory.
    staging_tensor[: windows[0].shape[0]].copy_(windows[0], non_blocking=True)
    if len(windows) > 1:
        staging_tensor[windows[0].shape[0] :].copy_(windows[1], non_blocking=True)

    return BufferSlice(
        transforms(staging_tensor) if transforms else staging_tensor,
        np.array([window_start_s], dtype=np.float64),
        np.array([window_end_s], dtype=np.float64),
    ), [imu_buffer.release]


def preprocess_async_imu(
    staging_tensor: Tensor,
    imu_buffers: dict[NiclaLocation, SharedTensorCircularBuffer],
    num_samples: int,
    feature_mapping: dict[NiclaLocation, int],
    transforms: Optional[Compose],
) -> tuple[BufferSlice, list[Callable[[], None]]]:
    """Preprocess a window of individual pinned tensor slices of IMU data.

    In-place asynchronous copy of the latest per-IMU data directly into the correct contiguous memory.

    Args:
        staging_tensor (Tensor): Contiguous memory tensor on the target device for staging model input data.
        imu_buffers (dict[NiclaLocation, SharedTensorCircularBuffer]): View of the latest window of IMU data, per-device, in pinned shared memory.
        num_samples (int): Number of newest samples to preprocess.
        feature_mapping (dict[NiclaLocation, int]): Mapping between IMU identifier and the corresponding column in the Tensor.
        transforms (Compose, optional): A set of transformation functions to preprocess the data with. Defaults to `None`.

    Returns:
        tuple[BufferSlice, list[Callable]]: Original staging tensor with preprocessed IMU data on the CPU/GPU,
            and a list of release callbacks to unlock the shared buffers.
    """

    def _copy_logic(windows: List[Tensor], dev_id: int):
        start_acc = 3 * dev_id
        end_acc = 3 * (dev_id + 1)
        start_gyr = start_acc + 21
        end_gyr = end_acc + 21

        staging_tensor[: windows[0].shape[0], start_acc:end_acc].copy_(
            windows[0][:, :3], non_blocking=True
        )
        staging_tensor[: windows[0].shape[0], start_gyr:end_gyr].copy_(
            windows[0][:, 3:], non_blocking=True
        )
        if len(windows) > 1:
            staging_tensor[windows[0].shape[0] :, start_acc:end_acc].copy_(
                windows[1][:, :3], non_blocking=True
            )
            staging_tensor[windows[0].shape[0] :, start_gyr:end_gyr].copy_(
                windows[1][:, 3:], non_blocking=True
            )

    releasers = []
    windows_start: list[float] = []
    windows_end: list[float] = []
    for dev, buf in imu_buffers.items():
        windows, window_start_s, window_end_s = buf.reserve(num_samples)
        _copy_logic(windows, feature_mapping[dev])
        releasers.append(buf.release)
        windows_start.append(window_start_s)
        windows_end.append(window_end_s)

    return BufferSlice(
        transforms(staging_tensor) if transforms else staging_tensor,
        np.array(windows_start, dtype=np.float64),
        np.array(windows_end, dtype=np.float64),
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
    windows, window_start_s, window_end_s = frame_buffer.reserve(read_range)

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

    window, window_start_s, window_end_s = frame_buffer.reserve(1)
    # View over single contiguous [1, C, H, W] data in the pinned shared CPU memory.
    staging_tensor.copy_(window[0], non_blocking=True)

    return BufferSlice(
        transforms(staging_tensor) if transforms else staging_tensor,
        np.array([window_start_s], dtype=np.float64),
        np.array([window_end_s], dtype=np.float64),
    ), [frame_buffer.release]
