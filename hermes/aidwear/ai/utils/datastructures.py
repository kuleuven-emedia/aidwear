"""
Filename: hermes/revalexo/ai/utils/datastructures.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-19
Version: 1.0
Description: PyTorch circular buffer in shared pinned memory
    with non-blocking simultaneous access by HERMES and PyTorch inference 
    subprocess, for efficient and fast zero-copy DMA transfers to the GPU.
"""

from dataclasses import dataclass
from multiprocessing import Value
from multiprocessing.sharedctypes import Synchronized
from typing import Callable, Concatenate, List, ParamSpec, Sequence
import torch
from torch import Tensor
import torch.multiprocessing as mp
import numpy as np

P = ParamSpec("P")

@dataclass
class BufferSlice:
    tensor: Tensor
    window_start_s: np.ndarray
    window_end_s: np.ndarray


class SharedTensorCircularBuffer:
    """PyTorch circular zero-copy buffer in pinned shared memory, for non-blocking DMA transfers to the GPU.

    NOTE: It's the responsibility of the upstream module to overallocate space
    for the buffer to avoid contention, akin to ping-pong writing.

    NOTE: Currently assumes the NVIDIA Tegra SoC (Jetson Orin NX) with unified memory, where CPU and GPU
    can directly read/write the same physical memory.
    """

    def __init__(self, buf_len: int, num_features: Sequence[int], dtype_str: str = "float32", device: str = "cpu"):
        self.buf_len = buf_len
        self.buffer = torch.zeros(
            buf_len, *num_features,
            dtype=getattr(torch, dtype_str),
            pin_memory=True if device != "cpu" else False,
        ).share_memory_()
        self.toa_s = torch.zeros(buf_len, 1, dtype=torch.float64).share_memory_()

        self.metadata_lock = mp.Lock()
        self.is_reading: Synchronized[bool] = Value("b", False)
        self.is_writing: Synchronized[bool] = Value("b", False)
        self.write_head: Synchronized[int] = Value("i", False)  # Always points to the newest sample in the circular buffer
        self.read_head: Synchronized[int] = Value("i", False)
        self.read_tail: Synchronized[int] = Value("i", False)
        self.is_reading.value = False
        self.is_writing.value = False
        self.write_head.value = 0
        self.read_head.value = 0
        self.read_tail.value = 0

    def put(self, new_data: torch.Tensor, toa_s: torch.Tensor) -> None:
        """In-place update of the shared pinned circular buffer.

        Args:
            new_data (torch.Tensor): View over N newest rows of data in non-shared memory of newly arrived HERMES samples. 
            toa_s (torch.Tensor): View over the N corresponding times-of-arrival. 
        """

        num_rows = new_data.shape[0]

        # Check which ranges we are allowed to write to in a thread-safe way.
        with self.metadata_lock:
            self.is_writing.value = True
            write_tail = self.write_head.value
            write_head = (self.write_head.value + num_rows) % self.buf_len
            # In case reading is in progress, fill at most up to the reading tail, dropping oldest of newest samples that would overlap.
            if self.is_reading.value and self.is_write_overlap_read(
                self.read_tail.value, self.read_head.value, write_tail, write_head
            ):
                write_head = self.read_tail.value
                num_rows = (write_head - write_tail) % self.buf_len
                new_data = new_data[-num_rows:]
                toa_s = toa_s[-num_rows:]

        # Nothing to write (write range bumps into the tail of the read region, with no empty slots for new data).
        if num_rows == 0:
            with self.metadata_lock:
                self.is_writing.value = False
            return

        if write_tail < write_head:
            self.buffer[write_tail:write_head] = new_data
            self.toa_s[write_tail:write_head] = toa_s
        else:
            first_part = self.buf_len - write_tail
            self.buffer[write_tail:] = new_data[:first_part]
            self.buffer[:num_rows-first_part] = new_data[first_part:]
            self.toa_s[write_tail:] = toa_s[:first_part]
            self.toa_s[:num_rows-first_part] = toa_s[first_part:]

        # Update the datastructures metadata in a thread-safe way.
        with self.metadata_lock:
            self.is_writing.value = False
            self.write_head.value = write_head

    def reserve(self, num_samples: int) -> tuple[List[Tensor], float, float]:
        """Reserve the N newest samples for reading.

        Sets the reading flag to True to prevent the producer from overwriting
        this range until `release()` is called.

        Returns:
            tuple[List[Tensor], float, float]: The data slice (potentially non-contiguous), start time, end time.
        """
        with self.metadata_lock:
            read_head = self.write_head.value
            read_tail = (read_head - num_samples) % self.buf_len
            self.read_tail.value = read_tail
            self.read_head.value = read_head
            self.is_reading.value = True

        num_rows = (read_head - read_tail) % self.buf_len

        if num_rows == 0:
            return self.buffer[:0], 0.0, 0.0

        if read_tail < read_head:
            tensor_slices = [self.buffer[read_tail:read_head]]
        else:
            first_part = self.buffer[read_tail:]
            second_part = self.buffer[:read_head]
            tensor_slices = [first_part, second_part]

        window_start_s = float(self.toa_s[read_tail].item())
        window_end_s = float(self.toa_s[read_head - 1].item())

        return tensor_slices, window_start_s, window_end_s

    def release(self) -> None:
        """Release the read reservation, allowing producer to overwrite the range."""
        with self.metadata_lock:
            self.is_reading.value = False

    def get(
        self,
        num_samples: int,
        callback: Callable[Concatenate[Tensor, P], None],
        *args: P.args,
        **kwargs: P.kwargs,
    ) -> tuple[float, float]:

        with self.metadata_lock:
            read_head = self.write_head.value
            read_tail = (read_head - num_samples) % self.buf_len
            self.read_tail.value = read_tail  # This acts as range reservation by producer, complemented by `is_reading`.
            self.read_head.value = read_head
            self.is_reading.value = True

        num_rows = (read_head - read_tail) % self.buf_len
        if num_rows == 0:
            tensor_slice = self.buffer[:0]
            window_start_s = 0.0
            window_end_s = 0.0
        else:
            if read_tail < read_head:
                tensor_slice = self.buffer[read_tail:read_head]
                window_start_s = float(self.toa_s[read_tail].item())
                window_end_s = float(self.toa_s[read_head - 1].item())
            else:
                # Wrapped range, I don't like cat's
                first_part = self.buffer[read_tail:]
                second_part = self.buffer[:read_head]
                tensor_slice = torch.cat((first_part, second_part), dim=0)

                window_start_s = float(self.toa_s[read_tail].item())
                window_end_s = float(self.toa_s[read_head - 1].item())

        callback(tensor_slice, *args, **kwargs)

        with self.metadata_lock:
            self.is_reading.value = False

        return window_start_s, window_end_s

    def is_write_overlap_read(
        self,
        read_tail: int,
        read_head: int,
        write_tail: int,
        write_head: int,
    ) -> bool:
        """Predicate for write-during-read condition of overlapping memory ranges.

        Both, read and write, ranges are treated as half-open intervals [tail, head)
        on the circular buffer of length `self.buf_len`.
        Overlap means the selected write range extends past the currently being read
        elements of the read range.

        Args:
            read_tail (int): Index of the oldest sample in the current read range.
            read_head (int): Index of the newest sample in the current read range.
            write_tail (int): Index of the oldest sample in the new write range.
            write_head (int): Index of the newest sample in the new write range.

        Returns:
            bool: Whether the write range overlaps the read range.
        """

        def _intervals(start: int, end: int) -> list[tuple[int, int]]:
            if start == end:
                return []
            elif start < end:
                return [(start, end)]
            else:
                return [(start, self.buf_len), (0, end)]

        read_intervals = _intervals(read_tail, read_head)
        write_intervals = _intervals(write_tail, write_head)

        for rs, re in read_intervals:
            for ws, we in write_intervals:
                if ws < re and rs < we:
                    return True
        return False
