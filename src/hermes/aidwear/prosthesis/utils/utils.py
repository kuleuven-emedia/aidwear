"""
Filename: hermes/revalexo/exo/utils/utils.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2025-12-10
Version: 1.0
Description: Reusable decoupled utility functions to common uses.
"""

import os


def config_can_linux(channel: str = "can0") -> None:
    os.system("sudo ip link set can0 down")
    os.system(f"sudo ip link set {channel} type can bitrate 500000 dbitrate 500000 fd on loopback off restart-ms 1000 berr-reporting on")
    os.system("sudo ip link set can0 up")
    os.system(f"sudo ifconfig {channel} txqueuelen 65536")


def wrap_angle(x, y):
    return (x + y) % 360


def update_running_stats(
    count: int,
    mean: float,
    mean2: float,
    sample: float,
    min_sample: float,
    max_sample: float,
) -> tuple[int, float, float, float, float]:
    count += 1
    delta = sample - mean
    mean += delta / count
    delta2 = sample - mean
    mean2 += delta * delta2
    return count, mean, mean2, min(sample, min_sample), max(sample, max_sample)


def finalize_running_stats(
    count: int, mean: float, mean2: float
) -> tuple[int, float, float] | None:
    if count < 2:
        return None
    else:
        variance = mean2 / count
        sample_variance = mean2 / (count - 1)
        return mean, variance, sample_variance
