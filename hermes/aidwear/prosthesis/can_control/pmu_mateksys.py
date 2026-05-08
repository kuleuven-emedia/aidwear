"""
Filename: hermes/revalexo/exo/can_control/pmu_mateksys.py
Author: Elias Thiery <ethiery@vub.be>
Date: 2026-03-03
Version: 1.0
Description: CAN parsing primitives for MatekSys power monitoring unit messages.
"""

import struct

from ..utils.types import BatteryData


def parse_pmu_message(timestamp: float, data: bytes) -> BatteryData:
    """Parses the CAN frame message into a `BatteryData` object.

    Bytes from MatekSys power monitor arrive in little-endian order.
    [0-1]: CRC
    [2-3]: Temperature (float16)
    [4-5]: Voltage (float16)
    [7]: message type (?)
    [6 and 8]: Current (float16)

    Args:
        timestamp (float): Time since epoch when the CAN message was received, where possible, stamped in hardware.
        data (bytes): Decoded bytes from the CAN frame of a motor.

    Returns:
        A BatteryData object representing the battery's state.
    """
    _, temp_fp16, vol_fp16, cur_fp16 = struct.unpack("<heee", data)
    pow_fp32 = vol_fp16 * cur_fp16

    return BatteryData(
        timestamp=timestamp,
        temperature=temp_fp16,
        voltage=vol_fp16,
        current=cur_fp16,
        power=pow_fp32,
    )
