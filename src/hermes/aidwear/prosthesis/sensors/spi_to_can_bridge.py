"""
Filename: hermes/aidwear/prosthesis/can_control/spi_to_can_bridge.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-09-13
Version: 1.0
Description: CAN parsing primitives for the VUB's AS5048A SPI-to-CAN bridge messages.
"""

import struct

from ..utils.types import EncoderData


def parse_message(timestamp: float, data: bytes) -> EncoderData:
    """Parses the CAN frame message into a `EncoderData` object.

    Args:
        timestamp (float): Time since epoch when the CAN message was received, where possible, stamped in hardware.
        data (bytes): Decoded bytes from the CAN frame of an absolute encoder.

    Returns:
        An EncoderData object representing the encoder's state.
    """
    angle_raw = (data[0] << 8) | data[1]

    error = (angle_raw >> 14) & 0x01 # verification error flag (bit 14)

    angle_data = angle_raw & 0x3FFF  # 14 data bits
    angle_deg = angle_data * (360.0 / 16384.0) # Resolution of the 14 bit => 2^14 = 16384

    return EncoderData(
        timestamp=timestamp,
        angle=angle_deg,
        is_error=bool(error),
    )
