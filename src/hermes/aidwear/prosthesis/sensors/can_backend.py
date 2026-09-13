"""
Filename: hermes/aidwear/prosthesis/sensors/can_backend.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-09-13
Version: 1.0
Description: Listeners and parsing logic for data received from the CAN bus
    connected devices (e.g. encoders, power monitoring unit, etc.).
"""

from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
from collections import deque
import can
import copy

from .spi_to_can_bridge import parse_message
from ..utils.types import EncoderId, EncoderData


class CanBackend(can.Listener):
    def __init__(
        self,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        encoder_latest_data: dict[EncoderId, deque[EncoderData]],
        encoder_data_queue: "Queue[tuple[str, EncoderData]]",
    ):
        super().__init__()
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._encoder_latest_data = encoder_latest_data
        self._encoder_data_queue = encoder_data_queue

    def on_message_received(self, msg: can.Message) -> None:
        # Add support for other CAN devices (e.g. PMU, etc.), accounting for Arbitration IDs.
        if msg.arbitration_id in [EncoderId.KNEE.value, EncoderId.ANKLE.value] and len(msg.data) >= 2:
            src_id = EncoderId(msg.arbitration_id)
            encoder_data = parse_message(msg.timestamp, bytes(msg.data))
            self._encoder_latest_data[src_id].append(encoder_data)

            if (
                self._is_keep_data_event.is_set()
                and not self._is_stop_new_data_event.is_set()
            ):
                self._encoder_data_queue.put((src_id, encoder_data))
