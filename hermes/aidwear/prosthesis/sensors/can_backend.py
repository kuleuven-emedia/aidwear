"""
Filename: hermes/revalexo/exo/sensors/can_backend.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-02
Version: 1.0
Description: Listeners and parsing logic for data received from the CAN bus
    connected devices (e.g. motors, power monitoring unit, etc.).
"""

from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
from collections import deque
import can
import copy

from ..can_control.motor_cubemars import parse_servo_message
from ..can_control.pmu_mateksys import parse_pmu_message
from ..utils.types import ServoMotorData, BatteryData, ServoMotorEnum


class CanBackend(can.Listener):
    def __init__(
        self,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        motor_type_mapping: dict[int, ServoMotorEnum],
        motor_latest_data: dict[int, deque[ServoMotorData | None]],
        motor_data_queue: "Queue[tuple[str, ServoMotorData]]",
        pmu_id: int,
        pmu_multipart_msg_time_threshold: float,
        pmu_data_queue: "Queue[tuple[str, BatteryData]]",
    ):
        super().__init__()
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._motor_type_mapping = motor_type_mapping
        self._motor_latest_data = motor_latest_data
        self._motor_data_queue = motor_data_queue
        self._pmu_id = pmu_id
        self._pmu_data_queue = pmu_data_queue
        self._pmu_multipart_msg_time_threshold = pmu_multipart_msg_time_threshold
        self._pmu_multipart_msg: bytes | None = None
        self._pmu_multipart_msg_time: float | None = None

    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id == self._pmu_id and msg.data[-1] & 0x80:
            self._pmu_multipart_msg = copy.deepcopy(msg.data)
            self._pmu_multipart_msg_time = msg.timestamp

        elif msg.arbitration_id == self._pmu_id and msg.data[-1] & 0x20:
            if (
                self._pmu_multipart_msg is None
                or msg.timestamp - self._pmu_multipart_msg_time > self._pmu_multipart_msg_time_threshold
            ):
                return

            packet = self._pmu_multipart_msg + msg.data
            pmu_data = parse_pmu_message(msg.timestamp, packet[:7] + packet[8:9])
            self._pmu_multipart_msg = None

            if (
                self._is_keep_data_event.is_set()
                and not self._is_stop_new_data_event.is_set()
            ):
                self._pmu_data_queue.put(pmu_data)

        # TODO: Add support for other CAN devices (e.g. IMU, etc.), accounting for Arbitration IDs.
        #   For now, we only support the Cubemars servo motors, which have arbitration IDs in the range of 10496-10751 (inclusive).
        elif msg.arbitration_id & 0xFFFFFF00 == 0x00002900:
            src_id = msg.arbitration_id & 0x00000FF
            motor_type = self._motor_type_mapping[src_id]
            motor_data = parse_servo_message(msg.timestamp, bytes(msg.data), motor_type)
            self._motor_latest_data[src_id].append(motor_data)

            if (
                self._is_keep_data_event.is_set()
                and not self._is_stop_new_data_event.is_set()
            ):
                self._motor_data_queue.put((src_id, motor_data))
