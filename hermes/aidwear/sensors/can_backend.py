from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
from collections import deque
import can


from ..can_control.motor_epos import parse_servo_message
from ..can_control.pmu_mateksys import parse_monitor_message
from ..utils.types import ServoMotorData, BatteryData, ServoMotorEnum


class CanBackend(can.Listener):
    def __init__(
        self,
        motor_type_mapping: dict[int, ServoMotorEnum],
        motor_latest_data: dict[int, deque[ServoMotorData | None]],
        motor_data_queue: "Queue[tuple[str, ServoMotorData]]",
        battery_data_queue: "Queue[tuple[str, BatteryData]]",
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
    ):
        super().__init__()
        self._motor_type_mapping = motor_type_mapping
        self._motor_latest_data = motor_latest_data
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._motor_data_queue = motor_data_queue
        self._battery_data_queue = battery_data_queue

    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id & 0x18a1a900:
            # TODO: parse dronecan messages
            # print(bytes(msg.data), flush=True)
            # battery_state = parse_monitor_message(msg.timestamp, bytes(msg.data))

            # if (
            #     self._is_keep_data_event.is_set()
            #     and not self._is_stop_new_data_event.is_set()
            # ):
            #     self._battery_data_queue.put(battery_state)
            pass
        else:
            src_id = msg.arbitration_id & 0x00000FF
            motor_type = self._motor_type_mapping[src_id]
            motor_state = parse_servo_message(msg.timestamp, bytes(msg.data), motor_type)
            self._motor_latest_data[src_id].append(motor_state)

            if (
                self._is_keep_data_event.is_set()
                and not self._is_stop_new_data_event.is_set()
            ):
                self._motor_data_queue.put((src_id, motor_state))
