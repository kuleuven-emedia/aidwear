import struct
import random
import time
import can
from multiprocessing.synchronize import Event as _Event


class CanEmulator:
    def __init__(
        self,
        motor_mapping: dict,
        is_stop_new_data_event: _Event,
        sampling_rate_hz: int = 1,
    ):
        self._motor_ids = list(map(lambda m: m["can_id"], motor_mapping.values()))
        self._is_stop_new_data_event = is_stop_new_data_event
        self._sample_period = 1 / sampling_rate_hz
        self._bus = can.interface.Bus(channel="localhost:18881", interface="virtualcan")

    def __call__(self, *args, **kwds):
        while not self._is_stop_new_data_event.is_set():
            for id in self._motor_ids:
                pos_int16 = random.randint(-3200, 3200)
                spd_int16 = random.randint(-32000, 32000)
                cur_int16 = random.randint(-60, 60)
                temp_int8 = random.randint(-20, 127)
                err_uint8 = random.randint(0, 7)

                data = struct.pack(
                    ">hhhbB", pos_int16, spd_int16, cur_int16, temp_int8, err_uint8
                )

                msg = can.Message(arbitration_id=id, data=data, is_extended_id=True)
                self._bus.send(msg)

            time.sleep(self._sample_period)
        print(f"CAN emulator exited.", flush=True)
