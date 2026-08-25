"""
Filename: hermes/notes/producer.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-05-16
Version: 1.0
Description: HERMES Node that wraps user keyboard input into
    manually controlled triggers for experiment realtime annotation.
"""

from queue import Queue
import queue
from typing import Optional

from hermes.utils.time_utils import get_time
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_KILL, PORT_SYNC_HOST
from hermes.utils.types import LoggingSpec

from hermes.base.nodes.producer import Producer
import numpy as np

from .data_container import NotesDataContainer


class NotesProducer(Producer):
    def __init__(
        self,
        topic: str,
        host_ip: str,
        logging_spec: LoggingSpec,
        buf_len: Optional[int] = 1000,
        port_pub: Optional[str] = PORT_BACKEND,
        port_sync: Optional[str] = PORT_SYNC_HOST,
        port_killsig: Optional[str] = PORT_KILL,
        **_,
    ):
        self._input_queue: Queue[tuple[float, str]] = _["input_queue"]
        self._event_sequence_id = 0

        def event_callback(
            toa_s: float, event_type: int, process_time_s: float, sequence_id: int
        ) -> None:
            self._publish(
                "%s.data" % self.topic,
                process_time_s=process_time_s,
                data={
                    "event": {
                        "toa_s": np.array([[toa_s]], dtype=np.float64),
                        "type": np.array([[event_type]], dtype=np.uint8),
                        "sequence_id": np.array([[sequence_id]], dtype=np.uint32),
                    }
                },
            )

        self._event_keyboard_mapper = event_callback

        data_out_spec = {
            "buf_len": buf_len,
        }

        super().__init__(
            topic=topic,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            logging_spec=logging_spec,
            port_pub=port_pub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

    @classmethod
    def create_data_container(cls, data_spec: dict) -> NotesDataContainer:
        return NotesDataContainer(**data_spec)

    def _ping_device(self) -> None:
        return None

    def _connect(self) -> bool:
        return True

    def _keep_samples(self) -> None:
        pass

    def _process_data(self) -> None:
        if self._is_continue_capture:
            try:
                toa_s, user_input = self._input_queue.get(timeout=5)
                process_time_s = get_time()
                if user_input[0] == "f":
                    try:
                        event_type = int(user_input[1:])
                        self._event_keyboard_mapper(
                            toa_s, event_type, process_time_s, self._event_sequence_id
                        )
                        self._event_sequence_id += 1
                    except ValueError:
                        print(
                            f"{user_input[1:]} is not a valid number for event annotation",
                            flush=True,
                        )
            except queue.Empty:
                pass
            except Exception as e:
                print(e, flush=True)
                pass
        else:
            self._send_end_packet()

    def _stop_new_data(self):
        pass

    def _cleanup(self) -> None:
        super()._cleanup()
