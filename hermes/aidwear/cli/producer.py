"""
Filename: hermes/revalexo/cli/producer.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-02-20
Version: 1.0
Description: HERMES Node that wraps user keyboard input into
    manually controlled intent, fatigue, and event controller.
"""

from collections import defaultdict
from functools import partial
from queue import Queue
import queue
import numpy as np
from typing import Callable

from hermes.utils.time_utils import get_time
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_KILL, PORT_SYNC_HOST
from hermes.utils.types import LoggingSpec

from hermes.base.nodes.producer import Producer

from hermes.aidwear.prosthesis.utils.types import ModeEnum

from .data_container import CliDataContainer


class CliProducer(Producer):
    def __init__(
        self,
        node_id: str,
        host_ip: str,
        logging_spec: LoggingSpec,
        port_pub: str = PORT_BACKEND,
        port_sync: str = PORT_SYNC_HOST,
        port_killsig: str = PORT_KILL,
        **_,
    ):
        self._input_queue: Queue[tuple[float, str]] = _["input_queue"]
        self._intent_sequence_id = 0
        self._fatigue_sequence_id = 0
        self._event_sequence_id = 0
        self._is_pause = False

        def intent_callback(
            mode: ModeEnum, toa_s: float, process_time_s: float, sequence_id: int
        ) -> None:
            print(f"User selected transition to: {mode.value.text}", flush=True)
            self._publish(
                process_time_s=process_time_s,
                new_data={
                    "intent": {
                        "toa_s": np.array([[toa_s]], dtype=np.float64),
                        "mode": np.array([[mode.value.id]], dtype=np.uint8),
                        "sequence_id": np.array([[sequence_id]], dtype=np.uint32),
                    }
                },
            )

        def safety_stop_callback(
            toa_s: float, is_pause: bool, process_time_s: float, sequence_id: int
        ) -> None:
            print(f"Safety stop triggered", flush=True)
            self._publish(
                process_time_s=process_time_s,
                new_data={
                    "safety_stop": {
                        "toa_s": np.array([[toa_s]], dtype=np.float64),
                        "is_pause": np.array([[is_pause]], dtype=np.bool),
                        "sequence_id": np.array([[sequence_id]], dtype=np.uint32),
                    }
                },
            )

        self._intent_keyboard_mapper: defaultdict[
            str, Callable[[float, float, int], None]
        ] = defaultdict(lambda: (lambda toa_s, process_time_s, sequence_id: None))
        for mode in ModeEnum:
            self._intent_keyboard_mapper[str(mode.value.id)] = partial(
                intent_callback, mode
            )

        self._safety_stop_keyboard_mapper = safety_stop_callback

        data_out_spec = {}

        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            logging_spec=logging_spec,
            port_pub=port_pub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

    @classmethod
    def create_data_container(cls, stream_spec: dict) -> CliDataContainer:
        return CliDataContainer(**stream_spec)

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
                if user_input[0] == "s":
                    self._is_pause = not self._is_pause
                    self._safety_stop_keyboard_mapper(
                        toa_s, self._is_pause, process_time_s, self._event_sequence_id
                    )
                else:
                    self._intent_keyboard_mapper[user_input](
                        toa_s, process_time_s, self._intent_sequence_id
                    )
                    self._intent_sequence_id += 1
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
