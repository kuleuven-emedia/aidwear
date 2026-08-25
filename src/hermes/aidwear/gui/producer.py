"""
Filename: hermes/revalexo/gui/producer.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-12
Version: 1.0
Description: HERMES Node that wraps Android GUI app interactive input
    of intent and fatigue into the HERMES fframework.
"""

import socket
import struct
from typing import Optional

from hermes.utils.time_utils import get_time
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_KILL, PORT_SYNC_HOST
from hermes.utils.types import LoggingSpec
from hermes.base.nodes.producer import Producer
import numpy as np

from src.hermes.aidwear.prosthesis.utils.types import ModeEnum

from .utils.types import GuiCommandType
from .data_container import PhoneGuiDataContainer


class PhoneGuiProducer(Producer):
    def __init__(
        self,
        topic: str,
        host_ip: str,
        phone_ip: str,
        phone_port: int,
        logging_spec: LoggingSpec,
        buf_len: Optional[int] = 1000,
        port_pub: Optional[str] = PORT_BACKEND,
        port_sync: Optional[str] = PORT_SYNC_HOST,
        port_killsig: Optional[str] = PORT_KILL,
        **_,
    ):
        self._phone_ip = phone_ip
        self._phone_port = phone_port
        self._int_to_mode: dict[int, ModeEnum] = {cmd.value.id: cmd for cmd in ModeEnum}

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
    def create_data_container(cls, data_spec: dict) -> PhoneGuiDataContainer:
        return PhoneGuiDataContainer(**data_spec)

    def _ping_device(self) -> None:
        return None

    def _connect(self) -> bool:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(10)
        self._sock.connect((self._phone_ip, self._phone_port))
        return True

    def _keep_samples(self) -> None:
        # Indicate to the smartphone app that we are ready.
        self._sock.send("GO".encode("utf-8"))

    def _process_data(self) -> None:
        if self._is_continue_capture:
            try:
                payload = self._sock.recv(14)
            except socket.timeout:
                return

            toa_s: float = get_time()

            # Interpret the payload from the app and route to the exo via the HERMES system
            # [0]: Command type (0: Intent, 1: Fatigue)
            # [1]: Command value (0-10 for fatigue, 0-2 for intent)
            # [2-5]: UINT32 sequence id of the command
            # [6-13]: 64-bit unsigned long long Android system timestamp
            command_type, command_value, sequence_id, android_timestamp = struct.unpack(
                ">BBIQ", payload
            )
            gui_command = GuiCommandType(command_type)

            if gui_command == GuiCommandType.INTENT:
                mode = self._int_to_mode[command_value]

                print(f"User selected transition to: {mode.value.text}", flush=True)
                self._publish(
                    "%s.data" % self.topic,
                    process_time_s=get_time(),
                    data={
                        "intent": {
                            "toa_s": np.array([[toa_s]], dtype=np.float64),
                            "timestamp": np.array([[android_timestamp / 1000.0]], dtype=np.float64),
                            "mode": np.array([[mode.value.id]], dtype=np.uint8),
                            "sequence_id": np.array([[sequence_id]], dtype=np.uint32),
                        }
                    },
                )
            elif gui_command == GuiCommandType.FATIGUE:
                fatigue: float = 10 * (
                    command_value
                    if 0 <= command_value <= 10
                    else (10 if command_value > 10 else 0)
                )

                print(
                    f"User selected assistance lvl: {command_value}. Setting to {fatigue}%",
                    flush=True,
                )
                self._publish(
                    "%s.data" % self.topic,
                    process_time_s=get_time(),
                    data={
                        "fatigue": {
                            "toa_s": np.array([[toa_s]], dtype=np.float64),
                            "timestamp": np.array([[android_timestamp / 1000.0]], dtype=np.float64),
                            "level": np.array([[fatigue]], dtype=np.float32),
                            "sequence_id": np.array([[sequence_id]], dtype=np.uint32),
                        }
                    },
                )
        else:
            self._send_end_packet()

    def _stop_new_data(self):
        pass

    def _cleanup(self) -> None:
        self._sock.close()
        super()._cleanup()
