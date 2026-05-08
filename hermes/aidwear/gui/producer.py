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

from hermes.utils.time_utils import get_time
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_KILL, PORT_SYNC_HOST
from hermes.utils.types import LoggingSpec
from hermes.base.nodes.producer import Producer

from hermes.aidwear.prosthesis.utils.types import ModeEnum

from .utils.types import GuiCommandType
from .stream import PhoneGuiStream


class PhoneGuiProducer(Producer):
    def __init__(
        self,
        topic: str,
        host_ip: str,
        phone_ip: str,
        phone_port: int,
        logging_spec: LoggingSpec,
        port_pub: str = PORT_BACKEND,
        port_sync: str = PORT_SYNC_HOST,
        port_killsig: str = PORT_KILL,
        **_,
    ):
        self._phone_ip = phone_ip
        self._phone_port = phone_port
        self._int_to_mode: dict[int, ModeEnum] = {cmd.value.id: cmd for cmd in ModeEnum}
        stream_out_spec = {}

        super().__init__(
            topic=topic,
            host_ip=host_ip,
            stream_out_spec=stream_out_spec,
            logging_spec=logging_spec,
            port_pub=port_pub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

    @classmethod
    def create_stream(cls, stream_spec: dict) -> PhoneGuiStream:
        return PhoneGuiStream(**stream_spec)

    def _ping_device(self) -> None:
        return None

    def _connect(self) -> bool:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(10)
        self._sock.connect((self._phone_ip, self._phone_port))
        return True

    def _keep_samples(self) -> None:
        # Indicate to the smartphone app that we are ready.
        self._sock.send('GO'.encode('utf-8'))

    def _process_data(self) -> None:
        if self._is_continue_capture:
            try:
                payload = self._sock.recv(14)
            except socket.timeout:
                return

            process_time_s: float = get_time()

            # Interpret the payload from the app and route to the exo via the HERMES system
            # [0]: Command type (0: Intent, 1: Fatigue)
            # [1]: Command value (0-10 for fatigue, 0-2 for intent)
            # [2-5]: UINT32 sequence id of the command
            # [6-13]: 64-bit unsigned long long Android system timestamp
            command_type, command_value, sequence_id, android_timestamp = struct.unpack('>BBIQ', payload)
            gui_command = GuiCommandType(command_type)

            if gui_command == GuiCommandType.INTENT:
                mode = self._int_to_mode[command_value]

                print(f"User selected transition to: {mode.value.text}", flush=True)
                self._publish(
                    "%s.data" % self.topic,
                    process_time_s=process_time_s,
                    data={
                        "intent": {
                            "toa_s": android_timestamp / 1000.0,
                            "mode": mode.value.id,
                            "sequence_id": sequence_id,
                        }
                    },
                )
            elif gui_command == GuiCommandType.FATIGUE:
                fatigue: float = 10 * (
                    command_value
                    if 0 <= command_value <= 10
                    else (10 if command_value > 10 else 0)
                )

                print(f"User selected assistance lvl: {command_value}. Setting to {fatigue}%", flush=True)
                self._publish(
                    "%s.data" % self.topic,
                    process_time_s=process_time_s,
                    data={
                        "fatigue": {
                            "toa_s": android_timestamp / 1000.0,
                            "level": fatigue,
                            "sequence_id": sequence_id,
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
