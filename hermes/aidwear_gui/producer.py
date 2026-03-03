import socket

from hermes.utils.time_utils import get_time
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_KILL, PORT_SYNC_HOST, IP_LOOPBACK
from hermes.utils.types import LoggingSpec

from hermes.base.nodes.producer import Producer

from ..aidwear.utils.types import ModeEnum
from .stream import PhoneGuiStream


class PhoneGuiProducer(Producer):
    @classmethod
    def _log_source_tag(cls) -> str:
        return "aidwear_gui"

    def __init__(
        self,
        host_ip: str,
        phone_ip: str,
        logging_spec: LoggingSpec,
        port_pub: str = PORT_BACKEND,
        port_sync: str = PORT_SYNC_HOST,
        port_killsig: str = PORT_KILL,
        **_,
    ):
        self._phone_ip = phone_ip
        stream_out_spec = {}

        super().__init__(
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
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(10)
        return True

    def _keep_samples(self) -> None:
        self._sock.bind((IP_LOOPBACK, int(self._phone_ip)))

    def _process_data(self) -> None:
        if self._is_continue_capture:
            try:
                payload, address = self._sock.recvfrom(
                    1024 # TODO: update with max packet size of what is sent from the phone app 
                )
            except socket.timeout:
                print(
                    "Moticon insoles receive socket timed out on receive.", flush=True
                )
                return

            process_time_s: float = get_time()

            # TODO: interpret the payload from the app and route to the prosthesis via the HERMES system
            #       e.g. convert selected activity and its sequence_id to a `ModeEnum` value.
            payload = [
                float(word) for word in payload.split()
            ]  # splits byte string into array of (multiple) bytes, removing whitespace separators between measurements

            print(f"User selected transition to: {mode.value.text}", flush=True)
            self._publish(
                "%s.data" % self._log_source_tag(),
                process_time_s=process_time_s,
                data={
                    "intent": {
                        "toa_s": process_time_s,
                        "mode": mode.value.id,
                        "sequence_id": sequence_id,
                    }
                },
            )
        else:
            self._send_end_packet()

    def _stop_new_data(self):
        pass

    def _cleanup(self) -> None:
        super()._cleanup()
