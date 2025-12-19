from multiprocessing import Process, Queue, Event, Value
from multiprocessing.sharedctypes import Synchronized

from hermes.utils.types import LoggingSpec
from hermes.utils.time_utils import get_time
from hermes.utils.zmq_utils import (
    PORT_BACKEND,
    PORT_FRONTEND,
    PORT_KILL,
    PORT_SYNC_HOST,
)
from hermes.base.nodes.pipeline import Pipeline

from .stream import CyberlegStream
from .fsm import CyberlegHandler
from .utils.types import ModeTransition, NiclaSample, PhaseTransition, ServoMotorState
from .utils.utilities import launch_handler


class CyberlegPipeline(Pipeline):
    @classmethod
    def _log_source_tag(cls) -> str:
        return "revalexo"

    def __init__(
        self,
        host_ip: str,
        logging_spec: LoggingSpec,
        niclas: dict,
        motors: dict,
        dt: float,
        stream_in_specs: list[dict],
        port_pub: str = PORT_BACKEND,
        port_sub: str = PORT_FRONTEND,
        port_sync: str = PORT_SYNC_HOST,
        port_killsig: str = PORT_KILL,
        **_
    ):
        nicla_mapping: dict[str, str] = niclas["device_mapping"]
        motor_mapping: dict[str, dict] = motors["device_mapping"]
        service_uuid: str = niclas["service_uuid"]
        char_uuid: str = niclas["char_uuid"]

        self._input_queue: Queue[tuple[float, str]] = _["input_queue"]

        # Onboard data.
        self._mode_changed_queue: Queue[ModeTransition] = Queue()
        self._phase_changed_queue: Queue[PhaseTransition] = Queue()
        # TODO: update the used datastructure type to be friendlier to high writes.
        self._nicla_sample_queue: dict[str, Queue[tuple[float, NiclaSample]]] = {name: Queue() for name in nicla_mapping.keys()}
        self._motor_state_queue: dict[int, Queue[tuple[float, ServoMotorState]]] = {motor_spec["can_id"]: Queue() for motor_spec in motor_mapping.values()}

        # Shared controls for exo handler.
        self._next_mode: Synchronized[int] = Value('i')
        self._next_fatigue: Synchronized[float] = Value('f')
        self._next_mode_sequence_id: Synchronized[int] = Value('i')
        self._next_fatigue_sequence_id: Synchronized[int] = Value('i')

        # Synchronization primitives between background exo handler and foreground HERMES procs.
        self._is_ready_event = Event()
        self._is_cleanup_event = Event()
        self._is_finished_event = Event()
        self._is_keep_data_event = Event()

        self._handler_proc = Process(
            target=launch_handler,
            args=(CyberlegHandler,),
            kwargs={
                "nicla_mapping": nicla_mapping,
                "service_uuid": service_uuid,
                "char_uuid": char_uuid,
                "motor_mapping": motor_mapping,
                "ref_time_s": logging_spec.ref_time_s,

                "is_ready_event": self._is_ready_event,
                "is_cleanup_event": self._is_cleanup_event,
                "is_finished_event": self._is_finished_event,
                "is_keep_data_event": self._is_keep_data_event,

                "input_queue": self._input_queue,

                "nicla_sample_queue": self._nicla_sample_queue,
                "motor_state_queue": self._motor_state_queue,
                "mode_changed_queue": self._mode_changed_queue,
                "phase_changed_queue": self._phase_changed_queue,

                "next_mode": self._next_mode,
                "next_fatigue": self._next_fatigue,
                "next_mode_sequence_id": self._next_mode_sequence_id,
                "next_fatigue_sequence_id": self._next_fatigue_sequence_id,

                "dt": dt,
            },
        )
        self._handler_proc.start()
        self._is_ready_event.wait()

        stream_out_spec = {
            "niclas": niclas,
            "motors": motors,
        }

        super().__init__(
            host_ip=host_ip,
            stream_out_spec=stream_out_spec,
            stream_in_specs=stream_in_specs,
            logging_spec=logging_spec,
            port_pub=port_pub,
            port_sub=port_sub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

    @classmethod
    def create_stream(cls, stream_spec: dict) -> CyberlegStream:
        return CyberlegStream(**stream_spec)

    def _on_sync_complete(self) -> None:
        super()._on_sync_complete()
        self._is_keep_data_event.set()

    def _process_data(self, topic: str, msg: dict) -> None:
        if self._is_continue_produce:
            process_time_s: float = get_time()
            if topic == "intent":
                # Passes to the top-level exo module the next state to choose internally when to switch to.
                # NOTE: AI component will provide `int` matching one of the ModeStateEnum values.
                self._next_mode.value = msg["data"]["intent"]["mode"]
                self._next_mode_sequence_id.value = msg["data"]["intent"]["sequence_id"]

            elif topic == "fatigue":
                # Passes to the top-level exo module the next fatigue percentage to choose internally to scale torques.
                # NOTE: AI component will provide `float` in range [0, 1].
                self._next_fatigue.value = msg["data"]["fatigue"]["level"]
                self._next_fatigue_sequence_id.value = msg["data"]["intent"]["sequence_id"]


            # TODO: pass internally generated data to the middleware.
            try:
                pass
                # tag: str = "%s.data" % self._log_source_tag()
                # data = {
                #     "toa_s": process_time_s,
                #     "mode": msg["data"]["intent"]["mode"],
                #     "sequence_id": msg["data"]["intent"]["sequence_id"],
                # }
                # self._publish(tag, process_time_s=process_time_s, data={"intent": data})
            except:
                pass
        else:
            self._send_end_packet()

    def _generate_data(self) -> None:
        pass

    def _stop_new_data(self):
        # TODO: trigger exo handler to stop adding data to the timestamp alignment buffer for the AI model to consumer.
        pass

    def _cleanup(self) -> None:
        self._is_cleanup_event.set()
        self._is_finished_event.wait()
        self._handler_proc.join()
        super()._cleanup()
