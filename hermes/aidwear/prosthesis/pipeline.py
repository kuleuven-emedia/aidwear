"""
Filename: hermes/revalexo/exo/pipeline.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-04
Version: 1.0
Description: HERMES Node that wraps en masse exoskeleton system
    into the HERMES framework to expose internal data to synchronized sensing
    and to integrate live AI from companion device into its control.
"""

from multiprocessing import Process, Queue, Event, Value
from multiprocessing.sharedctypes import Synchronized
import numpy as np

from hermes.base.nodes.pipeline import Pipeline
from hermes.utils.types import LoggingSpec
from hermes.utils.time_utils import get_time
from hermes.utils.mp_utils import launch_handler
from hermes.utils.zmq_utils import (
    PORT_BACKEND,
    PORT_FRONTEND,
    PORT_SYNC_HOST,
    PORT_KILL,
)

from .controller import ProsthesisHandler
from .data_container import ProsthesisDataContainer
from .utils.types import (
    CLASS_TO_MODE,
    BatteryData,
    ModeTransition,
    MotorCommand,
    StateTransition,
    PhaseEstimate,
    ServoMotorData,
)
from ..utils.types import (
    NiclaData,
    NiclaLocation,
    NiclaPayloadMode,
)


class ProsthesisPipeline(Pipeline):
    def __init__(
        self,
        node_id: str,
        host_ip: str,
        data_out_spec: dict,
        data_in_specs: list[dict],
        logging_spec: LoggingSpec,
        port_pub: str = PORT_BACKEND,
        port_sub: str = PORT_FRONTEND,
        port_sync: str = PORT_SYNC_HOST,
        port_killsig: str = PORT_KILL,
        **_,
    ):
        niclas: dict = data_out_spec["niclas"]
        motors: dict = data_out_spec["motors"]
        telemetry: dict = data_out_spec["telemetry"]
        dt: float = data_out_spec["dt"]

        self._motor_mapping: dict[str, dict] = motors["device_mapping"]
        # Filter out AI-only Niclas if running exo without the AI and validate input mapping.
        if not niclas["is_pelvis_and_feet"]:
            niclas["device_mapping"] = dict(
                filter(
                    lambda x: x[0] not in [
                        NiclaLocation.PELVIS.value,
                        NiclaLocation.FOOT_RIGHT.value,
                        NiclaLocation.FOOT_LEFT.value,
                    ],
                    niclas["device_mapping"].items()
                )
            )

        self._nicla_mapping: dict[str, dict] = niclas["device_mapping"]
        self._nicla_payload_mode = NiclaPayloadMode(
            is_acc=niclas["is_acc"],
            is_gyr=niclas["is_gyr"],
            is_mag=niclas["is_mag"],
            is_euler=niclas["is_euler"],
            is_quat=niclas["is_quat"],
            is_temp=niclas["is_temp"],
            is_baro=niclas["is_baro"],
            is_hum=niclas["is_hum"],
        )

        # Keyboard stdin input queue.
        self._input_queue: Queue[tuple[float, str]] = _["input_queue"]

        # Onboard data.
        self._mode_changed_queue: Queue[ModeTransition] = Queue()
        self._state_changed_queue: Queue[StateTransition] = Queue()
        self._phase_estimate_queue: Queue[PhaseEstimate] = Queue()
        self._motor_command_queue: Queue[MotorCommand] = Queue()
        self._nicla_data_queue: Queue[tuple[str, float, NiclaData]] = Queue()
        self._motor_data_queue: Queue[tuple[str, ServoMotorData]] = Queue()
        self._battery_data_queue: Queue[tuple[str, BatteryData]] = Queue()

        # Shared controls for exo handler.
        self._next_mode: Synchronized[int] = Value("i")
        self._next_fatigue: Synchronized[float] = Value("f")
        self._next_is_pause: Synchronized[bool] = Value("b")
        self._next_mode_sequence_id: Synchronized[int] = Value("i")
        self._next_fatigue_sequence_id: Synchronized[int] = Value("i")
        self._next_is_pause_sequence_id: Synchronized[int] = Value("i")

        # Synchronization primitives between background exo handler and foreground HERMES procs.
        self._is_ready_event = Event()
        self._is_keep_data_event = Event()
        self._is_stop_new_data_event = Event()
        self._is_cleanup_event = Event()
        self._is_finished_event = Event()

        # Outgoing onboard data.
        telemetry_kwargs = {
            "nicla_data_queue": self._nicla_data_queue,
            "motor_data_queue": self._motor_data_queue,
            "battery_data_queue": self._battery_data_queue,
            "mode_changed_queue": self._mode_changed_queue,
            "state_changed_queue": self._state_changed_queue,
            "phase_estimate_queue": self._phase_estimate_queue,
            "motor_command_queue": self._motor_command_queue,
        }

        # Incoming AI controls.
        ai_kwargs = {
            "next_mode": self._next_mode,
            "next_fatigue": self._next_fatigue,
            "next_is_pause": self._next_is_pause,
            "next_mode_sequence_id": self._next_mode_sequence_id,
            "next_fatigue_sequence_id": self._next_fatigue_sequence_id,
            "next_is_pause_sequence_id": self._next_is_pause_sequence_id,
        }

        hermes_kwargs = {
            "ref_time_s": logging_spec.ref_time_s,
            "is_ready_event": self._is_ready_event,
            "is_keep_data_event": self._is_keep_data_event,
            "is_stop_new_data_event": self._is_stop_new_data_event,
            "is_cleanup_event": self._is_cleanup_event,
            "is_finished_event": self._is_finished_event,
            "input_queue": self._input_queue,
        }

        self._handler_proc = Process(
            target=launch_handler,
            args=(ProsthesisHandler,),
            kwargs={
                "niclas": niclas,
                "motors": motors,
                **telemetry_kwargs,
                **ai_kwargs,
                **hermes_kwargs,
                "dt": dt,
            },
        )
        self._handler_proc.start()
        self._is_ready_event.wait()

        data_out_spec = {
            "niclas": niclas,
            "motors": motors,
            "telemetry": telemetry,
        }

        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            data_in_specs=data_in_specs,
            logging_spec=logging_spec,
            is_async_generate=True,
            port_pub=port_pub,
            port_sub=port_sub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

    @classmethod
    def create_data_container(cls, stream_spec: dict) -> ProsthesisDataContainer:
        return ProsthesisDataContainer(**stream_spec)

    def _keep_samples(self) -> None:
        self._is_keep_data_event.set()

    def _process_data(self, topic: str, msg: dict) -> None:
        if topic in ["cli_control", "gui_control"]:
            if "intent" in msg:
                # Passes to the top-level exo module the next state to choose internally when to switch to.
                # NOTE: AI component will provide `int` matching one of the ModeEnum values.
                self._next_mode.value = msg["intent"]["mode"].item()
                self._next_mode_sequence_id.value = msg["intent"]["sequence_id"].item()
            elif "fatigue" in msg:
                # Passes to the top-level exo module the next fatigue percentage to choose internally to scale torques.
                # NOTE: AI component will provide `float` in range [0, 100].
                self._next_fatigue.value = msg["fatigue"]["level"].item()
                self._next_fatigue_sequence_id.value = msg["fatigue"]["sequence_id"].item()
            elif "safety_stop" in msg:
                self._next_is_pause.value = msg["safety_stop"]["is_pause"].item()
                self._next_is_pause_sequence_id.value = msg["safety_stop"]["sequence_id"].item()
        elif topic == "ai_intent":
            prediction: int = msg["classifier"]["predictions"][0].item()
            self._next_mode.value = CLASS_TO_MODE[prediction].value.id
            self._next_mode_sequence_id.value = msg["classifier"]["sequence_id"].item()

    def _generate_data(self) -> None:
        # Motor data.
        motor_data: dict[str, tuple[str, list[ServoMotorData]]] = {
            motor_spec["can_id"]: (motor_name, [])
            for motor_name, motor_spec in self._motor_mapping.items()
        }
        while not self._motor_data_queue.empty():
            can_id, motor_sample = self._motor_data_queue.get_nowait()
            motor_data[can_id][1].append(motor_sample)
        for motor_name, data in motor_data.values():
            if data:
                output = {
                    f"motor_{motor_name}": {
                        "toa_s": np.array(
                            [list(map(lambda m: m.timestamp, data))], dtype=np.float64
                        ).transpose((1, 0)),
                        "position": np.array(
                            [list(map(lambda m: m.position, data))], dtype=np.float32
                        ).transpose((1, 0)),
                        "velocity": np.array(
                            [list(map(lambda m: m.velocity, data))], dtype=np.float32
                        ).transpose((1, 0)),
                        "current": np.array(
                            [list(map(lambda m: m.current, data))], dtype=np.float32
                        ).transpose((1, 0)),
                        "temperature": np.array(
                            [list(map(lambda m: m.temperature, data))], dtype=np.int8
                        ).transpose((1, 0)),
                        "error": np.array(
                            [list(map(lambda m: m.error, data))], dtype=np.uint8
                        ).transpose((1, 0)),
                    }
                }
                self._publish(process_time_s=get_time(), new_data=output)

        # Battery data.
        battery_data: list[BatteryData] = []
        while not self._battery_data_queue.empty():
            battery_data.append(self._battery_data_queue.get_nowait())
        if battery_data:
            output = {
                "power_monitor": {
                    "toa_s": np.array(
                        [list(map(lambda m: m.timestamp, battery_data))],
                        dtype=np.float64,
                    ).transpose((1, 0)),
                    "temperature": np.array(
                        [list(map(lambda m: m.temperature, battery_data))],
                        dtype=np.float32,
                    ).transpose((1, 0)),
                    "voltage": np.array(
                        [list(map(lambda m: m.voltage, battery_data))],
                        dtype=np.float32,
                    ).transpose((1, 0)),
                    "current": np.array(
                        [list(map(lambda m: m.current, battery_data))],
                        dtype=np.float32,
                    ).transpose((1, 0)),
                    "power": np.array(
                        [list(map(lambda m: m.power, battery_data))],
                        dtype=np.float32,
                    ).transpose((1, 0)),
                }
            }
            self._publish(process_time_s=get_time(), new_data=output)

        # Nicla data.
        nicla_data: dict[str, list[NiclaData]] = {
            name: [] for name in self._nicla_mapping.keys()
        }
        nicla_toa: dict[str, list[float]] = {
            name: [] for name in self._nicla_mapping.keys()
        }
        while not self._nicla_data_queue.empty():
            nicla_name, toa_s, nicla_sample = self._nicla_data_queue.get_nowait()
            nicla_toa[nicla_name].append(toa_s)
            nicla_data[nicla_name].append(nicla_sample)
        for nicla_name, data in nicla_data.items():
            if data:
                output = {
                    f"nicla_{nicla_name}": {
                        "toa_s": np.array(
                            [nicla_toa[nicla_name]], dtype=np.float64
                        ).transpose((1, 0)),
                        "sequence_id": np.array(
                            [list(map(lambda n: n.sequence_id, data))], dtype=np.uint32
                        ).transpose((1, 0)),
                        "timestamp": np.array(
                            [list(map(lambda n: n.timestamp, data))], dtype=np.uint32
                        ).transpose((1, 0)),
                    }
                }
                for (
                    data_name,
                    data_getter,
                ) in self._nicla_payload_mode.get_data_getters().items():
                    output[f"nicla_{nicla_name}"][data_name] = data_getter(data)
                self._publish(process_time_s=get_time(), new_data=output)

        # Motor command data.
        # motor_command_data: dict[str, tuple[str, list[MotorCommand]]] = {
        #     motor_spec["can_id"]: (motor_name, [])
        #     for motor_name, motor_spec in self._motor_mapping.items()
        # }
        # while not self._motor_command_queue.empty():
        #     motor_command = self._motor_command_queue.get_nowait()
        #     motor_command_data[motor_command.motor_id][1].append(motor_command)
        # for motor_name, data in motor_command_data.values():
        #     if data:
        #         output = {
        #             f"command_{motor_name}": {
        #                 "toa_s": np.array(
        #                     [list(map(lambda m: m.timestamp, data))], dtype=np.float64
        #                 ).transpose((1, 0)),
        #                 "data": np.array(
        #                     [list(map(lambda m: bytes(m.data), data))]
        #                 ).transpose((1, 0)),
        #                 "control_mode": np.array(
        #                     [list(map(lambda m: m.control_mode, data))], dtype=np.uint8
        #                 ).transpose((1, 0)),
        #             }
        #         }
        #         self._publish(process_time_s=get_time(), new_data=output)

        # Locomotion mode.
        mode_transitions: list[ModeTransition] = []
        while not self._mode_changed_queue.empty():
            mode_transitions.append(self._mode_changed_queue.get_nowait())
        if mode_transitions:
            output = {
                "mode": {
                    "toa_s": np.array(
                        [list(map(lambda m: m.timestamp, mode_transitions))],
                        dtype=np.float64,
                    ).transpose((1, 0)),
                    "mode": np.array(
                        [list(map(lambda m: m.mode, mode_transitions))], dtype=np.uint8
                    ).transpose((1, 0)),
                    "sequence_id": np.array(
                        [list(map(lambda m: m.sequence_id, mode_transitions))],
                        dtype=np.uint32,
                    ).transpose((1, 0)),
                }
            }
            self._publish(process_time_s=get_time(), new_data=output)

        # Mid-level state.
        state_transitions: list[StateTransition] = []
        while not self._state_changed_queue.empty():
            state_transitions.append(self._state_changed_queue.get_nowait())
        if state_transitions:
            output = {
                "state": {
                    "toa_s": np.array(
                        [list(map(lambda s: s.timestamp, state_transitions))],
                        dtype=np.float64,
                    ).transpose((1, 0)),
                    "state": np.array(
                        [list(map(lambda s: s.state, state_transitions))], dtype=np.uint8
                    ).transpose((1, 0)),
                }
            }
            self._publish(process_time_s=get_time(), new_data=output)

        # Gait cycle phase.
        phase_estimates: list[PhaseEstimate] = []
        while not self._phase_estimate_queue.empty():
            phase_estimates.append(self._phase_estimate_queue.get_nowait())
        if phase_estimates:
            output = {
                "phase": {
                    "toa_s": np.array(
                        [list(map(lambda p: p.timestamp, phase_estimates))],
                        dtype=np.float64,
                    ).transpose((1, 0)),
                    "phase": np.array(
                        [list(map(lambda p: p.phase, phase_estimates))], dtype=np.float32
                    ).transpose((1, 0)),
                }
            }
            self._publish(process_time_s=get_time(), new_data=output)

        if (
            self._is_finished_event.is_set()
            and self._motor_data_queue.empty()
            and self._nicla_data_queue.empty()
            and self._mode_changed_queue.empty()
            and self._state_changed_queue.empty()
            and self._phase_estimate_queue.empty()
        ):
            self._notify_no_more_data_out()

    def _stop_new_data(self):
        # Trigger exo handler to stop adding data to the timestamp alignment buffer for the AI model to consumer.
        self._is_cleanup_event.set()
        self._is_stop_new_data_event.set()

    def _cleanup(self) -> None:
        self._handler_proc.join()
        super()._cleanup()
