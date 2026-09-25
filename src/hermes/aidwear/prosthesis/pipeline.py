"""
Filename: hermes/aidwear/prosthesis/pipeline.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-04
Version: 1.0
Description: HERMES Node that wraps en masse prosthesis system
    into the HERMES framework to expose internal data to synchronized sensing
    and to integrate live AI from companion device into its control.
"""

from multiprocessing import Process, Queue, Event
import numpy as np
from collections import deque

from hermes.base.nodes.pipeline import Pipeline
from hermes.utils.types import LoggingSpec, NewData
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
    EncoderData,
    FatigueCommandSource,
    IntentCommandSource,
    ModeTransition,
    MotorCommand,
    NextFatigueSynchronized,
    NextIsPauseSynchronized,
    NextModeSynchronized,
    NextCalibrationSynchronized,
    StateTransition,
    PhaseEstimate,
    ServoMotorData,
    MotorId,
    EncoderId,
)
from hermes.nicla_sense_me.utils.types import (
    NiclaData,
    NiclaLocation,
    NiclaPayloadMode,
    CalibrationEvent,
    CalibrationEventType,
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
        niclas_spec: dict = data_out_spec["niclas"]
        motors_spec: dict = data_out_spec["motors"]
        telemetry_spec: dict = data_out_spec["telemetry"]
        dt: float = data_out_spec["dt"]
        base_assistance: float = data_out_spec["base_assistance"]
        is_immediate_mode_switch: bool = data_out_spec["is_immediate_mode_switch"]
        fsm_config_path: str = data_out_spec["fsm_config_path"]
        self._intent_majority_vote_buf = deque(
            maxlen=data_out_spec["num_majority_vote"]
        )

        self._motor_mapping: dict[str, dict] = motors_spec["device_mapping"]
        # Filter out AI-only Niclas if running exo without the AI and validate input mapping.
        if not niclas_spec["is_pelvis_and_feet"]:
            niclas_spec["device_mapping"] = dict(
                filter(
                    lambda x: (
                        x[0]
                        not in [
                            NiclaLocation.PELVIS.value,
                            NiclaLocation.FOOT_RIGHT.value,
                            NiclaLocation.FOOT_LEFT.value,
                        ]
                    ),
                    niclas_spec["device_mapping"].items(),
                )
            )

        self._nicla_mapping: dict[str, dict] = niclas_spec["device_mapping"]
        self._nicla_payload_mode = NiclaPayloadMode(
            is_acc=niclas_spec["is_acc"],
            is_gyr=niclas_spec["is_gyr"],
            is_mag=niclas_spec["is_mag"],
            is_euler=niclas_spec["is_euler"],
            is_quat=niclas_spec["is_quat"],
            is_temp=niclas_spec["is_temp"],
            is_baro=niclas_spec["is_baro"],
            is_hum=niclas_spec["is_hum"],
        )

        # Keyboard stdin input queue.
        self._input_queue: Queue[tuple[float, str]] = _["input_queue"]

        # Onboard data.
        self._mode_changed_queue: Queue[ModeTransition] = Queue()
        self._state_changed_queue: Queue[StateTransition] = Queue()
        self._phase_estimate_queue: Queue[PhaseEstimate] = Queue()
        self._motor_command_queue: Queue[MotorCommand] = Queue()
        self._nicla_data_queue: Queue[tuple[str, float, bytearray]] = Queue()
        self._encoder_data_queue: Queue[tuple[EncoderId, EncoderData]] = Queue()
        self._motor_data_queue: Queue[tuple[MotorId, ServoMotorData]] = Queue()
        self._calibration_event_queue: Queue[CalibrationEvent] = Queue()

        # Shared controls for exo handler.
        self._next_mode = NextModeSynchronized()
        self._next_fatigue = NextFatigueSynchronized()
        self._next_is_pause = NextIsPauseSynchronized()
        self._next_calibration_event = NextCalibrationSynchronized()

        # Synchronization primitives between background exo handler and foreground HERMES procs.
        self._is_ready_event = Event()
        self._is_keep_data_event = Event()
        self._is_stop_new_data_event = Event()
        self._is_exo_cleanup_event = Event()
        self._is_finished_event = Event()

        # Outgoing onboard data.
        telemetry_kwargs = {
            "nicla_data_queue": self._nicla_data_queue,
            "motor_data_queue": self._motor_data_queue,
            "encoder_data_queue": self._encoder_data_queue,
            "mode_changed_queue": self._mode_changed_queue,
            "state_changed_queue": self._state_changed_queue,
            "phase_estimate_queue": self._phase_estimate_queue,
            "motor_command_queue": self._motor_command_queue,
            "calibration_event_queue": self._calibration_event_queue,
        }

        # Incoming AI controls.
        ai_kwargs = {
            "next_mode_synchronized": self._next_mode,
            "next_fatigue_synchronized": self._next_fatigue,
            "next_is_pause_synchronized": self._next_is_pause,
            "next_calibration_event_synchronized": self._next_calibration_event,
        }

        hermes_kwargs = {
            "ref_time_s": logging_spec.ref_time_s,
            "is_ready_event": self._is_ready_event,
            "is_keep_data_event": self._is_keep_data_event,
            "is_stop_new_data_event": self._is_stop_new_data_event,
            "is_exo_cleanup_event": self._is_exo_cleanup_event,
            "is_finished_event": self._is_finished_event,
            "input_queue": self._input_queue,
        }

        self._handler_proc = Process(
            target=launch_handler,
            args=(ProsthesisHandler,),
            kwargs={
                "niclas": niclas_spec,
                "motors": motors_spec,
                "fsm_config_path": fsm_config_path,
                "output_dir": logging_spec.log_dir,
                **telemetry_kwargs,
                **ai_kwargs,
                **hermes_kwargs,
                "base_assistance": base_assistance,
                "is_immediate_mode_switch": is_immediate_mode_switch,
                "dt": dt,
            },
        )
        self._handler_proc.start()
        self._is_ready_event.wait()

        data_out_spec = {
            "niclas": niclas_spec,
            "motors": motors_spec,
            "telemetry": telemetry_spec,
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
    def create_data_container(cls, data_spec: dict) -> ProsthesisDataContainer:
        return ProsthesisDataContainer(**data_spec)

    def _keep_samples(self) -> None:
        self._is_keep_data_event.set()

    def _process_data(self, topic: str, msg: NewData) -> None:
        if topic in ["cli_control", "gui_control"]:
            if "intent" in msg:
                # Passes to the top-level exo module the next state to choose internally when to switch to.
                # NOTE: AI component will provide `int` matching one of the ModeEnum values.
                with self._next_mode.lock:
                    self._next_mode.next_value.value = msg["intent"]["mode"].item()
                    self._next_mode.sequence_id.value = msg["intent"][
                        "sequence_id"
                    ].item()
                    self._next_mode.source.value = (
                        IntentCommandSource.CLI.value
                        if topic == "cli_control"
                        else IntentCommandSource.GUI.value
                    )
            elif "fatigue" in msg:
                # Passes to the top-level exo module the next fatigue percentage to choose internally to scale torques.
                # NOTE: AI component will provide `float` in range [0, 100].
                with self._next_fatigue.lock:
                    self._next_fatigue.next_value.value = msg["fatigue"]["level"].item()
                    self._next_fatigue.sequence_id.value = msg["fatigue"][
                        "sequence_id"
                    ].item()
                    self._next_fatigue.source.value = (
                        FatigueCommandSource.CLI.value
                        if topic == "cli_control"
                        else FatigueCommandSource.GUI.value
                    )
            elif "safety_stop" in msg:
                with self._next_is_pause.lock:
                    self._next_is_pause.next_value.value = msg["safety_stop"][
                        "is_pause"
                    ].item()
                    self._next_is_pause.sequence_id.value = msg["safety_stop"][
                        "sequence_id"
                    ].item()
            elif "calibration_cmd" in msg:
                with self._next_calibration_event.lock:
                    self._next_calibration_event.next_value.value = msg["calibration_cmd"][
                        "type"
                    ].item()
                    self._next_is_pause.sequence_id.value = msg["calibration_cmd"][
                        "sequence_id"
                    ].item()
        elif topic == "ai_intent":
            # TODO: factor confidence into the majority voting.
            # Currently considers only the 0.0s forecasting horizon.
            prediction: int = msg["intent"]["predictions"][0, 0].item()
            self._intent_majority_vote_buf.appendleft(prediction)
            if all(map(lambda x: x == prediction, self._intent_majority_vote_buf)):
                # Convert AI high-level activity to the mid-level exo state machine used for the activity.
                with self._next_mode.lock:
                    self._next_mode.next_value.value = CLASS_TO_MODE[
                        prediction
                    ].value.id
                    self._next_mode.sequence_id.value = msg["intent"][
                        "sequence_id"
                    ].item()
                    self._next_mode.source.value = IntentCommandSource.AI.value
        elif topic == "ai_fatigue":
            with self._next_fatigue.lock:
                self._next_fatigue.next_value.value = msg["fatigue"]["rpe"][0].item()
                self._next_fatigue.sequence_id.value = msg["fatigue"][
                    "sequence_id"
                ].item()
                self._next_fatigue.source.value = FatigueCommandSource.AI.value

    def _generate_data(self) -> None:
        # Motor data.
        motor_data_dict: dict[MotorId, tuple[str, list[ServoMotorData]]] = {
            MotorId(motor_spec["can_id"]): (motor_name, [])
            for motor_name, motor_spec in self._motor_mapping.items()
        }
        while not self._motor_data_queue.empty():
            can_id, motor_sample = self._motor_data_queue.get_nowait()
            motor_data_dict[can_id][1].append(motor_sample)
        for motor_name, motor_data in motor_data_dict.values():
            if motor_data:
                output = {
                    f"motor_{motor_name}": {
                        "toa_s": np.array(
                            [list(map(lambda m: m.timestamp, motor_data))],
                            dtype=np.float64,
                        ).transpose((1, 0)),
                        "position": np.array(
                            [list(map(lambda m: m.position, motor_data))],
                            dtype=np.float32,
                        ).transpose((1, 0)),
                        "velocity": np.array(
                            [list(map(lambda m: m.velocity, motor_data))],
                            dtype=np.float32,
                        ).transpose((1, 0)),
                        "current": np.array(
                            [list(map(lambda m: m.current, motor_data))],
                            dtype=np.float32,
                        ).transpose((1, 0)),
                        "error": np.array(
                            [list(map(lambda m: m.error, motor_data))], dtype=np.uint8
                        ).transpose((1, 0)),
                    }
                }
                self._publish(process_time_s=get_time(), new_data=output)

        # Absolute encoder data.
        encoder_data_dict: dict[EncoderId, tuple[str, list[EncoderData]]] = {
            EncoderId[MotorId(motor_spec["can_id"]).name]: (motor_name, [])
            for motor_name, motor_spec in self._motor_mapping.items()
        }
        while not self._encoder_data_queue.empty():
            encoder_id, encoder_sample = self._encoder_data_queue.get_nowait()
            encoder_data_dict[encoder_id][1].append(encoder_sample)
        for motor_name, encoder_data in encoder_data_dict.values():
            if encoder_data:
                output = {
                    f"encoder_{motor_name}": {
                        "toa_s": np.array(
                            [list(map(lambda sample: sample.timestamp, encoder_data))],
                            dtype=np.float64,
                        ).transpose((1, 0)),
                        "angle": np.array(
                            [list(map(lambda sample: sample.angle, encoder_data))],
                            dtype=np.float32,
                        ).transpose((1, 0)),
                        "is_error": np.array(
                            [list(map(lambda sample: sample.is_error, encoder_data))],
                            dtype=np.bool,
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
            nicla_data[nicla_name].append(NiclaData.from_bytes(nicla_sample))
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
        # TODO: verify.
        # motor_command_data: dict[str, tuple[str, list[MotorCommand]]] = {
        #     motor_spec["can_id"]: (motor_name, [])
        #     for motor_name, motor_spec in self._motor_mapping.items()
        # }
        # while not self._motor_command_queue.empty():
        #     motor_command = self._motor_command_queue.get_nowait()
        #     cmd_id = (
        #         motor_command.motor_id.value
        #         if isinstance(motor_command.motor_id, MotorId)
        #         else motor_command.motor_id
        #     )
        #     if cmd_id in motor_command_data:
        #         motor_command_data[cmd_id][1].append(motor_command)
        #     elif str(cmd_id) in motor_command_data:
        #         motor_command_data[str(cmd_id)][1].append(motor_command)
        #     elif isinstance(cmd_id, str) and cmd_id.isdigit() and int(cmd_id) in motor_command_data:
        #         motor_command_data[int(cmd_id)][1].append(motor_command)
        # for motor_name, data in motor_command_data.values():
        #     if data:
        #         output = {
        #             f"command_{motor_name}": {
        #                 "toa_s": np.array(
        #                     [list(map(lambda m: m.timestamp, data))], dtype=np.float64
        #                 ).transpose((1, 0)),
        #                 "data": np.array(
        #                     [list(map(lambda m: m.command_data, data))], dtype="V8"
        #                 ).transpose((1, 0)),
        #                 "log_data": np.array(
        #                     [list(map(lambda m: m.log_data, data))], dtype="V20"
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
                    "source": np.array(
                        [list(map(lambda m: m.source, mode_transitions))],
                        dtype=np.uint8,
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
                        [list(map(lambda s: s.state, state_transitions))],
                        dtype=np.uint8,
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
                        [list(map(lambda p: p.phase, phase_estimates))],
                        dtype=np.float32,
                    ).transpose((1, 0)),
                }
            }
            self._publish(process_time_s=get_time(), new_data=output)

        if (
            self._is_finished_event.is_set()
            and self._motor_data_queue.empty()
            and self._encoder_data_queue.empty()
            and self._nicla_data_queue.empty()
            and self._mode_changed_queue.empty()
            and self._state_changed_queue.empty()
            and self._phase_estimate_queue.empty()
            # and self._motor_command_queue.empty()
        ):
            # Calibration event data.
            calibration_data: dict[CalibrationEventType, list[CalibrationEvent]] = {
                CalibrationEventType.NICLA: [],
                CalibrationEventType.ENCODER: [],
            }
            while not self._calibration_event_queue.empty():
                calibration_event = self._calibration_event_queue.get_nowait()
                calibration_data[calibration_event.sensor_type].append(
                    calibration_event
                )
            if calibration_data[CalibrationEventType.NICLA]:
                output = {
                    "nicla_calibration": {
                        "toa_s": np.array(
                            [
                                list(
                                    map(
                                        lambda c: c.timestamp,
                                        calibration_data[CalibrationEventType.NICLA],
                                    )
                                )
                            ],
                            dtype=np.float64,
                        ).transpose((1, 0)),
                        "offsets": np.array(
                            list(
                                map(
                                    lambda c: list(c.offsets.values()),
                                    calibration_data[CalibrationEventType.NICLA],
                                )
                            ),
                            dtype=np.float32,
                        ).transpose((1, 0)),
                    }
                }
                self._publish(process_time_s=get_time(), new_data=output)
            if calibration_data[CalibrationEventType.ENCODER]:
                output = {
                    "encoder_calibration": {
                        "toa_s": np.array(
                            [
                                list(
                                    map(
                                        lambda c: c.timestamp,
                                        calibration_data[CalibrationEventType.ENCODER],
                                    )
                                )
                            ],
                            dtype=np.float64,
                        ).transpose((1, 0)),
                        "offsets": np.array(
                            list(
                                map(
                                    lambda c: list(c.offsets.values()),
                                    calibration_data[CalibrationEventType.ENCODER],
                                )
                            ),
                            dtype=np.float32,
                        ).transpose((1, 0)),
                    }
                }
                self._publish(process_time_s=get_time(), new_data=output)

            self._notify_no_more_data_out()

    def _stop_new_data(self):
        # Trigger exo handler to stop adding data to the timestamp alignment buffer for the AI model to consumer.
        self._is_exo_cleanup_event.set()
        self._is_stop_new_data_event.set()

    def _cleanup(self) -> None:
        self._handler_proc.join()
        super()._cleanup()
