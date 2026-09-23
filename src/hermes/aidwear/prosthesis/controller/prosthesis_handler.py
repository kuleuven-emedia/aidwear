"""
Filename: hermes/aidwear/prosthesis/controller/exo_handler.py
Description: "Operating system" of the prosthesis that (1) binds together
    interactions between controllers, sensors, and actuators, (2) manages
    the lifecycle and connectivity to on-board devices over specified
    communication interfaces, and (3) exposes connection to the HERMES
    framework for integrating exo into upstream multimodal sensing setup and
    upstream AI predictions into the downstream prosthesis.
"""

import asyncio
from multiprocessing import Process, Queue
from multiprocessing.synchronize import Event as _Event
from queue import Empty
from typing import Callable, Dict
import can
from collections import deque
from dataclasses import fields

from hermes.utils.time_utils import get_time, init_time
from hermes.utils.mp_utils import launch_handler

from .mode_selection import ModeSelectionMachine
from ..motor_control.epos_facade import EposFacade
from ..motor_control.types import (
    EposDeviceConfig,
    HomingConfig,
)
from ..utils.config_manager import ConfigManager
from ..sensors.can_backend import CanBackend

from ..utils.utils import (
    config_can_linux,
    finalize_running_stats,
    update_running_stats,
    wrap_angle,
)
from ..utils.types import (
    ProsthesisMotorMapping,
    EncoderData,
    ModeContext,
    MotorCommand,
    MotorId,
    EncoderId,
    ServoMotorEnum,
    ServoImpedanceGains,
    ServoMotorData,
    ModeEnum,
    ModeTransition,
    NextFatigueSynchronized,
    NextIsPauseSynchronized,
    NextModeSynchronized,
    StateTransition,
    PhaseEstimate,
    CalibrationEvent,
    CalibrationEventType,
    AbsoluteEncoderOffset,
    NiclaSamples,
)
from hermes.nicla_sense_me.utils.abstract_backend import NiclaBackend
from hermes.nicla_sense_me.utils.ble_backend import NiclaBleBackend
from hermes.nicla_sense_me.utils.types import (
    NiclaSampleSynchronized,
    NiclaOffsetsSynchronized,
    NiclaMappingFull,
    NiclaMappingNoPelvisAndFeet,
    NiclaConnectionType,
    NiclaData,
    calculate_nicla_sample_size,
)


class ProsthesisHandler:
    def __init__(
        self,
        niclas: dict,
        motors: dict,
        fsm_config_path: str,
        output_dir: str,
        nicla_data_queue: "Queue[tuple[str, float, bytearray]]",
        encoder_data_queue: "Queue[tuple[EncoderId, EncoderData]]",
        motor_data_queue: "Queue[tuple[MotorId, ServoMotorData]]",
        mode_changed_queue: "Queue[ModeTransition]",
        state_changed_queue: "Queue[StateTransition]",
        phase_estimate_queue: "Queue[PhaseEstimate]",
        motor_command_queue: "Queue[MotorCommand]",
        calibration_event_queue: "Queue[CalibrationEvent]",
        next_mode_synchronized: NextModeSynchronized,
        next_fatigue_synchronized: NextFatigueSynchronized,
        next_is_pause_synchronized: NextIsPauseSynchronized,
        ref_time_s: float,
        is_ready_event: _Event,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_exo_cleanup_event: _Event,
        is_finished_event: _Event,
        input_queue: "Queue[tuple[float, str]]",
        base_assistance: float = 0.0,
        is_immediate_mode_switch: bool = False,
        dt: float = 0.01,
    ):
        self._ref_time_s = ref_time_s
        self._dt = dt
        self._motor_dt = 1.0 / motors["sampling_rate_hz"]

        # ------------------------------------------------------------------------
        # Inter-process communication related variables
        # ------------------------------------------------------------------------
        self._input_queue = input_queue
        self._motor_data_queue = motor_data_queue
        self._motor_command_queue = motor_command_queue
        self._calibration_event_queue = calibration_event_queue
        self._nicla_backend_proc_input_queue: "Queue[tuple[float, str]]" = Queue()

        self._next_mode = next_mode_synchronized
        self._next_fatigue = next_fatigue_synchronized
        self._next_is_pause = next_is_pause_synchronized

        self._is_ready_event = is_ready_event
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_exo_cleanup_event = is_exo_cleanup_event
        self._is_finished_event = is_finished_event

        # ------------------------------------------------------------------------
        # Nicla Sense ME related variables
        # ------------------------------------------------------------------------
        nicla_connection_type = NiclaConnectionType[niclas["connection_type"]]
        self._gravity_scaling_factor = (
            niclas["gravity_scaling_factor"] * 9.80665 / 32768.0
        )
        self._gyroscope_scaling_factor = niclas["gyroscope_scaling_factor"] / 32768.0

        # Filter out AI-only Niclas if running exo without the AI and validate input mapping.
        nicla_mapping: Dict[str, dict] = niclas["device_mapping"]
        if niclas["is_pelvis_and_feet"]:
            self._nicla_name_mapping = NiclaMappingFull(
                **dict(zip(nicla_mapping.keys(), nicla_mapping.keys()))
            )
        else:
            self._nicla_name_mapping = NiclaMappingNoPelvisAndFeet(
                **dict(zip(nicla_mapping.keys(), nicla_mapping.keys()))
            )

        # Extract expected raw data size from Nicla config YAML spec (9-byte header + active modalities)
        nicla_sample_size: int = calculate_nicla_sample_size(niclas)

        # Datastructures for managing the incoming Nicla data.
        self._nicla_latest_data: Dict[str, NiclaSampleSynchronized] = {
            field.name: NiclaSampleSynchronized(nicla_sample_size)
            for field in fields(self._nicla_name_mapping)
        }
        self._nicla_offsets = NiclaOffsetsSynchronized(
            [field.name for field in fields(self._nicla_name_mapping)]
        )

        nicla_backend: type[NiclaBackend]
        if nicla_connection_type == NiclaConnectionType.BLE:
            nicla_backend = NiclaBleBackend
        else:
            raise NotImplementedError(
                f"{nicla_connection_type} is not implemented in prosthesis handler"
            )

        self._nicla_backend_proc = Process(
            target=launch_handler,
            args=(nicla_backend,),
            kwargs={
                "niclas": niclas,
                "nicla_latest_data": {
                    name: data.get_metadata()
                    for name, data in self._nicla_latest_data.items()
                },
                "nicla_data_queue": nicla_data_queue,
                "nicla_offsets": self._nicla_offsets,
                "calibration_event_queue": calibration_event_queue,
                "is_keep_data_event": is_keep_data_event,
                "is_stop_new_data_event": is_stop_new_data_event,
                "is_cleanup_event": is_exo_cleanup_event,
                "ref_time_s": self._ref_time_s,
                "input_queue": self._nicla_backend_proc_input_queue,
            },
        )
        self._nicla_backend_proc.start()

        # ------------------------------------------------------------------------
        # Motor related variables
        # ------------------------------------------------------------------------
        motor_mapping: Dict[str, dict] = motors["device_mapping"]
        self._motor_name_mapping = ProsthesisMotorMapping(
            **dict(zip(motor_mapping.keys(), motor_mapping.keys()))
        )  # validates input mapping.

        self._motor_latest_data: Dict[MotorId, deque[ServoMotorData]] = {
            MotorId(motor_spec["can_id"]): deque(maxlen=1)
            for motor_spec in motor_mapping.values()
        }

        self._encoder_latest_data: Dict[EncoderId, deque[EncoderData]] = {
            EncoderId[MotorId(motor_spec["can_id"]).name]: deque(maxlen=1)
            for motor_spec in motor_mapping.values()
        }
        self._encoder_offsets: Dict[EncoderId, AbsoluteEncoderOffset] = {
            EncoderId[MotorId(motor_spec["can_id"]).name]: AbsoluteEncoderOffset(
                motor_spec["absolute_encoder_reference"], 0.0
            )
            for motor_spec in motor_mapping.values()
        }
        self._encoder_offsets_lock = asyncio.Lock()

        config_can_linux(channel="can0")
        self._can_bus = can.interface.Bus(
            channel="can0", interface="socketcan", fd=True
        )

        # CAN bus multithreaded async listener.
        self._can_listener = CanBackend(
            is_keep_data_event=is_keep_data_event,
            is_stop_new_data_event=is_stop_new_data_event,
            encoder_latest_data=self._encoder_latest_data,
            encoder_data_queue=encoder_data_queue,
        )
        self._can_notifier = can.Notifier(
            bus=self._can_bus, listeners=[self._can_listener]
        )

        self._epos_homing_spec = {
            MotorId(motor_spec["can_id"]): HomingConfig(**motor_spec["homing"])
            for motor_spec in motor_mapping.values()
        }

        # TODO: move motor facade to a separate subprocess?
        epos_handle_config = EposDeviceConfig(**motors["epos"])
        self._epos = EposFacade(
            command_queue=motor_command_queue,
            config=epos_handle_config,
        )

        self._active_motors: list[MotorId] = [MotorId.ANKLE, MotorId.KNEE]
        for motor_id in self._active_motors:
            self._epos.connect(motor_id)
            self._epos.enable(motor_id)

        # Low-level motor controller gains.
        # TODO: use `watchdog` to live update gains parameters from a local text file for tunning motors response.
        K: Dict[str, ServoImpedanceGains] = {
            ServoMotorEnum.AK10_9.name: ServoImpedanceGains(
                position=1.5, velocity=0.08, acceleration=0
            ),
            ServoMotorEnum.AK80_8.name: ServoImpedanceGains(
                position=0.3, velocity=0.015, acceleration=0
            ),
        }

        # ------------------------------------------------------------------------
        # High-level locomotion mode selection FSM
        # ------------------------------------------------------------------------
        # State machine live updater from a config file.
        self._fsm_config_manager = ConfigManager(fsm_config_path, output_dir)

        ctx = ModeContext(
            epos=self._epos,
            K=K,
            next_mode=self._next_mode,
            next_fatigue=self._next_fatigue,
            mode_changed_queue=mode_changed_queue,
            state_changed_queue=state_changed_queue,
            phase_estimate_queue=phase_estimate_queue,
            motor_command_queue=motor_command_queue,
            is_stop_new_data_event=is_stop_new_data_event,
            is_keep_data_event=is_keep_data_event,
            config_manager=self._fsm_config_manager,
        )
        self._mode_fsm = ModeSelectionMachine(ctx, is_immediate_mode_switch)
        self._next_mode.next_value.value = self._mode_fsm.current_state.value
        self._is_paused = False
        self._is_motors_calibrated = False

    async def _calibrate_motors(self) -> bool:
        try:
            for motor_id in self._active_motors:
                self._epos.start_homing(
                    motor_id=motor_id,
                    config=self._epos_homing_spec[motor_id],
                )
            return await self._wait_for_homing_completion()
        except Exception as e:
            print(f"Warning: Failed to initiate motor homing: {e}", flush=True)
            return False

    async def _wait_for_homing_completion(
        self,
        timeout_s: float = 60.0,
        poll_interval_s: float = 0.05,
    ) -> bool:
        """Awaits homing completion for all active motors before proceeding."""
        print("Awaiting completion of motor homing routine...", flush=True)
        # Give motor drives a brief interval to transition into the homing sequence
        await asyncio.sleep(0.1)

        pending_motors = set(self._active_motors)
        t0 = get_time()

        while pending_motors:
            if (elapsed_s := get_time() - t0) > timeout_s:
                print(
                    f"Warning: Homing routine timed out after {elapsed_s:.1f}s for motors: "
                    f"{[m.name for m in pending_motors]}",
                    flush=True,
                )
                return False

            for motor_id in list(pending_motors):
                try:
                    attained, homing_err = self._epos.get_homing_state(motor_id)
                    if homing_err:
                        print(
                            f"Warning: EPOS reported Homing Error flag on {motor_id.name}!",
                            flush=True,
                        )
                        return False
                    if attained:
                        print(f"Homing attained on motor {motor_id.name}.", flush=True)
                        pending_motors.remove(motor_id)

                        encoder_id = EncoderId[motor_id.name]
                        async with self._encoder_offsets_lock:
                            while (
                                latest_enc := self._encoder_latest_data[encoder_id][-1]
                            ) is None:
                                await asyncio.sleep(0.01)
                            self._encoder_offsets[encoder_id].offset = latest_enc.angle
                            print(
                                f"Encoder offset recorded for {encoder_id.name}: {self._encoder_offsets[encoder_id].offset:.2f}",
                                flush=True,
                            )
                except Exception as e:
                    print(
                        f"Warning: Failed to query homing state for {motor_id.name}: {e}",
                        flush=True,
                    )
                    return False

            if pending_motors:
                await asyncio.sleep(poll_interval_s)

        print(
            "Motor homing procedure successfully completed for all active motors.",
            flush=True,
        )
        async with self._encoder_offsets_lock:
            encoder_offsets = {
                k.name: v.offset for k, v in self._encoder_offsets.items()
            }
        self._calibration_event_queue.put(
            CalibrationEvent(
                timestamp=get_time(),
                sensor_type=CalibrationEventType.ENCODER,
                offsets=encoder_offsets,
            )
        )
        return True

    async def _watch_for_offset_recalibration(self) -> None:
        loop = asyncio.get_event_loop()
        while not self._is_exo_cleanup_event.is_set():
            try:
                toa_s, user_input = await loop.run_in_executor(
                    None, self._input_queue.get, True, 0.1
                )
                if user_input == "M":
                    await self._calibrate_motors()
                elif user_input == "I":
                    self._nicla_backend_proc_input_queue.put((toa_s, user_input))
            except Empty:
                pass
            except Exception as e:
                print(f"Failed to connect: {e}", flush=True)
            finally:
                await asyncio.sleep(0.5)

    async def _recv_calibration_trigger(self, calibrate_fn: Callable) -> bool:
        loop = asyncio.get_event_loop()
        try:
            toa_s, user_input = await loop.run_in_executor(
                None, self._input_queue.get, True, 1.0
            )
            if user_input == "M":
                res = await calibrate_fn()
                self._is_motors_calibrated = True if res is None else bool(res)
                if not self._is_motors_calibrated:
                    print(
                        "Motors calibration procedure was not attained. Press 'M' to retry.",
                        flush=True,
                    )
            elif user_input == "I":
                self._nicla_backend_proc_input_queue.put((toa_s, user_input))
        except Empty:
            pass
        except Exception as e:
            print(f"Failed to connect: {e}", flush=True)

    async def _run_state_machine(self):
        nicla_euler_samples: Dict[str, float] = {
            field.name: None for field in fields(self._nicla_name_mapping)
        }
        nicla_gyro_samples: Dict[str, float] = {
            field.name: None for field in fields(self._nicla_name_mapping)
        }
        encoder_samples: Dict[MotorId, EncoderData] = {
            key: None for key in self._encoder_latest_data.keys()
        }
        motor_samples: Dict[MotorId, ServoMotorData] = {
            key: None for key in self._motor_latest_data.keys()
        }

        count = 0
        mean = 0.0
        mean2 = 0.0
        min_loop_time = 0.0
        max_loop_time = 0.0
        next_period_s = get_time()

        # Initial sensor data buffering.
        while any(len(dq) == 0 for dq in self._encoder_latest_data.values()) or any(
            len(dq) == 0 for dq in self._motor_latest_data.values()
        ):
            start_time_s = get_time()
            next_period_s += self._dt
            if (sleep_s := (next_period_s - start_time_s)) > 0:
                await asyncio.sleep(sleep_s)
            continue

        # Actual loop
        while not self._is_exo_cleanup_event.is_set():
            start_time_s = get_time()
            next_period_s += self._dt

            with self._nicla_offsets.lock:
                for device_name, device_data in self._nicla_latest_data.items():
                    nicla_euler_samples[device_name] = (
                        device_data.data.euler[0]
                        - self._nicla_offsets.offsets[device_name].value
                    )
                    nicla_gyro_samples[device_name] = device_data.data.gyroscope[0]

            async with self._encoder_offsets_lock:
                for encoder_id, encoder_data in self._encoder_latest_data.items():
                    sample = encoder_data[-1]
                    encoder_samples[encoder_id] = EncoderData(
                        timestamp=sample.timestamp,
                        angle=self._encoder_offsets[encoder_id].offset - sample.angle,
                        is_error=sample.is_error,
                    )

            for motor_id, motor_data in self._motor_latest_data.items():
                motor_samples[motor_id] = motor_data[-1]

            nicla_samples = NiclaSamples.from_measurements(
                nicla_euler_samples,
                nicla_gyro_samples,
                self._gyroscope_scaling_factor,
            )

            # If safe-stop was entered via researcher input, exo will be forced into `IDLE` mode
            with self._next_is_pause.lock:
                next_is_pause = self._next_is_pause.next_value.value

            if next_is_pause and not self._is_paused:
                # Stop any in-progress motor movements if safety stop triggered.
                for motor_id in self._active_motors:
                    self._epos.quick_stop(motor_id)
                self._is_paused = True
            elif not next_is_pause and self._is_paused:
                # Re-enable motors if safety is switched off.
                for motor_id in self._active_motors:
                    self._epos.enable(motor_id)
                self._is_paused = False
                self._mode_fsm.to_idle()
            elif not next_is_pause and not self._is_paused:
                # Check if the handler received an update of state from the parent Pipeline mode with the new AI prediction.
                # The condition is evaluated on each loop iteration to recognize asynchronously received intent prediction.
                #   Internal logic of the `ModeSelectionMachine` will judge on which iteration to permit transition using conditional FSM transitions.
                with self._next_mode.lock:
                    next_mode = self._next_mode.next_value.value
                    next_mode_sequence_id = self._next_mode.sequence_id.value
                    next_mode_source = self._next_mode.source.value

                if next_mode != self._mode_fsm.current_state.value:
                    # Store next mode's sequence ID to guarantee it doesn't change while switching states.
                    self._mode_fsm._sequence_id = next_mode_sequence_id
                    self._mode_fsm._source = next_mode_source

                    # Triggers potential transition to the next locomotion mode.
                    if next_mode == ModeEnum.WALKING.value.id:
                        self._mode_fsm.to_walking()
                    elif next_mode == ModeEnum.SIT_TO_STAND.value.id:
                        self._mode_fsm.to_sit_to_stand()
                    elif next_mode == ModeEnum.STAIR_ASCENT.value.id:
                        self._mode_fsm.to_stair_ascent()
                    elif next_mode == ModeEnum.STAIR_DESCENT.value.id:
                        self._mode_fsm.to_stair_descent()
                    elif next_mode == ModeEnum.IDLE.value.id:
                        self._mode_fsm.to_idle()
                    elif next_mode == ModeEnum.HURDLE.value.id:
                        self._mode_fsm.to_hurdle()

            self._mode_fsm.update_sensor_values(
                nicla_samples=nicla_samples,
                encoder_samples=encoder_samples,
                motor_samples=motor_samples,
                dt=self._dt,
            )

            if not self._is_paused:
                self._mode_fsm.step()

            end_time_s = get_time()
            if (sleep_s := next_period_s - end_time_s) > 0:
                await asyncio.sleep(sleep_s)
            count, mean, mean2, min_loop_time, max_loop_time = update_running_stats(
                count,
                mean,
                mean2,
                end_time_s - start_time_s,
                min_loop_time,
                max_loop_time,
            )

        # Transition back to `IDLE`.
        if self._mode_fsm.current_state.value != ModeEnum.IDLE.value.id:
            # Stop any in-progress motor movements.
            self._mode_fsm.to_idle()
            self._mode_fsm.step()

        # Finalize and print exo statistics.
        if (res := finalize_running_stats(count, mean, mean2)) is not None:
            print(
                f"Prosthesis {self._dt}s FSM loop timing: mean = {res[0]} | variance = {res[1]} | sample variance = {res[2]} | min loop time = {min_loop_time} | max loop time = {max_loop_time}",
                flush=True,
            )

    async def _poll_motor_data(self) -> None:
        """Continuously captures motor telemetry (position, velocity, current) at the desired sampling rate."""
        next_period_s = get_time()
        while not self._is_exo_cleanup_event.is_set():
            next_period_s += self._motor_dt

            for motor_id in self._active_motors:
                try:
                    motor_sample = self._epos.get_motor_data(motor_id)
                    self._motor_latest_data[motor_id].append(motor_sample)

                    if (
                        self._is_keep_data_event.is_set()
                        and not self._is_stop_new_data_event.is_set()
                    ):
                        self._motor_data_queue.put((motor_id, motor_sample))
                except Exception as e:
                    print(
                        f"Warning: Failed to read telemetry for motor {motor_id.name}: {e}",
                        flush=True,
                    )

            end_time_s = get_time()
            if (sleep_s := next_period_s - end_time_s) > 0:
                await asyncio.sleep(sleep_s)
            elif next_period_s < end_time_s:
                next_period_s = end_time_s
                await asyncio.sleep(0)

    async def _cleanup(self) -> None:
        for motor_id in self._active_motors:
            try:
                self._epos.disable(motor_id)
            except Exception as e:
                print(
                    f"Warning: Failed to disable motor {motor_id.name}: {e}", flush=True
                )
        self._epos.shutdown(self._active_motors)

        print("Stopping CAN bus encoder data notifier.", flush=True)
        self._can_notifier.stop()

        self._nicla_backend_proc.join()
        print("Releasing Niclas shared memory latest data buffers.", flush=True)
        for nicla_shm in self._nicla_latest_data.values():
            nicla_shm.close()
            nicla_shm.unlink()

    async def main(self) -> None:
        # Initialize time utils for temporal alignment (data synchronization) with other HERMES components and networked host devices.
        init_time(ref_time=self._ref_time_s)

        # 1) Perform initial calibration of motors and IMUs.
        print("Press 'M' for power-on motors calibration.", flush=True)
        while not self._is_motors_calibrated or not self._nicla_offsets.is_calibrated():
            await self._recv_calibration_trigger(self._calibrate_motors)

        # 3) Indicate to `Pipeline` that handler finished connecting and exo calibrated.
        # NOTE: Begins streaming IMU and motor data, but stores only after upstream HERMES node triggers saving via `_is_keep_data_event` event.
        self._is_ready_event.set()

        # 4) Main working loop.
        # NOTE: Loops until upstream HERMES node triggers closure via `_is_cleanup_event` event.
        # TODO: Add a coroutine with `watchdog` of the motors gains file.
        await asyncio.gather(
            self._run_state_machine(),
            self._poll_motor_data(),
            self._watch_for_offset_recalibration(),
        )
        # Indicate to `ExoPipeline` that no new data will be produced.
        self._is_finished_event.set()

        # 5) Cleanup on exit.
        await self._cleanup()

    def __call__(self) -> None:
        asyncio.run(self.main())
