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
from multiprocessing import Process
from multiprocessing.synchronize import Event as _Event
from queue import Queue, Empty
from typing import Callable
import can
import numpy as np
from collections import deque
from dataclasses import fields

from hermes.utils.time_utils import get_time, init_time
from hermes.utils.mp_utils import launch_handler

from .mode_selection import ModeSelectionMachine
from ..motor_control import epos_facade
from ..motor_control.types import (
    EposDeviceConfig,
    HomingConfig,
)
from ..utils.config_manager import ConfigManager
from ..sensors.can_backend import CanBackend
from ..sensors.nicla.abstract_backend import NiclaBackend
from ..sensors.nicla.ble_backend import NiclaBleBackend
from ..sensors.nicla.i2c_backend import NiclaI2cBackend
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
from ...utils.types import (
    NiclaMappingFull,
    NiclaMappingNoPelvisAndFeet,
    NiclaConnectionType,
    NiclaData,
)


class ProsthesisHandler:
    def __init__(
        self,
        niclas: dict,
        motors: dict,
        fsm_config_path: str,
        output_dir: str,
        nicla_data_queue: "Queue[tuple[str, float, NiclaData]]",
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

        ##### Inter-process communication related variables.
        self._input_queue = input_queue
        self._motor_data_queue = motor_data_queue
        self._calibration_event_queue = calibration_event_queue

        self._next_mode = next_mode_synchronized
        self._next_fatigue = next_fatigue_synchronized
        self._next_is_pause = next_is_pause_synchronized

        self._is_ready_event = is_ready_event
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_exo_cleanup_event = is_exo_cleanup_event
        self._is_finished_event = is_finished_event

        self._factor_prev = {k: 0.0 for k in MotorId}
        self._token = 0

        ##### Nicla Sense ME related variables.
        nicla_connection_type = NiclaConnectionType[niclas["connection_type"]]

        # Filter out AI-only Niclas if running exo without the AI and validate input mapping.
        nicla_mapping: dict[str, dict] = niclas["device_mapping"]
        if niclas["is_pelvis_and_feet"]:
            self._nicla_name_mapping = NiclaMappingFull(
                **dict(zip(nicla_mapping.keys(), nicla_mapping.keys()))
            )
        else:
            self._nicla_name_mapping = NiclaMappingNoPelvisAndFeet(
                **dict(zip(nicla_mapping.keys(), nicla_mapping.keys()))
            )

        # Datastructures for managing the incoming Nicla data.
        self._nicla_latest_data: dict[str, deque[NiclaData]] = dict(
            map(
                lambda field: (field.name, deque([], maxlen=2)),
                fields(self._nicla_name_mapping),
            )
        )
        self._nicla_offsets: dict[str, float] = dict(
            map(lambda field: (field.name, 0.0), fields(self._nicla_name_mapping))
        )
        self._nicla_offsets_lock = asyncio.Lock()

        self._nicla_backend: NiclaBackend
        if nicla_connection_type == NiclaConnectionType.BLE:
            self._nicla_backend = NiclaBleBackend(
                niclas=niclas,
                nicla_latest_data=self._nicla_latest_data,
                nicla_data_queue=nicla_data_queue,
                is_keep_data_event=is_keep_data_event,
                is_stop_new_data_event=is_stop_new_data_event,
                is_cleanup_event=is_exo_cleanup_event,
            )
        elif nicla_connection_type == NiclaConnectionType.I2C:
            self._nicla_backend = NiclaI2cBackend()

        ##### Motor related variables.
        motor_mapping: dict[str, dict] = motors["device_mapping"]
        # self._is_emulate_can = "is_emulate_can" in motors and motors["is_emulate_can"]
        self._motor_name_mapping = ProsthesisMotorMapping(
            **dict(zip(motor_mapping.keys(), motor_mapping.keys()))
        )  # validates input mapping.

        self._motor_latest_data: dict[MotorId, deque[ServoMotorData]] = {
            MotorId(motor_spec["can_id"]): deque([], maxlen=2)
            for motor_spec in motor_mapping.values()
        }

        self._encoder_latest_data: dict[EncoderId, deque[EncoderData]] = {
            EncoderId[MotorId(motor_spec["can_id"]).name]: deque([], maxlen=2)
            for motor_spec in motor_mapping.values()
        }
        self._encoder_offsets: dict[EncoderId, AbsoluteEncoderOffset] = {
            EncoderId[MotorId(motor_spec["can_id"]).name]: AbsoluteEncoderOffset(
                motor_spec["absolute_encoder_reference"], 0.0
            )
            for motor_spec in motor_mapping.values()
        }
        self._encoder_offsets_lock = asyncio.Lock()

        # Main CAN bus for absolute encoder reading.
        # if self._is_emulate_can:
        #     self._can_bus = can.interface.Bus(
        #         channel="localhost:18881", interface="virtualcan"
        #     )
        #     # Launch process that generates dummy motor data for all 2 encoders.
        #     self._can_emulator_proc = Process(
        #         target=launch_handler,
        #         args=(CanEmulator,),
        #         kwargs={
        #             "motor_mapping": motor_mapping,
        #             "is_stop_new_data_event": is_stop_new_data_event,
        #             "sampling_rate_hz": motors["sampling_rate_hz"],
        #         },
        #     )
        #     self._can_emulator_proc.start()
        # else:
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

        epos_handle_config = EposDeviceConfig(**motors["epos"])
        self._epos_handle = epos_facade.init(epos_handle_config)

        # for motor_id in MotorId:
        self._active_motors: list[MotorId] = [MotorId.ANKLE, MotorId.KNEE]
        for motor_id in self._active_motors:
            epos_facade.connect(self._epos_handle, motor_id)
            epos_facade.enable(self._epos_handle, motor_id)

        # Low-level motor controller gains.
        # TODO: use `watchdog` to live update gains parameters from a local text file for tunning motors response.
        K: dict[str, ServoImpedanceGains] = {
            ServoMotorEnum.AK10_9.name: ServoImpedanceGains(
                position=1.5, velocity=0.08, acceleration=0
            ),
            ServoMotorEnum.AK80_8.name: ServoImpedanceGains(
                position=0.3, velocity=0.015, acceleration=0
            ),
        }

        # State machine live updater from a config file.
        self._fsm_config_manager = ConfigManager(fsm_config_path, output_dir)

        # High-level locomotion mode selection FSM.
        ctx = ModeContext(
            handle=self._epos_handle,
            K=K,
            #################################################
            # NOTE: provided for "unsafe" operations.
            _nicla_latest_data=self._nicla_latest_data,
            _encoder_latest_data=self._encoder_latest_data,
            _motor_latest_data=self._motor_latest_data,
            #################################################
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
        self._motor_command_queue = ctx.motor_command_queue
        self._is_keep_data_event = ctx.is_keep_data_event

    async def _calibrate_motors(self) -> bool:
        try:
            for motor_id in self._active_motors:
                epos_facade.start_homing(
                    handle=self._epos_handle,
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
                    attained, homing_err = epos_facade.get_homing_state(
                        self._epos_handle, motor_id
                    )
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

    async def _calibrate_imus(self, duration: float = 2.0) -> None:
        """Measure average torso, thigh, and knee offsets over given seconds."""

        print("Measuring IMU offsets... Please stand still.", flush=True)

        samples = dict(
            map(lambda field: (field.name, []), fields(self._nicla_name_mapping))
        )

        end_s = asyncio.get_event_loop().time() + duration
        while asyncio.get_event_loop().time() < end_s:
            try:
                for device_name, device_data in self._nicla_latest_data.items():
                    samples[device_name].append(
                        wrap_angle(device_data[-1].euler[0], 90)
                    )
            except (KeyError, IndexError) as e:
                print(
                    f"Error getting offset sample for {device_name}:\n", e, flush=True
                )
                await asyncio.sleep(self._dt)
                continue
            await asyncio.sleep(self._dt)

        async with self._nicla_offsets_lock:
            print("Offsets measured:", flush=True)
            for device_name, device_samples in samples.items():
                self._nicla_offsets[device_name] = np.mean(device_samples)
                print(f"{device_name}: {self._nicla_offsets[device_name]:.2f}")
            nicla_offsets = dict(self._nicla_offsets)
        self._calibration_event_queue.put(
            CalibrationEvent(
                timestamp=get_time(),
                sensor_type=CalibrationEventType.NICLA,
                offsets=nicla_offsets,
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
                if user_input == "m":
                    await self._calibrate_motors()
                elif user_input == "i":
                    await self._calibrate_imus()
            except Empty:
                pass
            except Exception as e:
                print(f"Failed to connect: {e}", flush=True)
            finally:
                await asyncio.sleep(5)

    async def _recv_calibration_trigger(self, calibrate_fn: Callable) -> bool:
        loop = asyncio.get_event_loop()
        try:
            toa_s, user_input = await loop.run_in_executor(
                None, self._input_queue.get, True, 0.1
            )
            if user_input == "Y":
                res = await calibrate_fn()
                success = True if res is None else bool(res)
                if not success:
                    print(
                        "Calibration procedure was not attained. Press 'Y' to retry.",
                        flush=True,
                    )
                return success
        except Empty:
            pass
        except Exception as e:
            print(f"Failed to connect: {e}", flush=True)
        return False

    async def _run_state_machine(self):
        nicla_euler_samples: dict[str, float] = dict(
            map(lambda field: (field.name, None), fields(self._nicla_name_mapping))
        )
        nicla_gyro_samples: dict[str, float] = dict(
            map(lambda field: (field.name, None), fields(self._nicla_name_mapping))
        )
        encoder_samples: dict[MotorId, EncoderData] = dict(
            map(lambda key: (key, None), self._encoder_latest_data.keys())
        )
        motor_samples: dict[MotorId, ServoMotorData] = dict(
            map(lambda key: (key, None), self._motor_latest_data.keys())
        )

        count = 0
        mean = 0.0
        mean2 = 0.0
        min_loop_time = 0.0
        max_loop_time = 0.0
        next_period_s = get_time()
        while not self._is_exo_cleanup_event.is_set():
            start_time_s = get_time()
            next_period_s += self._dt

            if (
                any(len(dq) == 0 for dq in self._nicla_latest_data.values())
                or any(len(dq) == 0 for dq in self._encoder_latest_data.values())
                or any(len(dq) == 0 for dq in self._motor_latest_data.values())
            ):
                if (sleep_s := (next_period_s - start_time_s)) > 0:
                    await asyncio.sleep(sleep_s)
                continue

            # TODO: not super clean. Improve consistency in the future.
            async with self._nicla_offsets_lock:
                for device_name, device_data in self._nicla_latest_data.items():
                    nicla_euler_samples[device_name] = (
                        wrap_angle(device_data[-1].euler[0], 90)
                        - self._nicla_offsets[device_name]
                    )
                    nicla_gyro_samples[device_name] = device_data[-1].gyroscope[0]

            async with self._encoder_offsets_lock:
                for encoder_id, encoder_data in self._encoder_latest_data.items():
                    encoder_samples[encoder_id] = encoder_data[-1]
                    encoder_samples[encoder_id].angle = (
                        self._encoder_offsets[encoder_id].reference
                        + encoder_data[-1].angle
                        - self._encoder_offsets[encoder_id].offset
                    )

            for motor_id, motor_data in self._motor_latest_data.items():
                motor_samples[motor_id] = motor_data[-1]

            nicla_samples = NiclaSamples(nicla_euler_samples, nicla_gyro_samples)

            # If safe-stop was entered via researcher input, exo will be forced into `IDLE` mode
            with self._next_is_pause.lock:
                next_is_pause = self._next_is_pause.next_value.value

            # Stop any in-progress motor movements if safety stop triggered.
            if next_is_pause and not self._is_paused:
                for motor_id in self._active_motors:
                    epos_facade.quick_stop(self._epos_handle, motor_id)
                self._is_paused = True

            # Re-enable motors if safety is switched off.
            elif not next_is_pause and self._is_paused:
                for motor_id in self._active_motors:
                    epos_facade.enable(self._epos_handle, motor_id)
                self._is_paused = False
                self._mode_fsm.to_idle()

            # Check if the handler received an update of state from the parent Pipeline mode with the new AI prediction.
            # The condition is evaluated on each loop iteration to recognize asynchronously received intent prediction.
            #   Internal logic of the `ModeSelectionMachine` will judge on which iteration to permit transition using conditional FSM transitions.
            elif not next_is_pause and not self._is_paused:
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
                    motor_sample = epos_facade.get_motor_data(
                        self._epos_handle, motor_id
                    )
                    # TODO: hold the async lock while adding the new sample
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
                epos_facade.disable(self._epos_handle, motor_id)
            except Exception as e:
                print(
                    f"Warning: Failed to disable motor {motor_id.name}: {e}", flush=True
                )
        epos_facade.shutdown(self._epos_handle, self._active_motors)

        self._can_notifier.stop()

        print("Cleaning up Niclas.", flush=True)
        await self._nicla_backend.cleanup()
        # if self._is_emulate_can:
        #     print("Cleaning up CAN emulator.", flush=True)
        #     self._can_emulator_proc.join()

    async def main(self) -> None:
        # Initialize time utils for temporal alignment (data synchronization) with other HERMES components and networked host devices.
        init_time(ref_time=self._ref_time_s)

        # 1) Perform initial calibration of motors.
        print("Press 'Y' for power-on motors calibration.", flush=True)
        is_calibrated = False
        while not is_calibrated:
            is_calibrated = await self._recv_calibration_trigger(self._calibrate_motors)

        # for motor_id in self._active_motors:
        #     try:
        #         epos_facade.disable(self._epos_handle, motor_id)
        #     except Exception as e:
        #         print(f"Warning: Failed to disable motor {motor_id.name}: {e}", flush=True)
        # epos_facade.shutdown(self._epos_handle, self._active_motors)

        # 2) Connect to the Nicla Sense ME sensors and calibrate offsets.
        await self._nicla_backend.connect()
        print("Press 'Y' for power-on IMU offset calibration.", flush=True)
        is_calibrated = False
        while not is_calibrated:
            is_calibrated = await self._recv_calibration_trigger(self._calibrate_imus)

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
            self._nicla_backend.run(),
        )
        # Indicate to `ExoPipeline` that no new data will be produced.
        self._is_finished_event.set()

        # 5) Cleanup on exit.
        await self._cleanup()

    def __call__(self) -> None:
        asyncio.run(self.main())
