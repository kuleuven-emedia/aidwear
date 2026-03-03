import asyncio
from multiprocessing import Process
from multiprocessing.sharedctypes import Synchronized
from multiprocessing.synchronize import Event as _Event
from queue import Queue, Empty
import can
import numpy as np
from collections import deque
from dataclasses import fields

from hermes.utils.time_utils import get_time, init_time
from hermes.utils.mp_utils import launch_handler

from .mode_selection import ModeSelectionMachine
from ..sensors.can_backend import CanBackend
from ..sensors.nicla_backend import NiclaBackend, NiclaBleBackend, NiclaI2cBackend
from ..can_control.emulator_cubemars import CanEmulator
from ..utils.utilities import (
    config_can_linux,
    finalize_running_stats,
    update_running_stats,
    wrap_angle,
)
from ..utils.types import (
    BatteryData,
    ProsthesisNiclaMapping,
    ProsthesisMotorMapping,
    ModeContext,
    MotorCommand,
    NiclaConnectionType,
    ServoMotorEnum,
    ServoImpedanceGains,
    NiclaData,
    ServoMotorData,
    ModeEnum,
    ModeTransition,
    StateTransition,
    PhaseEstimate,
)


class ProsthesisHandler:
    def __init__(
        self,
        niclas: dict,
        motors: dict,
        nicla_data_queue: "Queue[tuple[str, float, NiclaData]]",
        motor_data_queue: "Queue[tuple[str, ServoMotorData]]",
        battery_data_queue: "Queue[tuple[str, BatteryData]]",
        mode_changed_queue: "Queue[ModeTransition]",
        state_changed_queue: "Queue[StateTransition]",
        phase_estimate_queue: "Queue[PhaseEstimate]",
        motor_command_queue: "Queue[MotorCommand]",
        next_mode: "Synchronized[int]",
        next_fatigue: "Synchronized[float]",
        next_mode_sequence_id: "Synchronized[int]",
        next_fatigue_sequence_id: "Synchronized[int]",
        ref_time_s: float,
        is_ready_event: _Event,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_cleanup_event: _Event,
        is_finished_event: _Event,
        input_queue: "Queue[tuple[float, str]]",
        dt: float = 0.01,
    ):
        self._ref_time_s = ref_time_s
        self._dt = dt

        ##### Inter-process communication related variables.
        self._input_queue = input_queue

        self._next_mode = next_mode
        self._next_mode_sequence_id = next_mode_sequence_id

        self._is_ready_event = is_ready_event
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_cleanup_event = is_cleanup_event
        self._is_finished_event = is_finished_event

        ##### Nicla Sense ME related variables.
        nicla_mapping: dict[str, dict] = niclas["device_mapping"]
        nicla_connection_type = NiclaConnectionType[niclas["connection_type"]]
        self._nicla_name_mapping = ProsthesisNiclaMapping(
            **dict(zip(nicla_mapping.keys(), nicla_mapping.keys()))
        )  # validates input mapping.
        self._nicla_latest_data: dict[str, deque[NiclaData]] = dict(
            map(
                lambda field: (field.name, deque(maxlen=2)),
                fields(self._nicla_name_mapping),
            )
        )
        self._offsets: dict[str, float] = dict(
            map(lambda field: (field.name, 0.0), fields(self._nicla_name_mapping))
        )
        self._offsets_lock = asyncio.Lock()

        self._nicla_backend: NiclaBackend
        if nicla_connection_type == NiclaConnectionType.BLE:
            self._nicla_backend = NiclaBleBackend(
                niclas=niclas,
                nicla_latest_data=self._nicla_latest_data,
                nicla_data_queue=nicla_data_queue,
                is_keep_data_event=is_keep_data_event,
                is_stop_new_data_event=is_stop_new_data_event,
                is_cleanup_event=is_cleanup_event,
            )
        elif nicla_connection_type == NiclaConnectionType.I2C:
            self._nicla_backend = NiclaI2cBackend()

        ##### Motor related variables.
        motor_mapping: dict[str, dict] = motors["device_mapping"]
        self._is_emulate_can = "is_emulate_can" in motors and motors["is_emulate_can"]
        self._motor_name_mapping = ProsthesisMotorMapping(
            **dict(zip(motor_mapping.keys(), motor_mapping.keys()))
        )  # validates input mapping.
        motor_type_mapping: dict[int, ServoMotorEnum] = dict(
            [
                (motor_spec["can_id"], ServoMotorEnum[motor_spec["type"]])
                for motor_spec in motor_mapping.values()
            ]
        )
        self._motor_latest_data: dict[int, deque[ServoMotorData | None]] = {
            motor_spec["can_id"]: deque([None], maxlen=1)
            for motor_spec in motor_mapping.values()
        }

        # CAN bus for motor control.
        if self._is_emulate_can:
            self._can_bus = can.interface.Bus(
                channel="localhost:18881", interface="virtualcan"
            )
            # Launch process that generates dummy motor data for all 4 motors.
            self._can_emulator_proc = Process(
                target=launch_handler,
                args=(CanEmulator,),
                kwargs={
                    "motor_mapping": motor_mapping,
                    "is_stop_new_data_event": is_stop_new_data_event,
                    "sampling_rate_hz": motors["sampling_rate_hz"],
                },
            )
            self._can_emulator_proc.start()
        else:
            config_can_linux(channel="can0")
            self._can_bus = can.interface.Bus(channel="can0", interface="socketcan")

        self._can_listener = CanBackend(
            motor_type_mapping=motor_type_mapping,
            motor_latest_data=self._motor_latest_data,
            motor_data_queue=motor_data_queue,
            battery_data_queue=battery_data_queue,
            is_keep_data_event=is_keep_data_event,
            is_stop_new_data_event=is_stop_new_data_event,
        )
        self._can_notifier = can.Notifier(
            bus=self._can_bus, listeners=[self._can_listener]
        )

        # Low-level motor controller gains.
        # TODO: use `watchdog` to live update gains parameters from a local text file for tunning motors response.
        K: dict[str, ServoImpedanceGains] = {
            ServoMotorEnum.AK10_9.name: ServoImpedanceGains(
                position=0.5,
                velocity=0.06,
                acceleration=0
            ),
            ServoMotorEnum.AK80_8.name: ServoImpedanceGains(
                position=0.1,
                velocity=0.01,
                acceleration=0
            ),
        }

        # High-level locomotion mode selection FSM.
        ctx = ModeContext(
            bus=self._can_bus,
            K=K,
            motor_latest_data=self._motor_latest_data,
            fatigue=next_fatigue,
            mode_changed_queue=mode_changed_queue,
            state_changed_queue=state_changed_queue,
            phase_estimate_queue=phase_estimate_queue,
            motor_command_queue=motor_command_queue,
            is_stop_new_data_event=is_stop_new_data_event,
        )
        self._mode_fsm = ModeSelectionMachine(ctx)
        self._next_mode.value = self._mode_fsm.current_state.value

    async def _measure_offsets(self, duration: float = 2.0) -> None:
        """Measure average torso, thigh, and knee offsets over given seconds."""

        print("Measuring offsets... Please stand still.", flush=True)

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

        async with self._offsets_lock:
            print("Offsets measured:", flush=True)
            for device_name, device_samples in samples.items():
                self._offsets[device_name] = np.mean(device_samples)
                print(f"{device_name}: {self._offsets[device_name]:.2f}")

    async def _watch_for_offset_recalibration(self):
        while not self._is_cleanup_event.is_set():
            await self._recv_calibration_trigger()
            await asyncio.sleep(5)

    async def _recv_calibration_trigger(self) -> bool:
        loop = asyncio.get_event_loop()
        try:
            toa_s, user_input = await loop.run_in_executor(
                None,
                self._input_queue.get,
                True,
                0.1
            )
            if user_input == "m":
                await self._measure_offsets()
                return True
        except Empty:
            pass
        except Exception as e:
            print(f"Failed to connect: {e}", flush=True)

    async def _run_state_machine(self):
        samples = dict(
            map(lambda field: (field.name, 0.0), fields(self._nicla_name_mapping))
        )

        count = 0
        mean = 0.0
        mean2 = 0.0
        min_loop_time = 0.0
        max_loop_time = 0.0
        next_period_s = get_time()
        while not self._is_cleanup_event.is_set():
            start_time_s = get_time()
            next_period_s += self._dt
            # Check if all devices have data.
            if any(
                [
                    not self._nicla_latest_data[field.name]
                    for field in fields(self._nicla_name_mapping)
                ]
            ):
                if (sleep_s := (next_period_s - start_time_s)) > 0:
                    await asyncio.sleep(sleep_s)
                continue

            async with self._offsets_lock:
                for device_name, device_data in self._nicla_latest_data.items():
                    samples[device_name] = (
                        wrap_angle(device_data[-1].euler[0], 90)
                        - self._offsets[device_name]
                    )

            torso_angle = samples[self._nicla_name_mapping.torso]
            thigh_left_angle = samples[self._nicla_name_mapping.thigh_left]
            thigh_right_angle = samples[self._nicla_name_mapping.thigh_right]

            thigh_left_roll = torso_angle - thigh_left_angle
            thigh_right_roll = torso_angle - thigh_right_angle
            knee_left_roll = (
                samples[self._nicla_name_mapping.shank_left] - thigh_left_angle
            )
            knee_right_roll = (
                samples[self._nicla_name_mapping.shank_right] - thigh_right_angle
            )

            thigh_left_gyr = (
                self._nicla_latest_data[self._nicla_name_mapping.thigh_left][
                    -1
                ].gyroscope[0]
                - self._nicla_latest_data[self._nicla_name_mapping.torso][-1].gyroscope[
                    0
                ]
            )
            thigh_right_gyr = (
                self._nicla_latest_data[self._nicla_name_mapping.thigh_right][
                    -1
                ].gyroscope[0]
                - self._nicla_latest_data[self._nicla_name_mapping.torso][-1].gyroscope[
                    0
                ]
            )

            # Check if the handler received an update of state from the parent Pipeline mode with the new AI prediction.
            # The condition is evaluated on each loop iteration to recognize asynchronously received intent prediction.
            #   Internal logic of the `ModeSelectionMachine` will judge on which iteration to permit transition using conditional FSM transitions.
            if self._next_mode.value != self._mode_fsm.current_state.value:
                # Store next mode's sequence ID to guarantee it doesn't change while switching states.
                self._mode_fsm._sequence_id = self._next_mode_sequence_id.value
                # Triggers potential transition to the next locomotion mode.
                if self._next_mode.value == ModeEnum.WALKING.value.id:
                    self._mode_fsm.to_walking()
                elif self._next_mode.value == ModeEnum.SIT_TO_STAND.value.id:
                    self._mode_fsm.to_sit_to_stand()
                elif self._next_mode.value == ModeEnum.STAIR_ASCENT.value.id:
                    self._mode_fsm.to_stair_ascent()
                elif self._next_mode.value == ModeEnum.STAIR_DESCENT.value.id:
                    self._mode_fsm.to_stair_descent()
                elif self._next_mode.value == ModeEnum.IDLE.value.id:
                    self._mode_fsm.to_idle()

            self._mode_fsm.update_sensor_values(
                torso_angle=torso_angle,
                thigh_left_angle=thigh_left_angle,
                thigh_right_angle=thigh_right_angle,
                thigh_left_roll=thigh_left_roll,
                thigh_right_roll=thigh_right_roll,
                knee_left_roll=knee_left_roll,
                knee_right_roll=knee_right_roll,
                thigh_left_gyr=thigh_left_gyr,
                thigh_right_gyr=thigh_right_gyr,
                dt=self._dt,
            )
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
            self._mode_fsm.to_idle()
            self._mode_fsm.step()

        # Finalize and print prosthesis statistics.
        if (res := finalize_running_stats(count, mean, mean2)) is not None:
            print(
                f"Prosthesis {self._dt}s FSM loop timing: mean = {res[0]} | variance = {res[1]} | sample variance = {res[2]} | min loop time = {min_loop_time} | max loop time = {max_loop_time}",
                flush=True,
            )

    async def _cleanup(self) -> None:
        self._can_notifier.stop()
        print("Cleaning up Niclas.", flush=True)
        await self._nicla_backend.cleanup()
        if self._is_emulate_can:
            print("Cleaning up CAN emulator.", flush=True)
            self._can_emulator_proc.join()

    async def main(self) -> None:
        # Initialize time utils for temporal alignment (data synchronization) with other HERMES components and networked host devices.
        init_time(ref_time=self._ref_time_s)

        # 1) Connect to the Nicla Sense ME sensors.
        await self._nicla_backend.connect()

        # 2) Perform initial calibration.
        print("Press 'm' for initial offset calibration.", flush=True)
        is_calibrated = False
        while not is_calibrated:
            is_calibrated = await self._recv_calibration_trigger()

        # 3) Indicate to `Pipeline` that handler finished connecting and prosthesis calibrated.
        # NOTE: Begins streaming IMU and motor data, but stores only after upstream HERMES node triggers saving via `_is_keep_data_event` event.
        self._is_ready_event.set()

        # 4) Main working loop.
        # NOTE: Loops until upstream HERMES node triggers closure via `_is_cleanup_event` event.
        # TODO: Add a coroutine with `watchdog` of the motors gains file.
        await asyncio.gather(
            self._run_state_machine(),
            self._watch_for_offset_recalibration(),
            self._nicla_backend.run(),
        )
        # Indicate to `ProsthesisPipeline` that no new data will be produced.
        self._is_finished_event.set()

        # 5) Cleanup on exit.
        await self._cleanup()

    def __call__(self) -> None:
        asyncio.run(self.main())
