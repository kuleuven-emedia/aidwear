import asyncio
from multiprocessing import Queue
from multiprocessing.synchronize import Event as EventClass
from multiprocessing.sharedctypes import Synchronized
from queue import Empty
import json
import can
import numpy as np
from collections import deque
from dataclasses import fields

import paho.mqtt.client as mqtt

from bleak import BleakScanner, BleakClient
from bleak.uuids import normalize_uuid_str
from bleak.backends.device import BLEDevice
from bleak.backends.characteristic import BleakGATTCharacteristic

from hermes.utils.time_utils import get_time, init_time

from .mode_selection import ModeSelectionMachine
from ..motor_control.epos_can_commands import parse_servo_message
from ..utils.types import (
    CyberlegNiclaMapping,
    CyberlegMotorMapping,
    ModeStateEnum,
    ModeTransition,
    NiclaSample,
    PhaseTransition,
    ServoImpedanceGains,
    ServoMotorState,
    ServoMotorEnum,
)
from ..utils.utilities import config_can_linux, wrap_angle


class CyberlegHandler:
    def __init__(
        self,
        nicla_mapping: dict[str, str],
        service_uuid: str,
        char_uuid: str,
        motor_mapping: dict,
        ref_time_s: float,

        is_ready_event: EventClass,
        is_cleanup_event: EventClass,
        is_finished_event: EventClass,
        is_keep_data_event: EventClass,

        input_queue: "Queue[tuple[float, str]]",
        nicla_sample_queue: "dict[str, Queue[tuple[float, NiclaSample]]]",
        motor_state_queue: "dict[int, Queue[tuple[float, ServoMotorState]]]",
        mode_changed_queue: "Queue[ModeTransition]",
        phase_changed_queue: "Queue[PhaseTransition]",

        next_mode: Synchronized[int],
        next_fatigue: Synchronized[float],
        next_mode_sequence_id: Synchronized[int],
        next_fatigue_sequence_id: Synchronized[int],

        dt: float = 0.01,
    ):
        self._nicla_name_mapping = CyberlegNiclaMapping(**dict(zip(nicla_mapping.keys(), nicla_mapping.keys())))  # validates input mapping.
        self._nicla_mac_mapping = nicla_mapping

        self._motor_name_mapping = CyberlegMotorMapping(**dict(zip(motor_mapping.keys(), motor_mapping.keys())))  # validates input mapping.
        self._motor_type_mapping: dict[int, ServoMotorEnum] = dict([(motor_spec["can_id"], ServoMotorEnum[motor_spec["type"]]) for motor_spec in motor_mapping.values()])
        self._motor_latest_state: dict[int, deque[ServoMotorState | None]] = {motor_spec["can_id"]: deque([None], maxlen=1) for motor_spec in motor_mapping.values()}

        self._input_queue = input_queue
        self._nicla_sample_queue = nicla_sample_queue
        self._motor_state_queue = motor_state_queue

        self._next_mode = next_mode
        self._next_fatigue = next_fatigue
        self._next_mode_sequence_id = next_mode_sequence_id
        self._next_fatigue_sequence_id = next_fatigue_sequence_id

        self._is_ready_event = is_ready_event
        self._is_cleanup_event = is_cleanup_event
        self._is_finished_event = is_finished_event
        self._is_keep_data_event = is_keep_data_event
        self._ref_time_s = ref_time_s
        self._dt = dt

        self._K: dict[str, ServoImpedanceGains] = {
            "AK10-9": ServoImpedanceGains(
                position=0.5,
                velocity=0.06,
                acceleration=0
            ),
            "AK80-8": ServoImpedanceGains(
                position=0.1,
                velocity=0.01,
                acceleration=0
            ),
        }
        self._device_data: dict[str, deque[NiclaSample]] = dict(map(lambda field: (field.name, deque(maxlen=2)), fields(self._nicla_name_mapping)))
        self._offsets: dict[str, float] = dict(map(lambda field: (field.name, 0.0), fields(self._nicla_name_mapping)))
        self._offsets_lock = asyncio.Lock()
        self._discovered_devices: dict[str, BLEDevice] = {}
        self._connected_devices: dict[str, BleakClient] = {}
        self._disconnected_devices: dict[str, tuple[BleakClient, float]] = {}
        self._service_uuid = normalize_uuid_str(service_uuid)
        self._char_uuid = normalize_uuid_str(char_uuid)

        self._mqtt_client = mqtt.Client()
        self._mqtt_client.connect("localhost", 1883, 60)  # Public MQTT broker for telemetry overview.

        config_can_linux()
        self._can_bus = can.interface.Bus(channel="can0", interface="socketcan")
        self._can_notifier = can.Notifier(bus=self._can_bus, listeners=self._on_can_message_received)

        self._mode_fsm = ModeSelectionMachine(  # Top-level mode selection FSM.
            bus=self._can_bus,
            K=self._K,
            motor_latest_state=self._motor_latest_state,
            next_fatigue=self._next_fatigue,
            mode_changed_queue=mode_changed_queue,
            phase_changed_queue=phase_changed_queue,
        )
        self._next_mode.value = self._mode_fsm.current_state.value

    def _on_can_message_received(self, msg: can.Message) -> None:
        toa_s = get_time()
        src_id = msg.arbitration_id & 0x00000FF
        motor_type = self._motor_type_mapping[src_id]
        motor_state = parse_servo_message(msg.timestamp, bytes(msg.data), motor_type)
        self._motor_latest_state[src_id].append(motor_state)

        # TODO: placing each sample separately without alignment may overwhelm MPI and middleware.
        #       -> Use alignment buffers.
        if self._is_keep_data_event.is_set():
            self._motor_state_queue[src_id].put((toa_s, motor_state))

    def _make_data_callback(self, name):
        def callback(characteristic: BleakGATTCharacteristic, raw_data: bytearray) -> None:
            toa_s = get_time()
            sample = NiclaSample.from_bytes(raw_data)
            self._device_data[name].append(sample)

            # TODO: insert into a `hermes.datastructures.fifo.TimestampAlignedFifoBuffer` wrapper Node
            #       will pull the data from it to send to the AI model.
            if self._is_keep_data_event.is_set():
                self._nicla_sample_queue[name].put((toa_s, sample))
        return callback

    def _make_disconnection_callback(self, name: str, device: BLEDevice):
        def callback(client: BleakClient) -> None:
            print(f"Device {name} [{device.address}] disconnected.", flush=True)
            self._connected_devices.pop(name, None)
            self._disconnected_devices[name] = (device, get_time())
        return callback

    async def _measure_offsets(self, duration: float = 2.0) -> None:
        """Measure average torso, thigh, and knee offsets over given seconds."""

        print("Measuring offsets... Please stand still.", flush=True)

        samples = dict(map(lambda field: (field.name, []), fields(self._nicla_name_mapping)))

        end_s = asyncio.get_event_loop().time() + duration
        while asyncio.get_event_loop().time() < end_s:
            try:
                for device_name, device_data in self._device_data.items():
                    samples[device_name].append(wrap_angle(device_data[-1].euler[0], 90))
            except (KeyError, IndexError) as e:
                print(f"Error getting offset sample for {device_name}:\n", e, flush=True)
                await asyncio.sleep(self._dt)
                continue
            await asyncio.sleep(self._dt)

        async with self._offsets_lock:
            print("Offsets measured:", flush=True)
            for device_name, device_samples in samples.items():
                self._offsets[device_name] = np.mean(device_samples)
                print(f"{device_name}: {self._offsets[device_name]:.2f}")

    async def _discover(self) -> bool:
        discovered_devices = await BleakScanner.discover(timeout=10.0, service_uuids=[self._service_uuid])
        found = list(map(lambda device: device.address, discovered_devices))
        if not all([(mac in found) for mac in self._nicla_mac_mapping.values()]):
            not_found = [name for name, mac in self._nicla_mac_mapping.items() if mac not in found]
            print(f"Couldn't find {not_found}.\n", "Make sure all Niclas are advertising.", flush=True)
            return False

        inverted_mac_mapping = {v: k for k, v in self._nicla_mac_mapping.items()}
        self._discovered_devices = {
            inverted_mac_mapping[device.address]: device
            for device in filter(lambda d: d.address in self._nicla_mac_mapping.values(), discovered_devices)
        }
        return True

    async def _connect_all(self) -> bool:
        try:
            async with asyncio.TaskGroup() as tg:
                client_tasks = [
                    tg.create_task(self._connect_and_subscribe(name, device))
                    for name, device in self._discovered_devices.items()
                ]
            return all([t.result() for t in client_tasks])
        except Exception as e:
            print("Failed to connect to some of the Niclas.\n", e, flush=True)
            return False

    async def _connect_and_subscribe(self, name: str, device: BLEDevice) -> bool:
        client = BleakClient(device, disconnected_callback=self._make_disconnection_callback(name, device))
        try:
            await client.connect()
            print(f"Connected to {name} [{device.address}]", flush=True)
            await client.start_notify(self._char_uuid, self._make_data_callback(name))
            self._connected_devices[name] = client
            return True
        except Exception as e:
            print(f"Failed to connect to {name}: {e}", flush=True)
            return False

    async def _watch_for_offset_recalibration(self):
        loop = asyncio.get_event_loop()
        while not self._is_cleanup_event.is_set():
            try:
                toa_s, user_input = await loop.run_in_executor(
                    None,
                    self._input_queue.get,
                    True,
                    0.1
                )
                if user_input == 'm':
                    await self._measure_offsets()
            except Empty:
                pass
            except Exception as e:
                print(f"Failed to connect: {e}", flush=True)
            finally:
                await asyncio.sleep(5)

    async def _reconnect_manager(self):
        while not self._is_cleanup_event.is_set():
            for name, (device, _) in list(self._disconnected_devices.items()):
                print(f"Trying to reconnect to {name}...", flush=True)

                fresh_device = await BleakScanner.find_device_by_address(device.address, timeout=1.0)
                if not fresh_device:
                    print(f"Device {name} not found in scan.", flush=True)
                    continue

                success = await self._connect_and_subscribe(name, fresh_device)
                if success:
                    print(f"Reconnected to {name}.", flush=True)
                    self._disconnected_devices.pop(name, None)
                else:
                    print(f"Reconnect to {name} failed, will retry later.", flush=True)
            await asyncio.sleep(1.5)

    async def _run_state_machine(self):
        samples = dict(map(lambda field: (field.name, 0.0), fields(self._nicla_name_mapping)))

        while not self._is_cleanup_event.is_set():
            # Check if all devices have data.
            if any([
                not self._device_data[field.name]
                for field in fields(self._nicla_name_mapping)
            ]):
                await asyncio.sleep(self._dt)
                continue

            async with self._offsets_lock:
                for device_name, device_data in self._device_data.items():
                    samples[device_name] = (wrap_angle(device_data[-1].euler[0], 90) - self._offsets[device_name])

            torso_angle = samples[self._nicla_name_mapping.torso]
            thigh_left_angle = samples[self._nicla_name_mapping.thigh_left]
            thigh_right_angle = samples[self._nicla_name_mapping.thigh_right]

            thigh_left_roll = torso_angle - thigh_left_angle
            thigh_right_roll = torso_angle - thigh_right_angle
            knee_left_roll = samples[self._nicla_name_mapping.shank_left] - thigh_left_angle
            knee_right_roll = (samples[self._nicla_name_mapping.shank_right] - thigh_right_angle)

            thigh_left_gyr = (
                self._device_data[self._nicla_name_mapping.thigh_left][-1].gyroscope[0]
                - self._device_data[self._nicla_name_mapping.torso][-1].gyroscope[0]
            )
            thigh_right_gyr = (
                self._device_data[self._nicla_name_mapping.thigh_right][-1].gyroscope[0]
                - self._device_data[self._nicla_name_mapping.torso][-1].gyroscope[0]
            )

            # Check if the handler received an update of state from the parent Pipeline mode with the new AI prediction.
            # The condition is evaluated on each loop iteration to recognize asynchronously received intent prediction.
            #   Internal logic of the `ModeSelectionMachine` will judge on which iteration to permit transition using conditional FSM transitions.
            if self._next_mode.value != self._mode_fsm.current_state.value:
                # Store next mode's sequence ID to guarantee it doesn't change while switching states.
                self._mode_fsm._sequence_id = self._next_mode_sequence_id.value
                # Triggers potential transition to the next locomotion mode.
                if self._next_mode.value == ModeStateEnum.WALKING.value.id:
                    self._mode_fsm.to_walking()
                elif self._next_mode.value == ModeStateEnum.SIT_TO_STAND.value.id:
                    self._mode_fsm.to_sit_to_stand()
                elif self._next_mode.value == ModeStateEnum.STAIR_ASCENT.value.id:
                    self._mode_fsm.to_stair_ascent()
                elif self._next_mode.value == ModeStateEnum.STAIR_DESCENT.value.id:
                    self._mode_fsm.to_stair_descent()

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
            data = self._mode_fsm.send_data()

            self._mqtt_client.publish("revalexo/data", json.dumps(data))
            await asyncio.sleep(self._dt)

    async def _cleanup(self) -> None:
        print("Cleaning up Niclas.", flush=True)
        await asyncio.gather(*(c.stop_notify(self._char_uuid) for c in self._connected_devices.values()))
        await asyncio.gather(*(c.disconnect() for c in self._connected_devices.values()))

    async def main(self) -> None:
        # Initialize time utils for temporal alignment (data synchronization) with other HERMES components and networked host devices.
        init_time(ref_time=self._ref_time_s)

        # 1) Discover IMUs.
        while not (result := await self._discover()):
            print("Trying to rediscover Niclas. Make sure all are on.", flush=True)
            await asyncio.sleep(2)

        # 2) Connect all BLE IMUs.
        while not (result := await self._connect_all()):
            await self._cleanup()
            print("Trying to reconnect to Niclas.", flush=True)
            await asyncio.sleep(2)
        print("Press 'm' for initial offset calibration.", flush=True)
        # Blocks until keypress for the initial offset calibration.
        loop = asyncio.get_event_loop()
        is_calibrated = False
        while not is_calibrated:
            try:
                toa_s, user_input = await loop.run_in_executor(
                    None,
                    self._input_queue.get,
                    True,
                    0.1
                )
                if user_input == 'm':
                    is_calibrated = True
                    await self._measure_offsets()
            except Empty:
                print(f"User input queue empty", flush=True)
                await asyncio.sleep(5)
            except Exception as e:
                print(f"Failed to connect: {e}", flush=True)
                await asyncio.sleep(5)

        # Indicate to `Pipeline` that handler finished connecting and exo calibrated.
        # NOTE: Begins streaming IMU and motor data, but stores only after upstream HERMES node triggers saving via `_is_keep_data_event` event.
        self._is_ready_event.set()

        # 3) Main working loop.
        # NOTE: Loops until upstream HERMES node triggers closure via `_is_cleanup_event` event.
        await asyncio.gather(
            self._reconnect_manager(),
            self._watch_for_offset_recalibration(),
            self._run_state_machine(),
        )

        # 4) Cleanup on exit.
        await self._cleanup()
        self._is_finished_event.set()

    def __call__(self) -> None:
        asyncio.run(self.main())
        self._can_notifier.stop()
