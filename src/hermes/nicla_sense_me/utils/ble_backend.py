"""
Filename: hermes/nicla_sense_me/utils/ble_backend.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-02
Version: 1.0
Description: Listener and parsing logic for BLE data received from the
    Nicla Sense ME devices.
"""

import asyncio
import numpy as np
from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
from queue import Empty
from typing import Dict, Tuple, Callable, List

from bleak import BleakScanner, BleakClient
from bleak.uuids import normalize_uuid_str
from bleak.backends.device import BLEDevice
from bleak.backends.characteristic import BleakGATTCharacteristic

from hermes.utils.time_utils import get_time

from hermes.nicla_sense_me.utils.abstract_backend import NiclaBackend
from hermes.nicla_sense_me.utils.types import (
    NiclaSampleSynchronized,
    NiclaSampleSynchronizedMetadata,
    NiclaOffsetsSynchronized,
    CalibrationEvent,
    CalibrationEventType,
)


class NiclaBleBackend(NiclaBackend):
    def __init__(
        self,
        niclas: dict,
        nicla_latest_data: "Dict[str, NiclaSampleSynchronizedMetadata]",
        nicla_data_queue: "Queue[Tuple[str, float, bytearray]]",
        nicla_offsets: "NiclaOffsetsSynchronized",
        calibration_event_queue: "Queue[CalibrationEvent]",
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_cleanup_event: _Event,
        ref_time_s: float,
        input_queue: "Queue[tuple[float, str]]",
    ):
        self._dt = 1.0 / niclas["sampling_rate_hz"]
        self._ref_time_s = ref_time_s
        self._input_queue = input_queue
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_cleanup_event = is_cleanup_event

        self._nicla_data_queue = nicla_data_queue
        self._nicla_offsets = nicla_offsets
        self._nicla_latest_data: Dict[str, NiclaSampleSynchronized] = {
            name: NiclaSampleSynchronized.from_metadata(data)
            for name, data in nicla_latest_data.items()
        }
        self._calibration_event_queue = calibration_event_queue

        self._service_uuid = normalize_uuid_str(niclas["service_uuid"])
        self._char_uuid = normalize_uuid_str(niclas["char_uuid"])
        self._nicla_mac_mapping: Dict[str, str] = niclas["device_mapping"]
        self._discovered_devices: Dict[str, BLEDevice] = {}
        self._connected_devices: Dict[str, BleakClient] = {}
        self._disconnected_devices: Dict[str, Tuple[BleakClient, float]] = {}

    def _make_data_callback(self, name):
        def callback(
            characteristic: BleakGATTCharacteristic, raw_data: bytearray
        ) -> None:
            toa_s = get_time()
            self._nicla_latest_data[name].data = raw_data
            if (
                self._is_keep_data_event.is_set()
                and not self._is_stop_new_data_event.is_set()
            ):
                self._nicla_data_queue.put((name, toa_s, raw_data))

        return callback

    def _make_disconnection_callback(self, name: str, device: BLEDevice):
        def callback(client: BleakClient) -> None:
            print(f"Device {name} [{device.address}] disconnected.", flush=True)
            self._connected_devices.pop(name, None)
            self._disconnected_devices[name] = (device, get_time())

        return callback

    async def _discover(self) -> bool:
        discovered_devices = await BleakScanner.discover(
            timeout=10.0, service_uuids=[self._service_uuid]
        )
        found = [device.address for device in discovered_devices]
        if not all([(mac in found) for mac in self._nicla_mac_mapping.values()]):
            not_found = [
                name
                for name, mac in self._nicla_mac_mapping.items()
                if mac not in found
            ]
            print(
                f"Couldn't find {not_found}.\n",
                "Make sure all Niclas are advertising.",
                flush=True,
            )
            return False

        inverted_mac_mapping = {v: k for k, v in self._nicla_mac_mapping.items()}
        self._discovered_devices = {
            inverted_mac_mapping[device.address]: device
            for device in filter(
                lambda d: d.address in self._nicla_mac_mapping.values(),
                discovered_devices,
            )
        }
        return True

    async def _connect_all(self) -> bool:
        try:
            for name, device in self._discovered_devices.items():
                if name in self._connected_devices:
                    continue
                success = await self._connect_and_subscribe(name, device)
                if not success:
                    print(f"Failed to connect to {name}.", flush=True)
                    return False
                # Settle delay between connection setups to let BlueZ & controller stabilize
                await asyncio.sleep(0.3)
            return True
        except Exception as e:
            print("Failed to connect to some of the Niclas.\n", e, flush=True)
            return False

    async def _connect_and_subscribe(self, name: str, device: BLEDevice) -> bool:
        client = BleakClient(
            device,
            disconnected_callback=self._make_disconnection_callback(name, device),
        )
        try:
            await client.connect(timeout=5.0)
            print(f"Connected to {name} [{device.address}]", flush=True)
            await client.start_notify(self._char_uuid, self._make_data_callback(name))
            self._connected_devices[name] = client
            return True
        except Exception as e:
            print(f"Failed to connect to {name}: {e}", flush=True)
            try:
                if client.is_connected:
                    await client.disconnect()
            except Exception:
                pass
            return False

    async def _calibrate_imus(self, duration: float = 5.0) -> None:
        """Measure average torso, thigh, and knee offsets over given seconds."""

        print(
            f"Measuring IMU offsets over {duration} seconds... Please stand still.",
            flush=True,
        )

        samples: Dict[str, List[float]] = {
            name: [] for name in self._nicla_latest_data.keys()
        }

        end_s = asyncio.get_event_loop().time() + duration
        while asyncio.get_event_loop().time() < end_s:
            try:
                for device_name, device_data in self._nicla_latest_data.items():
                    sample = device_data.data
                    if sample.euler is not None:
                        samples[device_name].append(sample.euler[0])
            except (KeyError, IndexError) as e:
                print(
                    f"Error getting offset sample for {device_name}:\n", e, flush=True
                )
                await asyncio.sleep(self._dt)
                continue
            await asyncio.sleep(self._dt)

        if any(len(l) == 0 for l in samples.values()):
            return False

        # Update the synchronized offsets
        offsets = {}
        with self._nicla_offsets.lock:
            res_str = ""
            for device_name, device_samples in samples.items():
                offset = np.mean(device_samples)
                self._nicla_offsets.offsets[device_name].value = offset
                offsets[device_name] = offset
                res_str += f"{device_name}: {offset:.2f}\n"

        self._calibration_event_queue.put(
            CalibrationEvent(
                timestamp=get_time(),
                sensor_type=CalibrationEventType.NICLA,
                offsets=offsets,
            )
        )

        print(f"Offsets measured:\n{res_str}", flush=True)
        return True

    async def _watch_for_offset_recalibration(self) -> None:
        loop = asyncio.get_event_loop()
        while not self._is_cleanup_event.is_set():
            try:
                toa_s, user_input = await loop.run_in_executor(
                    None, self._input_queue.get, True, 0.1
                )
                if user_input == "I":
                    await self._calibrate_imus()
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
            if user_input == "I":
                res = await calibrate_fn()
                success = True if res is None else bool(res)
                if not success:
                    print(
                        "IMUs calibration procedure was not attained. Press 'I' to retry.",
                        flush=True,
                    )
                return success
        except Empty:
            pass
        except Exception as e:
            print(f"Failed to connect: {e}", flush=True)
        return False

    async def _connect(self):
        # Discover IMUs.
        while not (result := await self._discover()):
            print("Trying to rediscover Niclas. Make sure all are on.", flush=True)
            await asyncio.sleep(2)

        # Connect all BLE IMUs.
        while not (result := await self._connect_all()):
            await self._cleanup()
            print("Trying to reconnect to Niclas.", flush=True)
            await asyncio.sleep(2)

    async def _run(self):
        while not self._is_cleanup_event.is_set():
            for name, (device, _) in list(self._disconnected_devices.items()):
                print(
                    f"Trying to directly reconnect to {name} [{device.address}]...",
                    flush=True,
                )

                # Attempt direct connection without BleakScanner active scan,
                # which would interrupt streaming on currently connected sensors.
                success = await self._connect_and_subscribe(name, device)
                if success:
                    print(f"Reconnected to {name}.", flush=True)
                    self._disconnected_devices.pop(name, None)
                else:
                    print(f"Reconnect to {name} failed, will retry later.", flush=True)
            await asyncio.sleep(1.5)

    async def _cleanup(self):
        try:
            await asyncio.gather(
                *(
                    c.stop_notify(self._char_uuid)
                    for c in self._connected_devices.values()
                )
            )
        except Exception as e:
            print(e, flush=True)

        for dev_name, dev in list(self._connected_devices.items()):
            try:
                await dev.disconnect()
            except Exception as e:
                print(f"Failed to disconnect {dev_name}", e, flush=True)

    async def main(self):
        await self._connect()

        print("Press 'I' for power-on IMU offset calibration.", flush=True)
        is_calibrated = False
        while not is_calibrated:
            is_calibrated = await self._recv_calibration_trigger(self._calibrate_imus)

        await asyncio.gather(
            self._run(),
            self._watch_for_offset_recalibration(),
        )

        await self._cleanup()

        for nicla_shm in self._nicla_latest_data.values():
            nicla_shm.close()
