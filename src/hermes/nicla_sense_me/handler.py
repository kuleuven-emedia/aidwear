"""
Filename: hermes/nicla_sense_me/handler.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-20
Version: 1.0
Description: Handler for the Nicla Sense ME HERMES Node,
    responsible for interfacing with the Nicla Sense ME sensors.
    Manages the lifecycle and connectivity to on-board devices over specified
    communication interfaces, and exposes connection to the HERMES
    framework for integrating Niclas into upstream multimodal sensing setup.
"""

import asyncio
from dataclasses import fields
from multiprocessing import Process, Queue
from multiprocessing.synchronize import Event as _Event
from queue import Empty
from typing import Callable, Dict, Optional

from hermes.utils.mp_utils import launch_handler
from hermes.utils.time_utils import init_time

from hermes.aidwear.prosthesis.utils.types import CalibrationEvent

from hermes.nicla_sense_me.utils.abstract_backend import NiclaBackend
from hermes.nicla_sense_me.utils.ble_backend import NiclaBleBackend
from hermes.nicla_sense_me.utils.types import (
    NiclaConnectionType,
    NiclaMappingFull,
    NiclaMappingNoPelvisAndFeet,
    NiclaOffsetsSynchronized,
    NiclaSampleSynchronized,
    calculate_nicla_sample_size,
)


class NiclaSenseMeHandler:
    def __init__(
        self,
        niclas: dict,
        nicla_data_queue: "Queue[tuple[str, float, bytearray]]",
        ref_time_s: float,
        is_ready_event: _Event,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_dev_cleanup_event: _Event,
        is_finished_event: _Event,
        input_queue: "Queue[tuple[float, str]]",
        calibration_event_queue: "Queue[CalibrationEvent]",
    ):
        self._ref_time_s = ref_time_s

        # ------------------------------------------------------------------------
        # Inter-process communication related variables
        # ------------------------------------------------------------------------
        self._input_queue = input_queue
        self._nicla_backend_proc_input_queue: "Queue[tuple[float, str]]" = Queue()
        self._calibration_event_queue = calibration_event_queue

        self._is_ready_event = is_ready_event
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_dev_cleanup_event = is_dev_cleanup_event
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
                f"{nicla_connection_type} is not implemented in nicla sense me handler"
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
                "calibration_event_queue": self._calibration_event_queue,
                "is_keep_data_event": is_keep_data_event,
                "is_stop_new_data_event": is_stop_new_data_event,
                "is_cleanup_event": is_dev_cleanup_event,
                "ref_time_s": self._ref_time_s,
                "input_queue": self._nicla_backend_proc_input_queue,
            },
        )
        self._nicla_backend_proc.start()

    async def _watch_for_offset_recalibration(self) -> None:
        loop = asyncio.get_event_loop()
        while not self._is_dev_cleanup_event.is_set():
            try:
                toa_s, user_input = await loop.run_in_executor(
                    None, self._input_queue.get, True, 0.1
                )
                if user_input == "I":
                    self._nicla_backend_proc_input_queue.put((toa_s, user_input))
            except Empty:
                pass
            except Exception as e:
                print(f"Failed to connect: {e}", flush=True)
            finally:
                await asyncio.sleep(0.5)

    async def _recv_calibration_trigger(
        self, calibrate_fn: Optional[Callable] = None
    ) -> None:
        loop = asyncio.get_event_loop()
        try:
            toa_s, user_input = await loop.run_in_executor(
                None, self._input_queue.get, True, 1.0
            )
            if user_input == "I":
                self._nicla_backend_proc_input_queue.put((toa_s, user_input))
        except Empty:
            pass
        except Exception as e:
            print(f"Failed to connect: {e}", flush=True)

    async def _cleanup(self) -> None:
        print("Cleaning up Niclas.", flush=True)
        self._nicla_backend_proc.join()
        print("Releasing Niclas shared memory latest data buffers.", flush=True)
        for nicla_shm in self._nicla_latest_data.values():
            nicla_shm.close()
            nicla_shm.unlink()

    async def main(self) -> None:
        # Initialize time utils for temporal alignment (data synchronization) with other HERMES components and networked host devices.
        init_time(ref_time=self._ref_time_s)

        # 1) Wait for initial calibration of IMUs.
        while not self._nicla_offsets.is_calibrated():
            if self._is_dev_cleanup_event.is_set():
                break
            await self._recv_calibration_trigger()

        # 2) Indicate to `Producer` that handler finished connecting and IMUs calibrated.
        # NOTE: Begins streaming IMU data, but stores only after upstream HERMES node triggers saving via `_is_keep_data_event` event.
        self._is_ready_event.set()

        # 3) Main working loop.
        # NOTE: Loops until upstream HERMES node triggers closure via `_is_dev_cleanup_event` event.
        await self._watch_for_offset_recalibration()

        # Indicate to `Producer` that no new data will be produced.
        self._is_finished_event.set()

        # 4) Cleanup on exit.
        await self._cleanup()

    def __call__(self) -> None:
        asyncio.run(self.main())
