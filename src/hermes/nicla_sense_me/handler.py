"""
Filename: hermes/nicla_sense_me/handler.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-20
Version: 1.0
Description: Handler for the Nicla Sense ME HERMES Node,
    responsible for interfacing with the Nicla Sense ME sensors.
    Connects to the sensors, continuously receives data, and pushes it to
    a multiprocessing queue for the HERMES Producer to manage.
"""

import asyncio
from multiprocessing.synchronize import Event as _Event
from collections import deque
from dataclasses import fields
from multiprocessing import Queue

from hermes.utils.time_utils import init_time

from .utils.abstract_backend import NiclaBackend
from .utils.ble_backend import NiclaBleBackend
from src.hermes.aidwear.utils.types import (
    NiclaMappingPelvisAndFeet,
    NiclaConnectionType,
    NiclaData,
)


class NiclaSenseMeHandler:
    def __init__(
        self,
        niclas: dict,
        nicla_data_queue: "Queue[tuple[str, float, NiclaData]]",
        ref_time_s: float,
        is_ready_event: _Event,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_dev_cleanup_event: _Event,
        is_finished_event: _Event,
        input_queue: "Queue[tuple[float, str]]",
    ):
        self._ref_time_s = ref_time_s

        ##### Inter-process communication related variables.
        self._input_queue = input_queue
        self._is_ready_event = is_ready_event
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_dev_cleanup_event = is_dev_cleanup_event
        self._is_finished_event = is_finished_event

        ##### Nicla Sense ME related variables.
        nicla_connection_type = NiclaConnectionType[niclas["connection_type"]]

        # Filter out AI-only Niclas if running exo without the AI and validate input mapping.
        nicla_mapping: dict[str, dict] = niclas["device_mapping"]
        self._nicla_name_mapping = NiclaMappingPelvisAndFeet(
            **dict(zip(nicla_mapping.keys(), nicla_mapping.keys()))
        )

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
                is_cleanup_event=is_dev_cleanup_event,
            )
        else:
            raise ValueError(
                f"Unsupported Nicla connection type: {nicla_connection_type}"
            )

    async def _cleanup(self) -> None:
        print("Cleaning up Niclas.", flush=True)
        await self._nicla_backend.cleanup()

    async def main(self) -> None:
        # Initialize time utils for temporal alignment (data synchronization) with other HERMES components and networked host devices.
        init_time(ref_time=self._ref_time_s)

        # 1) Connect to the Nicla Sense ME sensors.
        await self._nicla_backend.connect()

        # 2) Indicate to `Producer` that handler finished connecting and IMUs are connected.
        # NOTE: Begins streaming IMU and motor data, but stores only after upstream HERMES node triggers saving via `_is_keep_data_event` event.
        self._is_ready_event.set()

        # 3) Main working loop.
        # NOTE: Loops until upstream HERMES node triggers closure via `_is_cleanup_event` event.
        await self._nicla_backend.run()
        # Indicate to `Producer` that no new data will be produced.
        self._is_finished_event.set()

        # 4) Cleanup on exit.
        await self._cleanup()

    def __call__(self) -> None:
        asyncio.run(self.main())
