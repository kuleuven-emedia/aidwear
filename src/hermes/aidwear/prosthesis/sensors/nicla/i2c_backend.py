"""
Filename: hermes/revalexo/exo/sensors/nicla/i2c_backend.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-02
Version: 1.0
Description: Listener and parsing logic for I2C data received from the
    Nicla Sense ME devices.
"""

import asyncio
from collections import deque
from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
import time

# from RPi.GPIO import GPIO
# import board
# import busio

from hermes.utils.time_utils import get_time

from .abstract_backend import NiclaBackend
from hermes.aidwear.utils.types import NiclaData, NiclaI2cCommand, NiclaPacketMask


class NiclaI2cBackend(NiclaBackend):
    def __init__(
        self,
        niclas: dict,
        nicla_latest_data: "dict[str, deque[NiclaData]]",
        nicla_data_queue: "Queue[tuple[str, float, NiclaData]]",
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_cleanup_event: _Event,
    ):
        self._nicla_mapping: dict[str, dict[str, int]] = niclas["device_mapping"]
        self._nicla_pin_mapping: dict[int, str] = {
            v["int_pin"]: k for k, v in self._nicla_mapping.items()
        }
        self._nicla_latest_data = nicla_latest_data
        self._nicla_data_queue = nicla_data_queue
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_cleanup_event = is_cleanup_event
        self._i2c: busio.I2C
        # Provide the overoptimistic buffer to accomodate the largest expected variable-size Nicla packet.
        self._recv_buffer_size = sum([v.value.num_bytes for v in NiclaPacketMask]) + 9

    def _writeto(self, address: int, data: int):
        if isinstance(data, int):
            data = bytes((data,))

        while not self._i2c.try_lock():
            time.sleep(0)
        try:
            self._i2c.writeto(address, data)
        finally:
            self._i2c.unlock()

        return self

    def _writeto_then_readfrom(self, address: int, data: int, buffer: bytearray):
        if isinstance(data, int):
            data = bytes((data,))

        while not self._i2c.try_lock():
            time.sleep(0)
        try:
            self._i2c.writeto_then_readfrom(address, data, buffer)
        finally:
            self._i2c.unlock()

        return buffer

    def _check_device(self, address: int) -> bool:
        while not self._i2c.try_lock():
            time.sleep(0)
        try:
            self._i2c.writeto(address, b"")
        except OSError:
            try:
                result = bytearray(1)
                self._i2c.readfrom_into(address, result)
            except OSError:
                raise ValueError(f"No I2C device at address {hex(address)}.")
        finally:
            self._i2c.unlock()

    def _callback(self, channel: int) -> None:
        # Fetch new packet from the matched Nicla over I2C.
        name = self._nicla_pin_mapping[channel]
        buffer = bytearray(self._recv_buffer_size)
        raw_data = self._writeto_then_readfrom(
            self._nicla_mapping[name]["address"], NiclaI2cCommand.CMD_SEND, buffer
        )

        # Parse received bytes and push to the exo handler.
        toa_s = get_time()
        sample = NiclaData.from_bytes(raw_data)
        self._nicla_latest_data[name].append(sample)

        if (
            self._is_keep_data_event.is_set()
            and not self._is_stop_new_data_event.is_set()
        ):
            self._nicla_data_queue.put((name, toa_s, sample))

    async def connect(self):
        self._i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C devices found: ", [hex(i) for i in self._i2c.scan()], flush=True)

        # Validate that all devices have been found.
        while not all(
            map(
                lambda el: el["is_found"],
                (
                    found := {
                        name: {
                            "is_found": self._check_device(nicla["address"]),
                            "address": nicla["address"],
                        }
                        for name, nicla in self._nicla_mapping.items()
                    }
                ).values(),
            )
        ):
            not_found = " || ".join(
                [f"({name}: {nicla['address']})" for name, nicla in found.items()]
            )
            print(
                f"Couldn't find {not_found}.\n",
                "Make sure all Niclas have a unique I2C address and run correct firmware.",
                flush=True,
            )
            asyncio.sleep(5)

        # Register interrupts on the provided pins to retrieve in callbacks the corresponding ready sensor's data.
        for nicla in self._nicla_mapping.values():
            GPIO.add_event_detect(nicla["int_pin"], GPIO.RISING, self._callback)

        # Send command to sensors to start streaming.
        for nicla in self._nicla_mapping.values():
            self._writeto(nicla["address"], NiclaI2cCommand.CMD_START)

    async def run(self):
        while not self._is_cleanup_event.is_set():
            asyncio.sleep(10)
        # Unregister interrupts.
        for nicla in self._nicla_mapping.values():
            GPIO.remove_event_detect(nicla["int_pin"])

    async def cleanup(self):
        # Send command to sensors to stop streaming.
        for nicla in self._nicla_mapping.values():
            self._writeto(nicla["address"], NiclaI2cCommand.CMD_STOP)
        self._i2c.deinit()
