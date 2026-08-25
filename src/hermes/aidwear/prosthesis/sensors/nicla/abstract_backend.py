"""
Filename: hermes/revalexo/exo/sensors/nicla/abstract_backend.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-02
Version: 1.0
Description: Abstract class to interface the Nicla Sense ME devices.
"""

from abc import ABC, abstractmethod


class NiclaBackend(ABC):
    @abstractmethod
    async def connect(self):
        pass

    @abstractmethod
    async def run(self):
        pass

    @abstractmethod
    async def cleanup(self):
        pass
