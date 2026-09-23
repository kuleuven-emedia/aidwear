"""
Filename: hermes/nicla_sense_me/utils/abstract_backend.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-02
Version: 1.0
Description: Abstract class to interface the Nicla Sense ME devices.
"""

from hermes.utils.time_utils import init_time
import asyncio
from abc import ABC, abstractmethod


class NiclaBackend(ABC):
    @abstractmethod
    def main() -> None:
        pass

    def __call__(self) -> None:
        init_time(ref_time=self._ref_time_s)
        asyncio.run(self.main())
