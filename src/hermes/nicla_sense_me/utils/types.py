"""
Filename: hermes/aidwear/prosthesis/sensors/nicla/types.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2025-12-10
Version: 1.0
Description: Nicla Sense ME specific data types.
"""

from __future__ import annotations

from typing import List
from typing import Dict
from multiprocessing import Value
from multiprocessing.sharedctypes import Synchronized
from dataclasses import field
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import struct
from typing import Optional, Union, Any
import numpy as np
from multiprocessing import Lock
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.synchronize import Lock as _Lock


class CalibrationEventType(Enum):
    NICLA = "nicla"
    ENCODER = "encoder"


@dataclass
class CalibrationEvent:
    timestamp: float
    sensor_type: CalibrationEventType
    offsets: dict[str, float]


class NiclaLocation(Enum):
    TORSO = "torso"
    PELVIS = "pelvis"
    THIGH_RIGHT = "thigh_right"
    THIGH_LEFT = "thigh_left"
    SHANK_RIGHT = "shank_right"
    SHANK_LEFT = "shank_left"
    FOOT_RIGHT = "foot_right"
    FOOT_LEFT = "foot_left"


@dataclass
class NiclaMappingFull:
    torso: str
    pelvis: str
    thigh_right: str
    thigh_left: str
    shank_right: str
    shank_left: str
    foot_right: str
    foot_left: str


@dataclass
class NiclaMappingNoPelvisAndFeet:
    torso: str
    thigh_right: str
    thigh_left: str
    shank_right: str
    shank_left: str


@dataclass
class NiclaMappingPelvisAndFeet:
    pelvis: str
    foot_right: str
    foot_left: str


@dataclass
class MaskParsingTuple:
    mask: int
    format: str
    key: str
    num_bytes: int


class NiclaI2cCommand(Enum):
    CMD_START = 0xF0
    CMD_SEND = 0xAA
    CMD_STOP = 0x0F
    CMD_DEFAULT = 0x00
    CMD_DEFAULT_REPLY = 0xFF


class NiclaConnectionType(Enum):
    BLE = 0
    I2C = 1


class NiclaPacketMask(Enum):
    ACC = MaskParsingTuple(mask=0x01, format="3h", key="acceleration", num_bytes=6)
    GYR = MaskParsingTuple(mask=0x02, format="3h", key="gyroscope", num_bytes=6)
    MAG = MaskParsingTuple(mask=0x04, format="3h", key="magnetometer", num_bytes=6)
    EULER = MaskParsingTuple(mask=0x08, format="3f", key="euler", num_bytes=12)
    QUAT = MaskParsingTuple(mask=0x10, format="4f", key="quaternion", num_bytes=16)
    TEMP = MaskParsingTuple(mask=0x20, format="f", key="temperature", num_bytes=4)
    BARO = MaskParsingTuple(mask=0x40, format="f", key="pressure", num_bytes=4)
    HUM = MaskParsingTuple(mask=0x80, format="f", key="humidity", num_bytes=4)


NICLA_HEADER_FORMAT: str = "<BII"
NICLA_HEADER_SIZE: int = struct.calcsize(
    NICLA_HEADER_FORMAT
)  # 9 bytes: mask (1B), timestamp (4B), sequence_id (4B)

NICLA_CONFIG_TO_PACKET_MASK: dict[str, NiclaPacketMask] = {
    "is_acc": NiclaPacketMask.ACC,
    "is_gyr": NiclaPacketMask.GYR,
    "is_mag": NiclaPacketMask.MAG,
    "is_euler": NiclaPacketMask.EULER,
    "is_quat": NiclaPacketMask.QUAT,
    "is_temp": NiclaPacketMask.TEMP,
    "is_baro": NiclaPacketMask.BARO,
    "is_hum": NiclaPacketMask.HUM,
}


def calculate_nicla_sample_size(nicla_config: dict[str, Any]) -> int:
    """Calculates the expected total raw packet size in bytes for a Nicla sensor.

    Fixed 9-byte header (modality mask, timestamp, sequence_id) plus the payload size
    of each enabled modality defined in the nicla config specification.
    """
    return NICLA_HEADER_SIZE + sum(
        mask.value.num_bytes
        for key, mask in NICLA_CONFIG_TO_PACKET_MASK.items()
        if nicla_config.get(key, False)
    )


_PARSER_CACHE: dict[int, Callable[[Any], "NiclaData"]] = {}


def _create_packet_parser(mask: int) -> Callable[[Any], "NiclaData"]:
    fmt = NICLA_HEADER_FORMAT
    field_specs: list[tuple[str, int]] = []
    for modality in NiclaPacketMask:
        if mask & modality.value.mask:
            fmt += modality.value.format
            count = (
                int(modality.value.format[:-1])
                if modality.value.format[:-1].isdigit()
                else 1
            )
            field_specs.append((modality.value.key, count))
    compiled_struct = struct.Struct(fmt)

    def parser(data: Union[bytes, bytearray]) -> "NiclaData":
        vals = compiled_struct.unpack_from(data)
        kwargs = {"timestamp": vals[1], "sequence_id": vals[2]}
        idx = 3
        for key, count in field_specs:
            if count == 1:
                kwargs[key] = vals[idx]
            else:
                kwargs[key] = vals[idx : idx + count]
            idx += count
        return NiclaData(**kwargs)

    return parser


@dataclass
class NiclaData:
    """Data class for representing Nicla Sense ME sensor data.

    The `from_bytes` method parses raw bytes received from the Nicla devices into structured data fields.
    It uses the packet's header's mask to determine which sensor modalities are included in the packet and unpacks the data accordingly.

    The expected format of the incoming packets is:
        Fixed-length header (9 bytes):
            [0]: modality mask (1 byte).
            [1-4]: timestamp (4 bytes).
            [5-8]: sequence_id (4 bytes).
        Variable-length body:
            [9-...]: sensor data (variable length based on modalities included, determined by the mask).

    Accelerometer range is expected to be in units of g (1g = ~9.80665 m/s^2). Possible settings:
        2 (+/-2g), 4 (+/-4g), 8 (+/-8g), 16 (+/-16g).
    Gyroscope range is expected to be in units of dps (degrees per second). Possible settings:
        125 (+/-125 dps), 250 (+/-250 dps), 500 (+/-500 dps), 1000 (+/-1000 dps), 2000 (+/-2000 dps).

    Acceleration: [x,y,z] in g.
    Gyroscope: [x,y,z] in dps.
    Magnetometer: [x,y,z] in uT.
    Euler angles: [pitch,roll,heading] in degrees.
    Quaternion: [w,x,y,z] unitless (Hamilton convention).
    Temperature: in degrees Celsius.
    Pressure: in hPa.
    Humidity: in %.
    """

    timestamp: int
    sequence_id: int
    acceleration: Optional[tuple[int, int, int]] = None
    gyroscope: Optional[tuple[int, int, int]] = None
    magnetometer: Optional[tuple[int, int, int]] = None
    euler: Optional[tuple[float, float, float]] = None
    quaternion: Optional[tuple[float, float, float, float]] = None
    temperature: Optional[float] = None
    pressure: Optional[float] = None
    humidity: Optional[float] = None

    @classmethod
    def from_bytes(cls, data: Union[bytes, bytearray]) -> "NiclaData":
        mask = data[0]
        parser = _PARSER_CACHE.get(mask)
        if parser is None:
            parser = _create_packet_parser(mask)
            _PARSER_CACHE[mask] = parser
        return parser(data)


@dataclass(frozen=True)
class NiclaSampleSynchronizedMetadata:
    size: int
    name: str
    lock: _Lock


class NiclaSampleSynchronized:
    def __init__(
        self,
        size: int,
        name: Optional[str] = None,
        lock: Optional[_Lock] = None,
    ):
        if name is not None:
            assert lock is not None
            self._lock = lock
            self._shm = SharedMemory(
                name=name, size=size
            )  # NOTE: creates a 4096 shared mem
            self.size = size
        else:
            self._lock = Lock()
            self._shm = SharedMemory(create=True, size=size)
            self._shm.buf[:] = bytes(size)
            self.size = size

    @classmethod
    def from_metadata(
        cls, metadata: NiclaSampleSynchronizedMetadata
    ) -> "NiclaSampleSynchronized":
        return cls(
            size=metadata.size,
            name=metadata.name,
            lock=metadata.lock,
        )

    def get_metadata(self) -> NiclaSampleSynchronizedMetadata:
        return NiclaSampleSynchronizedMetadata(
            size=self.size,
            name=self._shm.name,
            lock=self._lock,
        )

    @property
    def data(self) -> NiclaData:
        with self._lock:
            return NiclaData.from_bytes(data=self._shm.buf[: self.size])

    @data.setter
    def data(self, raw_data: bytearray) -> None:
        with self._lock:
            self._shm.buf[: self.size] = raw_data

    def close(self) -> None:
        self._shm.close()

    def unlink(self) -> None:
        self._shm.unlink()


@dataclass
class NiclaOffsetsSynchronized:
    _device_names: List[str]
    lock: _Lock = field(init=False)
    offsets: "Dict[str, Synchronized[float]]" = field(init=False)

    def __post_init__(self):
        self.lock = Lock()
        self.offsets = dict(
            map(
                lambda dev_name: (dev_name, Value("f", np.nan, lock=False)),
                self._device_names,
            )
        )

    def is_calibrated(self) -> bool:
        with self.lock:
            return not any(np.isnan(v.value) for v in self.offsets.values())


@dataclass
class NiclaNumpyGetter:
    func: Callable[[list[NiclaData]], np.ndarray]


class NiclaDataGetMethods(Enum):
    acceleration = NiclaNumpyGetter(
        func=lambda data: np.array(
            list(map(lambda n: n.acceleration, data)), dtype=np.int16
        )
    )
    gyroscope = NiclaNumpyGetter(
        func=lambda data: np.array(
            list(map(lambda n: n.gyroscope, data)), dtype=np.int16
        )
    )
    magnetometer = NiclaNumpyGetter(
        func=lambda data: np.array(
            list(map(lambda n: n.magnetometer, data)), dtype=np.int16
        )
    )
    euler = NiclaNumpyGetter(
        func=lambda data: np.array(list(map(lambda n: n.euler, data)), dtype=np.float32)
    )
    quaternion = NiclaNumpyGetter(
        func=lambda data: np.array(
            list(map(lambda n: n.quaternion, data)), dtype=np.float32
        )
    )
    temperature = NiclaNumpyGetter(
        func=lambda data: np.array(
            list(map(lambda n: n.temperature, data)), dtype=np.float32
        )
    )
    pressure = NiclaNumpyGetter(
        func=lambda data: np.array(
            list(map(lambda n: n.pressure, data)), dtype=np.float32
        )
    )
    humidity = NiclaNumpyGetter(
        func=lambda data: np.array(
            list(map(lambda n: n.humidity, data)), dtype=np.float32
        )
    )


class NiclaPayloadMode:
    def __init__(
        self,
        is_acc: bool,
        is_gyr: bool,
        is_mag: bool,
        is_euler: bool,
        is_quat: bool,
        is_temp: bool,
        is_baro: bool,
        is_hum: bool,
    ):
        self._data_getters: dict[str, Callable[[list[NiclaData]], np.ndarray]] = {}

        if is_acc:
            self._data_getters[NiclaDataGetMethods.acceleration.name] = (
                NiclaDataGetMethods.acceleration.value.func
            )
        if is_gyr:
            self._data_getters[NiclaDataGetMethods.gyroscope.name] = (
                NiclaDataGetMethods.gyroscope.value.func
            )
        if is_mag:
            self._data_getters[NiclaDataGetMethods.magnetometer.name] = (
                NiclaDataGetMethods.magnetometer.value.func
            )
        if is_euler:
            self._data_getters[NiclaDataGetMethods.euler.name] = (
                NiclaDataGetMethods.euler.value.func
            )
        if is_quat:
            self._data_getters[NiclaDataGetMethods.quaternion.name] = (
                NiclaDataGetMethods.quaternion.value.func
            )
        if is_temp:
            self._data_getters[NiclaDataGetMethods.temperature.name] = (
                NiclaDataGetMethods.temperature.value.func
            )
        if is_baro:
            self._data_getters[NiclaDataGetMethods.pressure.name] = (
                NiclaDataGetMethods.pressure.value.func
            )
        if is_hum:
            self._data_getters[NiclaDataGetMethods.humidity.name] = (
                NiclaDataGetMethods.humidity.value.func
            )

    def get_data_getters(self) -> dict[str, Callable[[list[NiclaData]], np.ndarray]]:
        return self._data_getters
