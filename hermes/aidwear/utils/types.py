"""
Filename: hermes/revalexo/exo/sensors/nicla/types.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2025-12-10
Version: 1.0
Description: Nicla Sense ME specific data types.
"""

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import struct
from typing import Optional
import numpy as np


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
    ACC     = MaskParsingTuple(mask=0x01, format="3h", key="acceleration", num_bytes=6)
    GYR     = MaskParsingTuple(mask=0x02, format="3h", key="gyroscope", num_bytes=6)
    MAG     = MaskParsingTuple(mask=0x04, format="3h", key="magnetometer", num_bytes=6)
    EULER   = MaskParsingTuple(mask=0x08, format="3f", key="euler", num_bytes=12)
    QUAT    = MaskParsingTuple(mask=0x10, format="4f", key="quaternion", num_bytes=16)
    TEMP    = MaskParsingTuple(mask=0x20, format="f", key="temperature", num_bytes=4)
    BARO    = MaskParsingTuple(mask=0x40, format="f", key="pressure", num_bytes=4)
    HUM     = MaskParsingTuple(mask=0x80, format="f", key="humidity", num_bytes=4)


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
    def from_bytes(cls, data: bytearray):
        mask, timestamp, sequence_id = struct.unpack_from("<BII", data)
        kwargs = {"timestamp": timestamp, "sequence_id": sequence_id}
        offset: int = 9

        for modality in NiclaPacketMask:
            if mask & modality.value.mask:
                val = struct.unpack_from(modality.value.format, data, offset)
                kwargs[modality.value.key] = val[0] if len(val) == 1 else val
                offset += modality.value.num_bytes
        return cls(**kwargs)


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
