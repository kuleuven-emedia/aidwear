"""
Filename: hermes/nicla_sense_me/data_container.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-20
Version: 1.0
Description: HERMES DataContainer object for the management of the BLE data
    from the Nicla Sense ME devices.
"""

from collections import OrderedDict
from typing import Optional

from hermes.base.data_container import DataContainer


class NiclaSenseMeDataContainer(DataContainer):
    """A structure to store Nicla Sense ME stream's data."""

    def __init__(
        self,
        niclas: dict,
        buf_len: Optional[int] = 10000,
        **_,
    ) -> None:
        super().__init__()

        self._niclas = niclas

        self._define_data_notes()

        # Nicla Sense ME data.
        nicla_specs: dict = niclas["device_mapping"]
        for nicla_name in nicla_specs.keys():
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="toa_s",
                data_type="float64",
                sample_size=[1],
                buf_len=buf_len,
                sampling_rate_hz=niclas["sampling_rate_hz"],
                is_measure_rate_hz=True,
                data_notes=self._data_notes[f"nicla_{nicla_name}"]["toa_s"],
            )
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="timestamp",
                data_type="uint32",
                sample_size=[1],
                buf_len=buf_len,
                sampling_rate_hz=niclas["sampling_rate_hz"],
                data_notes=self._data_notes[f"nicla_{nicla_name}"]["timestamp"],
            )
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="sequence_id",
                data_type="uint32",
                sample_size=[1],
                buf_len=buf_len,
                sampling_rate_hz=niclas["sampling_rate_hz"],
                data_notes=self._data_notes[f"nicla_{nicla_name}"]["sequence_id"],
            )
            if niclas["is_acc"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="acceleration",
                    data_type="int16",
                    sample_size=[3],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["acceleration"],
                )
            if niclas["is_gyr"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="gyroscope",
                    data_type="int16",
                    sample_size=[3],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["gyroscope"],
                )
            if niclas["is_mag"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="magnetometer",
                    data_type="int16",
                    sample_size=[3],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["magnetometer"],
                )
            if niclas["is_euler"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="euler",
                    data_type="float32",
                    sample_size=[3],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["euler"],
                )
            if niclas["is_quat"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="quaternion",
                    data_type="float32",
                    sample_size=[4],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["quaternion"],
                )
            if niclas["is_temp"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="temperature",
                    data_type="float32",
                    sample_size=[1],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["temperature"],
                )
            if niclas["is_baro"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="pressure",
                    data_type="float32",
                    sample_size=[1],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["pressure"],
                )
            if niclas["is_hum"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="humidity",
                    data_type="float32",
                    sample_size=[1],
                    buf_len=buf_len,
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["humidity"],
                )

    def _define_data_notes(self) -> None:
        self._data_notes = {}
        for nicla_name in self._niclas["device_mapping"].keys():
            self._data_notes.setdefault(f"nicla_{nicla_name}", {})

        # Niclas IMU data.
        niclas: dict = self._niclas["device_mapping"]
        for nicla_name in niclas.keys():
            dev = f"nicla_{nicla_name}"
            self._data_notes[dev]["toa_s"] = OrderedDict(
                [
                    (
                        "Description",
                        "Time of arrival of the samples since Epoch w.r.t. system clock",
                    ),
                    ("Units", "seconds since Epoch"),
                ]
            )
            self._data_notes[dev]["timestamp"] = OrderedDict(
                [
                    (
                        "Description",
                        "Sensor time w.r.t. its onboard oscillator",
                    ),
                    (
                        "Units",
                        "number of ticks w.r.t. clock frequency of the onboard oscillator",
                    ),
                ]
            )
            self._data_notes[dev]["sequence_id"] = OrderedDict(
                [
                    (
                        "Description",
                        "Monotonically increasing sequence number of sent sensor samples, used to track drop out",
                    ),
                ]
            )
            self._data_notes[dev]["acceleration"] = OrderedDict(
                [
                    (
                        "Description",
                        "Local raw linear acceleration of the IMU [x,y,z]",
                    ),
                    (
                        "Units",
                        f"arbitrary units in the 16-bit range, for the {self._niclas['gravity_scaling_factor']} gravity factor",
                    ),
                    (
                        "Conversion formula",
                        f"{self._niclas['gravity_scaling_factor']} * 9.80665 / 32768.0 (m/s^2)",
                    ),
                ]
            )
            self._data_notes[dev]["gyroscope"] = OrderedDict(
                [
                    (
                        "Description",
                        "Local raw angular velocity of the IMU [x,y,z]",
                    ),
                    (
                        "Units",
                        f"arbitrary units in the 16-bit range, for the {self._niclas['gyroscope_scaling_factor']} gyroscope factor",
                    ),
                    (
                        "Conversion formula",
                        f"{self._niclas['gyroscope_scaling_factor']} / 32768.0 (degree/s)",
                    ),
                ]
            )
            self._data_notes[dev]["magnetometer"] = OrderedDict(
                [
                    (
                        "Description",
                        "Local raw magnetic field of the IMU [x,y,z]",
                    ),
                    (
                        "Units",
                        "arbitrary units in the 16-bit range w.r.t. magnetic field at the calibration site",
                    ),
                ]
            )
            self._data_notes[dev]["euler"] = OrderedDict(
                [
                    (
                        "Description",
                        "Absolute orientation of the sensor with respect to the global coordinate system, measured at the boot",
                    ),
                    ("Units", "degrees"),
                ]
            )
            self._data_notes[dev]["quaternion"] = OrderedDict(
                [
                    (
                        "Description",
                        "Quaternion orientation vector of the sensor with respect to the global coordinate system, measured at the boot",
                    ),
                    ("Units", "no physical units for quaternion orientation"),
                ]
            )
            self._data_notes[dev]["temperature"] = OrderedDict(
                [
                    (
                        "Description",
                        "Ambient temperature near the sensor",
                    ),
                    ("Units", "Celsius"),
                ]
            )
            self._data_notes[dev]["pressure"] = OrderedDict(
                [
                    (
                        "Description",
                        "Barometric pressure sensor",
                    ),
                    ("Units", "hPa"),
                ]
            )
            self._data_notes[dev]["humidity"] = OrderedDict(
                [
                    (
                        "Description",
                        "Relative humidity sensor",
                    ),
                    ("Units", "%"),
                ]
            )
