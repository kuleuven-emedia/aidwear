"""
Filename: hermes/nicla_sense_me/stream.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-20
Version: 1.0
Description: HERMES Stream object for the management of the BLE data
    from the Nicla Sense ME devices.
"""

from hermes.base.stream import Stream


class NiclaSenseMeStream(Stream):
    """A structure to store Nicla Sense ME stream's data."""

    def __init__(
        self,
        niclas: dict,
        **_
    ) -> None:
        super().__init__()

        # TODO: use details to add HDF5 metadata.

        sample_rate_niclas = niclas["sampling_rate_hz"]

        # Nicla Sense ME data.
        nicla_specs: dict = niclas["device_mapping"]
        for nicla_name in nicla_specs.keys():
            self.add_stream(
                device_name=f"nicla_{nicla_name}",
                stream_name="toa_s",
                data_type="float64",
                sample_size=(1,),
                sampling_rate_hz=sample_rate_niclas,
                is_measure_rate_hz=True,
            )
            self.add_stream(
                device_name=f"nicla_{nicla_name}",
                stream_name="timestamp",
                data_type="uint32",
                sample_size=(1,),
                sampling_rate_hz=sample_rate_niclas,
            )
            self.add_stream(
                device_name=f"nicla_{nicla_name}",
                stream_name="sequence_id",
                data_type="uint32",
                sample_size=(1,),
                sampling_rate_hz=sample_rate_niclas,
            )
            self.add_stream(
                device_name=f"nicla_{nicla_name}",
                stream_name="count",
                data_type="uint16",
                sample_size=(1,),
                sampling_rate_hz=sample_rate_niclas,
            )
            if niclas["is_acc"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="acceleration",
                    data_type="int16",
                    sample_size=(3,),
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_gyr"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="gyroscope",
                    data_type="int16",
                    sample_size=(3,),
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_mag"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="magnetometer",
                    data_type="int16",
                    sample_size=(3,),
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_euler"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="euler",
                    data_type="float32",
                    sample_size=(3,),
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_quat"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="quaternion",
                    data_type="float32",
                    sample_size=(4,),
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_temp"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="temperature",
                    data_type="float32",
                    sample_size=(1,),
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_baro"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="pressure",
                    data_type="float32",
                    sample_size=(1,),
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_hum"]:
                self.add_stream(
                    device_name=f"nicla_{nicla_name}",
                    stream_name="humidity",
                    data_type="float32",
                    sample_size=(1,),
                    sampling_rate_hz=sample_rate_niclas,
                )

    def get_fps(self) -> dict[str, float | None]:
        return {
            "nicla": super()._get_fps("nicla", "toa_s"),
        }
