"""
Filename: hermes/revalexo/exo/stream.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-01-04
Version: 1.0
Description: HERMES Stream object for the management of the exoskeleton
    internal data from the corresponding Pipeline Node.
"""

from hermes.base.data_container import DataContainer


class ProsthesisDataContainer(DataContainer):
    def __init__(
        self,
        niclas: dict,
        motors: dict,
        telemetry: dict,
        **_,
    ) -> None:
        super().__init__()

        # TODO: use details to add HDF5 metadata.
        sample_rate_niclas = niclas["sampling_rate_hz"]
        sample_rate_motors = motors["sampling_rate_hz"]

        # Nicla Sense ME data.
        nicla_specs: dict = niclas["device_mapping"]
        for nicla_name in nicla_specs.keys():
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="toa_s",
                data_type="float64",
                sample_size=(1,),
                buf_len=niclas["buf_len"],
                sampling_rate_hz=sample_rate_niclas,
            )
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="timestamp",
                data_type="uint32",
                sample_size=(1,),
                buf_len=niclas["buf_len"],
                sampling_rate_hz=sample_rate_niclas,
            )
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="sequence_id",
                data_type="uint32",
                sample_size=(1,),
                buf_len=niclas["buf_len"],
                sampling_rate_hz=sample_rate_niclas,
            )
            if niclas["is_acc"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="acceleration",
                    data_type="int16",
                    sample_size=(3,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_gyr"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="gyroscope",
                    data_type="int16",
                    sample_size=(3,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_mag"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="magnetometer",
                    data_type="int16",
                    sample_size=(3,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_euler"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="euler",
                    data_type="float32",
                    sample_size=(3,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_quat"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="quaternion",
                    data_type="float32",
                    sample_size=(4,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_temp"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="temperature",
                    data_type="float32",
                    sample_size=(1,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_baro"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="pressure",
                    data_type="float32",
                    sample_size=(1,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )
            if niclas["is_hum"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="humidity",
                    data_type="float32",
                    sample_size=(1,),
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=sample_rate_niclas,
                )

        # Motor data.
        motor_specs: dict = motors["device_mapping"]
        for motor_name in motor_specs.keys():
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="toa_s",
                data_type="float64",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=sample_rate_motors,
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="position",
                data_type="float32",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=sample_rate_motors,
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="velocity",
                data_type="float32",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=sample_rate_motors,
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="current",
                data_type="float32",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=sample_rate_motors,
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="temperature",
                data_type="int8",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=sample_rate_motors,
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="error",
                data_type="uint8",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=sample_rate_motors,
            )

        # Motor commands.
        for motor_name in motor_specs.keys():
            self.add_channel(
                bundle_name=f"command_{motor_name}",
                channel_name="toa_s",
                data_type="float64",
                sample_size=[1],
                buf_len=telemetry["buf_len"],
            )
            self.add_channel(
                bundle_name=f"command_{motor_name}",
                channel_name="control_mode",
                data_type=f"uint8",
                sample_size=[1],
                buf_len=telemetry["buf_len"],
            )
            self.add_channel(
                bundle_name=f"command_{motor_name}",
                channel_name="data",
                data_type=f"V8",  # CubeMars motors driver receives up to 8 bytes payloads for control
                sample_size=[1],
                buf_len=telemetry["buf_len"],
            )

        # Locomotion mode transitions, w/ reference to upstream switch command sequence id.
        self.add_channel(
            bundle_name="mode",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
        )
        self.add_channel(
            bundle_name="mode",
            channel_name="mode",
            data_type="uint8",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
        )
        self.add_channel(
            bundle_name="mode",
            channel_name="sequence_id",
            data_type="uint32",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
        )

        # State machine transitions (intra-mode).
        self.add_channel(
            bundle_name="state",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
        )
        self.add_channel(
            bundle_name="state",
            channel_name="state",
            data_type="uint8",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
        )

        # State machine transitions (intra-mode).
        self.add_channel(
            bundle_name="phase",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
        )
        self.add_channel(
            bundle_name="phase",
            channel_name="phase",
            data_type="float32",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
        )

    def get_fps(self) -> dict[str, float | None]:
        return {
            "nicla": super()._get_fps("nicla", "toa_s"),
            "motor": super()._get_fps("motor", "timestamp"),
        }
