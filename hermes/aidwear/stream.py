from hermes.base.stream import Stream


class CyberlegStream(Stream):
    def __init__(
        self,
        niclas: dict,
        motors: dict,
        **_,
    ) -> None:
        super().__init__()

        # Nicla Sense ME data.
        num_niclas = len(niclas["device_mapping"])
        sample_rate_niclas = niclas["nicla_sampling_rate_hz"]
        num_motors = len(motors["device_mapping"])
        sample_rate_motors = motors["nicla_sampling_rate_hz"]

        self.add_stream(
            device_name="nicla",
            stream_name="toa_s",
            data_type="float64",
            sample_size=[num_niclas, 1],
            sampling_rate_hz=sample_rate_niclas,
            is_measure_rate_hz=True,
        )
        self.add_stream(
            device_name="nicla",
            stream_name="timestamp",
            data_type="uint32",
            sample_size=[num_niclas, 1],
            sampling_rate_hz=sample_rate_niclas,
        )
        self.add_stream(
            device_name="nicla",
            stream_name="sequence_id",
            data_type="uint32",
            sample_size=[num_niclas, 1],
            sampling_rate_hz=sample_rate_niclas,
        )
        if niclas["is_acc"]:
            self.add_stream(
                device_name="nicla",
                stream_name="acceleration",
                data_type="int16",
                sample_size=[num_niclas, 3],
                sampling_rate_hz=sample_rate_niclas,
            )
        if niclas["is_gyr"]:
            self.add_stream(
                device_name="nicla",
                stream_name="gyroscope",
                data_type="int16",
                sample_size=[num_niclas, 3],
                sampling_rate_hz=sample_rate_niclas,
            )
        if niclas["is_mag"]:
            self.add_stream(
                device_name="nicla",
                stream_name="magnetometer",
                data_type="int16",
                sample_size=[num_niclas, 3],
                sampling_rate_hz=sample_rate_niclas,
            )
        if niclas["is_euler"]:
            self.add_stream(
                device_name="nicla",
                stream_name="euler",
                data_type="float32",
                sample_size=[num_niclas, 3],
                sampling_rate_hz=sample_rate_niclas,
            )
        if niclas["is_quat"]:
            self.add_stream(
                device_name="nicla",
                stream_name="quaternion",
                data_type="float32",
                sample_size=[num_niclas, 4],
                sampling_rate_hz=sample_rate_niclas,
            )
        if niclas["is_temp"]:
            self.add_stream(
                device_name="nicla",
                stream_name="temperature",
                data_type="float32",
                sample_size=[num_niclas, 1],
                sampling_rate_hz=sample_rate_niclas,
            )
        if niclas["is_baro"]:
            self.add_stream(
                device_name="nicla",
                stream_name="pressure",
                data_type="float32",
                sample_size=[num_niclas, 1],
                sampling_rate_hz=sample_rate_niclas,
            )
        if niclas["is_hum"]:
            self.add_stream(
                device_name="nicla",
                stream_name="humidity",
                data_type="float32",
                sample_size=[num_niclas, 1],
                sampling_rate_hz=sample_rate_niclas,
            )

        # Motor data.
        self.add_stream(
            device_name="motor",
            stream_name="toa_s",
            data_type="float64",
            sample_size=[num_motors, 1],
            sampling_rate_hz=sample_rate_motors,
            is_measure_rate_hz=True,
        )
        self.add_stream(
            device_name="motor",
            stream_name="timestamp",
            data_type="float64",
            sample_size=[num_motors, 1],
            sampling_rate_hz=sample_rate_motors,
        )
        self.add_stream(
            device_name="motor",
            stream_name="position",
            data_type="float32",
            sample_size=[num_motors, 1],
            sampling_rate_hz=sample_rate_motors,
        )
        self.add_stream(
            device_name="motor",
            stream_name="velocity",
            data_type="float32",
            sample_size=[num_motors, 1],
            sampling_rate_hz=sample_rate_motors,
        )
        self.add_stream(
            device_name="motor",
            stream_name="acceleration",
            data_type="float32",
            sample_size=[num_motors, 1],
            sampling_rate_hz=sample_rate_motors,
        )
        self.add_stream(
            device_name="motor",
            stream_name="temperature",
            data_type="int8",
            sample_size=[num_motors, 1],
            sampling_rate_hz=sample_rate_motors,
        )
        self.add_stream(
            device_name="motor",
            stream_name="error",
            data_type="uint8",
            sample_size=[num_motors, 1],
            sampling_rate_hz=sample_rate_motors,
        )

        # Control.
        self.add_stream(
            device_name="mode",
            stream_name="toa_s",
            data_type="float64",
            sample_size=[1],
        )
        self.add_stream(
            device_name="mode",
            stream_name="timestamp",
            data_type="float32",
            sample_size=[1],
        )
        self.add_stream(
            device_name="mode",
            stream_name="sequence_id",
            data_type="uint32",
            sample_size=[1],
        )
        self.add_stream(
            device_name="mode",
            stream_name="task",
            data_type="uint8",
            sample_size=[1],
        )

        self.add_stream(
            device_name="phase",
            stream_name="toa_s",
            data_type="float64",
            sample_size=[1],
        )
        self.add_stream(
            device_name="phase",
            stream_name="timestamp",
            data_type="float32",
            sample_size=[1],
        )
        self.add_stream(
            device_name="phase",
            stream_name="cycle",
            data_type="uint8",
            sample_size=[1],
        )


    def get_fps(self) -> dict[str, float | None]:
        return {
            "nicla": super()._get_fps("nicla", "toa_s"),
            "motor": super()._get_fps("motor", "toa_s")
        }
