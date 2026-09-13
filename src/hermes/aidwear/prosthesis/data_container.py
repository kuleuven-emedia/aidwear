"""
Filename: hermes/aidwear/prosthesis/data_container.py
Description: HERMES DataContainer object for the management of the prosthesis
    internal data from the corresponding Pipeline Node.
"""

from collections import OrderedDict
from enum import Enum

from hermes.base.data_container import DataContainer

from .utils.types import (
    IntentCommandSource,
    ModeEnum,
    ServoCanPacketEnum,
    ServoErrorCode,
    StateEnum,
)


class ProsthesisDataContainer(DataContainer):
    def __init__(
        self,
        niclas: dict,
        motors: dict,
        telemetry: dict,
        **_,
    ) -> None:
        super().__init__()

        self._niclas = niclas
        self._motors = motors
        self._telemetry = telemetry

        self._define_data_notes()

        # Nicla Sense ME data.
        nicla_specs: dict = niclas["device_mapping"]
        for nicla_name in nicla_specs.keys():
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="toa_s",
                data_type="float64",
                sample_size=[1],
                buf_len=niclas["buf_len"],
                sampling_rate_hz=niclas["sampling_rate_hz"],
                is_measure_rate_hz=True,
                data_notes=self._data_notes[f"nicla_{nicla_name}"]["toa_s"],
            )
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="timestamp",
                data_type="uint32",
                sample_size=[1],
                buf_len=niclas["buf_len"],
                sampling_rate_hz=niclas["sampling_rate_hz"],
                data_notes=self._data_notes[f"nicla_{nicla_name}"]["timestamp"],
            )
            self.add_channel(
                bundle_name=f"nicla_{nicla_name}",
                channel_name="sequence_id",
                data_type="uint32",
                sample_size=[1],
                buf_len=niclas["buf_len"],
                sampling_rate_hz=niclas["sampling_rate_hz"],
                data_notes=self._data_notes[f"nicla_{nicla_name}"]["sequence_id"],
            )
            if niclas["is_acc"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="acceleration",
                    data_type="int16",
                    sample_size=[3],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["acceleration"],
                )
            if niclas["is_gyr"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="gyroscope",
                    data_type="int16",
                    sample_size=[3],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["gyroscope"],
                )
            if niclas["is_mag"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="magnetometer",
                    data_type="int16",
                    sample_size=[3],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["magnetometer"],
                )
            if niclas["is_euler"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="euler",
                    data_type="float32",
                    sample_size=[3],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["euler"],
                )
            if niclas["is_quat"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="quaternion",
                    data_type="float32",
                    sample_size=[4],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["quaternion"],
                )
            if niclas["is_temp"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="temperature",
                    data_type="float32",
                    sample_size=[1],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["temperature"],
                )
            if niclas["is_baro"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="pressure",
                    data_type="float32",
                    sample_size=[1],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["pressure"],
                )
            if niclas["is_hum"]:
                self.add_channel(
                    bundle_name=f"nicla_{nicla_name}",
                    channel_name="humidity",
                    data_type="float32",
                    sample_size=[1],
                    buf_len=niclas["buf_len"],
                    sampling_rate_hz=niclas["sampling_rate_hz"],
                    data_notes=self._data_notes[f"nicla_{nicla_name}"]["humidity"],
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
                sampling_rate_hz=motors["sampling_rate_hz"],
                is_measure_rate_hz=True,
                data_notes=self._data_notes[f"motor_{motor_name}"]["toa_s"],
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="position",
                data_type="float32",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=motors["sampling_rate_hz"],
                data_notes=self._data_notes[f"motor_{motor_name}"]["position"],
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="velocity",
                data_type="float32",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=motors["sampling_rate_hz"],
                data_notes=self._data_notes[f"motor_{motor_name}"]["velocity"],
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="current",
                data_type="float32",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=motors["sampling_rate_hz"],
                data_notes=self._data_notes[f"motor_{motor_name}"]["current"],
            )
            self.add_channel(
                bundle_name=f"motor_{motor_name}",
                channel_name="error",
                data_type="uint8",
                sample_size=[1],
                buf_len=motors["buf_len"],
                sampling_rate_hz=motors["sampling_rate_hz"],
                data_notes=self._data_notes[f"motor_{motor_name}"]["error"],
            )

        # Encoder data.
        for motor_name in motor_specs.keys():
            self.add_channel(
                bundle_name=f"encoder_{motor_name}",
                channel_name="toa_s",
                data_type="float64",
                sample_size=[1],
                buf_len=telemetry["buf_len"],
                data_notes=self._data_notes[f"encoder_{motor_name}"]["toa_s"],
            )
            self.add_channel(
                bundle_name=f"encoder_{motor_name}",
                channel_name="angle",
                data_type="float32",
                sample_size=[1],
                buf_len=telemetry["buf_len"],
                data_notes=self._data_notes[f"encoder_{motor_name}"]["angle"],
            )
            self.add_channel(
                bundle_name=f"encoder_{motor_name}",
                channel_name="is_error",
                data_type="bool",
                sample_size=[1],
                buf_len=telemetry["buf_len"],
                data_notes=self._data_notes[f"encoder_{motor_name}"]["is_error"],
            )

        # Motor commands.
        for motor_name in motor_specs.keys():
            self.add_channel(
                bundle_name=f"command_{motor_name}",
                channel_name="toa_s",
                data_type="float64",
                sample_size=[1],
                buf_len=telemetry["buf_len"],
                data_notes=self._data_notes[f"command_{motor_name}"]["toa_s"],
            )
            self.add_channel(
                bundle_name=f"command_{motor_name}",
                channel_name="control_mode",
                data_type=f"uint8",
                sample_size=[1],
                buf_len=telemetry["buf_len"],
                data_notes=self._data_notes[f"command_{motor_name}"]["control_mode"],
            )
            self.add_channel(
                bundle_name=f"command_{motor_name}",
                channel_name="data",
                data_type="V8",  # CubeMars motors driver receives up to 8 bytes payloads for control
                sample_size=[1],
                buf_len=telemetry["buf_len"],
                data_notes=self._data_notes[f"command_{motor_name}"]["data"],
            )
            self.add_channel(
                bundle_name=f"command_{motor_name}",
                channel_name="log_data",
                data_type="V20",  # Commands sent to CubeMars motors driver can contain up to 5x 4-byte floats
                sample_size=[1],
                buf_len=telemetry["buf_len"],
                data_notes=self._data_notes[f"command_{motor_name}"]["log_data"],
            )

        # Locomotion mode transitions (inter-mode).
        self.add_channel(
            bundle_name="mode",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["mode"]["toa_s"],
        )
        self.add_channel(
            bundle_name="mode",
            channel_name="mode",
            data_type="uint8",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["mode"]["mode"],
        )
        self.add_channel(
            bundle_name="mode",
            channel_name="sequence_id",
            data_type="uint32",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["mode"]["sequence_id"],
        )
        self.add_channel(
            bundle_name="mode",
            channel_name="source",
            data_type="uint8",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["mode"]["source"],
        )

        # State machine transitions (intra-mode).
        self.add_channel(
            bundle_name="state",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["state"]["toa_s"],
        )
        self.add_channel(
            bundle_name="state",
            channel_name="state",
            data_type="uint8",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["state"]["state"],
        )

        # Gait phase.
        self.add_channel(
            bundle_name="phase",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["phase"]["toa_s"],
        )
        self.add_channel(
            bundle_name="phase",
            channel_name="phase",
            data_type="float32",
            sample_size=[1],
            buf_len=telemetry["buf_len"],
            data_notes=self._data_notes["phase"]["phase"],
        )

        # Calibration event data.
        calibration_buf_len = 100
        self.add_channel(
            bundle_name="nicla_calibration",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=calibration_buf_len,
            data_notes=self._data_notes["nicla_calibration"]["toa_s"],
        )
        self.add_channel(
            bundle_name="nicla_calibration",
            channel_name="offsets",
            data_type="float32",
            sample_size=[len(nicla_specs)],
            buf_len=calibration_buf_len,
            data_notes=self._data_notes["nicla_calibration"]["offsets"],
        )

        self.add_channel(
            bundle_name="encoder_calibration",
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=calibration_buf_len,
            data_notes=self._data_notes["encoder_calibration"]["toa_s"],
        )
        self.add_channel(
            bundle_name="encoder_calibration",
            channel_name="offsets",
            data_type="float32",
            sample_size=[len(motor_specs)],
            buf_len=calibration_buf_len,
            data_notes=self._data_notes["encoder_calibration"]["offsets"],
        )

    def _define_data_notes(self) -> None:
        self._data_notes = {}
        self._data_notes.setdefault("mode", {})
        self._data_notes.setdefault("state", {})
        self._data_notes.setdefault("phase", {})
        self._data_notes.setdefault("nicla_calibration", {})
        self._data_notes.setdefault("encoder_calibration", {})
        for nicla_name in self._niclas["device_mapping"].keys():
            self._data_notes.setdefault(f"nicla_{nicla_name}", {})
        for motor_name in self._motors["device_mapping"].keys():
            self._data_notes.setdefault(f"motor_{motor_name}", {})
        for motor_name in self._motors["device_mapping"].keys():
            self._data_notes.setdefault(f"encoder_{motor_name}", {})
        for motor_name in self._motors["device_mapping"].keys():
            self._data_notes.setdefault(f"command_{motor_name}", {})

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

        # Motor incoming data.
        motors: dict = self._motors["device_mapping"]
        for motor_name in motors.keys():
            dev = f"motor_{motor_name}"
            self._data_notes[dev]["toa_s"] = OrderedDict(
                [
                    (
                        "Description",
                        "Time of arrival of the samples since Epoch w.r.t. system clock",
                    ),
                    ("Units", "seconds since Epoch"),
                ]
            )
            self._data_notes[dev]["position"] = OrderedDict(
                [
                    (
                        "Description",
                        "Encoder absolute position (angle) w.r.t. user-calibrated motor origin position [-3200, 3200]",
                    ),
                    ("Units", "degrees"),
                ]
            )
            self._data_notes[dev]["velocity"] = OrderedDict(
                [
                    (
                        "Description",
                        "Angular speed of the motor [-320000, 320000]",
                    ),
                    ("Units", "RPM"),
                ]
            )
            self._data_notes[dev]["current"] = OrderedDict(
                [
                    (
                        "Description",
                        "Encoder absolute position w.r.t. user-calibrated motor origin position [-60, 60]",
                    ),
                    ("Units", "Ampere"),
                ]
            )
            self._data_notes[dev]["error"] = OrderedDict(
                [
                    (
                        "Description",
                        "Error codes identifying internal state of the motor",
                    ),
                    # ("Error codes", [f"{e.name}: {e.value}" for e in ServoErrorCode]),
                ]
            )

        # Absolute joint encoder data. 
        for motor_name in motors.keys():
            dev = f"encoder_{motor_name}"
            self._data_notes[dev]["toa_s"] = OrderedDict(
                [
                    (
                        "Description",
                        "Time of arrival of the samples since Epoch w.r.t. system clock",
                    ),
                    ("Units", "seconds since Epoch"),
                ]
            )
            self._data_notes[dev]["angle"] = OrderedDict(
                [
                    (
                        "Description",
                        "Joint's absolute encoder position (angle) w.r.t. user-calibrated motor origin position.",
                    ),
                    ("Units", "degrees"),
                ]
            )
            self._data_notes[dev]["is_error"] = OrderedDict(
                [
                    (
                        "Description",
                        "Error codes identifying internal state of the motor",
                    ),
                    # ("Error codes", [f"{e.name}: {e.value}" for e in ServoErrorCode]),
                ]
            )

        # Motor control outgoing commands.
        motors: dict = self._motors["device_mapping"]
        for motor_name in motors.keys():
            dev = f"command_{motor_name}"
            self._data_notes[dev]["toa_s"] = OrderedDict(
                [
                    (
                        "Description",
                        "Time of arrival of the samples since Epoch w.r.t. system clock",
                    ),
                    ("Units", "seconds since Epoch"),
                ]
            )
            self._data_notes[dev]["control_mode"] = OrderedDict(
                [
                    (
                        "Description",
                        "Control modes of CubeMars motors identifying the loop in which the motor is commanded to operate",
                    ),
                    (
                        "Control modes",
                        [f"{e.name}: {e.value}" for e in ServoCanPacketEnum],
                    ),
                ]
            )
            self._data_notes[dev]["data"] = OrderedDict(
                [
                    (
                        "Description",
                        "Motor command recorded w.r.t present 'control_mode' as raw bytes in the big-endian format. Refer to CubeMars datasheet for interpretation in post-processing",
                    ),
                    (
                        "Bytes",
                        "Uses only the first 4 out of 8 bytes for most motor control modes (except position-velocity mode, which uses 8)",
                    ),
                ]
            )
            self._data_notes[dev]["log_data"] = OrderedDict(
                [
                    (
                        "Description",
                        "Readable motor commands sent to CubeMars motors w.r.t present 'control_mode' as raw bytes in the big-endian format. Float values mapping to joint control loops (e.g. impedance: target position, target velocity, and torque, current loop: target Iq current).",
                    ),
                ]
            )

        # Locomotion mode.
        self._data_notes["mode"]["toa_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Time of arrival of the samples since Epoch w.r.t. system clock",
                ),
                ("Units", "seconds since Epoch"),
            ]
        )
        self._data_notes["mode"]["mode"] = OrderedDict(
            [
                (
                    "Description",
                    "Actual taken locomotion mode transition of the mid-level exo controller",
                ),
                ("Ambulation modes", [f"{e.name}: {e.value.id}" for e in ModeEnum]),
            ]
        )
        self._data_notes["mode"]["sequence_id"] = OrderedDict(
            [
                (
                    "Description",
                    "Reference to upstream switch commands (i.e. manual CLI switch or specific AI prediction). Tracks which of the predictions exo actually enacted on while smoothing switching behavior in the presence of AI oversegmentation",
                ),
            ]
        )
        self._data_notes["mode"]["source"] = OrderedDict(
            [
                (
                    "Description",
                    "Command source that triggered the locomotion mode transition of the mid-level prosthesis controller",
                ),
                (
                    "Command sources",
                    [f"{e.name}: {e.value}" for e in IntentCommandSource],
                ),
            ]
        )

        # Intra-ambulation mode FSM.
        self._data_notes["state"]["toa_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Time of arrival of the samples since Epoch w.r.t. system clock",
                ),
                ("Units", "seconds since Epoch"),
            ]
        )
        self._data_notes["state"]["state"] = OrderedDict(
            [
                (
                    "Description",
                    "Substates of each ambulation mode mid-level state machine controller",
                ),
                (
                    "State machines",
                    [
                        f"{attr_value.__name__}.{s.name}: {s.value}"
                        for attr_value in vars(StateEnum).values()
                        if isinstance(attr_value, type) and issubclass(attr_value, Enum)
                        for s in attr_value
                    ],
                ),
            ]
        )

        # Intra-ambulation mode FSM.
        self._data_notes["phase"]["toa_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Time of arrival of the samples since Epoch w.r.t. system clock",
                ),
                ("Units", "seconds since Epoch"),
            ]
        )
        self._data_notes["phase"]["phase"] = OrderedDict(
            [
                (
                    "Description",
                    "State machine gait phase estimator. Used mostly in Walking",
                ),
            ]
        )

        # Nicla calibration.
        self._data_notes["nicla_calibration"]["toa_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Time of arrival of the Nicla calibration event since Epoch w.r.t. system clock",
                ),
                ("Units", "seconds since Epoch"),
            ]
        )
        self._data_notes["nicla_calibration"]["offsets"] = OrderedDict(
            [
                (
                    "Description",
                    "Measured Euler angle calibration offsets for each Nicla sensor in degrees",
                ),
                ("Units", "degrees"),
            ]
        )

        # Encoder calibration.
        self._data_notes["encoder_calibration"]["toa_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Time of arrival of the motor homing calibration completion event since Epoch w.r.t. system clock",
                ),
                ("Units", "seconds since Epoch"),
            ]
        )
        self._data_notes["encoder_calibration"]["offsets"] = OrderedDict(
            [
                (
                    "Description",
                    "Recorded absolute joint encoder angles at the home position in degrees",
                ),
                ("Units", "degrees"),
            ]
        )
