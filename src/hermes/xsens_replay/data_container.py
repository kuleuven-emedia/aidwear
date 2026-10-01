"""
HERMES Stream definition for Xsens MVN data replayed from recorded datasets.

Declares the data bundles and channels for:
- "xsens_motion_trackers": raw 3D IMU data (acceleration, gyroscope, quaternion,
  free acceleration, magnetometer, counter, time_since_start_s, toa_s).
- "xsens_pose": 3D pose data (position, quaternion, counter, time_since_start_s, toa_s).
"""

from collections import OrderedDict
from typing import Optional

from hermes.base.data_container import DataContainer

# Standard Xsens 17-sensor layout headings
DEFAULT_SENSOR_HEADINGS = [
    "Pelvis",
    "T8",
    "Head",
    "Right Shoulder",
    "Right Upper Arm",
    "Right Forearm",
    "Right Hand",
    "Left Shoulder",
    "Left Upper Arm",
    "Left Forearm",
    "Left Hand",
    "Right Upper Leg",
    "Right Lower Leg",
    "Right Foot",
    "Left Upper Leg",
    "Left Lower Leg",
    "Left Foot",
]

# Standard Xsens 23-segment layout headings
DEFAULT_SEGMENT_HEADINGS = [
    "Pelvis",
    "L5",
    "L3",
    "T12",
    "T8",
    "Neck",
    "Head",
    "Right Shoulder",
    "Right Upper Arm",
    "Right Forearm",
    "Right Hand",
    "Left Shoulder",
    "Left Upper Arm",
    "Left Forearm",
    "Left Hand",
    "Right Upper Leg",
    "Right Lower Leg",
    "Right Foot",
    "Right Toe",
    "Left Upper Leg",
    "Left Lower Leg",
    "Left Foot",
    "Left Toe",
]


class ImuReplayDataContainer(DataContainer):
    """Data container for replayed Xsens MVN data matching raw IMU and 3D pose formats."""

    def __init__(
        self,
        num_sensors: int = 17,
        num_segments: int = 23,
        sampling_rate_hz: int = 60,
        buf_len: int = 3_000,
        sensor_headings: Optional[list[str]] = None,
        segment_headings: Optional[list[str]] = None,
        **_,
    ) -> None:
        super().__init__(**_)

        self._num_sensors = num_sensors
        self._num_segments = num_segments
        self._sensor_headings = sensor_headings or DEFAULT_SENSOR_HEADINGS[:num_sensors]
        self._segment_headings = (
            segment_headings or DEFAULT_SEGMENT_HEADINGS[:num_segments]
        )

        self._define_data_notes()

        # ==================== xsens_motion_trackers Bundle ====================
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="acceleration",
            data_type="float32",
            sample_size=(num_sensors, 3),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_motion_trackers"]["acceleration"],
        )
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="gyroscope",
            data_type="float32",
            sample_size=(num_sensors, 3),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_motion_trackers"]["gyroscope"],
        )
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="quaternion",
            data_type="float32",
            sample_size=(num_sensors, 4),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_motion_trackers"]["quaternion"],
        )
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="free_acceleration",
            data_type="float32",
            sample_size=(num_sensors, 3),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_motion_trackers"]["free_acceleration"],
        )
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="magnetometer",
            data_type="float32",
            sample_size=(num_sensors, 3),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_motion_trackers"]["magnetometer"],
        )
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="counter",
            data_type="uint32",
            sample_size=(1,),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_motion_trackers"]["counter"],
        )
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="time_since_start_s",
            data_type="float64",
            sample_size=(1,),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_motion_trackers"]["time_since_start_s"],
        )
        self.add_channel(
            bundle_name="xsens_motion_trackers",
            channel_name="toa_s",
            data_type="float64",
            sample_size=(1,),
            buf_len=buf_len,
            data_notes=self._data_notes["xsens_motion_trackers"]["toa_s"],
        )

        # ==================== xsens_pose Bundle ====================
        self.add_channel(
            bundle_name="xsens_pose",
            channel_name="position",
            data_type="float32",
            sample_size=(num_segments, 3),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_pose"]["position"],
        )
        self.add_channel(
            bundle_name="xsens_pose",
            channel_name="quaternion",
            data_type="float32",
            sample_size=(num_segments, 4),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_pose"]["quaternion"],
        )
        self.add_channel(
            bundle_name="xsens_pose",
            channel_name="counter",
            data_type="uint32",
            sample_size=(1,),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_pose"]["counter"],
        )
        self.add_channel(
            bundle_name="xsens_pose",
            channel_name="time_since_start_s",
            data_type="float64",
            sample_size=(1,),
            buf_len=buf_len,
            sampling_rate_hz=sampling_rate_hz,
            data_notes=self._data_notes["xsens_pose"]["time_since_start_s"],
        )
        self.add_channel(
            bundle_name="xsens_pose",
            channel_name="toa_s",
            data_type="float64",
            sample_size=(1,),
            buf_len=buf_len,
            data_notes=self._data_notes["xsens_pose"]["toa_s"],
        )

    def _define_data_notes(self) -> None:
        self._data_notes = {}
        self._data_notes.setdefault("xsens_motion_trackers", {})
        self._data_notes.setdefault("xsens_pose", {})

        # Motion Trackers
        self._data_notes["xsens_motion_trackers"]["acceleration"] = OrderedDict(
            [
                ("Description", "Local raw linear acceleration of the IMU"),
                ("Units", "meter/second^2"),
                (DataContainer.metadata_data_headings_key, self._sensor_headings),
            ]
        )
        self._data_notes["xsens_motion_trackers"]["gyroscope"] = OrderedDict(
            [
                ("Description", "Local raw angular velocity of the IMU"),
                ("Units", "meter/second"),
                (DataContainer.metadata_data_headings_key, self._sensor_headings),
            ]
        )
        self._data_notes["xsens_motion_trackers"]["quaternion"] = OrderedDict(
            [
                (
                    "Description",
                    "Quaternion orientation vector of the sensor with respect to the global coordinate system",
                ),
                (DataContainer.metadata_data_headings_key, self._sensor_headings),
            ]
        )
        self._data_notes["xsens_motion_trackers"]["free_acceleration"] = OrderedDict(
            [
                (
                    "Description",
                    "Local linear acceleration of the IMU, with the gravitational component subtracted",
                ),
                ("Units", "meter/second^2"),
                (DataContainer.metadata_data_headings_key, self._sensor_headings),
            ]
        )
        self._data_notes["xsens_motion_trackers"]["magnetometer"] = OrderedDict(
            [
                ("Description", "Local raw magnetic field of the IMU"),
                ("Units", "a.u. w.r.t. magnetic field at the calibration site"),
                (DataContainer.metadata_data_headings_key, self._sensor_headings),
            ]
        )
        self._data_notes["xsens_motion_trackers"]["counter"] = OrderedDict(
            [("Description", "Index of the sample provisioned by MVN Analyze")]
        )
        self._data_notes["xsens_motion_trackers"]["time_since_start_s"] = OrderedDict(
            [
                ("Description", "MVN timecode from the datagram metadata"),
                ("Units", "seconds"),
            ]
        )
        self._data_notes["xsens_motion_trackers"]["toa_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Time of arrival of the multi-sensor snapshot w.r.t. synchronized system clock",
                ),
                ("Units", "seconds"),
            ]
        )

        # 3D Pose
        self._data_notes["xsens_pose"]["position"] = OrderedDict(
            [
                (
                    "Description",
                    "Global position of segments in the Z-up right-handed coordinate system",
                ),
                ("Units", "cm"),
                (DataContainer.metadata_data_headings_key, self._segment_headings),
            ]
        )
        self._data_notes["xsens_pose"]["quaternion"] = OrderedDict(
            [
                (
                    "Description",
                    "Global orientation of segments as unit quaternions in the Z-up right-handed coordinate system",
                ),
                (DataContainer.metadata_data_headings_key, self._segment_headings),
            ]
        )
        self._data_notes["xsens_pose"]["counter"] = OrderedDict(
            [("Description", "Index of the sample provisioned by MVN Analyze")]
        )
        self._data_notes["xsens_pose"]["time_since_start_s"] = OrderedDict(
            [
                ("Description", "MVN timecode from the datagram metadata"),
                ("Units", "seconds"),
            ]
        )
        self._data_notes["xsens_pose"]["toa_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Time of arrival of the 3D pose snapshot w.r.t. synchronized system clock",
                ),
                ("Units", "seconds"),
            ]
        )
