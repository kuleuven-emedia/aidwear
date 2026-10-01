"""
HERMES Producer for replaying recorded Xsens MVN MoCap datasets.

Loads .hdf5 files from the KU Leuven dataset and streams frame-by-frame:
- Raw IMU data bundle ("xsens_motion_trackers"): 3D acceleration, gyroscope, quaternion,
  free acceleration, magnetometer, counter, time_since_start_s, toa_s.
- 3D pose bundle ("xsens_pose"): 3D position, quaternion, counter, time_since_start_s, toa_s.

Published at the original 60 Hz rate into the HERMES pipeline.
"""

import time
from typing import Optional
import h5py
import numpy as np

from hermes.base.nodes.producer import Producer
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_KILL, PORT_SYNC_HOST

from .data_container import ImuReplayDataContainer


class ImuReplayProducer(Producer):
    """Replays recorded Xsens MoCap data from .hdf5 files as a HERMES Producer."""

    def __init__(
        self,
        node_id: str,
        host_ip: str,
        logging_spec: LoggingSpec,
        file_path: str,
        sampling_rate_hz: int = 60,
        sensor_indices: Optional[list[int]] = None,
        segment_indices: Optional[list[int]] = None,
        buf_len: int = 3_000,
        is_loop: bool = True,
        port_pub: str = PORT_BACKEND,
        port_sync: str = PORT_SYNC_HOST,
        port_killsig: str = PORT_KILL,
        **_,
    ):
        self._sensor_indices = sensor_indices
        self._segment_indices = segment_indices
        self._is_loop = is_loop
        self._sampling_rate_hz = sampling_rate_hz
        self._period = 1.0 / sampling_rate_hz
        self._next_period: float = 0.0

        (
            self._mt_data,
            self._pose_data,
            self._start_counter,
            self._end_counter,
            num_sensors,
            num_segments,
        ) = self._load_dataset(file_path, sensor_indices, segment_indices)

        self._current_counter: int = self._start_counter

        data_out_spec = {
            "num_sensors": num_sensors,
            "num_segments": num_segments,
            "sampling_rate_hz": sampling_rate_hz,
            "buf_len": buf_len,
        }

        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            logging_spec=logging_spec,
            port_pub=port_pub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

        # Map common subscriber topic aliases to the replayed bundles
        self._topic_map = {
            self.node_id: ["xsens_motion_trackers", "xsens_pose"],
            "all": ["xsens_motion_trackers", "xsens_pose"],
            "data": ["xsens_motion_trackers", "xsens_pose"],
            "mvn": ["xsens_motion_trackers", "xsens_pose"],
            "xsens_motion_trackers": ["xsens_motion_trackers"],
            "xsens_pose": ["xsens_pose"],
            "xsens_pose_quaternion": ["xsens_pose"],
        }

    def _load_dataset(
        self,
        file_path: str,
        sensor_indices: Optional[list[int]] = None,
        segment_indices: Optional[list[int]] = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], int, int, int, int]:
        """Loads and prepares raw IMU and 3D pose data from an HDF5 dataset file."""
        with h5py.File(file_path, "r") as f:
            root = (
                "mvn-analyze"
                if "mvn-analyze" in f
                else "mvn"
                if "mvn" in f
                else list(f.keys())[0]
            )
            grp = f[root]

            # 1. Motion Trackers (Raw IMU data)
            mt_grp_name = (
                "xsens-motion-trackers"
                if "xsens-motion-trackers" in grp
                else "xsens_motion_trackers"
            )
            mt_grp = grp[mt_grp_name]
            acc = np.array(mt_grp["acceleration"], dtype=np.float32)
            gyr = np.array(mt_grp["gyroscope"], dtype=np.float32)
            mt_counter = np.array(mt_grp["counter"], dtype=np.uint32)
            mt_toa_s = np.array(mt_grp["process_time_s"], dtype=np.float64)
            mt_time = np.array(mt_grp["time_since_start_s"], dtype=np.float64)

            # 2. 3D Pose
            pose_grp_name = "xsens-pose" if "xsens-pose" in grp else "xsens_pose"
            pose_grp = grp[pose_grp_name]
            pos = np.array(pose_grp["position"], dtype=np.float32)
            pose_quat = np.array(pose_grp["quaternion"], dtype=np.float32)
            pose_counter = np.array(pose_grp["counter"], dtype=np.uint32)
            pose_toa_s = np.array(pose_grp["process_time_s"], dtype=np.float64)
            pose_time = np.array(pose_grp["time_since_start_s"], dtype=np.float64)

        # Slice selected sensors if requested
        if sensor_indices is not None:
            acc = acc[:, sensor_indices, :]
            gyr = gyr[:, sensor_indices, :]

        # Slice selected segments if requested
        if segment_indices is not None:
            pos = pos[:, segment_indices, :]
            pose_quat = pose_quat[:, segment_indices, :]

        # 1. Get the largest of the 2 starting counter values and smallest ending counter value
        mt_end_counter, _ = np.argwhere(mt_counter == 0)[0]
        pose_end_counter, _ = np.argwhere(pose_counter == 0)[0]

        mt_counter = mt_counter.flatten()[:mt_end_counter]
        pose_counter = pose_counter.flatten()[:pose_end_counter]

        start_counter = min(mt_counter[0], pose_counter[0])
        end_counter = max(mt_counter[-1], pose_counter[-1])

        # 2. Crop each corresponding series using the counter bounds
        # Motion Trackers
        acc = acc[:mt_end_counter]
        gyr = gyr[:mt_end_counter]
        mt_time = mt_time[:mt_end_counter]
        mt_toa_s = mt_toa_s[:mt_end_counter]

        # 3D Pose
        pos = pos[:pose_end_counter]
        pose_quat = pose_quat[:pose_end_counter]
        pose_counter = pose_counter[:pose_end_counter]
        pose_time = pose_time[:pose_end_counter]
        pose_toa_s = pose_toa_s[:pose_end_counter]

        num_sensors = acc.shape[1]
        num_segments = pos.shape[1]

        mt_data = {
            "acceleration": acc,
            "gyroscope": gyr,
            "counter": mt_counter,
            "time_since_start_s": mt_time,
            "toa_s": mt_toa_s,
        }

        pose_data = {
            "position": pos,
            "quaternion": pose_quat,
            "counter": pose_counter,
            "time_since_start_s": pose_time,
            "toa_s": pose_toa_s,
        }

        return mt_data, pose_data, start_counter, end_counter, num_sensors, num_segments

    @classmethod
    def create_data_container(cls, data_spec: dict) -> ImuReplayDataContainer:
        return ImuReplayDataContainer(**data_spec)

    def _ping_device(self) -> None:
        return None

    def _connect(self) -> bool:
        return True

    def _keep_samples(self) -> None:
        self._next_period = get_time() + self._period

    def _process_data(self) -> None:
        if not self._is_continue_capture:
            self._send_end_packet()
            return

        if self._current_counter > self._end_counter:
            self._current_counter = self._start_counter
            print("Xsens dataset replay looping back to start...", flush=True)

        # Check if the counter wrapped around or reset in the remaining recorded data
        mt_idx = np.argwhere(self._mt_data["counter"] == self._current_counter)
        pose_idx = np.argwhere(self._pose_data["counter"] == self._current_counter)
        toa_s_arr = np.array([[get_time()]], dtype=np.float64)

        new_data = {}

        if mt_idx.shape[0]:
            id = mt_idx[0].item()
            mt_payload = {
                k: v[id : id + 1] for k, v in self._mt_data.items() if k != "toa_s"
            }
            mt_payload["toa_s"] = toa_s_arr
            new_data["xsens_motion_trackers"] = mt_payload

        if pose_idx.shape[0]:
            id = pose_idx[0].item()
            pose_payload = {
                k: v[id : id + 1] for k, v in self._pose_data.items() if k != "toa_s"
            }
            pose_payload["toa_s"] = toa_s_arr
            new_data["xsens_pose"] = pose_payload

        self._current_counter += 1

        # Rate limiting to 60 Hz
        while (toa_s := get_time()) < self._next_period:
            time.sleep((self._next_period - toa_s) * 0.9)

        if new_data:
            self._publish(process_time_s=toa_s, new_data=new_data)

        self._next_period += self._period
        if self._next_period < toa_s - self._period:
            self._next_period = toa_s + self._period

    def _stop_new_data(self):
        self._send_end_packet()

    def _cleanup(self) -> None:
        super()._cleanup()
