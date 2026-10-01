"""
Filename: hermes/nicla_sense_me/dummy_producer.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-10-01
Version: 1.0
Description: Synthetic HERMES Producer Node for Nicla Sense ME IMUs.
    Generates realistic multimodal locomotion data (acceleration, gyroscope,
    euler angles, toa_s, etc.) for testing AI intent classification and
    visualization pipelines without requiring physical BLE hardware.
"""

from typing import Optional
import numpy as np

from hermes.base.nodes.producer import Producer
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_SYNC_HOST, PORT_KILL

from .data_container import NiclaSenseMeDataContainer


class DummyNiclaSenseMeProducer(Producer):
    """Synthetic Nicla Sense ME producer for testing AI inference and visualization without hardware."""

    def __init__(
        self,
        node_id: str,
        host_ip: str,
        niclas: dict,
        logging_spec: LoggingSpec,
        buf_len: Optional[int] = 10_000,
        port_pub: Optional[str] = PORT_BACKEND,
        port_sync: Optional[str] = PORT_SYNC_HOST,
        port_killsig: Optional[str] = PORT_KILL,
        transmit_delay_sample_period_s: Optional[float] = float("nan"),
        timesteps_before_solidified: Optional[int] = 0,
        **_,
    ):
        self._niclas = niclas
        self._sampling_rate_hz = float(niclas.get("sampling_rate_hz", 90))
        self._period = 1.0 / self._sampling_rate_hz
        self._device_mapping: dict[str, str] = niclas.get("device_mapping", {})
        self._nicla_names: list[str] = list(self._device_mapping.keys())

        # Feature flags matching config spec
        self._is_acc = niclas.get("is_acc", True)
        self._is_gyr = niclas.get("is_gyr", True)
        self._is_mag = niclas.get("is_mag", False)
        self._is_euler = niclas.get("is_euler", True)
        self._is_quat = niclas.get("is_quat", False)
        self._is_temp = niclas.get("is_temp", False)
        self._is_baro = niclas.get("is_baro", False)
        self._is_hum = niclas.get("is_hum", False)

        # Scale factors to convert real-world units to raw Nicla int16 counts
        # Nicla acceleration: 32768 counts = gravity_scaling_factor * 1g (9.80665 m/s^2)
        # Nicla gyroscope: 32768 counts = gyroscope_scaling_factor deg/s
        gravity_scaling = float(niclas.get("gravity_scaling_factor", 4))
        gyro_scaling = float(niclas.get("gyroscope_scaling_factor", 500))
        self._acc_counts_per_g = 32768.0 / gravity_scaling
        self._gyr_counts_per_degs = 32768.0 / gyro_scaling

        # Per-sensor sequence numbers and sub-millisecond clock offsets to emulate async BLE streams
        self._sequence_ids: dict[str, int] = {name: 0 for name in self._nicla_names}
        self._sensor_offsets: dict[str, float] = {
            name: (i * 0.0015) % self._period
            for i, name in enumerate(self._nicla_names)
        }

        self._start_time_s = get_time()
        self._next_period = self._start_time_s + self._period
        self._last_process_time = self._start_time_s

        data_out_spec = {
            "niclas": niclas,
            "buf_len": buf_len,
        }

        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            logging_spec=logging_spec,
            sampling_rate_hz=self._sampling_rate_hz,
            port_pub=port_pub,
            port_sync=port_sync,
            port_killsig=port_killsig,
            transmit_delay_sample_period_s=transmit_delay_sample_period_s,
        )

        # Register hierarchical and grouped topics for flexible subscription
        nicla_bundles = [f"nicla_{name}" for name in self._nicla_names]
        topic_map = {
            # Full telemetry / all Niclas
            "telemetry.all": nicla_bundles,
            "telemetry": nicla_bundles,
            "all": nicla_bundles,
            "data": nicla_bundles,
            # Nicla grouping
            "telemetry.nicla.all": nicla_bundles,
            "telemetry.nicla": nicla_bundles,
            "nicla.all": nicla_bundles,
            "nicla": nicla_bundles,
        }

        for name in self._nicla_names:
            topic_map[f"telemetry.nicla.{name}"] = [f"nicla_{name}"]
            topic_map[f"nicla.{name}"] = [f"nicla_{name}"]
            topic_map[f"nicla_{name}"] = [f"nicla_{name}"]

        if hasattr(self, "register_topic_map"):
            self.register_topic_map(topic_map)
        else:
            self._topic_map = topic_map

    @classmethod
    def create_data_container(cls, data_spec: dict) -> NiclaSenseMeDataContainer:
        return NiclaSenseMeDataContainer(**data_spec)

    def _ping_device(self) -> None:
        return None

    def _connect(self) -> bool:
        return True

    def _keep_samples(self) -> None:
        self._start_time_s = get_time()
        self._next_period = self._start_time_s + self._period
        self._last_process_time = self._start_time_s

    def _generate_sensor_batch(
        self, name: str, num_samples: int, process_time_s: float
    ) -> dict[str, np.ndarray]:
        """Generate a batch of synthetic IMU data samples for a specific sensor location."""
        # Calculate sample times with realistic async sensor offsets
        time_indices = np.arange(num_samples) - (num_samples - 1)
        sample_times = (
            process_time_s
            + time_indices * self._period
            + self._sensor_offsets.get(name, 0.0)
        )
        t = sample_times - self._start_time_s

        # Human walking cadence: ~0.9 Hz (approx 1.8 steps/sec)
        w = 2.0 * np.pi * 0.9

        # Limb phase handling (left leg is anti-phase relative to right leg)
        is_left = "left" in name.lower()
        phase = np.pi if is_left else 0.0

        if "thigh" in name.lower():
            # Thigh kinematics
            pitch_deg = 28.0 * np.sin(w * t + phase)
            roll_deg = 4.0 * np.cos(w * t + phase)
            yaw_deg = 3.0 * np.sin(w * t + phase)
            pitch_rate = 28.0 * w * np.cos(w * t + phase)
            roll_rate = -4.0 * w * np.sin(w * t + phase)
            yaw_rate = 3.0 * w * np.cos(w * t + phase)

            acc_x = 0.35 * np.cos(w * t + phase)
            acc_y = 0.15 * np.sin(w * t + phase)
            acc_z = 1.0 + 0.35 * np.sin(2.0 * w * t + phase)
        elif "shank" in name.lower():
            # Shank kinematics (knee swing lag)
            pitch_deg = 42.0 * np.sin(w * t + phase - 0.35)
            roll_deg = 3.0 * np.cos(w * t + phase)
            yaw_deg = 4.0 * np.sin(w * t + phase)
            pitch_rate = 42.0 * w * np.cos(w * t + phase - 0.35)
            roll_rate = -3.0 * w * np.sin(w * t + phase)
            yaw_rate = 4.0 * w * np.cos(w * t + phase)

            acc_x = 0.55 * np.cos(w * t + phase - 0.35)
            acc_y = 0.2 * np.sin(w * t + phase)
            acc_z = 1.0 + 0.5 * np.sin(2.0 * w * t + phase)
        elif "foot" in name.lower():
            # Foot kinematics
            pitch_deg = 30.0 * np.sin(w * t + phase - 0.6)
            roll_deg = 2.0 * np.cos(w * t + phase)
            yaw_deg = 3.0 * np.sin(w * t + phase)
            pitch_rate = 30.0 * w * np.cos(w * t + phase - 0.6)
            roll_rate = -2.0 * w * np.sin(w * t + phase)
            yaw_rate = 3.0 * w * np.cos(w * t + phase)

            acc_x = 0.7 * np.cos(w * t + phase - 0.6)
            acc_y = 0.2 * np.sin(w * t + phase)
            acc_z = 1.0 + 0.75 * np.sin(2.0 * w * t + phase)
        else:
            # Torso / pelvis trunk dynamics
            pitch_deg = 5.0 * np.sin(2.0 * w * t)
            roll_deg = 3.0 * np.cos(w * t)
            yaw_deg = 3.0 * np.sin(w * t)
            pitch_rate = 10.0 * w * np.cos(2.0 * w * t)
            roll_rate = -3.0 * w * np.sin(w * t)
            yaw_rate = 3.0 * w * np.cos(w * t)

            acc_x = 0.15 * np.cos(w * t)
            acc_y = 0.1 * np.sin(w * t)
            acc_z = 1.0 + 0.25 * np.sin(2.0 * w * t)

        bundle: dict[str, np.ndarray] = {
            "toa_s": sample_times.reshape(-1, 1).astype(np.float64),
            "sequence_id": np.arange(
                self._sequence_ids[name],
                self._sequence_ids[name] + num_samples,
                dtype=np.uint32,
            ).reshape(-1, 1),
            "timestamp": (np.mod(t * 1000.0, 2**32)).astype(np.uint32).reshape(-1, 1),
        }
        self._sequence_ids[name] = (self._sequence_ids[name] + num_samples) % (2**32)

        if self._is_acc:
            acc_raw = np.stack([acc_x, acc_y, acc_z], axis=1) * self._acc_counts_per_g
            acc_noise = np.random.normal(0.0, 30.0, size=acc_raw.shape)
            bundle["acceleration"] = np.clip(acc_raw + acc_noise, -32768, 32767).astype(
                np.int16
            )

        if self._is_gyr:
            gyr_raw = (
                np.stack([roll_rate, pitch_rate, yaw_rate], axis=1)
                * self._gyr_counts_per_degs
            )
            gyr_noise = np.random.normal(0.0, 20.0, size=gyr_raw.shape)
            bundle["gyroscope"] = np.clip(gyr_raw + gyr_noise, -32768, 32767).astype(
                np.int16
            )

        if self._is_euler:
            bundle["euler"] = np.stack([roll_deg, pitch_deg, yaw_deg], axis=1).astype(
                np.float32
            )

        if self._is_quat:
            # Euler to Quaternion (XYZ intrinsic)
            cr = np.cos(np.radians(roll_deg) * 0.5)
            sr = np.sin(np.radians(roll_deg) * 0.5)
            cp = np.cos(np.radians(pitch_deg) * 0.5)
            sp = np.sin(np.radians(pitch_deg) * 0.5)
            cy = np.cos(np.radians(yaw_deg) * 0.5)
            sy = np.sin(np.radians(yaw_deg) * 0.5)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            bundle["quaternion"] = np.stack([qw, qx, qy, qz], axis=1).astype(np.float32)

        if self._is_mag:
            bundle["magnetometer"] = np.tile(
                np.array([180, -120, 420], dtype=np.int16), (num_samples, 1)
            )

        if self._is_temp:
            bundle["temperature"] = (
                (24.0 + 0.1 * np.sin(0.1 * t)).reshape(-1, 1).astype(np.float32)
            )

        if self._is_baro:
            bundle["pressure"] = (
                (1013.25 + 0.2 * np.sin(0.05 * t)).reshape(-1, 1).astype(np.float32)
            )

        if self._is_hum:
            bundle["humidity"] = (
                (45.0 + 0.5 * np.cos(0.05 * t)).reshape(-1, 1).astype(np.float32)
            )

        return bundle

    def _process_data(self) -> None:
        if self._is_continue_capture:
            process_time_s = get_time()
            if process_time_s >= self._next_period:
                num_due = int((process_time_s - self._last_process_time) / self._period)
                num_due = max(1, min(num_due, 4))
                self._last_process_time += num_due * self._period
                self._next_period = self._last_process_time + self._period

                packet = {}
                for name in self._nicla_names:
                    packet[f"nicla_{name}"] = self._generate_sensor_batch(
                        name, num_due, process_time_s
                    )

                if packet:
                    self._publish(process_time_s=process_time_s, new_data=packet)
        else:
            self._send_end_packet()

    def _stop_new_data(self) -> None:
        pass

    def _cleanup(self) -> None:
        super()._cleanup()
