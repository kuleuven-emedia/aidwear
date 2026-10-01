"""
Filename: hermes/aidwear/visualizer/producer.py
Description: HERMES Producer Node that generates synthetic multimodal prosthesis
    telemetry, joint encoder angles, live AI intent predictions, and factual
    prosthesis operating modes to drive the visualizer GUI without real hardware.
"""

from typing import Optional
import numpy as np

from hermes.base.nodes.producer import Producer
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec
from hermes.utils.zmq_utils import (
    PORT_BACKEND,
    PORT_KILL,
    PORT_SYNC_HOST,
)

from hermes.aidwear.prosthesis.utils.types import ModeEnum, IntentCommandSource
from .data_container import VisualizerDummyDataContainer


class DummyProducer(Producer):
    """HERMES Producer Node that generates synthetic telemetry, AI intent predictions,

    and prosthesis mode transitions to drive the visualizer without requiring real hardware.
    """

    def __init__(
        self,
        node_id: str,
        host_ip: str,
        logging_spec: LoggingSpec,
        sampling_rate_hz: float = 25.0,
        buf_len: int = 1000,
        num_classes: int = 8,
        port_pub: Optional[str] = PORT_BACKEND,
        port_sync: Optional[str] = PORT_SYNC_HOST,
        port_killsig: Optional[str] = PORT_KILL,
        transmit_delay_sample_period_s: Optional[float] = float("nan"),
        **_,
    ):
        sampling_rate_hz = float(sampling_rate_hz)
        buf_len = int(buf_len)
        num_classes = int(num_classes)

        self._sampling_rate_hz = sampling_rate_hz
        self._period = 1.0 / sampling_rate_hz
        self._buf_len = buf_len
        self._num_classes = num_classes
        self._step = 0
        self._start_time_s = get_time()
        self._next_period = self._start_time_s + self._period

        data_out_spec = {
            "sampling_rate_hz": sampling_rate_hz,
            "buf_len": buf_len,
            "num_classes": num_classes,
        }

        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            logging_spec=logging_spec,
            sampling_rate_hz=sampling_rate_hz,
            port_pub=port_pub,
            port_sync=port_sync,
            port_killsig=port_killsig,
            transmit_delay_sample_period_s=transmit_delay_sample_period_s,
        )

        nicla_locs = ["torso", "thigh_right", "thigh_left", "shank_right", "shank_left"]
        motor_joints = ["knee", "ankle"]

        nicla_bundles = [f"nicla_{loc}" for loc in nicla_locs]
        motor_bundles = [f"motor_{joint}" for joint in motor_joints]
        encoder_bundles = [f"encoder_{joint}" for joint in motor_joints]
        all_telemetry = nicla_bundles + motor_bundles + encoder_bundles + ["intent", "mode"]

        topic_map = {
            # Full telemetry
            "telemetry.all": all_telemetry,
            "telemetry": all_telemetry,
            "all": all_telemetry,
            "data": all_telemetry,

            # Nicla grouping
            "telemetry.nicla.all": nicla_bundles,
            "telemetry.nicla": nicla_bundles,
            "nicla.all": nicla_bundles,
            "nicla": nicla_bundles,

            # Motor grouping
            "telemetry.motor.all": motor_bundles,
            "telemetry.motor": motor_bundles,
            "motor.all": motor_bundles,
            "motor": motor_bundles,

            # Encoder grouping
            "telemetry.encoder.all": encoder_bundles,
            "telemetry.encoder": encoder_bundles,
            "encoder.all": encoder_bundles,
            "encoder": encoder_bundles,

            # Intent and mode
            "telemetry.intent": ["intent"],
            "intent": ["intent"],
            "telemetry.mode": ["mode"],
            "mode": ["mode"],
        }

        for loc in nicla_locs:
            topic_map[f"telemetry.nicla.{loc}"] = [f"nicla_{loc}"]
            topic_map[f"nicla.{loc}"] = [f"nicla_{loc}"]
            topic_map[f"nicla_{loc}"] = [f"nicla_{loc}"]

        for joint in motor_joints:
            topic_map[f"telemetry.motor.{joint}"] = [f"motor_{joint}"]
            topic_map[f"motor.{joint}"] = [f"motor_{joint}"]
            topic_map[f"motor_{joint}"] = [f"motor_{joint}"]

            topic_map[f"telemetry.encoder.{joint}"] = [f"encoder_{joint}"]
            topic_map[f"encoder.{joint}"] = [f"encoder_{joint}"]
            topic_map[f"encoder_{joint}"] = [f"encoder_{joint}"]

        self.register_topic_map(topic_map)

    @classmethod
    def create_data_container(cls, data_spec: dict) -> VisualizerDummyDataContainer:
        return VisualizerDummyDataContainer(**data_spec)

    def _ping_device(self) -> None:
        return None

    def _connect(self) -> bool:
        return True

    def _keep_samples(self) -> None:
        self._start_time_s = get_time()
        self._next_period = self._start_time_s + self._period

    def _process_data(self) -> None:
        if self._is_continue_capture:
            process_time_s = get_time()
            if self._next_period <= process_time_s:
                t = process_time_s - self._start_time_s
                sample_time = process_time_s

                # Simulated mode cycle:
                # 0-6s: Walking (1)
                # 6-12s: Stair Ascent (3)
                # 12-18s: Walking (1)
                # 18-24s: Sit to Stand (2)
                # 24-30s: Stair Descent (4)
                # 30-36s: Hurdle (5)
                cycle_t = t % 36.0
                if cycle_t < 6.0:
                    current_mode = ModeEnum.WALKING.value.id
                    top_class_idx = 0  # Level-Ground Walking
                elif cycle_t < 12.0:
                    current_mode = ModeEnum.STAIR_ASCENT.value.id
                    top_class_idx = 2  # Stair Ascent
                elif cycle_t < 18.0:
                    current_mode = ModeEnum.WALKING.value.id
                    top_class_idx = 0  # Level-Ground Walking
                elif cycle_t < 24.0:
                    current_mode = ModeEnum.SIT_TO_STAND.value.id
                    top_class_idx = 0
                elif cycle_t < 30.0:
                    current_mode = ModeEnum.STAIR_DESCENT.value.id
                    top_class_idx = 3  # Stair Descent
                else:
                    current_mode = ModeEnum.HURDLE.value.id
                    top_class_idx = 1  # Hurdles

                # Generate probability distribution for intent predictions
                probs = np.full(self._num_classes, 0.02, dtype=np.float32)
                if top_class_idx < self._num_classes:
                    probs[top_class_idx] = float(0.75 + 0.15 * np.sin(t * 1.5))
                # Add slight secondary prediction oscillation
                secondary_idx = (top_class_idx + 2) % self._num_classes
                probs[secondary_idx] = float(0.08 + 0.05 * np.cos(t * 1.5))
                # Normalize probabilities
                probs = probs / np.sum(probs)

                logits = np.log(np.clip(probs, 1e-6, 1.0)).astype(np.float32)

                packet = {
                    "nicla_torso": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "euler": np.array(
                            [[np.sin(t * 2.0), np.cos(t * 1.5), np.sin(t * 3.0)]],
                            dtype=np.float32,
                        )
                        * 30.0,
                    },
                    "nicla_thigh_right": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "euler": np.array(
                            [[np.sin(t * 2.2), np.cos(t * 1.7), np.sin(t * 2.8)]],
                            dtype=np.float32,
                        )
                        * 45.0,
                    },
                    "nicla_thigh_left": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "euler": np.array(
                            [
                                [
                                    np.sin(t * 2.2 + 1.0),
                                    np.cos(t * 1.7 + 1.0),
                                    np.sin(t * 2.8 + 1.0),
                                ]
                            ],
                            dtype=np.float32,
                        )
                        * 45.0,
                    },
                    "nicla_shank_right": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "euler": np.array(
                            [[np.sin(t * 3.0), np.cos(t * 2.5), np.sin(t * 3.5)]],
                            dtype=np.float32,
                        )
                        * 60.0,
                    },
                    "nicla_shank_left": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "euler": np.array(
                            [
                                [
                                    np.sin(t * 3.0 + 1.0),
                                    np.cos(t * 2.5 + 1.0),
                                    np.sin(t * 3.5 + 1.0),
                                ]
                            ],
                            dtype=np.float32,
                        )
                        * 60.0,
                    },
                    "motor_knee": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "position": np.array(
                            [[np.sin(t * 1.8) * 5.0]], dtype=np.float32
                        ),
                        "velocity": np.array(
                            [[np.cos(t * 1.8) * 120.0]], dtype=np.float32
                        ),
                        "current": np.array(
                            [[1.5 + 0.8 * np.sin(t * 4.0)]], dtype=np.float32
                        ),
                    },
                    "motor_ankle": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "position": np.array(
                            [[np.sin(t * 2.5) * 3.0]], dtype=np.float32
                        ),
                        "velocity": np.array(
                            [[np.cos(t * 2.5) * 80.0]], dtype=np.float32
                        ),
                        "current": np.array(
                            [[1.2 + 0.6 * np.cos(t * 3.5)]], dtype=np.float32
                        ),
                    },
                    "encoder_knee": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "angle": np.array(
                            [[30.0 + 25.0 * np.sin(t * 1.8)]], dtype=np.float32
                        ),
                    },
                    "encoder_ankle": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "angle": np.array(
                            [[10.0 + 15.0 * np.sin(t * 2.5)]], dtype=np.float32
                        ),
                    },
                    "intent": {
                        "predictions": probs[None, :],
                        "logits": logits[None, :],
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "compute_time_s": np.array(
                            [[0.0125 + 0.003 * np.sin(t * 5.0)]], dtype=np.float64
                        ),
                        "sequence_id": np.array([[self._step]], dtype=np.uint32),
                    },
                    "mode": {
                        "toa_s": np.array([[sample_time]], dtype=np.float64),
                        "mode": np.array([[current_mode]], dtype=np.uint8),
                        "sequence_id": np.array([[self._step]], dtype=np.uint32),
                        "source": np.array([[IntentCommandSource.AI.value]], dtype=np.uint8),
                    },
                }

                self._publish(process_time_s=process_time_s, new_data=packet)
                self._step += 1
                self._next_period += self._period
                if self._next_period < process_time_s:
                    self._next_period = process_time_s + self._period
        else:
            self._send_end_packet()

    def _stop_new_data(self) -> None:
        pass

    def _cleanup(self) -> None:
        super()._cleanup()
