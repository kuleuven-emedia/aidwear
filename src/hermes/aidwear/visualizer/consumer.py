"""
Filename: hermes/aidwear/visualizer/consumer.py
Description: HERMES visualization Consumer Node.
    Streams prosthesis multimodal telemetry (5 IMUs, 2 Motors, 2 Joint Encoders)
    and visualizes it in real-time using PyQt6 and PyQtGraph with a rolling time window.

    Architectural Pattern:
        - `VisualizerConsumer` - interface HERMES framework to injest multimodal data.
        - `VisualizerGuiHandler` - runs the PyQt6 GUI inside a dedicated subprocess
                and relays data via Queue.
        - `VisualizerMainWindow` - live plots the PyQt6 dashboard.
"""

from multiprocessing import Event, Process, Queue
from queue import Full
import time
from typing import Optional
import numpy as np

from hermes.base.nodes.consumer import Consumer

from hermes.utils.mp_utils import launch_handler
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec, NewData
from hermes.utils.zmq_utils import (
    PORT_FRONTEND,
    PORT_KILL,
    PORT_SYNC_HOST,
)

from hermes.aidwear.visualizer.handler import VisualizerGuiHandler
from hermes.aidwear.prosthesis.utils.types import EncoderId, MotorId
from hermes.nicla_sense_me.utils.types import NiclaLocation


class VisualizerConsumer(Consumer):
    """Real-time PyQt6 + PyQtGraph visualization consumer node for prosthesis multimodal telemetry."""

    IMU_LOCATIONS: list[NiclaLocation] = [
        NiclaLocation.TORSO,
        NiclaLocation.THIGH_RIGHT,
        NiclaLocation.THIGH_LEFT,
        NiclaLocation.SHANK_RIGHT,
        NiclaLocation.SHANK_LEFT,
    ]
    MOTOR_IDS: list[MotorId] = [MotorId.KNEE, MotorId.ANKLE]
    ENCODER_IDS: list[EncoderId] = [EncoderId.KNEE, EncoderId.ANKLE]

    NICLA_BY_NAME: dict[str, NiclaLocation] = {loc.value: loc for loc in IMU_LOCATIONS}
    MOTOR_BY_NAME: dict[str, MotorId] = {m.name.lower(): m for m in MOTOR_IDS}
    ENCODER_BY_NAME: dict[str, EncoderId] = {e.name.lower(): e for e in ENCODER_IDS}

    IMU_NAMES: list[str] = [loc.value for loc in IMU_LOCATIONS]
    MOTOR_NAMES: list[str] = [m.name.lower() for m in MOTOR_IDS]
    ENCODER_NAMES: list[str] = [e.name.lower() for e in ENCODER_IDS]

    def __init__(
        self,
        node_id: str,
        host_ip: str,
        data_in_specs: list[dict],
        logging_spec: LoggingSpec,
        port_sub: Optional[str] = PORT_FRONTEND,
        port_sync: Optional[str] = PORT_SYNC_HOST,
        port_killsig: Optional[str] = PORT_KILL,
        history_len: int = 150,
        draw_interval_s: float = 0.04,
        dark_mode: bool = True,
        time_window_s: float = 5.0,
        **kwargs,
    ):
        """Constructor of the VisualizerConsumer Node.

        Spawns the dedicated GUI process in accordance with HERMES architecture.
        """
        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_in_specs=data_in_specs,
            logging_spec=logging_spec,
            port_sub=port_sub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

        self._history_len = history_len
        self._draw_interval_s = draw_interval_s
        self._dark_mode = dark_mode
        self.time_window_s = time_window_s

        # Dedicated IPC Queue and Cleanup Event for the GUI subprocess
        self._data_queue = Queue(maxsize=1000)
        self._is_gui_cleanup = Event()
        self._is_windows_closed_event = Event()
        self._is_babykill_sent = False

        # Extract class names if provided in ai_intent data_in_specs
        class_names = None
        for spec in data_in_specs:
            settings = spec.get("settings", {})
            if "classes" in settings and isinstance(settings["classes"], dict):
                sorted_classes = sorted(settings["classes"].items(), key=lambda x: x[1])
                class_names = [name for name, _ in sorted_classes]
                break

        # Spawn the GUI subprocess via HERMES launch_handler
        self._gui_proc = Process(
            target=launch_handler,
            args=(VisualizerGuiHandler,),
            kwargs={
                "node_id": self.node_id,
                "data_queue": self._data_queue,
                "is_cleanup_event": self._is_gui_cleanup,
                "is_windows_closed_event": self._is_windows_closed_event,
                "time_window_s": self.time_window_s,
                "draw_interval_s": self._draw_interval_s,
                "dark_mode": self._dark_mode,
                "class_names": class_names,
            },
        )
        self._gui_proc.start()

    def _process_data(self, topic: str, msg: NewData) -> None:
        """Relay incoming multimodal data packet to the GUI subprocess."""
        try:
            self._data_queue.put_nowait(msg)
        except Full:
            # Drop frame if visualizer queue is saturated to never throttle sensing
            pass
        finally:
            if not self._is_babykill_sent and self._is_windows_closed_event.is_set():
                self._send_kill_to_broker()
                self._is_babykill_sent = True

    def _cleanup(self) -> None:
        self._is_gui_cleanup.set()
        try:
            self._data_queue.put_nowait(None)
        except Exception:
            pass

        if hasattr(self, "_gui_proc") and self._gui_proc.is_alive():
            self._gui_proc.join(timeout=3.0)
            if self._gui_proc.is_alive():
                self._gui_proc.terminate()

        if hasattr(self, "_sync"):
            super()._cleanup()


if __name__ == "__main__":
    # Demo / standalone testing runner
    print("Launching Visualizer in standalone demo mode...")
    now = get_time()
    dummy_logging_spec = LoggingSpec(
        log_dir="./logs",
        experiment={"name": "test"},
        log_time_s=now,
        ref_time_s=now,
    )

    vis = VisualizerConsumer(
        node_id="visualizer_demo",
        host_ip="127.0.0.1",
        data_in_specs=[],
        logging_spec=dummy_logging_spec,
        time_window_s=5.0,
    )

    # Feed simulated multimodal data for 2.5 seconds
    t0 = get_time()
    for step in range(500):
        t = get_time() - t0
        sample_time = t0 + t
        packet = {
            "nicla_torso": {
                "toa_s": np.array([sample_time]),
                "euler": np.array([[np.sin(t * 2.0), np.cos(t * 1.5), np.sin(t * 3.0)]])
                * 30.0,
            },
            "nicla_thigh_right": {
                "toa_s": np.array([sample_time]),
                "euler": np.array([[np.sin(t * 2.2), np.cos(t * 1.7), np.sin(t * 2.8)]])
                * 45.0,
            },
            "nicla_thigh_left": {
                "toa_s": np.array([sample_time]),
                "euler": np.array(
                    [
                        [
                            np.sin(t * 2.2 + 1.0),
                            np.cos(t * 1.7 + 1.0),
                            np.sin(t * 2.8 + 1.0),
                        ]
                    ]
                )
                * 45.0,
            },
            "nicla_shank_right": {
                "toa_s": np.array([sample_time]),
                "euler": np.array([[np.sin(t * 3.0), np.cos(t * 2.5), np.sin(t * 3.5)]])
                * 60.0,
            },
            "nicla_shank_left": {
                "toa_s": np.array([sample_time]),
                "euler": np.array(
                    [
                        [
                            np.sin(t * 3.0 + 1.0),
                            np.cos(t * 2.5 + 1.0),
                            np.sin(t * 3.5 + 1.0),
                        ]
                    ]
                )
                * 60.0,
            },
            "motor_knee": {
                "toa_s": np.array([sample_time]),
                "position": np.array([np.sin(t * 1.8) * 5.0]),
                "velocity": np.array([np.cos(t * 1.8) * 120.0]),
                "current": np.array([1.5 + 0.8 * np.sin(t * 4.0)]),
            },
            "motor_ankle": {
                "toa_s": np.array([sample_time]),
                "position": np.array([np.sin(t * 2.5) * 3.0]),
                "velocity": np.array([np.cos(t * 2.5) * 80.0]),
                "current": np.array([1.2 + 0.6 * np.cos(t * 3.5)]),
            },
            "encoder_knee": {
                "toa_s": np.array([sample_time]),
                "angle": np.array([30.0 + 25.0 * np.sin(t * 1.8)]),
            },
            "encoder_ankle": {
                "toa_s": np.array([sample_time]),
                "angle": np.array([10.0 + 15.0 * np.sin(t * 2.5)]),
            },
            "intent": {
                "predictions": np.array(
                    [
                        [
                            0.75 + 0.15 * np.sin(t * 1.5),
                            0.05,
                            0.08 + 0.05 * np.cos(t * 1.5),
                            0.04,
                            0.03,
                            0.02,
                            0.02,
                            0.01,
                        ]
                    ]
                ),
                "logits": np.array([[2.5, -0.5, 0.2, -0.8, -1.0, -1.5, -1.5, -2.0]]),
                "toa_s": np.array([[sample_time]]),
                "compute_time_s": np.array([[0.0125 + 0.003 * np.sin(t * 5.0)]]),
                "sequence_id": np.array([[step]], dtype=np.uint32),
            },
            "mode": {
                "toa_s": np.array([[sample_time]]),
                "mode": np.array([[1 if int(t) % 4 != 2 else 3]], dtype=np.uint8),
                "sequence_id": np.array([[step]], dtype=np.uint32),
                "source": np.array([[2]], dtype=np.uint8),
            },
        }
        vis._process_data("prosthesis", packet)
        time.sleep(0.04)

    print("Cleaning up standalone demo...")
    vis._cleanup()
    print("Standalone demo completed cleanly!")
