"""
Filename: hermes/aidwear/visualizer/consumer.py
Author: Antigravity AI
Description: HERMES Consumer Node that streams multimodal data from the
    medical prosthesis device and visualizes it in real time using Matplotlib.
"""

from collections import deque
import time
from typing import Optional
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from hermes.base.nodes.consumer import Consumer
from hermes.utils.types import LoggingSpec, NewData
from hermes.utils.zmq_utils import (
    PORT_FRONTEND,
    PORT_KILL,
    PORT_SYNC_HOST,
)
from hermes.aidwear.utils.types import NiclaLocation
from hermes.aidwear.prosthesis.utils.types import MotorId, EncoderId


class VisualizerConsumer(Consumer):
    """Real-time Matplotlib visualization node for prosthesis multimodal telemetry."""

    IMU_LOCATIONS: list[NiclaLocation] = [
        NiclaLocation.TORSO,
        NiclaLocation.THIGH_RIGHT,
        NiclaLocation.THIGH_LEFT,
        NiclaLocation.SHANK_RIGHT,
        NiclaLocation.SHANK_LEFT,
    ]
    MOTOR_IDS: list[MotorId] = [MotorId.KNEE, MotorId.ANKLE]
    ENCODER_IDS: list[EncoderId] = [EncoderId.KNEE, EncoderId.ANKLE]

    # Lookups from string bundle tokens to enum types
    NICLA_BY_NAME: dict[str, NiclaLocation] = {loc.value: loc for loc in IMU_LOCATIONS}
    MOTOR_BY_NAME: dict[str, MotorId] = {m.name.lower(): m for m in MOTOR_IDS}
    ENCODER_BY_NAME: dict[str, EncoderId] = {e.name.lower(): e for e in ENCODER_IDS}

    # String aliases for convenience
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
        **kwargs,
    ):
        """Constructor of the VisualizerConsumer Node.

        Args:
            node_id: Unique identifier for this consumer node.
            host_ip: Host IP of the master broker.
            data_in_specs: Specification of incoming modalities to subscribe to.
            logging_spec: Logging specification for storage.
            port_sub: Local subscription port from broker.
            port_sync: Coordination port with broker.
            port_killsig: Kill signal port from broker.
            history_len: Number of historical samples to retain per line on plots.
            draw_interval_s: Minimum interval in seconds between canvas redraws.
            dark_mode: Whether to apply a sleek dark theme.
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
        self._last_draw_time = 0.0

        # Rolling history buffers for 5 IMUs (3 channels each: Roll, Pitch, Yaw / X, Y, Z)
        self._imu_data: dict[NiclaLocation, list[deque]] = {
            loc: [deque(maxlen=self._history_len) for _ in range(3)]
            for loc in self.IMU_LOCATIONS
        }

        # Rolling history buffers for 2 Motors (Position, Velocity, Current)
        self._motor_pos: dict[MotorId, deque] = {
            m_id: deque(maxlen=self._history_len) for m_id in self.MOTOR_IDS
        }
        self._motor_vel: dict[MotorId, deque] = {
            m_id: deque(maxlen=self._history_len) for m_id in self.MOTOR_IDS
        }
        self._motor_cur: dict[MotorId, deque] = {
            m_id: deque(maxlen=self._history_len) for m_id in self.MOTOR_IDS
        }

        # Rolling history buffers for 2 Joint Absolute Encoders (Angle)
        self._encoder_angle: dict[EncoderId, deque] = {
            enc_id: deque(maxlen=self._history_len) for enc_id in self.ENCODER_IDS
        }

        # Matplotlib figure and plot references
        self._fig: Optional[plt.Figure] = None
        self._axes: Optional[np.ndarray] = None
        self._lines_imu: dict[NiclaLocation, list[plt.Line2D]] = {}
        self._lines_motor_pos: dict[MotorId, plt.Line2D] = {}
        self._lines_motor_vel: dict[MotorId, plt.Line2D] = {}
        self._lines_motor_cur: dict[MotorId, plt.Line2D] = {}
        self._lines_encoder: dict[EncoderId, plt.Line2D] = {}

    def _initialize(self) -> None:
        """Initialize ZeroMQ sockets and build the Matplotlib UI."""
        super()._initialize()
        self._setup_plots()

    def _setup_plots(self) -> None:
        """Create the 3x3 Matplotlib subplot grid and line objects."""
        if self._dark_mode:
            plt.style.use("dark_background")
        plt.ion()

        fig, axes = plt.subplots(3, 3, figsize=(15, 9.5))
        if fig.canvas.manager is not None and hasattr(
            fig.canvas.manager, "set_window_title"
        ):
            fig.canvas.manager.set_window_title(
                f"HERMES Prosthesis Live Telemetry [{self.node_id}]"
            )
        fig.suptitle(
            "Prosthesis Live Multimodal Telemetry", fontsize=16, fontweight="bold"
        )
        self._fig = fig
        self._axes = axes

        # Grid allocation:
        # Row 0: IMU Torso, IMU Thigh Right, IMU Thigh Left
        # Row 1: IMU Shank Right, IMU Shank Left, Absolute Joint Encoders (Knee & Ankle)
        # Row 2: Motor Positions (Knee & Ankle), Motor Velocities, Motor Currents

        imu_grid_coords: dict[NiclaLocation, tuple[int, int]] = {
            NiclaLocation.TORSO: (0, 0),
            NiclaLocation.THIGH_RIGHT: (0, 1),
            NiclaLocation.THIGH_LEFT: (0, 2),
            NiclaLocation.SHANK_RIGHT: (1, 0),
            NiclaLocation.SHANK_LEFT: (1, 1),
        }

        channel_labels = ["Roll / X", "Pitch / Y", "Yaw / Z"]
        channel_colors = ["#ff595e", "#8ac926", "#1982c4"]

        # Setup 5 IMU subplots
        for loc, (r, c) in imu_grid_coords.items():
            ax = axes[r, c]
            nice_name = loc.value.replace("_", " ").title()
            ax.set_title(f"IMU: {nice_name}", fontsize=11, fontweight="semibold")
            ax.set_xlabel("Samples", fontsize=8)
            ax.set_ylabel("deg / (m/s²)", fontsize=8)
            ax.grid(True, linestyle="--", alpha=0.35)
            self._lines_imu[loc] = []
            for ch_idx in range(3):
                (line,) = ax.plot(
                    [],
                    [],
                    label=channel_labels[ch_idx],
                    color=channel_colors[ch_idx],
                    linewidth=1.5,
                )
                self._lines_imu[loc].append(line)
            ax.legend(loc="upper right", fontsize=7)

        # Setup Joint Absolute Encoders subplot at (1, 2)
        ax_enc = axes[1, 2]
        ax_enc.set_title("Joint Absolute Encoders", fontsize=11, fontweight="semibold")
        ax_enc.set_xlabel("Samples", fontsize=8)
        ax_enc.set_ylabel("Angle [deg]", fontsize=8)
        ax_enc.grid(True, linestyle="--", alpha=0.35)
        enc_colors = {EncoderId.KNEE: "#06d6a0", EncoderId.ANKLE: "#ffd166"}
        for enc_id in self.ENCODER_IDS:
            (line,) = ax_enc.plot(
                [],
                [],
                label=f"{enc_id.name.title()} Joint",
                color=enc_colors[enc_id],
                linewidth=1.8,
            )
            self._lines_encoder[enc_id] = line
        ax_enc.legend(loc="upper right", fontsize=8)

        # Setup Motor Positions subplot at (2, 0)
        ax_mpos = axes[2, 0]
        ax_mpos.set_title("Motor Positions", fontsize=11, fontweight="semibold")
        ax_mpos.set_xlabel("Samples", fontsize=8)
        ax_mpos.set_ylabel("Position [turns / deg]", fontsize=8)
        ax_mpos.grid(True, linestyle="--", alpha=0.35)
        motor_colors = {MotorId.KNEE: "#118ab2", MotorId.ANKLE: "#f78c6b"}
        for motor_id in self.MOTOR_IDS:
            (line,) = ax_mpos.plot(
                [],
                [],
                label=f"{motor_id.name.title()} Motor",
                color=motor_colors[motor_id],
                linewidth=1.8,
            )
            self._lines_motor_pos[motor_id] = line
        ax_mpos.legend(loc="upper right", fontsize=8)

        # Setup Motor Velocities subplot at (2, 1)
        ax_mvel = axes[2, 1]
        ax_mvel.set_title("Motor Velocities", fontsize=11, fontweight="semibold")
        ax_mvel.set_xlabel("Samples", fontsize=8)
        ax_mvel.set_ylabel("Velocity [rpm / rad/s]", fontsize=8)
        ax_mvel.grid(True, linestyle="--", alpha=0.35)
        for motor_id in self.MOTOR_IDS:
            (line,) = ax_mvel.plot(
                [],
                [],
                label=f"{motor_id.name.title()} Motor",
                color=motor_colors[motor_id],
                linewidth=1.8,
            )
            self._lines_motor_vel[motor_id] = line
        ax_mvel.legend(loc="upper right", fontsize=8)

        # Setup Motor Currents subplot at (2, 2)
        ax_mcur = axes[2, 2]
        ax_mcur.set_title("Motor Currents", fontsize=11, fontweight="semibold")
        ax_mcur.set_xlabel("Samples", fontsize=8)
        ax_mcur.set_ylabel("Current [A]", fontsize=8)
        ax_mcur.grid(True, linestyle="--", alpha=0.35)
        for motor_id in self.MOTOR_IDS:
            (line,) = ax_mcur.plot(
                [],
                [],
                label=f"{motor_id.name.title()} Motor",
                color=motor_colors[motor_id],
                linewidth=1.8,
            )
            self._lines_motor_cur[motor_id] = line
        ax_mcur.legend(loc="upper right", fontsize=8)

        fig.tight_layout()
        fig.canvas.mpl_connect("close_event", self._on_window_close)
        backend_name = matplotlib.get_backend().lower()
        if backend_name not in ["agg", "pdf", "ps", "svg", "cairo"]:
            plt.show(block=False)
        fig.canvas.flush_events()

    def _on_window_close(self, _event) -> None:
        """Handle user closing the plot window by signaling termination to broker."""
        print(f"[{self.node_id}] Visualizer window closed by user.", flush=True)
        # TODO: do something.
        # self._send_kill_to_broker()

    def _process_data(self, topic: str, msg: NewData) -> None:
        """Plot updated incoming multimodal data from the prosthesis using matplotlib.

        Args:
            topic: Identifier of the node publishing the data (e.g. 'prosthesis').
            msg: Multimodal packet dictionary mapping bundle names to channel arrays.
        """
        axes_to_relim: set[plt.Axes] = set()

        for bundle_name, bundle_data in msg.items():
            # 1. Nicla IMU updates
            if bundle_name.startswith("nicla_"):
                loc_name = bundle_name[6:]  # strip 'nicla_'
                loc = self.NICLA_BY_NAME.get(loc_name)
                if loc is not None and loc in self._imu_data:
                    # Select orientation (euler) if present, else fallback to acceleration or gyroscope
                    signal_key = None
                    if "euler" in bundle_data:
                        signal_key = "euler"
                    elif "acceleration" in bundle_data:
                        signal_key = "acceleration"
                    elif "gyroscope" in bundle_data:
                        signal_key = "gyroscope"

                    if signal_key is not None:
                        samples = np.asarray(bundle_data[signal_key])
                        if samples.ndim == 2 and samples.shape[1] >= 3:
                            for row in samples:
                                for ch in range(3):
                                    self._imu_data[loc][ch].append(float(row[ch]))
                        elif samples.ndim == 1 and len(samples) >= 3:
                            for ch in range(3):
                                self._imu_data[loc][ch].append(float(samples[ch]))

                        # Update line artists
                        for ch in range(3):
                            y_vals = list(self._imu_data[loc][ch])
                            x_vals = list(range(len(y_vals)))
                            self._lines_imu[loc][ch].set_data(x_vals, y_vals)
                        axes_to_relim.add(self._lines_imu[loc][0].axes)

            # 2. Motor updates (knee, ankle)
            elif bundle_name.startswith("motor_"):
                motor_name = bundle_name[6:]  # strip 'motor_'
                motor_id = self.MOTOR_BY_NAME.get(motor_name)
                if motor_id is not None and motor_id in self._motor_pos:
                    if "position" in bundle_data:
                        pos_samples = np.asarray(bundle_data["position"]).ravel()
                        for p in pos_samples:
                            self._motor_pos[motor_id].append(float(p))
                        y_vals = list(self._motor_pos[motor_id])
                        self._lines_motor_pos[motor_id].set_data(
                            range(len(y_vals)), y_vals
                        )
                        axes_to_relim.add(self._lines_motor_pos[motor_id].axes)

                    if "velocity" in bundle_data:
                        vel_samples = np.asarray(bundle_data["velocity"]).ravel()
                        for v in vel_samples:
                            self._motor_vel[motor_id].append(float(v))
                        y_vals = list(self._motor_vel[motor_id])
                        self._lines_motor_vel[motor_id].set_data(
                            range(len(y_vals)), y_vals
                        )
                        axes_to_relim.add(self._lines_motor_vel[motor_id].axes)

                    if "current" in bundle_data:
                        cur_samples = np.asarray(bundle_data["current"]).ravel()
                        for c in cur_samples:
                            self._motor_cur[motor_id].append(float(c))
                        y_vals = list(self._motor_cur[motor_id])
                        self._lines_motor_cur[motor_id].set_data(
                            range(len(y_vals)), y_vals
                        )
                        axes_to_relim.add(self._lines_motor_cur[motor_id].axes)

            # 3. Absolute encoder updates (knee, ankle)
            elif bundle_name.startswith("encoder_"):
                encoder_name = bundle_name[8:]  # strip 'encoder_'
                encoder_id = self.ENCODER_BY_NAME.get(encoder_name)
                if (
                    encoder_id is not None
                    and encoder_id in self._encoder_angle
                    and "angle" in bundle_data
                ):
                    angle_samples = np.asarray(bundle_data["angle"]).ravel()
                    for a in angle_samples:
                        self._encoder_angle[encoder_id].append(float(a))
                    y_vals = list(self._encoder_angle[encoder_id])
                    self._lines_encoder[encoder_id].set_data(range(len(y_vals)), y_vals)
                    axes_to_relim.add(self._lines_encoder[encoder_id].axes)

        # Autoscale affected axes
        for ax in axes_to_relim:
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=True)

        # Render throttled to draw_interval_s to keep performance smooth
        now = time.time()
        if now - self._last_draw_time >= self._draw_interval_s:
            if self._fig is not None and plt.fignum_exists(self._fig.number):
                self._fig.canvas.draw_idle()
                self._fig.canvas.flush_events()
            self._last_draw_time = now

    def _cleanup(self) -> None:
        """Close Matplotlib figures and clean up resources."""
        if self._fig is not None and plt.fignum_exists(self._fig.number):
            plt.close(self._fig)
        super()._cleanup()
