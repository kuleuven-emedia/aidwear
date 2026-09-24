"""
Filename: hermes/aidwear/visualizer/consumer.py
Description: High-performance HERMES Consumer Node that streams multimodal
    telemetry from the prosthesis device (5 IMUs, 2 Motors, 2 Joint Encoders)
    and visualizes it in real time using PyQt6 and PyQtGraph with a rolling
    relative time window.
"""

from collections import deque
import sys
import threading
import time
from typing import Optional
import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from hermes.base.nodes.consumer import Consumer
from hermes.nicla_sense_me.utils.types import NiclaLocation
from hermes.aidwear.prosthesis.utils.types import EncoderId, MotorId
from hermes.utils.types import LoggingSpec, NewData
from hermes.utils.time_utils import get_time
from hermes.utils.zmq_utils import (
    PORT_FRONTEND,
    PORT_KILL,
    PORT_SYNC_HOST,
)


class VisualizerMainWindow(QtWidgets.QMainWindow):
    """Main telemetry dashboard window displaying 3x3 real-time line plots."""

    def __init__(
        self,
        consumer: "VisualizerConsumer",
        time_window_s: float = 5.0,
        draw_interval_s: float = 0.04,
        dark_mode: bool = True,
    ) -> None:
        super().__init__()
        self.consumer = consumer
        self.time_window_s = time_window_s
        self.draw_interval_s = draw_interval_s
        self.dark_mode = dark_mode
        self.is_paused = False

        self.setWindowTitle(
            f"HERMES / OpenAssist / X-LEG Live Telemetry [{self.consumer.node_id}]"
        )
        self.resize(1400, 920)

        # Plot references
        self._plots: list[pg.PlotItem] = []
        self._curves_imu: dict[NiclaLocation, list[pg.PlotCurveItem]] = {}
        self._curves_encoder: dict[EncoderId, pg.PlotCurveItem] = {}
        self._curves_motor_pos: dict[MotorId, pg.PlotCurveItem] = {}
        self._curves_motor_vel: dict[MotorId, pg.PlotCurveItem] = {}
        self._curves_motor_cur: dict[MotorId, pg.PlotCurveItem] = {}

        # Frame rate calculation
        self._frame_count = 0
        self._fps_time = get_time()

        self._build_ui()

        # Render loop driven by QTimer
        self._render_timer = QtCore.QTimer(self)
        self._render_timer.timeout.connect(self._update_plots)
        self._render_timer.start(max(10, int(self.draw_interval_s * 1000)))

    def _build_ui(self) -> None:
        """Construct the dashboard layout, controls, and pyqtgraph grid."""
        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 8, 10, 10)
        main_layout.setSpacing(6)

        # 1. Header Toolbar
        header_layout = QtWidgets.QHBoxLayout()
        header_layout.setContentsMargins(4, 2, 4, 4)
        header_layout.setSpacing(12)

        title_label = QtWidgets.QLabel("PROSTHESIS MULTIMODAL TELEMETRY")
        title_label.setStyleSheet(
            "font-size: 15px; font-weight: 700; color: #38bdf8; letter-spacing: 0.8px;"
        )
        header_layout.addWidget(title_label)
        header_layout.addStretch()

        self.status_badge = QtWidgets.QLabel("● LIVE STREAMING")
        self.status_badge.setStyleSheet(
            "color: #22c55e; font-weight: bold; font-size: 12px; margin-right: 8px;"
        )
        header_layout.addWidget(self.status_badge)

        self.fps_label = QtWidgets.QLabel("FPS: --")
        self.fps_label.setStyleSheet("color: #94a3b8; font-size: 12px; margin-right: 12px;")
        header_layout.addWidget(self.fps_label)

        # Time Window Selector
        window_label = QtWidgets.QLabel("Time Window:")
        window_label.setStyleSheet("color: #cbd5e1; font-size: 12px;")
        header_layout.addWidget(window_label)

        self.window_combo = QtWidgets.QComboBox()
        self.window_combo.addItems(["2.0 s", "5.0 s", "10.0 s", "20.0 s", "30.0 s"])
        self.window_combo.setCurrentText(f"{self.time_window_s:.1f} s")
        self.window_combo.currentTextChanged.connect(self._on_window_changed)
        self.window_combo.setStyleSheet("""
            QComboBox {
                background-color: #1e2230;
                color: #f1f5f9;
                border: 1px solid #334155;
                border-radius: 4px;
                padding: 3px 8px;
                font-size: 12px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #1e2230;
                color: #f1f5f9;
                selection-background-color: #3b82f6;
            }
        """)
        header_layout.addWidget(self.window_combo)

        # Pause / Resume Button
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setStyleSheet("""
            QPushButton {
                background-color: #272738;
                color: #e2e8f0;
                border: 1px solid #3b3f54;
                border-radius: 4px;
                padding: 4px 14px;
                font-weight: 600;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #35394d; border-color: #6366f1; }
        """)
        self.pause_btn.clicked.connect(self._toggle_pause)
        header_layout.addWidget(self.pause_btn)

        # Autoscale Button
        self.autoscale_btn = QtWidgets.QPushButton("Autoscale Y")
        self.autoscale_btn.setStyleSheet("""
            QPushButton {
                background-color: #272738;
                color: #e2e8f0;
                border: 1px solid #3b3f54;
                border-radius: 4px;
                padding: 4px 14px;
                font-weight: 600;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #35394d; border-color: #38bdf8; }
        """)
        self.autoscale_btn.clicked.connect(self._autoscale_all)
        header_layout.addWidget(self.autoscale_btn)

        # Clear Buffers Button
        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #272738;
                color: #e2e8f0;
                border: 1px solid #3b3f54;
                border-radius: 4px;
                padding: 4px 12px;
                font-weight: 600;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #45222a; border-color: #ef4444; color: #fca5a5; }
        """)
        self.clear_btn.clicked.connect(self.consumer.clear_buffers)
        header_layout.addWidget(self.clear_btn)

        main_layout.addLayout(header_layout)

        # 2. GraphicsLayoutWidget (3x3 grid)
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.ci.layout.setSpacing(8)
        main_layout.addWidget(self.glw)

        self._setup_plot_grid()

    def _setup_plot_grid(self) -> None:
        """Create the 3x3 subplot grid with line curves and legends."""
        # Grid Coordinates:
        # Row 0: IMU Torso, IMU Thigh Right, IMU Thigh Left
        # Row 1: IMU Shank Right, IMU Shank Left, Absolute Joint Encoders
        # Row 2: Motor Positions, Motor Velocities, Motor Currents

        imu_grid_coords: dict[NiclaLocation, tuple[int, int]] = {
            NiclaLocation.TORSO: (0, 0),
            NiclaLocation.THIGH_RIGHT: (0, 1),
            NiclaLocation.THIGH_LEFT: (0, 2),
            NiclaLocation.SHANK_RIGHT: (1, 0),
            NiclaLocation.SHANK_LEFT: (1, 1),
        }

        imu_channel_labels = ["Roll / X", "Pitch / Y", "Yaw / Z"]
        imu_channel_colors = ["#ff595e", "#8ac926", "#1982c4"]

        # Setup 5 IMU subplots
        for loc, (r, c) in imu_grid_coords.items():
            p = self.glw.addPlot(row=r, col=c)
            self._plots.append(p)
            nice_name = loc.value.replace("_", " ").title()
            p.setTitle(
                f"<span style='font-size: 11pt; font-weight: 700; color: #f8fafc;'>IMU: {nice_name}</span>"
            )
            p.showGrid(x=True, y=True, alpha=0.22)
            p.setLabel("bottom", "Time", units="s")
            p.setLabel("left", "deg / (m/s²)")
            p.setXRange(-self.time_window_s, 0.0, padding=0.0)
            p.enableAutoRange(axis="y", enable=True)

            leg = p.addLegend(offset=(5, 5))
            leg.setBrush(pg.mkBrush("#181922c0"))
            leg.setPen(pg.mkPen("#334155"))

            self._curves_imu[loc] = []
            for ch_idx in range(3):
                pen = pg.mkPen(color=imu_channel_colors[ch_idx], width=1.8)
                curve = p.plot(name=imu_channel_labels[ch_idx], pen=pen)
                self._curves_imu[loc].append(curve)

        # Setup Joint Absolute Encoders at (1, 2)
        p_enc = self.glw.addPlot(row=1, col=2)
        self._plots.append(p_enc)
        p_enc.setTitle(
            "<span style='font-size: 11pt; font-weight: 700; color: #f8fafc;'>Joint Absolute Encoders</span>"
        )
        p_enc.showGrid(x=True, y=True, alpha=0.22)
        p_enc.setLabel("bottom", "Time", units="s")
        p_enc.setLabel("left", "Angle", units="deg")
        p_enc.setXRange(-self.time_window_s, 0.0, padding=0.0)
        p_enc.enableAutoRange(axis="y", enable=True)

        leg_enc = p_enc.addLegend(offset=(5, 5))
        leg_enc.setBrush(pg.mkBrush("#181922c0"))
        leg_enc.setPen(pg.mkPen("#334155"))

        enc_colors = {EncoderId.KNEE: "#06d6a0", EncoderId.ANKLE: "#ffd166"}
        for enc_id in self.consumer.ENCODER_IDS:
            pen = pg.mkPen(color=enc_colors[enc_id], width=2.0)
            curve = p_enc.plot(name=f"{enc_id.name.title()} Joint", pen=pen)
            self._curves_encoder[enc_id] = curve

        # Setup Motor Positions at (2, 0)
        p_mpos = self.glw.addPlot(row=2, col=0)
        self._plots.append(p_mpos)
        p_mpos.setTitle(
            "<span style='font-size: 11pt; font-weight: 700; color: #f8fafc;'>Motor Positions</span>"
        )
        p_mpos.showGrid(x=True, y=True, alpha=0.22)
        p_mpos.setLabel("bottom", "Time", units="s")
        p_mpos.setLabel("left", "Position", units="turns / deg")
        p_mpos.setXRange(-self.time_window_s, 0.0, padding=0.0)
        p_mpos.enableAutoRange(axis="y", enable=True)

        leg_mpos = p_mpos.addLegend(offset=(5, 5))
        leg_mpos.setBrush(pg.mkBrush("#181922c0"))
        leg_mpos.setPen(pg.mkPen("#334155"))

        motor_colors = {MotorId.KNEE: "#118ab2", MotorId.ANKLE: "#f78c6b"}
        for motor_id in self.consumer.MOTOR_IDS:
            pen = pg.mkPen(color=motor_colors[motor_id], width=2.0)
            curve = p_mpos.plot(name=f"{motor_id.name.title()} Motor", pen=pen)
            self._curves_motor_pos[motor_id] = curve

        # Setup Motor Velocities at (2, 1)
        p_mvel = self.glw.addPlot(row=2, col=1)
        self._plots.append(p_mvel)
        p_mvel.setTitle(
            "<span style='font-size: 11pt; font-weight: 700; color: #f8fafc;'>Motor Velocities</span>"
        )
        p_mvel.showGrid(x=True, y=True, alpha=0.22)
        p_mvel.setLabel("bottom", "Time", units="s")
        p_mvel.setLabel("left", "Velocity", units="rpm / rad/s")
        p_mvel.setXRange(-self.time_window_s, 0.0, padding=0.0)
        p_mvel.enableAutoRange(axis="y", enable=True)

        leg_mvel = p_mvel.addLegend(offset=(5, 5))
        leg_mvel.setBrush(pg.mkBrush("#181922c0"))
        leg_mvel.setPen(pg.mkPen("#334155"))

        for motor_id in self.consumer.MOTOR_IDS:
            pen = pg.mkPen(color=motor_colors[motor_id], width=2.0)
            curve = p_mvel.plot(name=f"{motor_id.name.title()} Motor", pen=pen)
            self._curves_motor_vel[motor_id] = curve

        # Setup Motor Currents at (2, 2)
        p_mcur = self.glw.addPlot(row=2, col=2)
        self._plots.append(p_mcur)
        p_mcur.setTitle(
            "<span style='font-size: 11pt; font-weight: 700; color: #f8fafc;'>Motor Currents</span>"
        )
        p_mcur.showGrid(x=True, y=True, alpha=0.22)
        p_mcur.setLabel("bottom", "Time", units="s")
        p_mcur.setLabel("left", "Current", units="A")
        p_mcur.setXRange(-self.time_window_s, 0.0, padding=0.0)
        p_mcur.enableAutoRange(axis="y", enable=True)

        leg_mcur = p_mcur.addLegend(offset=(5, 5))
        leg_mcur.setBrush(pg.mkBrush("#181922c0"))
        leg_mcur.setPen(pg.mkPen("#334155"))

        for motor_id in self.consumer.MOTOR_IDS:
            pen = pg.mkPen(color=motor_colors[motor_id], width=2.0)
            curve = p_mcur.plot(name=f"{motor_id.name.title()} Motor", pen=pen)
            self._curves_motor_cur[motor_id] = curve

    def _on_window_changed(self, text: str) -> None:
        """Handle user changing the time window duration via dropdown."""
        try:
            val = float(text.replace("s", "").strip())
            self.time_window_s = val
            self.consumer.time_window_s = val
            for p in self._plots:
                p.setXRange(-self.time_window_s, 0.0, padding=0.0)
        except ValueError:
            pass

    def _toggle_pause(self) -> None:
        """Toggle live plotting pause/resume."""
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.pause_btn.setText("Resume")
            self.status_badge.setText("⏸ PAUSED")
            self.status_badge.setStyleSheet(
                "color: #eab308; font-weight: bold; font-size: 12px; margin-right: 8px;"
            )
        else:
            self.pause_btn.setText("Pause")
            self.status_badge.setText("● LIVE STREAMING")
            self.status_badge.setStyleSheet(
                "color: #22c55e; font-weight: bold; font-size: 12px; margin-right: 8px;"
            )

    def _autoscale_all(self) -> None:
        """Trigger autoscale on Y-axes for all 9 subplots."""
        for p in self._plots:
            p.enableAutoRange(axis="y", enable=True)

    def _update_plots(self) -> None:
        """Periodic timer slot to refresh line curves with relative time."""
        if self.is_paused:
            return

        # Snapshot data under thread lock
        with self.consumer._lock:
            cur_t = self.consumer._latest_time
            if cur_t <= 0.0:
                return

            imu_snaps = {}
            for loc in self.consumer.IMU_LOCATIONS:
                imu_snaps[loc] = []
                for ch in range(3):
                    t_buf = self.consumer._imu_times[loc][ch]
                    y_buf = self.consumer._imu_values[loc][ch]
                    imu_snaps[loc].append(
                        (
                            np.asarray(t_buf, dtype=np.float64),
                            np.asarray(y_buf, dtype=np.float32),
                        )
                    )

            enc_snaps = {}
            for enc_id in self.consumer.ENCODER_IDS:
                t_buf = self.consumer._encoder_times[enc_id]
                y_buf = self.consumer._encoder_values[enc_id]
                enc_snaps[enc_id] = (
                    np.asarray(t_buf, dtype=np.float64),
                    np.asarray(y_buf, dtype=np.float32),
                )

            mpos_snaps = {}
            mvel_snaps = {}
            mcur_snaps = {}
            for m_id in self.consumer.MOTOR_IDS:
                t_buf = self.consumer._motor_pos_times[m_id]
                y_buf = self.consumer._motor_pos_values[m_id]
                mpos_snaps[m_id] = (
                    np.asarray(t_buf, dtype=np.float64),
                    np.asarray(y_buf, dtype=np.float32),
                )

                t_buf = self.consumer._motor_vel_times[m_id]
                y_buf = self.consumer._motor_vel_values[m_id]
                mvel_snaps[m_id] = (
                    np.asarray(t_buf, dtype=np.float64),
                    np.asarray(y_buf, dtype=np.float32),
                )

                t_buf = self.consumer._motor_cur_times[m_id]
                y_buf = self.consumer._motor_cur_values[m_id]
                mcur_snaps[m_id] = (
                    np.asarray(t_buf, dtype=np.float64),
                    np.asarray(y_buf, dtype=np.float32),
                )

        # Update IMU curves: x = t - cur_t
        for loc in self.consumer.IMU_LOCATIONS:
            for ch in range(3):
                t_arr, y_arr = imu_snaps[loc][ch]
                if t_arr.size > 0:
                    x_arr = t_arr - cur_t
                    self._curves_imu[loc][ch].setData(x_arr, y_arr)

        # Update Encoder curves
        for enc_id in self.consumer.ENCODER_IDS:
            t_arr, y_arr = enc_snaps[enc_id]
            if t_arr.size > 0:
                self._curves_encoder[enc_id].setData(t_arr - cur_t, y_arr)

        # Update Motor curves
        for m_id in self.consumer.MOTOR_IDS:
            t_arr, y_arr = mpos_snaps[m_id]
            if t_arr.size > 0:
                self._curves_motor_pos[m_id].setData(t_arr - cur_t, y_arr)

            t_arr, y_arr = mvel_snaps[m_id]
            if t_arr.size > 0:
                self._curves_motor_vel[m_id].setData(t_arr - cur_t, y_arr)

            t_arr, y_arr = mcur_snaps[m_id]
            if t_arr.size > 0:
                self._curves_motor_cur[m_id].setData(t_arr - cur_t, y_arr)

        # Calculate and display live GUI rendering FPS
        self._frame_count += 1
        now = get_time()
        dt = now - self._fps_time
        if dt >= 1.0:
            fps = self._frame_count / dt
            self.fps_label.setText(f"FPS: {fps:.1f}")
            self._frame_count = 0
            self._fps_time = now

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Handle user closing the plot window."""
        self.consumer._on_window_close()
        event.accept()


class VisualizerConsumer(Consumer):
    """Real-time PyQt6 + PyQtGraph visualization node for prosthesis multimodal telemetry."""

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
        time_window_s: float = 5.0,
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
            time_window_s: Fixed duration in seconds of the horizontal time axis.
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

        self._lock = threading.Lock()
        self._latest_time: float = 0.0
        self._window: Optional[VisualizerMainWindow] = None

        # Max buffer size per channel (approx. 50 Hz * 30s max window + safety margin)
        max_buf = max(2000, int(50 * (self.time_window_s + 10)))

        # Rolling history buffers for 5 IMUs (times and values for 3 channels each)
        self._imu_times: dict[NiclaLocation, list[deque]] = {
            loc: [deque(maxlen=max_buf) for _ in range(3)] for loc in self.IMU_LOCATIONS
        }
        self._imu_values: dict[NiclaLocation, list[deque]] = {
            loc: [deque(maxlen=max_buf) for _ in range(3)] for loc in self.IMU_LOCATIONS
        }

        # Rolling history buffers for 2 Motors (Position, Velocity, Current)
        self._motor_pos_times: dict[MotorId, deque] = {
            m_id: deque(maxlen=max_buf) for m_id in self.MOTOR_IDS
        }
        self._motor_pos_values: dict[MotorId, deque] = {
            m_id: deque(maxlen=max_buf) for m_id in self.MOTOR_IDS
        }

        self._motor_vel_times: dict[MotorId, deque] = {
            m_id: deque(maxlen=max_buf) for m_id in self.MOTOR_IDS
        }
        self._motor_vel_values: dict[MotorId, deque] = {
            m_id: deque(maxlen=max_buf) for m_id in self.MOTOR_IDS
        }

        self._motor_cur_times: dict[MotorId, deque] = {
            m_id: deque(maxlen=max_buf) for m_id in self.MOTOR_IDS
        }
        self._motor_cur_values: dict[MotorId, deque] = {
            m_id: deque(maxlen=max_buf) for m_id in self.MOTOR_IDS
        }

        # Rolling history buffers for 2 Joint Absolute Encoders (Angle)
        self._encoder_times: dict[EncoderId, deque] = {
            enc_id: deque(maxlen=max_buf) for enc_id in self.ENCODER_IDS
        }
        self._encoder_values: dict[EncoderId, deque] = {
            enc_id: deque(maxlen=max_buf) for enc_id in self.ENCODER_IDS
        }

    def clear_buffers(self) -> None:
        """Clear all historical sensor data buffers."""
        with self._lock:
            for loc in self.IMU_LOCATIONS:
                for ch in range(3):
                    self._imu_times[loc][ch].clear()
                    self._imu_values[loc][ch].clear()

            for m_id in self.MOTOR_IDS:
                self._motor_pos_times[m_id].clear()
                self._motor_pos_values[m_id].clear()
                self._motor_vel_times[m_id].clear()
                self._motor_vel_values[m_id].clear()
                self._motor_cur_times[m_id].clear()
                self._motor_cur_values[m_id].clear()

            for enc_id in self.ENCODER_IDS:
                self._encoder_times[enc_id].clear()
                self._encoder_values[enc_id].clear()

            self._latest_time = 0.0

    def __call__(self) -> None:
        # Injects Qt app launch before the Node begins running its lifecycle FSM.
        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication(sys.argv if hasattr(sys, "argv") else [])

        # Configure pyqtgraph global options.
        if self._dark_mode:
            pg.setConfigOption("background", "#12131a")
            pg.setConfigOption("foreground", "#e2e8f0")
        else:
            pg.setConfigOption("background", "#f8fafc")
            pg.setConfigOption("foreground", "#0f172a")

        pg.setConfigOption("antialias", True)

        self._window = VisualizerMainWindow(
            consumer=self,
            time_window_s=self.time_window_s,
            draw_interval_s=self._draw_interval_s,
            dark_mode=self._dark_mode,
        )
        self._window.show()

        # Start HERMES consumer state machine on a background daemon thread.
        worker = threading.Thread(
            target=super().__call__,
            name=f"{self.node_id}_hermes_worker",
            daemon=True,
        )
        worker.start()

        # Execute Qt main event loop.
        app.exec()

        # Mark done and wait briefly for worker to complete.
        self._is_done = True
        worker.join(timeout=1.0)

    def _extract_timestamps(
        self, bundle_data: dict, num_samples: int, now: float
    ) -> np.ndarray:
        """Extract or compute timestamp array for a given number of samples."""
        if "toa_s" in bundle_data:
            raw_t = np.asarray(bundle_data["toa_s"]).ravel()
            if len(raw_t) == num_samples:
                return raw_t.astype(np.float64)
            if len(raw_t) == 1 and num_samples > 1:
                return np.linspace(
                    raw_t[0] - (num_samples - 1) * 0.02,
                    raw_t[0],
                    num_samples,
                    dtype=np.float64,
                )

        if num_samples <= 1:
            return np.array([now], dtype=np.float64)
        return np.linspace(
            now - (num_samples - 1) * 0.02, now, num_samples, dtype=np.float64
        )

    def _process_data(self, topic: str, msg: NewData) -> None:
        # Ingest incoming multimodal data packet from the prosthesis into rolling buffers.
        now = get_time()
        with self._lock:
            for bundle_name, bundle_data in msg.items():
                # 1. Nicla IMU updates
                if bundle_name.startswith("nicla_"):
                    loc_name = bundle_name[6:]  # strip 'nicla_'
                    loc = self.NICLA_BY_NAME.get(loc_name)
                    if loc is not None and loc in self._imu_times:
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
                                num_samples = samples.shape[0]
                                t_arr = self._extract_timestamps(
                                    bundle_data, num_samples, now
                                )
                                for ch in range(3):
                                    self._imu_times[loc][ch].extend(t_arr)
                                    self._imu_values[loc][ch].extend(samples[:, ch])
                                if t_arr.size > 0:
                                    self._latest_time = max(
                                        self._latest_time, float(t_arr[-1])
                                    )

                            elif samples.ndim == 1 and len(samples) >= 3:
                                t_arr = self._extract_timestamps(bundle_data, 1, now)
                                for ch in range(3):
                                    self._imu_times[loc][ch].extend(t_arr)
                                    self._imu_values[loc][ch].append(float(samples[ch]))
                                if t_arr.size > 0:
                                    self._latest_time = max(
                                        self._latest_time, float(t_arr[-1])
                                    )

                # 2. Motor updates (knee, ankle)
                elif bundle_name.startswith("motor_"):
                    motor_name = bundle_name[6:]  # strip 'motor_'
                    motor_id = self.MOTOR_BY_NAME.get(motor_name)
                    if motor_id is not None and motor_id in self._motor_pos_times:
                        if "position" in bundle_data:
                            pos_samples = np.asarray(bundle_data["position"]).ravel()
                            num_samples = len(pos_samples)
                            if num_samples > 0:
                                t_arr = self._extract_timestamps(
                                    bundle_data, num_samples, now
                                )
                                self._motor_pos_times[motor_id].extend(t_arr)
                                self._motor_pos_values[motor_id].extend(pos_samples)
                                self._latest_time = max(
                                    self._latest_time, float(t_arr[-1])
                                )

                        if "velocity" in bundle_data:
                            vel_samples = np.asarray(bundle_data["velocity"]).ravel()
                            num_samples = len(vel_samples)
                            if num_samples > 0:
                                t_arr = self._extract_timestamps(
                                    bundle_data, num_samples, now
                                )
                                self._motor_vel_times[motor_id].extend(t_arr)
                                self._motor_vel_values[motor_id].extend(vel_samples)
                                self._latest_time = max(
                                    self._latest_time, float(t_arr[-1])
                                )

                        if "current" in bundle_data:
                            cur_samples = np.asarray(bundle_data["current"]).ravel()
                            num_samples = len(cur_samples)
                            if num_samples > 0:
                                t_arr = self._extract_timestamps(
                                    bundle_data, num_samples, now
                                )
                                self._motor_cur_times[motor_id].extend(t_arr)
                                self._motor_cur_values[motor_id].extend(cur_samples)
                                self._latest_time = max(
                                    self._latest_time, float(t_arr[-1])
                                )

                # 3. Absolute encoder updates (knee, ankle)
                elif bundle_name.startswith("encoder_"):
                    encoder_name = bundle_name[8:]  # strip 'encoder_'
                    encoder_id = self.ENCODER_BY_NAME.get(encoder_name)
                    if (
                        encoder_id is not None
                        and encoder_id in self._encoder_times
                        and "angle" in bundle_data
                    ):
                        angle_samples = np.asarray(bundle_data["angle"]).ravel()
                        num_samples = len(angle_samples)
                        if num_samples > 0:
                            t_arr = self._extract_timestamps(
                                bundle_data, num_samples, now
                            )
                            self._encoder_times[encoder_id].extend(t_arr)
                            self._encoder_values[encoder_id].extend(angle_samples)
                            self._latest_time = max(
                                self._latest_time, float(t_arr[-1])
                            )

            # Prune samples older than time_window_s + 2.0s
            cutoff = self._latest_time - self.time_window_s - 2.0
            if cutoff > 0:
                for loc in self.IMU_LOCATIONS:
                    for ch in range(3):
                        t_buf = self._imu_times[loc][ch]
                        y_buf = self._imu_values[loc][ch]
                        while t_buf and t_buf[0] < cutoff:
                            t_buf.popleft()
                            y_buf.popleft()

                for m_id in self.MOTOR_IDS:
                    for t_buf, y_buf in [
                        (self._motor_pos_times[m_id], self._motor_pos_values[m_id]),
                        (self._motor_vel_times[m_id], self._motor_vel_values[m_id]),
                        (self._motor_cur_times[m_id], self._motor_cur_values[m_id]),
                    ]:
                        while t_buf and t_buf[0] < cutoff:
                            t_buf.popleft()
                            y_buf.popleft()

                for enc_id in self.ENCODER_IDS:
                    t_buf = self._encoder_times[enc_id]
                    y_buf = self._encoder_values[enc_id]
                    while t_buf and t_buf[0] < cutoff:
                        t_buf.popleft()
                        y_buf.popleft()

    def _on_window_close(self) -> None:
        """Handle user closing the plot window by signaling termination to broker."""
        print(f"[{self.node_id}] Visualizer window closed by user.", flush=True)
        if hasattr(self, "_babykillsig"):
            try:
                self._send_kill_to_broker()
            except Exception as e:
                print(
                    f"[{self.node_id}] Error notifying broker of window close: {e}",
                    flush=True,
                )

    def _cleanup(self) -> None:
        # Close Qt window and clean up node resources.
        if self._window is not None:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self._window, "close", QtCore.Qt.ConnectionType.QueuedConnection
                )
            except Exception:
                pass
        super()._cleanup()


if __name__ == "__main__":
    # Demo / standalone testing runner
    print("Launching Visualizer in standalone demo mode...", flush=True)
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    pg.setConfigOption("background", "#12131a")
    pg.setConfigOption("foreground", "#e2e8f0")
    pg.setConfigOption("antialias", True)

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
    win = VisualizerMainWindow(vis, time_window_s=5.0, draw_interval_s=0.033)
    vis._window = win
    win.show()

    # Simulate incoming data stream in background
    running = True

    def sim_stream():
        t0 = get_time()
        step = 0
        while running:
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
                    "euler": np.array([[np.sin(t * 2.2 + 1.0), np.cos(t * 1.7 + 1.0), np.sin(t * 2.8 + 1.0)]])
                    * 45.0,
                },
                "nicla_shank_right": {
                    "toa_s": np.array([sample_time]),
                    "euler": np.array([[np.sin(t * 3.0), np.cos(t * 2.5), np.sin(t * 3.5)]])
                    * 60.0,
                },
                "nicla_shank_left": {
                    "toa_s": np.array([sample_time]),
                    "euler": np.array([[np.sin(t * 3.0 + 1.0), np.cos(t * 2.5 + 1.0), np.sin(t * 3.5 + 1.0)]])
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
            }
            vis._process_data("prosthesis", packet)
            step += 1
            time.sleep(0.02)  # 50 Hz

    sim_thread = threading.Thread(target=sim_stream, daemon=True)
    sim_thread.start()

    # Run for 2 seconds then exit in automated test
    QtCore.QTimer.singleShot(25000, app.quit)
    app.exec()
    running = False
    print("Standalone demo completed successfully!", flush=True)
