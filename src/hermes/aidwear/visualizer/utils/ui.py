"""
Filename: hermes/aidwear/visualizer/utils/ui.py
Description: Qt6 live telemetry visualization dashboard
"""

from collections import deque
from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
from queue import Empty
import numpy as np

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from hermes.nicla_sense_me.utils.types import NiclaLocation
from hermes.aidwear.prosthesis.utils.types import EncoderId, MotorId
from hermes.utils.time_utils import get_time


class VisualizerMainWindow(QtWidgets.QMainWindow):
    """Main telemetry dashboard window displaying 3x3 real-time line plots."""

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

    def __init__(
        self,
        node_id: str,
        data_queue: Queue,
        is_cleanup_event: _Event,
        is_windows_closed_event: _Event,
        time_window_s: float = 5.0,
        draw_interval_s: float = 0.04,
        dark_mode: bool = True,
    ):
        super().__init__()
        self.node_id = node_id
        self.data_queue = data_queue
        self.is_cleanup_event = is_cleanup_event
        self.is_windows_closed_event = is_windows_closed_event
        self.time_window_s = time_window_s
        self.draw_interval_s = draw_interval_s
        self.dark_mode = dark_mode
        self.is_paused = False

        self.setWindowTitle(
            f"HERMES / OpenAssist / X-LEG Live Telemetry [{self.node_id}]"
        )
        self.resize(1400, 920)

        # Telemetry rolling buffers
        max_buf = max(2000, int(50 * (self.time_window_s + 15)))

        self._imu_times: dict[NiclaLocation, list[deque]] = {
            loc: [deque(maxlen=max_buf) for _ in range(3)] for loc in self.IMU_LOCATIONS
        }
        self._imu_values: dict[NiclaLocation, list[deque]] = {
            loc: [deque(maxlen=max_buf) for _ in range(3)] for loc in self.IMU_LOCATIONS
        }

        self._motor_pos_times: dict[MotorId, deque] = {
            m: deque(maxlen=max_buf) for m in self.MOTOR_IDS
        }
        self._motor_pos_values: dict[MotorId, deque] = {
            m: deque(maxlen=max_buf) for m in self.MOTOR_IDS
        }

        self._motor_vel_times: dict[MotorId, deque] = {
            m: deque(maxlen=max_buf) for m in self.MOTOR_IDS
        }
        self._motor_vel_values: dict[MotorId, deque] = {
            m: deque(maxlen=max_buf) for m in self.MOTOR_IDS
        }

        self._motor_cur_times: dict[MotorId, deque] = {
            m: deque(maxlen=max_buf) for m in self.MOTOR_IDS
        }
        self._motor_cur_values: dict[MotorId, deque] = {
            m: deque(maxlen=max_buf) for m in self.MOTOR_IDS
        }

        self._encoder_times: dict[EncoderId, deque] = {
            e: deque(maxlen=max_buf) for e in self.ENCODER_IDS
        }
        self._encoder_values: dict[EncoderId, deque] = {
            e: deque(maxlen=max_buf) for e in self.ENCODER_IDS
        }

        self._latest_time: float = 0.0

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

        # Ingestion & rendering timer
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.timeout.connect(self._drain_queue_and_update)
        self._poll_timer.start(max(10, int(self.draw_interval_s * 1000)))

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
        self.clear_btn.clicked.connect(self.clear_buffers)
        header_layout.addWidget(self.clear_btn)

        main_layout.addLayout(header_layout)

        # 2. GraphicsLayoutWidget (3x3 grid)
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.ci.layout.setSpacing(8)
        main_layout.addWidget(self.glw)

        self._setup_plot_grid()

    def _setup_plot_grid(self) -> None:
        """Create the 3x3 subplot grid with line curves and legends."""
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
        for enc_id in self.ENCODER_IDS:
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
        for motor_id in self.MOTOR_IDS:
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

        for motor_id in self.MOTOR_IDS:
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

        for motor_id in self.MOTOR_IDS:
            pen = pg.mkPen(color=motor_colors[motor_id], width=2.0)
            curve = p_mcur.plot(name=f"{motor_id.name.title()} Motor", pen=pen)
            self._curves_motor_cur[motor_id] = curve

    def clear_buffers(self) -> None:
        """Clear all historical sensor data buffers."""
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

    def _on_window_changed(self, text: str) -> None:
        """Handle user changing the time window duration via dropdown."""
        try:
            val = float(text.replace("s", "").strip())
            self.time_window_s = val
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

    def _drain_queue_and_update(self) -> None:
        """Drain incoming telemetry packets from multiprocessing queue and update plots."""
        if self.is_cleanup_event.is_set():
            self.close()
            QtWidgets.QApplication.instance().quit()
            return

        now = get_time()
        packets_processed = 0

        # Drain up to 100 packets per tick to maintain real-time throughput
        while packets_processed < 100:
            try:
                msg = self.data_queue.get_nowait()
                if msg is None:  # Sentinel to exit
                    self.close()
                    QtWidgets.QApplication.instance().quit()
                    return
                packets_processed += 1
            except Empty:
                break

            for bundle_name, bundle_data in msg.items():
                # 1. Nicla IMU updates
                if bundle_name.startswith("nicla_"):
                    loc_name = bundle_name[6:]
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
                    motor_name = bundle_name[6:]
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
                    encoder_name = bundle_name[8:]
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

        # Render curves if not paused and telemetry exists
        if not self.is_paused and self._latest_time > 0.0:
            cur_t = self._latest_time

            for loc in self.IMU_LOCATIONS:
                for ch in range(3):
                    t_buf = self._imu_times[loc][ch]
                    if t_buf:
                        t_arr = np.asarray(t_buf, dtype=np.float64)
                        y_arr = np.asarray(self._imu_values[loc][ch], dtype=np.float32)
                        self._curves_imu[loc][ch].setData(t_arr - cur_t, y_arr)

            for enc_id in self.ENCODER_IDS:
                t_buf = self._encoder_times[enc_id]
                if t_buf:
                    t_arr = np.asarray(t_buf, dtype=np.float64)
                    y_arr = np.asarray(self._encoder_values[enc_id], dtype=np.float32)
                    self._curves_encoder[enc_id].setData(t_arr - cur_t, y_arr)

            for m_id in self.MOTOR_IDS:
                t_buf = self._motor_pos_times[m_id]
                if t_buf:
                    t_arr = np.asarray(t_buf, dtype=np.float64)
                    y_arr = np.asarray(self._motor_pos_values[m_id], dtype=np.float32)
                    self._curves_motor_pos[m_id].setData(t_arr - cur_t, y_arr)

                t_buf = self._motor_vel_times[m_id]
                if t_buf:
                    t_arr = np.asarray(t_buf, dtype=np.float64)
                    y_arr = np.asarray(self._motor_vel_values[m_id], dtype=np.float32)
                    self._curves_motor_vel[m_id].setData(t_arr - cur_t, y_arr)

                t_buf = self._motor_cur_times[m_id]
                if t_buf:
                    t_arr = np.asarray(t_buf, dtype=np.float64)
                    y_arr = np.asarray(self._motor_cur_values[m_id], dtype=np.float32)
                    self._curves_motor_cur[m_id].setData(t_arr - cur_t, y_arr)

        # FPS calculation
        self._frame_count += 1
        now = get_time()
        dt = now - self._fps_time
        if dt >= 1.0:
            fps = self._frame_count / dt
            self.fps_label.setText(f"FPS: {fps:.1f}")
            self._frame_count = 0
            self._fps_time = now

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        """Handle user closing the plot window by signaling termination to broker."""
        print(f"[{self.node_id}] Visualizer window closed by user.", flush=True)
        # Trigger to the handler/node to trigger "babykill".
        self._is_windows_closed_event.set()
        event.accept()
