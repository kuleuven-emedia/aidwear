"""
Filename: hermes/aidwear/visualizer/utils/ui.py
Description: Qt6 live telemetry visualization dashboard with real-time IMU,
    motor, and absolute encoder plots, along with live AI intent prediction
    horizontal bar charts and factual prosthesis mode indicators.
"""

from collections import deque
from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
from queue import Empty
from typing import Optional
import numpy as np

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from hermes.nicla_sense_me.utils.types import NiclaLocation
from hermes.aidwear.prosthesis.utils.types import (
    EncoderId,
    MotorId,
    ModeEnum,
    IntentCommandSource,
)
from hermes.utils.time_utils import get_time


class VisualizerMainWindow(QtWidgets.QMainWindow):
    """Main telemetry dashboard window displaying 3x3 real-time line plots,

    alongside live AI intent classification bar charts and factual prosthesis
    operating mode indicators.
    """

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

    # 23-segment Xsens MVN Kinematic Chains
    SPINE_BONES: list[tuple[int, int]] = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6)]
    RIGHT_ARM_BONES: list[tuple[int, int]] = [(4, 7), (7, 8), (8, 9), (9, 10)]
    LEFT_ARM_BONES: list[tuple[int, int]] = [(4, 11), (11, 12), (12, 13), (13, 14)]
    RIGHT_LEG_BONES: list[tuple[int, int]] = [(0, 15), (15, 16), (16, 17), (17, 18)]
    LEFT_LEG_BONES: list[tuple[int, int]] = [(0, 19), (19, 20), (20, 21), (21, 22)]

    # Default standing human skeleton rest pose in meters (23 segments)
    DEFAULT_REST_POSE: np.ndarray = np.array([
        [0.0, 0.0, 1.1],        # 0: Pelvis
        [0.0, 0.0, 1.18],       # 1: L5
        [0.0, 0.0, 1.26],       # 2: L3
        [0.0, 0.0, 1.35],       # 3: T12
        [0.0, 0.0, 1.45],       # 4: T8
        [0.0, 0.0, 1.57],       # 5: Neck
        [0.0, 0.0, 1.73],       # 6: Head
        [0.0, -0.18, 1.45],     # 7: Right Shoulder
        [0.0, -0.32, 1.30],     # 8: Right Upper Arm
        [0.0, -0.35, 1.05],     # 9: Right Forearm
        [0.0, -0.35, 0.9],      # 10: Right Hand
        [0.0, 0.18, 1.45],      # 11: Left Shoulder
        [0.0, 0.32, 1.30],      # 12: Left Upper Arm
        [0.0, 0.35, 1.05],      # 13: Left Forearm
        [0.0, 0.35, 0.9],       # 14: Left Hand
        [0.0, -0.10, 0.70],     # 15: Right Upper Leg
        [0.0, -0.10, 0.30],     # 16: Right Lower Leg
        [0.0, -0.10, 0.15],     # 17: Right Foot
        [0.15, -0.10, 0.15],    # 18: Right Toe
        [0.0, 0.10, 0.70],      # 19: Left Upper Leg
        [0.0, 0.10, 0.30],      # 20: Left Lower Leg
        [0.0, 0.10, 0.15],      # 21: Left Foot
        [0.15, 0.10, 0.15],     # 22: Left Toe
    ], dtype=np.float32)

    # Default locomotion / ambulation classes for AI Intent recognition
    DEFAULT_AI_CLASSES: list[str] = [
        "Level-Ground Walking",
        "Hurdles",
        "Stair Ascent",
        "Stair Descent",
        "Cross Country",
        "Ladder",
        "Slope Ascent",
        "Slope Descent",
    ]

    # Mode visual aesthetics mapped by ModeEnum integer IDs
    MODE_STYLE_CONFIG: dict[int, dict[str, str]] = {
        0: {
            "name": "IDLE",
            "color": "#94a3b8",
            "bg": "#1e293b",
            "border": "#475569",
        },
        1: {
            "name": "WALKING",
            "color": "#38bdf8",
            "bg": "#082f49",
            "border": "#0284c7",
        },
        2: {
            "name": "SIT TO STAND",
            "color": "#f59e0b",
            "bg": "#451a03",
            "border": "#d97706",
        },
        3: {
            "name": "STAIR ASCENT",
            "color": "#a855f7",
            "bg": "#3b0764",
            "border": "#9333ea",
        },
        4: {
            "name": "STAIR DESCENT",
            "color": "#ec4899",
            "bg": "#500724",
            "border": "#db2777",
        },
        5: {
            "name": "HURDLE",
            "color": "#10b981",
            "bg": "#022c22",
            "border": "#059669",
        },
    }

    SOURCE_NAMES: dict[int, str] = {
        IntentCommandSource.CLI.value: "CLI",
        IntentCommandSource.GUI.value: "GUI",
        IntentCommandSource.AI.value: "AI",
    }

    def __init__(
        self,
        node_id: str,
        data_queue: Queue,
        is_cleanup_event: _Event,
        is_windows_closed_event: _Event,
        time_window_s: float = 5.0,
        draw_interval_s: float = 0.04,
        dark_mode: bool = True,
        class_names: Optional[list[str]] = None,
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

        # AI Intent classes
        self._class_names: list[str] = (
            list(class_names) if class_names else list(self.DEFAULT_AI_CLASSES)
        )

        # Factual prosthesis mode state
        self._current_mode_id: int = 0
        self._current_mode_source: int = 0
        self._current_mode_seq: int = 0
        self._current_mode_time: float = 0.0

        # AI Intent latest predictions state
        self._latest_predictions: np.ndarray = np.zeros(
            len(self._class_names), dtype=np.float64
        )
        self._latest_ai_seq: int = 0
        self._latest_ai_time: float = 0.0
        self._latest_ai_latency_ms: float = 0.0

        self.setWindowTitle(
            f"HERMES / OpenAssist / X-LEG Live Telemetry [{self.node_id}]"
        )
        self.resize(1560, 920)

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
        self._imu_signal_preference: str = "Auto"
        self._active_imu_source: str = "None"

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
        """Construct the dashboard layout, controls, pyqtgraph grid, and AI dock."""
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

        # Header Mode Badge
        self.header_mode_badge = QtWidgets.QLabel("MODE: IDLE")
        self.header_mode_badge.setStyleSheet("""
            QLabel {
                color: #94a3b8;
                background-color: #1e293b;
                border: 1px solid #475569;
                border-radius: 4px;
                padding: 3px 12px;
                font-weight: 700;
                font-size: 12px;
                letter-spacing: 0.5px;
            }
        """)
        header_layout.addWidget(self.header_mode_badge)

        header_layout.addStretch()

        self.status_badge = QtWidgets.QLabel("● LIVE STREAMING")
        self.status_badge.setStyleSheet(
            "color: #22c55e; font-weight: bold; font-size: 12px; margin-right: 8px;"
        )
        header_layout.addWidget(self.status_badge)

        self.fps_label = QtWidgets.QLabel("FPS: --")
        self.fps_label.setStyleSheet(
            "color: #94a3b8; font-size: 12px; margin-right: 12px;"
        )
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

        # IMU Signal Selector
        imu_signal_label = QtWidgets.QLabel("IMU Signal:")
        imu_signal_label.setStyleSheet("color: #cbd5e1; font-size: 12px;")
        header_layout.addWidget(imu_signal_label)

        self.imu_signal_combo = QtWidgets.QComboBox()
        self.imu_signal_combo.addItems(["Auto", "Acceleration", "Gyroscope"])
        self.imu_signal_combo.setCurrentText("Auto")
        self.imu_signal_combo.currentTextChanged.connect(self._on_imu_signal_changed)
        self.imu_signal_combo.setStyleSheet("""
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
        header_layout.addWidget(self.imu_signal_combo)

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

        # 2. Body splitter: 3x3 telemetry grid on the left, AI & Mode dock on the right
        self.body_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.body_splitter.setStyleSheet("""
            QSplitter::handle {
                background-color: #232736;
                width: 4px;
            }
            QSplitter::handle:hover {
                background-color: #38bdf8;
            }
        """)
        main_layout.addWidget(self.body_splitter)

        # Left: 3x3 Telemetry grid
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.ci.layout.setSpacing(8)
        self.body_splitter.addWidget(self.glw)

        # Right: AI predictions and prosthesis mode dock
        self._setup_ai_and_mode_dock()
        self.body_splitter.addWidget(self.side_dock)

        self.body_splitter.setStretchFactor(0, 3)
        self.body_splitter.setStretchFactor(1, 1)

        self._setup_plot_grid()

    def _setup_ai_and_mode_dock(self) -> None:
        """Construct the right-hand panel for factual mode display and live AI intent bar chart."""
        self.side_dock = QtWidgets.QWidget()
        self.side_dock.setMinimumWidth(320)
        side_layout = QtWidgets.QVBoxLayout(self.side_dock)
        side_layout.setContentsMargins(6, 0, 0, 0)
        side_layout.setSpacing(8)

        card_bg = "#181926" if self.dark_mode else "#f8fafc"
        card_border = "#2d3142" if self.dark_mode else "#cbd5e1"

        # ----------------------------------------------------
        # Card 1: Factual Prosthesis Mode Indicator
        # ----------------------------------------------------
        self.mode_card = QtWidgets.QFrame()
        self.mode_card.setStyleSheet(f"""
            QFrame {{
                background-color: {card_bg};
                border: 1px solid {card_border};
                border-radius: 8px;
            }}
        """)
        mode_card_layout = QtWidgets.QVBoxLayout(self.mode_card)
        mode_card_layout.setContentsMargins(12, 10, 12, 10)
        mode_card_layout.setSpacing(6)

        mode_header_layout = QtWidgets.QHBoxLayout()
        mode_title = QtWidgets.QLabel("PROSTHESIS OPERATION MODE")
        mode_title.setStyleSheet(
            "font-size: 11px; font-weight: 700; color: #94a3b8; letter-spacing: 0.6px;"
        )
        mode_header_layout.addWidget(mode_title)
        mode_header_layout.addStretch()

        self.mode_status_dot = QtWidgets.QLabel("●")
        self.mode_status_dot.setStyleSheet("color: #94a3b8; font-size: 14px;")
        mode_header_layout.addWidget(self.mode_status_dot)
        mode_card_layout.addLayout(mode_header_layout)

        # Big active mode text display
        self.mode_text_label = QtWidgets.QLabel("IDLE")
        self.mode_text_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.mode_text_label.setStyleSheet("""
            QLabel {
                color: #94a3b8;
                background-color: #1e293b;
                border: 1px solid #475569;
                border-radius: 6px;
                padding: 8px 14px;
                font-size: 18px;
                font-weight: 800;
                letter-spacing: 1.0px;
            }
        """)
        mode_card_layout.addWidget(self.mode_text_label)

        # Metadata line
        self.mode_meta_label = QtWidgets.QLabel("Source: -- | Seq: #0 | Time: --")
        self.mode_meta_label.setStyleSheet("color: #64748b; font-size: 11px;")
        self.mode_meta_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        mode_card_layout.addWidget(self.mode_meta_label)

        side_layout.addWidget(self.mode_card)

        # ----------------------------------------------------
        # Card 2: Live AI Intent Predictions Bar Chart
        # ----------------------------------------------------
        self.ai_card = QtWidgets.QFrame()
        self.ai_card.setStyleSheet(f"""
            QFrame {{
                background-color: {card_bg};
                border: 1px solid {card_border};
                border-radius: 8px;
            }}
        """)
        ai_card_layout = QtWidgets.QVBoxLayout(self.ai_card)
        ai_card_layout.setContentsMargins(12, 10, 12, 10)
        ai_card_layout.setSpacing(6)

        ai_header_layout = QtWidgets.QHBoxLayout()
        ai_title = QtWidgets.QLabel("LIVE AI INTENT PREDICTIONS")
        ai_title.setStyleSheet(
            "font-size: 11px; font-weight: 700; color: #38bdf8; letter-spacing: 0.6px;"
        )
        ai_header_layout.addWidget(ai_title)
        ai_header_layout.addStretch()

        self.ai_latency_label = QtWidgets.QLabel("Latency: -- ms")
        self.ai_latency_label.setStyleSheet("color: #94a3b8; font-size: 11px;")
        ai_header_layout.addWidget(self.ai_latency_label)
        ai_card_layout.addLayout(ai_header_layout)

        # Top predicted class banner
        self.ai_top_label = QtWidgets.QLabel("Top: -- (--%)")
        self.ai_top_label.setStyleSheet(
            "color: #e2e8f0; font-size: 12px; font-weight: 600;"
        )
        ai_card_layout.addWidget(self.ai_top_label)

        # PyQtGraph PlotWidget for horizontal bar chart
        self.ai_plot_widget = pg.PlotWidget()
        self.ai_plot_widget.setBackground("#12131a" if self.dark_mode else "#ffffff")
        self.ai_plot: pg.PlotItem = self.ai_plot_widget.getPlotItem()
        self.ai_plot.invertY(True)
        self.ai_plot.showGrid(x=True, y=False, alpha=0.22)
        self.ai_plot.setXRange(0.0, 1.05, padding=0.0)
        self.ai_plot.setLabel("bottom", "Prediction Confidence", units="")
        self.ai_plot.setMouseEnabled(x=False, y=False)
        self.ai_plot.hideButtons()

        # Format left Y-axis ticks with class names
        y_ticks = [(i, name) for i, name in enumerate(self._class_names)]
        self.ai_plot.getAxis("left").setTicks([y_ticks])
        self.ai_plot.getAxis("left").setWidth(135)
        self.ai_plot.setYRange(-0.6, len(self._class_names) - 0.4, padding=0.0)

        # Horizontal BarGraphItem (x0=0, y=class_idx, width=prediction, height=0.45)
        self._bar_item = pg.BarGraphItem(
            x0=0,
            y=list(range(len(self._class_names))),
            width=[0.0] * len(self._class_names),
            height=0.45,
            brush=pg.mkBrush("#38bdf8"),
            pen=pg.mkPen(color="#1e293b", width=1),
        )
        self.ai_plot.addItem(self._bar_item)

        # Value text labels for each class
        self._ai_text_items: list[pg.TextItem] = []
        for _ in range(len(self._class_names)):
            ti = pg.TextItem("", color="#cbd5e1", anchor=(0.0, 0.5))
            ti.setFont(QtGui.QFont("Segoe UI", 8, QtGui.QFont.Weight.Bold))
            self.ai_plot.addItem(ti)
            self._ai_text_items.append(ti)

        ai_card_layout.addWidget(self.ai_plot_widget)
        self.ai_plot_widget.setMinimumHeight(180)
        side_layout.addWidget(self.ai_card, stretch=1)

        # ----------------------------------------------------
        # Card 3: 3D MVN Skeleton Pose Visualizer
        # ----------------------------------------------------
        self.pose_card = QtWidgets.QFrame()
        self.pose_card.setStyleSheet(f"""
            QFrame {{
                background-color: {card_bg};
                border: 1px solid {card_border};
                border-radius: 8px;
            }}
        """)
        pose_card_layout = QtWidgets.QVBoxLayout(self.pose_card)
        pose_card_layout.setContentsMargins(10, 8, 10, 8)
        pose_card_layout.setSpacing(6)

        # Pose Card Header
        pose_header = QtWidgets.QHBoxLayout()
        pose_title = QtWidgets.QLabel("3D SKELETON POSE [MVN]")
        pose_title.setStyleSheet(
            "font-size: 11px; font-weight: 700; color: #38bdf8; letter-spacing: 0.6px;"
        )
        pose_header.addWidget(pose_title)
        pose_header.addStretch()

        self.pose_center_checkbox = QtWidgets.QCheckBox("Center Root")
        self.pose_center_checkbox.setChecked(True)
        self.pose_center_checkbox.setToolTip("Keep Pelvis centered at origin in the ground plane")
        self.pose_center_checkbox.setStyleSheet("""
            QCheckBox {
                color: #94a3b8;
                font-size: 11px;
            }
            QCheckBox::indicator {
                width: 12px;
                height: 12px;
            }
        """)
        pose_header.addWidget(self.pose_center_checkbox)

        self.pose_reset_btn = QtWidgets.QPushButton("↺ View")
        self.pose_reset_btn.setToolTip("Reset camera view")
        self.pose_reset_btn.setStyleSheet("""
            QPushButton {
                background-color: #272738;
                color: #cbd5e1;
                border: 1px solid #3b3f54;
                border-radius: 3px;
                padding: 2px 7px;
                font-size: 10px;
                font-weight: 600;
            }
            QPushButton:hover { background-color: #35394d; border-color: #38bdf8; color: #f1f5f9; }
        """)
        self.pose_reset_btn.clicked.connect(self._reset_pose_camera)
        pose_header.addWidget(self.pose_reset_btn)

        self.pose_status_label = QtWidgets.QLabel("● STANDBY")
        self.pose_status_label.setStyleSheet("color: #64748b; font-size: 11px; font-weight: bold;")
        pose_header.addWidget(self.pose_status_label)
        pose_card_layout.addLayout(pose_header)

        # 3D GL Viewport
        self.pose_view = gl.GLViewWidget()
        self.pose_view.setBackgroundColor("#12131a" if self.dark_mode else "#f8fafc")
        self.pose_view.setMinimumHeight(240)
        self._reset_pose_camera()

        # Ground Grid (3.0m x 3.0m with 0.5m grid intervals)
        self._pose_grid = gl.GLGridItem()
        self._pose_grid.setSize(3.0, 3.0)
        self._pose_grid.setSpacing(0.5, 0.5)
        self._pose_grid.setColor((0.3, 0.35, 0.45, 0.35))
        self.pose_view.addItem(self._pose_grid)

        # Skeleton Bone Line Items
        self._pose_spine_lines = gl.GLLinePlotItem(
            pos=np.zeros((0, 3), dtype=np.float32), mode="lines", width=3.0, color=(0.22, 0.74, 0.97, 1.0)
        )
        self._pose_r_arm_lines = gl.GLLinePlotItem(
            pos=np.zeros((0, 3), dtype=np.float32), mode="lines", width=2.5, color=(0.2, 0.83, 0.6, 1.0)
        )
        self._pose_l_arm_lines = gl.GLLinePlotItem(
            pos=np.zeros((0, 3), dtype=np.float32), mode="lines", width=2.5, color=(0.65, 0.55, 0.98, 1.0)
        )
        self._pose_r_leg_lines = gl.GLLinePlotItem(
            pos=np.zeros((0, 3), dtype=np.float32), mode="lines", width=3.0, color=(0.98, 0.75, 0.14, 1.0)
        )
        self._pose_l_leg_lines = gl.GLLinePlotItem(
            pos=np.zeros((0, 3), dtype=np.float32), mode="lines", width=3.0, color=(0.96, 0.45, 0.71, 1.0)
        )
        for item in [
            self._pose_spine_lines,
            self._pose_r_arm_lines,
            self._pose_l_arm_lines,
            self._pose_r_leg_lines,
            self._pose_l_leg_lines,
        ]:
            self.pose_view.addItem(item)

        # Joints Scatter Item
        self._pose_joints = gl.GLScatterPlotItem(
            pos=np.zeros((0, 3), dtype=np.float32), size=7, color=(0.95, 0.96, 0.98, 1.0)
        )
        self.pose_view.addItem(self._pose_joints)

        # Render initial standing reference pose
        self._render_pose(self.DEFAULT_REST_POSE)

        pose_card_layout.addWidget(self.pose_view, stretch=1)
        side_layout.addWidget(self.pose_card, stretch=1)

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
            p.setLabel("left", "deg / (deg/s) / (m/s²)")
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
        """Clear all historical sensor data buffers, AI intent predictions, and reset mode."""
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

        # Reset AI predictions and mode display
        self._latest_predictions = np.zeros(len(self._class_names), dtype=np.float64)
        self._latest_ai_seq = 0
        self._latest_ai_latency_ms = 0.0
        self._render_ai_predictions()
        self._update_mode_display(mode_id=0, source_id=0, seq_id=0, toa_s=0.0)
        self._clear_pose_display()

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

    def _update_imu_y_label(self, unit_str: str) -> None:
        """Update Y-axis label on all 5 IMU subplots if changed."""
        if getattr(self, "_current_imu_unit", None) != unit_str:
            self._current_imu_unit = unit_str
            for p in self._plots[:5]:
                p.setLabel("left", unit_str)

    def _on_imu_signal_changed(self, text: str) -> None:
        """Handle user changing the IMU signal source (Auto, Acceleration, Gyroscope)."""
        self._imu_signal_preference = text
        for loc in self.IMU_LOCATIONS:
            for ch in range(3):
                self._imu_times[loc][ch].clear()
                self._imu_values[loc][ch].clear()

        unit_str = (
            "m/s²"
            if text == "Acceleration"
            else ("deg/s" if text == "Gyroscope" else "deg / (deg/s) / (m/s²)")
        )
        self._update_imu_y_label(unit_str)
        self._autoscale_all()

    @classmethod
    def get_xsens_sensor_map(cls, num_sensors: int) -> dict[NiclaLocation, int]:
        """Map Xsens sensor array indices to NiclaLocation subplots based on sensor count.

        - For full-body Xsens MVN layout (>= 16 sensors):
            0: Pelvis (Torso)
            11: Right Upper Leg (Thigh Right)
            14: Left Upper Leg (Thigh Left)
            12: Right Lower Leg (Shank Right)
            15: Left Lower Leg (Shank Left)
        - For 7-sensor lower-body layout [pelvis, thigh_r, shank_r, foot_r, thigh_l, shank_l, foot_l]:
            0: Pelvis, 1: Thigh Right, 2: Shank Right, 4: Thigh Left, 5: Shank Left
        - For 5-sensor lower-body layout [pelvis, thigh_r, thigh_l, shank_r, shank_l]:
            0: Pelvis, 1: Thigh Right, 2: Thigh Left, 3: Shank Right, 4: Shank Left
        """
        if num_sensors >= 16:
            return {
                NiclaLocation.TORSO: 0,
                NiclaLocation.THIGH_RIGHT: 11,
                NiclaLocation.THIGH_LEFT: 14,
                NiclaLocation.SHANK_RIGHT: 12,
                NiclaLocation.SHANK_LEFT: 15,
            }
        elif num_sensors == 7:
            return {
                NiclaLocation.TORSO: 0,
                NiclaLocation.THIGH_RIGHT: 1,
                NiclaLocation.SHANK_RIGHT: 2,
                NiclaLocation.THIGH_LEFT: 4,
                NiclaLocation.SHANK_LEFT: 5,
            }
        elif num_sensors >= 5:
            return {
                NiclaLocation.TORSO: 0,
                NiclaLocation.THIGH_RIGHT: 1,
                NiclaLocation.THIGH_LEFT: 2,
                NiclaLocation.SHANK_RIGHT: 3,
                NiclaLocation.SHANK_LEFT: 4,
            }
        else:
            locs = [
                NiclaLocation.TORSO,
                NiclaLocation.THIGH_RIGHT,
                NiclaLocation.THIGH_LEFT,
                NiclaLocation.SHANK_RIGHT,
                NiclaLocation.SHANK_LEFT,
            ]
            return {loc: i for i, loc in enumerate(locs[:num_sensors])}

    def _handle_xsens_imu(self, bundle_data: dict, now: float) -> None:
        """Parse incoming Xsens MVN motion trackers packet and update IMU subplots."""
        if not isinstance(bundle_data, dict):
            return

        pref = getattr(self, "_imu_signal_preference", "Auto")
        signal_key = None
        if pref == "Acceleration" and "acceleration" in bundle_data:
            signal_key = "acceleration"
        elif pref == "Gyroscope" and "gyroscope" in bundle_data:
            signal_key = "gyroscope"
        else:
            # Auto preference: default to linear acceleration, then free_acceleration, then gyroscope
            if "acceleration" in bundle_data:
                signal_key = "acceleration"
            elif "free_acceleration" in bundle_data:
                signal_key = "free_acceleration"
            elif "gyroscope" in bundle_data:
                signal_key = "gyroscope"

        if signal_key is None:
            return

        samples = np.asarray(bundle_data[signal_key])
        if samples.ndim == 2 and samples.shape[-1] >= 3:
            samples = samples[np.newaxis, :, :]  # (1, num_sensors, 3)
        elif samples.ndim != 3 or samples.shape[-1] < 3:
            return

        num_samples = samples.shape[0]
        num_sensors = samples.shape[1]
        sensor_map = self.get_xsens_sensor_map(num_sensors)
        t_arr = self._extract_timestamps(bundle_data, num_samples, now)

        for loc, s_idx in sensor_map.items():
            if s_idx < num_sensors and loc in self._imu_times:
                loc_samples = samples[:, s_idx, :]
                for ch in range(3):
                    self._imu_times[loc][ch].extend(t_arr.tolist())
                    self._imu_values[loc][ch].extend(loc_samples[:, ch].tolist())

        if t_arr.size > 0:
            self._latest_time = max(self._latest_time, float(t_arr[-1]))
            self._active_imu_source = "Xsens MVN"
            if pref == "Auto":
                self._update_imu_y_label(
                    "deg/s" if signal_key == "gyroscope" else "m/s²"
                )
            if not self.is_paused:
                self.status_badge.setText("● LIVE STREAMING [XSENS]")
                self.status_badge.setStyleSheet(
                    "color: #22c55e; font-weight: bold; font-size: 12px; margin-right: 8px;"
                )

    def _reset_pose_camera(self) -> None:
        """Reset the 3D pose view camera to default vantage point."""
        if hasattr(self, "pose_view"):
            self.pose_view.setCameraPosition(
                pos=QtGui.QVector3D(0.0, 0.0, 1.1), distance=2.3, elevation=15.0, azimuth=-60.0
            )

    @classmethod
    def get_pose_bones(cls, num_segments: int):
        """Return bone connection tuples for the given segment count."""
        if num_segments >= 23:
            return (
                cls.SPINE_BONES,
                cls.RIGHT_ARM_BONES,
                cls.LEFT_ARM_BONES,
                cls.RIGHT_LEG_BONES,
                cls.LEFT_LEG_BONES,
            )
        elif num_segments == 7:
            # Lower-body layout: [pelvis, thigh_r, shank_r, foot_r, thigh_l, shank_l, foot_l]
            r_leg = [(0, 1), (1, 2), (2, 3)]
            l_leg = [(0, 4), (4, 5), (5, 6)]
            return ([], [], [], r_leg, l_leg)
        else:
            chain = [(i, i + 1) for i in range(num_segments - 1)]
            return (chain, [], [], [], [])

    @staticmethod
    def _build_line_segments(
        bones: list[tuple[int, int]], pos: np.ndarray
    ) -> np.ndarray:
        """Convert bone index pairs into continuous line vertices for OpenGL lines mode."""
        n = len(pos)
        verts = []
        for i, j in bones:
            if i < n and j < n:
                verts.append(pos[i])
                verts.append(pos[j])
        if len(verts) == 0:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(verts, dtype=np.float32)

    def _render_pose(self, pos: np.ndarray) -> None:
        """Render joint positions and bone connections on the 3D GL canvas."""
        num_segments = pos.shape[0]
        if (
            getattr(self, "pose_center_checkbox", None)
            and self.pose_center_checkbox.isChecked()
            and num_segments > 0
        ):
            root_xy = pos[0, :2].copy()
            pos_render = pos.copy()
            pos_render[:, 0] -= root_xy[0]
            pos_render[:, 1] -= root_xy[1]
        else:
            pos_render = pos

        spine_b, r_arm_b, l_arm_b, r_leg_b, l_leg_b = self.get_pose_bones(num_segments)
        self._pose_spine_lines.setData(
            pos=self._build_line_segments(spine_b, pos_render)
        )
        self._pose_r_arm_lines.setData(
            pos=self._build_line_segments(r_arm_b, pos_render)
        )
        self._pose_l_arm_lines.setData(
            pos=self._build_line_segments(l_arm_b, pos_render)
        )
        self._pose_r_leg_lines.setData(
            pos=self._build_line_segments(r_leg_b, pos_render)
        )
        self._pose_l_leg_lines.setData(
            pos=self._build_line_segments(l_leg_b, pos_render)
        )
        self._pose_joints.setData(pos=pos_render)

    def _handle_xsens_pose(self, bundle_data: dict, now: float) -> None:
        """Parse incoming Xsens MVN 3D pose packet and update the skeleton visualizer."""
        if not isinstance(bundle_data, dict) or "position" not in bundle_data:
            return

        samples = np.asarray(bundle_data["position"])
        if samples.ndim == 3 and samples.shape[0] > 0:
            pos = samples[-1].astype(np.float32)
        elif samples.ndim == 2:
            pos = samples.astype(np.float32)
        else:
            return

        if pos.shape[-1] < 3:
            return

        if "toa_s" in bundle_data:
            raw_t = np.asarray(bundle_data["toa_s"]).ravel()
            if len(raw_t) > 0:
                self._latest_time = max(self._latest_time, float(raw_t[-1]))

        # Auto-detect units: if coordinates exceed 10.0 (e.g. 180 cm -> 1.8 m), convert to meters
        if np.max(np.abs(pos)) > 10.0:
            pos = pos / 100.0

        self._render_pose(pos)

        # Update tracking status badge
        num_segments = pos.shape[0]
        self.pose_status_label.setText(f"● TRACKING ({num_segments} Seg)")
        self.pose_status_label.setStyleSheet(
            "color: #22c55e; font-size: 11px; font-weight: bold;"
        )

    def _clear_pose_display(self) -> None:
        """Reset 3D pose visualizer to default standing rest pose."""
        if hasattr(self, "_pose_spine_lines"):
            self._render_pose(self.DEFAULT_REST_POSE)
            self.pose_status_label.setText("● STANDBY")
            self.pose_status_label.setStyleSheet(
                "color: #64748b; font-size: 11px; font-weight: bold;"
            )

    def _extract_timestamps(
        self, bundle_data: dict, num_samples: int, now: float
    ) -> np.ndarray:
        """Extract or compute timestamp array for a given number of samples."""
        if "toa_s" in bundle_data:
            raw_t = np.asarray(bundle_data["toa_s"]).ravel()
            if len(raw_t) == num_samples:
                return raw_t.astype(np.float64)
            if len(raw_t) == 1 and num_samples > 1:
                dt = 1.0 / float(bundle_data.get("sampling_rate_hz", 60.0))
                return np.linspace(
                    raw_t[0] - (num_samples - 1) * dt,
                    raw_t[0],
                    num_samples,
                    dtype=np.float64,
                )

        if num_samples <= 1:
            return np.array([now], dtype=np.float64)
        return np.linspace(
            now - (num_samples - 1) * (1.0 / 60.0), now, num_samples, dtype=np.float64
        )

    def _ensure_class_capacity(self, num_classes: int) -> None:
        """Dynamically expand class names and bar chart elements if more classes arrive."""
        if num_classes <= len(self._class_names):
            return
        old_len = len(self._class_names)
        for i in range(old_len, num_classes):
            self._class_names.append(f"Class {i}")
            ti = pg.TextItem("", color="#cbd5e1", anchor=(0.0, 0.5))
            ti.setFont(QtGui.QFont("Segoe UI", 8, QtGui.QFont.Weight.Bold))
            self.ai_plot.addItem(ti)
            self._ai_text_items.append(ti)

        y_ticks = [(i, name) for i, name in enumerate(self._class_names)]
        self.ai_plot.getAxis("left").setTicks([y_ticks])
        self.ai_plot.setYRange(-0.6, len(self._class_names) - 0.4, padding=0.0)
        self._bar_item.setOpts(y=list(range(len(self._class_names))))

    def _extract_probabilities(self, preds_raw, logits_raw) -> Optional[np.ndarray]:
        """Convert incoming predictions or logits array into normalized probabilities."""
        preds = None
        if preds_raw is not None:
            preds = np.asarray(preds_raw).squeeze()
            if preds.ndim == 0:
                preds = np.array([preds.item()])

        logits = None
        if logits_raw is not None:
            logits = np.asarray(logits_raw).squeeze()
            if logits.ndim == 0:
                logits = np.array([logits.item()])

        # If predictions is a vector across all classes
        if preds is not None and preds.size > 1:
            p = preds.astype(np.float64)
            if np.all(p >= 0.0) and np.max(p) <= 1.0:
                return p
            # If values are raw scores, apply softmax
            shifted = p - np.max(p)
            exp_p = np.exp(shifted)
            sum_exp = np.sum(exp_p)
            return exp_p / sum_exp if sum_exp > 0 else exp_p

        # If logits are provided
        if logits is not None and logits.size > 1:
            l = logits.astype(np.float64)
            shifted = l - np.max(l)
            exp_l = np.exp(shifted)
            sum_exp = np.sum(exp_l)
            return exp_l / sum_exp if sum_exp > 0 else exp_l

        # If predictions is a single discrete class index
        if preds is not None and preds.size == 1:
            pred_idx = int(preds.item())
            n_classes = max(len(self._class_names), pred_idx + 1)
            probs = np.zeros(n_classes, dtype=np.float64)
            if 0 <= pred_idx < n_classes:
                probs[pred_idx] = 1.0
            return probs

        return None

    def _handle_ai_intent(self, bundle_data: dict, now: float) -> None:
        """Parse incoming AI Intent prediction packet and refresh the bar chart."""
        preds_raw = bundle_data.get("predictions")
        logits_raw = bundle_data.get("logits")
        compute_time_raw = bundle_data.get("compute_time_s")
        seq_raw = bundle_data.get("sequence_id")
        toa_raw = bundle_data.get("toa_s")

        if compute_time_raw is not None:
            c_vals = np.asarray(compute_time_raw).ravel()
            if len(c_vals) > 0:
                self._latest_ai_latency_ms = float(c_vals[-1]) * 1000.0

        if seq_raw is not None:
            s_vals = np.asarray(seq_raw).ravel()
            if len(s_vals) > 0:
                self._latest_ai_seq = int(s_vals[-1])

        if toa_raw is not None:
            t_vals = np.asarray(toa_raw).ravel()
            if len(t_vals) > 0:
                self._latest_ai_time = float(t_vals[-1])
                self._latest_time = max(self._latest_time, self._latest_ai_time)

        probs = self._extract_probabilities(preds_raw, logits_raw)
        if probs is not None and len(probs) > 0:
            self._ensure_class_capacity(len(probs))
            self._latest_predictions = probs
            self._render_ai_predictions()

    def _render_ai_predictions(self) -> None:
        """Render the horizontal bar chart for AI intent predictions."""
        if self.is_paused or self._latest_predictions is None:
            return

        probs = self._latest_predictions
        n = len(self._class_names)
        if len(probs) < n:
            padded = np.zeros(n, dtype=np.float64)
            padded[: len(probs)] = probs
            probs = padded
        elif len(probs) > n:
            self._ensure_class_capacity(len(probs))

        top_idx = int(np.argmax(probs))
        top_prob = float(probs[top_idx])
        top_name = (
            self._class_names[top_idx]
            if top_idx < len(self._class_names)
            else f"Class {top_idx}"
        )

        # Update top label and compute time
        if top_prob > 0.0:
            self.ai_top_label.setText(f"Top: {top_name} ({top_prob * 100:.1f}%)")
        else:
            self.ai_top_label.setText("Top: -- (--%)")

        if self._latest_ai_latency_ms > 0:
            self.ai_latency_label.setText(
                f"Latency: {self._latest_ai_latency_ms:.1f} ms"
            )
        elif self._latest_ai_seq > 0:
            self.ai_latency_label.setText(f"Seq: #{self._latest_ai_seq}")

        brushes = []
        for i, p in enumerate(probs):
            if i == top_idx and top_prob > 0.0:
                brushes.append(pg.mkBrush("#38bdf8"))  # Vibrant cyan for top class
            else:
                brushes.append(pg.mkBrush("#334155"))  # Muted slate for others

            if i < len(self._ai_text_items):
                ti = self._ai_text_items[i]
                if p >= 0.005:
                    ti.setText(f" {p * 100:.1f}%")
                    ti.setColor("#38bdf8" if i == top_idx else "#94a3b8")
                else:
                    ti.setText("")
                ti.setPos(min(1.0, float(p)), i)

        self._bar_item.setOpts(width=probs.tolist(), brushes=brushes)

    def _handle_prosthesis_mode(self, bundle_data: dict, now: float) -> None:
        """Parse incoming factual prosthesis mode packet and update indicators."""
        if "mode" not in bundle_data:
            return

        mode_vals = np.asarray(bundle_data["mode"]).ravel()
        if len(mode_vals) == 0:
            return

        latest_mode = int(mode_vals[-1])
        latest_source = 0
        latest_seq = 0
        latest_toa = now

        if "source" in bundle_data:
            src_vals = np.asarray(bundle_data["source"]).ravel()
            if len(src_vals) > 0:
                latest_source = int(src_vals[-1])

        if "sequence_id" in bundle_data:
            seq_vals = np.asarray(bundle_data["sequence_id"]).ravel()
            if len(seq_vals) > 0:
                latest_seq = int(seq_vals[-1])

        if "toa_s" in bundle_data:
            toa_vals = np.asarray(bundle_data["toa_s"]).ravel()
            if len(toa_vals) > 0:
                latest_toa = float(toa_vals[-1])
                self._latest_time = max(self._latest_time, latest_toa)

        self._update_mode_display(
            mode_id=latest_mode,
            source_id=latest_source,
            seq_id=latest_seq,
            toa_s=latest_toa,
        )

    def _update_mode_display(
        self,
        mode_id: int,
        source_id: int = 0,
        seq_id: int = 0,
        toa_s: float = 0.0,
    ) -> None:
        """Update both the header badge and the dedicated sidebar card with the new mode."""
        self._current_mode_id = mode_id
        self._current_mode_source = source_id
        self._current_mode_seq = seq_id
        self._current_mode_time = toa_s

        style = self.MODE_STYLE_CONFIG.get(
            mode_id,
            {
                "name": f"MODE {mode_id}",
                "color": "#cbd5e1",
                "bg": "#1e293b",
                "border": "#475569",
            },
        )
        mode_name = style["name"]
        color = style["color"]
        bg = style["bg"]
        border = style["border"]

        source_str = self.SOURCE_NAMES.get(source_id, f"SRC_{source_id}")

        # Update Header badge
        self.header_mode_badge.setText(f"MODE: {mode_name}")
        self.header_mode_badge.setStyleSheet(f"""
            QLabel {{
                color: {color};
                background-color: {bg};
                border: 1px solid {border};
                border-radius: 4px;
                padding: 3px 12px;
                font-weight: 700;
                font-size: 12px;
                letter-spacing: 0.5px;
            }}
        """)

        # Update Sidebar card
        self.mode_status_dot.setStyleSheet(f"color: {color}; font-size: 14px;")
        self.mode_text_label.setText(mode_name)
        self.mode_text_label.setStyleSheet(f"""
            QLabel {{
                color: {color};
                background-color: {bg};
                border: 1px solid {border};
                border-radius: 6px;
                padding: 8px 14px;
                font-size: 18px;
                font-weight: 800;
                letter-spacing: 1.0px;
            }}
        """)

        time_str = f"{toa_s:.2f} s" if toa_s > 0 else "--"
        self.mode_meta_label.setText(
            f"Source: {source_str} | Seq: #{seq_id} | Time: {time_str}"
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
                        pref = getattr(self, "_imu_signal_preference", "Auto")
                        signal_key = None
                        if pref == "Acceleration" and "acceleration" in bundle_data:
                            signal_key = "acceleration"
                        elif pref == "Gyroscope" and "gyroscope" in bundle_data:
                            signal_key = "gyroscope"
                        else:
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
                                    self._imu_times[loc][ch].extend(t_arr.tolist())
                                    self._imu_values[loc][ch].extend(
                                        samples[:, ch].tolist()
                                    )
                                if t_arr.size > 0:
                                    self._latest_time = max(
                                        self._latest_time, float(t_arr[-1])
                                    )
                                    self._active_imu_source = "Nicla"
                                    if pref == "Auto":
                                        unit_lbl = (
                                            "deg"
                                            if signal_key == "euler"
                                            else (
                                                "deg/s"
                                                if signal_key == "gyroscope"
                                                else "m/s²"
                                            )
                                        )
                                        self._update_imu_y_label(unit_lbl)
                                    if not self.is_paused:
                                        self.status_badge.setText(
                                            "● LIVE STREAMING [NICLA]"
                                        )
                                        self.status_badge.setStyleSheet(
                                            "color: #22c55e; font-weight: bold; font-size: 12px; margin-right: 8px;"
                                        )

                            elif samples.ndim == 1 and len(samples) >= 3:
                                t_arr = self._extract_timestamps(bundle_data, 1, now)
                                for ch in range(3):
                                    self._imu_times[loc][ch].extend(t_arr.tolist())
                                    self._imu_values[loc][ch].append(
                                        float(samples[ch])
                                    )
                                if t_arr.size > 0:
                                    self._latest_time = max(
                                        self._latest_time, float(t_arr[-1])
                                    )
                                    self._active_imu_source = "Nicla"
                                    if pref == "Auto":
                                        unit_lbl = (
                                            "deg"
                                            if signal_key == "euler"
                                            else (
                                                "deg/s"
                                                if signal_key == "gyroscope"
                                                else "m/s²"
                                            )
                                        )
                                        self._update_imu_y_label(unit_lbl)
                                    if not self.is_paused:
                                        self.status_badge.setText(
                                            "● LIVE STREAMING [NICLA]"
                                        )
                                        self.status_badge.setStyleSheet(
                                            "color: #22c55e; font-weight: bold; font-size: 12px; margin-right: 8px;"
                                        )

                # 2. Xsens MVN replay IMU updates ("xsens_motion_trackers")
                elif (
                    bundle_name in ("xsens_motion_trackers", "mvn")
                    or bundle_name.startswith("xsens_motion_trackers")
                ):
                    payload = (
                        bundle_data.get("xsens_motion_trackers", bundle_data)
                        if isinstance(bundle_data, dict)
                        else bundle_data
                    )
                    self._handle_xsens_imu(payload, now)
                    if isinstance(bundle_data, dict) and "xsens_pose" in bundle_data:
                        self._handle_xsens_pose(bundle_data["xsens_pose"], now)

                # 3. Xsens MVN 3D Pose updates ("xsens_pose")
                elif (
                    bundle_name in ("xsens_pose", "pose")
                    or bundle_name.startswith("xsens_pose")
                ):
                    payload = (
                        bundle_data.get("xsens_pose", bundle_data)
                        if isinstance(bundle_data, dict)
                        else bundle_data
                    )
                    self._handle_xsens_pose(payload, now)

                # 4. Motor updates (knee, ankle)
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
                            self._latest_time = max(self._latest_time, float(t_arr[-1]))

                # 4. Live AI Intent prediction updates
                elif bundle_name in ("intent", "ai_intent") or bundle_name.startswith(
                    "intent"
                ):
                    self._handle_ai_intent(bundle_data, now)

                # 5. Factual Prosthesis Mode updates
                elif bundle_name == "mode" or bundle_name.startswith("mode"):
                    self._handle_prosthesis_mode(bundle_data, now)

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

            # Refresh AI intent bar chart
            self._render_ai_predictions()

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
        self.is_windows_closed_event.set()
        event.accept()
