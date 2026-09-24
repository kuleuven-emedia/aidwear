"""
Filename: hermes/aidwear/visualizer/handler.py
Description: Dedicated PyQt6 subprocess isolating slow GUI from fast HERMES sensing.
"""

from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
import sys

from PyQt6 import QtWidgets
import pyqtgraph as pg

from hermes.aidwear.visualizer.utils.ui import VisualizerMainWindow


class VisualizerGuiHandler:
    """Callable target executed inside the dedicated GUI subprocess."""

    def __init__(
        self,
        node_id: str,
        data_queue: Queue,
        is_cleanup_event: _Event,
        is_windows_closed_event: _Event,
        time_window_s: float = 5.0,
        draw_interval_s: float = 0.04,
        dark_mode: bool = True,
    ) -> None:
        self.node_id = node_id
        self.data_queue = data_queue
        self.is_cleanup_event = is_cleanup_event
        self.is_windows_closed_event = is_windows_closed_event
        self.time_window_s = time_window_s
        self.draw_interval_s = draw_interval_s
        self.dark_mode = dark_mode

    def __call__(self) -> None:
        """Start the Qt event loop on the main thread of the GUI subprocess."""
        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication(sys.argv if hasattr(sys, "argv") else [])

        if self.dark_mode:
            pg.setConfigOption("background", "#12131a")
            pg.setConfigOption("foreground", "#e2e8f0")
        else:
            pg.setConfigOption("background", "#f8fafc")
            pg.setConfigOption("foreground", "#0f172a")

        pg.setConfigOption("antialias", True)

        window = VisualizerMainWindow(
            node_id=self.node_id,
            data_queue=self.data_queue,
            is_cleanup_event=self.is_cleanup_event,
            is_windows_closed_event=self._is_windows_closed_event,
            time_window_s=self.time_window_s,
            draw_interval_s=self.draw_interval_s,
            dark_mode=self.dark_mode,
        )
        window.show()

        app.exec()
