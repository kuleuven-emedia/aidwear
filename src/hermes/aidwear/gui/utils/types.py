"""
Filename: revalexo_gui/utils/types.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-12
Version: 1.0
Description: Types for GUI command interpretation.
"""

from enum import Enum


class GuiCommandType(Enum):
    NULL = -1
    INTENT = 0
    FATIGUE = 1
    SAFETY_STOP = 2
    CALIBRATE_MOTORS = 3
    CALIBRATE_IMUS = 4
