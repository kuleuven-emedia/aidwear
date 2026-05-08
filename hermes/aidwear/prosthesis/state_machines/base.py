"""
Filename: hermes/revalexo/exo/state_machines/base.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2025-12-10
Version: 1.0
Description: Abstract class for the hierarchical exoskeleton controller
    of a predetermined ambulation mode.
"""

from abc import abstractmethod
import numpy as np


class ProsthesisStateMachine:
    @abstractmethod
    def step(self) -> None:
        """Cycle the state machine."""
        pass

    @abstractmethod
    def update_sensor_values(
        self,
        torso_angle: float = np.nan,
        thigh_left_angle: float = np.nan,
        thigh_right_angle: float = np.nan,
        thigh_left_roll: float = np.nan,
        thigh_right_roll: float = np.nan,
        knee_left_roll: float = np.nan,
        knee_right_roll: float = np.nan,
        thigh_left_gyr: int = 0,
        thigh_right_gyr: int = 0,
        dt: float = 0.01,
    ) -> None:
        """Does state machine dependent update with new sensor readings."""
        pass

    @abstractmethod
    def send_data(self) -> dict:
        """Gets data from the state machine to send over MQTT.

        Returns:
            dict: Data for MQTT client to plot for visualization.
        """
        pass
