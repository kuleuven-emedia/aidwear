"""
Filename: hermes/aidwear/prosthesis/state_machines/base.py
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
        thigh_left_gyr: float = np.nan,
        thigh_right_gyr: float = np.nan,
        knee_left_gyr: float = np.nan,
        knee_right_gyr: float = np.nan,
        knee_enc_angle: float = np.nan,
        ankle_enc_angle: float = np.nan,
        dt: float = 0.01,
    ) -> None:
        """Does state machine dependent update with new sensor readings."""
        pass

    @abstractmethod
    def is_safe_to_switch(self) -> bool:
        """Checks if switching to the next FSM is allowed.

        Returns:
            bool: Predicate whether switching to the next FSM is allowed.
        """
        pass
