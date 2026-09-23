"""
Filename: hermes/aidwear/prosthesis/state_machines/base.py
Description: Abstract class for the hierarchical exoskeleton controller
    of a predetermined ambulation mode.
"""

from hermes.aidwear.prosthesis.utils.types import (
    NiclaSamples,
    EncoderData,
    MotorId,
    ServoMotorData,
)
from abc import abstractmethod


class ProsthesisStateMachine:
    @abstractmethod
    def step(self) -> None:
        """Cycle the state machine."""
        pass

    @abstractmethod
    def update_sensor_values(
        self,
        nicla_samples: NiclaSamples,
        encoder_samples: dict[MotorId, EncoderData],
        motor_samples: dict[MotorId, ServoMotorData],
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
