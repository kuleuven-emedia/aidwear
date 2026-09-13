"""
Filename: hermes/aidwear/prosthesis/state_machines/idle.py
Description: Prosthesis-specific state machine for the hierarchical control
    of the idle ambulation mode.
"""

import numpy as np
from statemachine import Event, State, StateMachine

from .base import ProsthesisStateMachine
from ..utils.types import (
    ModeContext,
    ServoMotorEnum,
    StateEnum,
    MotorId,
)


class Idle(StateMachine, ProsthesisStateMachine):
    # States.
    idle = State(value=StateEnum.Idle.IDLE.value, initial=True)

    # Transitions.
    cycle = idle.to(idle)

    def __init__(self, ctx: ModeContext):
        self._ctx = ctx
        self._is_safe = True
        self._i = 0
        self._ctx.token = 0
        super(Idle, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        pass

    # Actions.
    def on_enter_idle(self):
        self._ctx.factor_prev = (0.0, 0.0)
        self._i += 1
        # TODO: add what to do in Idle.

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
        knee_left_gyr: int = 0,
        knee_right_gyr: int = 0,
        dt: float = 0.01,
    ):
        self._thigh_left_angle = thigh_left_angle
        self._thigh_right_angle = thigh_right_angle
        self._knee_left_roll = knee_left_roll
        self._knee_right_roll = knee_right_roll
        self._thigh_right_gyr = thigh_right_gyr

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True

    def send_data(self) -> dict:
        data = {
            "Lth_roll": self._thigh_left_angle,
            "Rth_roll": self._thigh_right_angle,
            "Lkn_roll": self._knee_left_roll,
            "Rkn_roll": self._knee_right_roll,
            "Rth_gyr": self._thigh_right_gyr,
            "phase": 0,
            "Rth_traj": float(0),
            "Lth_traj": float(0),
            "Rkn_traj": float(0),
            "Lkn_traj": float(0),
        }
        return data
