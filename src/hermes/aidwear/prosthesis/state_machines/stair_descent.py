"""
Filename: hermes/aidwear/prosthesis/state_machines/stair_descent.py
Description: AidWear-specific state machine for the hierarchical control
    of the stair descent ambulation mode.
"""

import numpy as np
from statemachine import Event, State, StateMachine
from dataclasses import asdict
from hermes.utils.time_utils import get_time
from scipy.interpolate import CubicHermiteSpline

from .base import ProsthesisStateMachine
from ..utils.types import (
    ModeContext,
    ServoImpedanceGains,
    ServoMotorEnum,
    ServoReference,
    StairDescentParameters,
    StairLeadingLegEnum,
    StateEnum,
    StateTransition,
    MotorId,
    ModeEnum,
)


class StairDescent(StateMachine, ProsthesisStateMachine):
    # States.
    idle = State(value=StateEnum.Idle.IDLE.value, initial=True)

    # Transitions.
    cycle = idle.to(idle)

    def __init__(self, ctx: ModeContext):
        self._motor_command_queue = ctx.motor_command_queue
        super(StairDescent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # TODO: record to the state transition queue that sub state changed
        # print(f"Completed transition to {state.name}", flush=True)
        pass

    # Actions.
    def on_enter_idle(self):
        # for the ankle and the knee we a position reference of 0,
        # fully extended for the knee and neutral for the ankle.
        # TODO: add motor control logic for idle.
        pass

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
