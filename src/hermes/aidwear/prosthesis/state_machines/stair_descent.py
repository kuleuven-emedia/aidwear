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
    idle = State(
        value=StateEnum.Idle.IDLE.value,
        initial=True
    )

    # Transitions.
    cycle = idle.to(idle)

    def __init__(self, ctx: ModeContext):
        self._bus = ctx.bus
        self._motor_command_queue = ctx.motor_command_queue
        super(StairDescent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        pass
        # print(f"Completed transition to {state.name}", flush=True)

    # Actions.
    def on_enter_idle(self):
        # for the ankle and the knee we a position reference of 0, 
        # fully extended for the knee and neutral for the ankle.
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

    def send_data(self) -> dict:
        data = {
            "Lth_roll": self._thigh_left_angle,
            "Rth_roll": self._thigh_right_angle,
            "Lkn_roll": self._knee_left_roll,
            "Rkn_roll": self._knee_right_roll,
            "Rth_gyr": self._thigh_right_gyr,
            "phase": 0,
            "Pkn_traj": float(0),
            "Pan_traj": float(0),
        }
        return data 
    
#    def send_data(self) -> dict:
        data = {
            "Lth_gyr": self._thigh_leading_leg_gyr,
            "Lth_roll": self._thigh_leading_leg_roll,
            "torso_roll": self._knee_leading_leg_roll,
            "phase": self._knee_lagging_leg_roll,
            "state": str(self.current_state),
        }
        return data
    
#    def step(self) -> None:
        if self.current_state == self.double_support:
            self.send("start_descent")
        else:
            self.send("continue_descent")