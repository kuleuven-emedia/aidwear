"""
Filename: hermes/revalexo/exo/state_machines/idle.py
Author: Stijn Hamelryckx <stijn.hamelryckx@gmail.com>
Date: 2025-12-10
Version: 1.0
Description: Revalexo-specific state machine for the hierarchical control
    of the idle ambulation mode.
"""

import numpy as np
from statemachine import Event, State, StateMachine

from .base import ProsthesisStateMachine
from ..can_control.motor_cubemars import can_set_torque
from ..utils.types import (
    ModeContext,
    ServoMotorEnum,
    StateEnum,
)


class Idle(StateMachine, ProsthesisStateMachine):
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
        super(Idle, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        pass
        # print(f"Completed transition to {state.name}", flush=True)

    # Actions.
    def on_enter_idle(self):
        can_set_torque(
            bus=self._bus,
            controller_id=1,
            torque=0.0,
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_torque(
            bus=self._bus,
            controller_id=4,
            torque=0.0,
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_torque(
            bus=self._bus,
            controller_id=2,
            torque=0.0,
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_torque(
            bus=self._bus,
            controller_id=3,
            torque=0.0,
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )

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
        pass

    def step(self) -> None:
        self.send("cycle")

    def send_data(self) -> dict:
        return {}
