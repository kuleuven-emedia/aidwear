"""
Filename: hermes/aidwear/prosthesis/state_machines/idle.py
Description: Prosthesis-specific state machine for the hierarchical control
    of the idle ambulation mode.
"""

from statemachine import Event, State, StateMachine

from .base import ProsthesisStateMachine
from ..utils.types import (
    ModeContext,
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
        super(Idle, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # NOTE: no need to keep storing Idle->Idle transitions.
        # NOTE: use this to change the drive operation mode when different states use
        #   different low-level controllers. (better yet, have specific transitions ref. `python-statemachine`).
        pass

    # Actions.
    def on_enter_idle(self):
        # NOTE: control of EPOS drives at `dt` of mid-level controller.
        #   Use values updated in `update_sensor_values` to decide when to change target.
        #   Commands will be logged by HERMES and drives will track target themselves,
        #   while reporting current values.
        # Set target positions using the high-level Facade
        #self._ctx.epos.set_target_position(MotorId.ANKLE, 0)
        #self._ctx.epos.set_target_position(MotorId.KNEE, 0)
        pass

    def update_sensor_values(self, **kwargs):
        pass

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
