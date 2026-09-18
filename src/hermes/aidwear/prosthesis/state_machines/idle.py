"""
Filename: hermes/aidwear/prosthesis/state_machines/idle.py
Description: Prosthesis-specific state machine for the hierarchical control
    of the idle ambulation mode.
"""

from statemachine import Event, State, StateMachine

from .base import ProsthesisStateMachine
from ..motor_control.epos_commands import activate_position_mode, pm_set_position_must
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
        self._is_enabled = False
        self._is_activated = False
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
        if self._is_enabled and not self._is_activated:
            activate_position_mode(self._ctx.handle, MotorId.ANKLE)
            activate_position_mode(self._ctx.handle, MotorId.KNEE)
        
            # TODO: replace later with live auto-loaded configs from the YAML file via the ConfigManager.
            # NOTE: this will allow live changes like in LabView (must move the position command to the `on_enter_idle` then).
            pm_set_position_must(self._ctx.handle, MotorId.ANKLE, 0)
            pm_set_position_must(self._ctx.handle, MotorId.KNEE, 0)
            self._is_activated = True

    def update_sensor_values(self, **kwargs):
        # TODO: update some local variables.
        self._is_enabled = True
        super().update_sensor_values(**kwargs)

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
