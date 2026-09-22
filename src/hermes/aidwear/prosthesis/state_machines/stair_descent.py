"""
Filename: hermes/aidwear/prosthesis/state_machines/stair_descent.py
Description: AidWear-specific state machine for the hierarchical control
    of the stair descent ambulation mode.
"""

import numpy as np
from hermes.aidwear.prosthesis.utils.types import (
    NiclaSamples,
    ServoMotorData,
    EncoderData,
)

from statemachine import Event, State, StateMachine
from hermes.utils.time_utils import get_time
from .base import ProsthesisStateMachine

from ..motor_control.epos_commands import (
    activate_position_mode,
    pm_set_position_must,
    activate_current_mode,
    cm_set_current_must,
)

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
    EncoderId,
    ModeEnum,
)


class StairDescent(StateMachine, ProsthesisStateMachine):
    # States.
    idle = State(value=StateEnum.Idle.IDLE.value, initial=True)

    # Transitions.
    cycle = (
        idle.to(idle)
    )

    def __init__(self, ctx: ModeContext):
        # Parameters for motor control
        self._swing_stiffness = 0.15
        self._swing_damping = 0.06
        self._swing_current_limit_ma = 1000
        self._knee_velocity = 0.0

        self._ctx = ctx
        self._K = ctx.K
        self._state_changed_queue = ctx.state_changed_queue
        self._motor_command_queue = ctx.motor_command_queue

        activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        activate_current_mode(self._ctx.handle, MotorId.KNEE)

        super(StairDescent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )

    # Actions.
    def on_enter_idle(self):
        # for the ankle and the knee we a position reference of 0,
        # fully extended for the knee and neutral for the ankle.
       
        # KNEE: impedance control from absolute encoder angle/velocity.
        knee_error = 0.0 - self._knee_pr_roll
        # TODO: Convert torque to current using a simple linear model.
        knee_torque = (
            self._swing_stiffness * knee_error
            - self._swing_damping * self._knee_velocity
        )

        knee_current = ((knee_torque * 8)*1000) / ((5 / 9) * self._knee_pr_roll + 10)
        knee_current = int(
            np.clip(knee_current, -self._swing_current_limit_ma, self._swing_current_limit_ma)
        )
        self._knee_current = knee_current

        cm_set_current_must(self._ctx.handle, MotorId.KNEE, knee_current)
        # ANKLE: stays in position mode at reference.
        pm_set_position_must(self._ctx.handle, MotorId.ANKLE, int(0))
            
    def update_sensor_values(
        self,
        nicla_samples: NiclaSamples,
        #nicla_euler_samples: NiclaSamples,
        encoder_samples: dict[EncoderId, EncoderData],
        motor_samples: dict[MotorId, ServoMotorData],
        dt: float = 0.01,
    ):
        # NOTE: get called on each loop iteration of the mid-level controller.
        #   Then, transition is invoked and corresponding `on_<state>` event is triggered.

        # TODO: save the variables of interest to the self._*, to use in the next "step()".
        self._thigh_left_gyr = nicla_samples.thigh_left_gyr
        #print(nicla_samples)
        self._thigh_left_roll = nicla_samples.thigh_left_roll
        self._shank_left_gyr = nicla_samples.knee_left_gyr
        self._shank_left_roll = nicla_samples.knee_left_roll
        # I'm using the right leg as prosthetic one
        self._thigh_pr_gyr = nicla_samples.thigh_right_gyr
        self._shank_pr_gyr = nicla_samples.knee_right_gyr
        self._shank_pr_roll = nicla_samples.knee_right_roll

        self._torso_roll = nicla_samples.torso_angle
        self._thigh_pr_roll = nicla_samples.thigh_right_roll
        self._knee_pr_roll = encoder_samples[EncoderId.KNEE].angle 

        if self._knee_pr_roll is not None:
            if self._knee_encoder_prev_angle is not None:
                self._knee_velocity = (self._knee_pr_roll - self._knee_encoder_prev_angle) / dt
            else:
                self._knee_velocity = 0.0
            #self._knee_encoder_prev_time = self._knee_pr_roll_timestamp
            self._knee_encoder_prev_angle = self._knee_pr_roll
        

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
