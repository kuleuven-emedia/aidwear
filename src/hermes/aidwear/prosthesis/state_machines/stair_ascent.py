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
    StairAscentParameters,
    StairLeadingLegEnum,
    StateEnum,
    StateTransition,
    MotorId,
    EncoderId,
    ModeEnum,
)


class StairAscent(StateMachine, ProsthesisStateMachine):
    # States.
    stance = State(
        value=StateEnum.StairAscent.STANCE.value,
        initial=True,
    )
    push_off = State(
        value=StateEnum.StairAscent.PUSH_OFF.value,
    )
    swing = State(
        value=StateEnum.StairAscent.SWING.value,
    )
    step_up = State(
        value=StateEnum.StairAscent.STEP_UP.value,
    )

    # Transitions.
    cycle = (  ### YOU SHOUD ADD THE "SAFETY" MEasure
        stance.to(stance, unless="is_stance_to_push_off")
        | stance.to(
            push_off, cond="is_stance_to_push_off", on="stance_to_push_off"
        )  # T1
        | push_off.to(push_off, unless="is_push_off_to_swing")
        | push_off.to(swing, cond="is_push_off_to_swing", on="push_off_to_swing")  # T2
        | swing.to(swing, unless="is_swing_to_step_up")
        | swing.to(step_up, cond="is_swing_to_step_up", on="swing_to_step_up")  # T3
        | step_up.to(step_up, unless="is_step_up_to_stance")
        | step_up.to(stance, cond="is_step_up_to_stance", on="step_up_to_stance")  # T4
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_intact_gyr = 0  # θ̇_thigh,intact
        self._thigh_intact_roll = 0  # θ_thigh,intact
        self._thigh_pr_gyr = 0  # θ̇_thigh,pr
        self._thigh_pr_roll = 0  # θ_thigh,pr
        self._inactivity_dur = 0  # counter_inactivity
        self._knee_pr_roll = 0  # θ_knee,pr

        self._angle_knee_reference = 0  # θ_knee,pr reference
        self._angle_ankle_reference = 0  # θ_ankle,pr reference

        self._step_up_dur = 0
        self._knee_encoder_prev_angle = None
        self._knee_velocity = 0.0

        self._torque_knee_reference = 0
        self._torque_ankle_reference = 0

        self._thigh_swing_start = 0
        self._knee_swing_start = 0
        self._knee_thigh_gain = 1.3
        self._gain_step = 0.01

        self._ctx = ctx
        self._K = ctx.K
        # self._motor_latest_data = ctx._motor_latest_data
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # Personalized parameters.
        self._param = StairAscentParameters(
            # --------------------------- Stance -> Push-off (T1) --------------------------
            stance_to_push_off_th_gyr=500,  # 1000,         # θ̇_thigh,intact < 60 deg/s
            stance_to_push_off_th_roll=50,  # θ_thigh,intact > 50 deg
            # --------------------------- Push-off -> Swing (T2) ---------------------------
            push_off_to_swing_th_gyr=0,  # θ̇_thigh,intact > 0 deg/s
            push_off_to_swing_th_roll=30,  # θ_thigh,intact > -30 deg
            push_off_to_swing_pr_roll=-20,  # θ_thigh,pr < 5 deg # before it was -10
            # --------------------------- Swing -> Step-up (T3) ----------------------------
            swing_to_step_up_th_gyr=150,  # θ̇_thigh,pr > -12 deg/s
            swing_to_step_up_th_roll=20,  # θ_thigh,pr < -20 deg
            swing_to_step_up_inactivity_dur=0.8,  # counter_inactivity > 0.5 s
            # --------------------------- Step-up -> Stance (T4) ---------------------------
            step_up_to_stance_th_gyr_range=100,  # θ̇_thigh,pr >< 6 deg/s
            step_up_to_stance_th_roll=10,  # θ_thigh,pr < 10 deg
            # ------------------------------------------------------------------------------
            risetime=0.5,  # risetime for torque reference
        )

        # activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        # activate_position_mode(self._ctx.handle, MotorId.KNEE)

        # Parameters for motor control
        self._swing_stiffness = 0.1
        self._swing_damping = 0.05
        self._swing_current_limit_ma = 1000
        self._knee_velocity = 0.0

        super(StairAscent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # NOTE: this records transition information between states of a state machine.
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )

    def stance_to_push_off(self):
        # activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        # activate_position_mode(self._ctx.handle, MotorId.KNEE)
        print("push")

    def push_off_to_swing(self):
        # activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        # activate_current_mode(self._ctx.handle, MotorId.KNEE)
        self._thigh_swing_start = self._thigh_pr_roll
        self._knee_swing_start = self._knee_pr_roll
        self._angle_knee_reference = self._knee_swing_start
        print("swing")

    def swing_to_step_up(self):
        # activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        # activate_current_mode(self._ctx.handle, MotorId.KNEE)
        self._target_knee_torque = -30
        print("step")

    def step_up_to_stance(self):
        # activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        # activate_position_mode(self._ctx.handle, MotorId.KNEE)
        self._target_knee_torque = 0
        print("stance")

    # T1: Stance -> Push-off
    def is_stance_to_push_off(self):
        return (
            self._thigh_intact_gyr < self._param.stance_to_push_off_th_gyr
            and self._thigh_intact_roll > self._param.stance_to_push_off_th_roll
        )

    # T2: Push-off -> Swing
    def is_push_off_to_swing(self):
        return (
            self._thigh_intact_gyr < self._param.push_off_to_swing_th_gyr
            and self._thigh_intact_roll < self._param.push_off_to_swing_th_roll
            and self._thigh_pr_roll > self._param.push_off_to_swing_pr_roll
        )

    # T3: Swing -> Step-up
    def is_swing_to_step_up(self):
        return (
            self._thigh_pr_gyr < self._param.swing_to_step_up_th_gyr
            and self._thigh_pr_roll > self._param.swing_to_step_up_th_roll
            and self._inactivity_dur > self._param.swing_to_step_up_inactivity_dur
        )

    # T4: Step-up -> Stance
    def is_step_up_to_stance(self):
        return (
            abs(self._thigh_pr_gyr) < self._param.step_up_to_stance_th_gyr_range
            and self._thigh_pr_roll < self._param.step_up_to_stance_th_roll
        )

    # Actions.
    def on_enter_stance(self):
        # pm_set_position_must(self._ctx.handle, MotorId.ANKLE, int(0))
        # pm_set_position_must(self._ctx.handle, MotorId.KNEE, int(0))
        self._ctx.epos.set_target_position(MotorId.ANKLE, int(0))
        self._ctx.epos.set_target_position(MotorId.KNEE, int(0))
        self._step_up_dur = 0
        self._inactivity_dur = 0

    def on_enter_push_off(self):
        self._torque_ankle_reference = 0
        self._ctx.epos.set_target_position(MotorId.KNEE, int(0))

    def on_enter_swing(self):
        # Start the reference from the measured knee angle at swing onset.
        knee_error = self._knee_reference - self._knee_pr_roll

        # TODO: Convert torque to current using a simple linear model.
        self._target_knee_torque = (
            self._swing_stiffness * knee_error
            - self._swing_damping * self._knee_velocity
        )

        knee_current = ((self._target_knee_torque * 8) * 1000) / (
            (5 / 9) * self._knee_pr_roll + 10
        )
        self._knee_current = int(
            np.clip(
                knee_current,
                -self._swing_current_limit_ma,
                self._swing_current_limit_ma,
            )
        )
        self._ctx.epos.set_target_current(MotorId.KNEE, self._knee_current)

        if abs(self._thigh_pr_gyr) < 25:
            self._inactivity_dur += self._gain_step
        else:
            self._inactivity_dur = 0
        # TODO: add motor control logic for swing.

    def on_enter_step_up(self):
        if self._step_up_dur < self._param.risetime:
            self._torque_knee_reference = max(
                self._target_knee_torque,
                self._torque_knee_reference
                * (self._step_up_dur / self._param.risetime),
            )
        knee_current = ((self._target_knee_torque * 8) * 1000) / (
            (5 / 9) * self._knee_pr_roll + 10
        )
        self._knee_current = int(
            np.clip(
                knee_current,
                -self._swing_current_limit_ma,
                self._swing_current_limit_ma,
            )
        )
        self._ctx.epos.set_target_current(MotorId.KNEE, self._knee_current)
        self._step_up_dur += self._gain_step
        # TODO: add motor control logic for step-up.

    def _update_motors_reference(self):
        """Make the knee reference follow changes in the thigh angle."""
        if not hasattr(self, "_thigh_swing_start") or not hasattr(
            self, "_knee_swing_start"
        ):
            return
        thigh_change = self._thigh_pr_roll - self._thigh_swing_start
        self._knee_reference = (
            self._knee_swing_start + self._knee_thigh_gain * thigh_change
        )
        self._ankle_reference = 0

    def update_sensor_values(
        self,
        nicla_samples: NiclaSamples,
        encoder_samples: dict[EncoderId, EncoderData],
        motor_samples: dict[MotorId, ServoMotorData],
        dt: float = 0.01,
    ):
        # NOTE: get called on each loop iteration of the mid-level controller.
        #   Then, transition is invoked and corresponding `on_<state>` event is triggered.

        # TODO: save the variables of interest to the self._*, to use in the next "step()".
        self._thigh_intact_gyr = nicla_samples.thigh_left_gyr
        self._thigh_intact_roll = nicla_samples.thigh_left_roll
        self._shank_intact_gyr = nicla_samples.knee_left_gyr
        self._shank_intact_roll = nicla_samples.knee_left_roll
        # I'm using the right leg as prosthetic one
        self._thigh_pr_gyr = nicla_samples.thigh_right_gyr
        self._shank_pr_gyr = nicla_samples.knee_right_gyr
        self._shank_pr_roll = nicla_samples.knee_right_roll

        self._torso_roll = nicla_samples.torso_angle
        self._thigh_pr_roll = nicla_samples.thigh_right_roll

        self._knee_pr_roll = encoder_samples[EncoderId.KNEE].angle
        # self._knee_pr_roll_timestamp = encoder_samples[EncoderId.KNEE].timestamp

        if self._knee_pr_roll is not None:
            if self._knee_encoder_prev_angle is not None:
                self._knee_velocity = (
                    self._knee_pr_roll - self._knee_encoder_prev_angle
                ) / dt
            else:
                self._knee_velocity = 0.0
            # self._knee_encoder_prev_time = self._knee_pr_roll_timestamp
            self._knee_encoder_prev_angle = self._knee_pr_roll

        # Update the reference trajectory for both motors.
        self._update_motors_reference()

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
