"""
Filename: hermes/aidwear/prosthesis/state_machines/stair_ascent.py
Description: AidWear-specific state machine for the hierarchical control
    of the stair ascent ambulation mode.
"""

from dataclasses import asdict
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from statemachine import Event, State, StateMachine

from hermes.utils.time_utils import get_time

from .base import ProsthesisStateMachine
from ..utils.types import (
    ModeContext,
    ModeEnum,
    ServoImpedanceGains,
    ServoMotorEnum,
    ServoReference,
    StairAscentParameters,
    StairLeadingLegEnum,
    StateEnum,
    StateTransition,
    MotorId,
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
        | stance.to(push_off, cond="is_stance_to_push_off")  # T1
        | push_off.to(push_off, unless="is_push_off_to_swing")
        | push_off.to(swing, cond="is_push_off_to_swing")  # T2
        | swing.to(swing, unless="is_swing_to_step_up")
        | swing.to(step_up, cond="is_swing_to_step_up")  # T3
        | step_up.to(step_up, unless="is_step_up_to_stance")
        | step_up.to(stance, cond="is_step_up_to_stance")  # T4
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_intact_gyr = 0      # θ̇_thigh,intact
        self._thigh_intact_roll = 0     # θ_thigh,intact
        self._thigh_pr_gyr = 0          # θ̇_thigh,pr
        self._thigh_pr_roll = 0         # θ_thigh,pr
        self._inactivity_dur = 0        # counter_inactivity
        self._knee_pr_roll = 0          # θ_knee,pr
        
        self._angle_knee_reference = 0        # θ_knee,pr reference   
        self._angle_ankle_reference = 0       # θ_ankle,pr reference
        
        self._step_up_dur = 0

        self._torque_knee_reference = 0
        self._torque_ankle_reference = 0

        self._thigh_swing_start = 0
        self._knee_swing_start = 0
        self._knee_thigh_gain = 1.3
        self._gain_step = 0.01

        self._K = ctx.K
        self._motor_latest_data = ctx._motor_latest_data
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # In a real scenario, we would load personalized parameters from the configuration file.
        # Personalized parameters.
        #        thresholds = ctx.config_manager.get_section("stairs_ascent")["thresholds"]
        #        timings = ctx.config_manager.get_section("stairs_ascent")["timings"]

        # Personalized parameters.
        #        self._param = StairsAscentParameters(
        #            stance_to_push_off_th_gyr = thresholds["stance_to_push_off_th_gyr"],
        #            stance_to_push_off_th_roll = thresholds[
        #                "stance_to_push_off_th_roll"
        #            ],
        #            push_off_to_swing_th_gyr = thresholds["push_off_to_swing_th_gyr"],
        #            push_off_to_swing_th_roll = thresholds["push_off_to_swing_th_roll"],
        #            push_off_to_swing_pr_roll = thresholds[
        #                "push_off_to_swing_pr_roll"
        #            ],
        #            swing_to_step_up_th_gyr = thresholds["swing_to_step_up_th_gyr"],
        #            swing_to_step_up_th_roll = thresholds["swing_to_step_up_th_roll"],
        #            swing_to_step_up_inactivity_dur = thresholds[
        #                "swing_to_step_up_inactivity_dur"
        #            ],
        #            step_up_to_stance_th_gyr_range = thresholds["step_up_to_stance_th_gyr_range"],
        #            step_up_to_stance_th_roll = thresholds[
        #                "step_up_to_stance_th_roll"
        #            ],
        #            risetime = timings[
        #                "risetime"
        #            ],
        #        )

        # Personalized parameters.
        self._param = StairAscentParameters(
            # --------------------------- Stance -> Push-off (T1) ---------------------------
            stance_to_push_off_th_gyr=1000,         # θ̇_thigh,intact < 60 deg/s
            stance_to_push_off_th_roll=50,          # θ_thigh,intact > 50 deg
            # --------------------------- Push-off -> Swing (T2) ---------------------------
            push_off_to_swing_th_gyr=0,             # θ̇_thigh,intact > 0 deg/s
            push_off_to_swing_th_roll=30,           # θ_thigh,intact > -30 deg
            push_off_to_swing_pr_roll=-5,           # θ_thigh,pr < 5 deg
            # --------------------------- Swing -> Step-up (T3) ---------------------------
            swing_to_step_up_th_gyr=200,            # θ̇_thigh,pr > -12 deg/s
            swing_to_step_up_th_roll=20,            # θ_thigh,pr < -20 deg
            swing_to_step_up_inactivity_dur=0.5,    # counter_inactivity > 0.5 s
            # --------------------------- Step-up -> Stance (T4) ---------------------------
            step_up_to_stance_th_gyr_range=100,     # θ̇_thigh,pr >< 6 deg/s
            step_up_to_stance_th_roll=10,           # θ_thigh,pr < 10 deg

            risetime=0.5,                           # risetime for torque reference
        )

        super(StairAscent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # NOTE: this records transition information between states of a state machine.
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )

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
            and self._thigh_pr_roll > self._param.step_up_to_stance_th_roll
        )

    # Actions.
    def on_enter_stance(self):
        self._step_up_dur = 0

    def on_enter_pushoff(self):
        self._torque_ankle_reference = 0

    def on_enter_swing(self):
        # Start the reference from the measured knee angle at swing onset.
        self._thigh_swing_start = self._thigh_pr_roll
        self._knee_swing_start = self._knee_pr_roll
        self._angle_knee_reference = self._knee_swing_start
        # TODO: add motor control logic for swing.

    def on_enter_step_up(self):
        self._target_knee_torque = 80
        self._torque_knee_reference = min(
            self._target_knee_torque,
            self._torque_knee_reference * (self._step_up_dur / self._param.risetime),
        )
        self._step_up_dur += self._gain_step
        # TODO: add motor control logic for step-up.

    def _update_motors_reference(self):
        """Make the knee reference follow changes in the thigh angle."""
        thigh_change = self._thigh_pr_roll - self._thigh_swing_start
        self._angle_knee_reference = (
            self._knee_swing_start + self._knee_thigh_gain * thigh_change
        )
        self._angle_ankle_reference = 0

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
        ########################################################################################
        # Here the right leg is treated as the prosthetic leg and the left leg as the intact leg#
        ########################################################################################
        self._thigh_intact_gyr = thigh_left_gyr
        self._thigh_intact_roll = thigh_left_roll
        self._thigh_pr_gyr = thigh_right_gyr
        self._thigh_pr_roll = thigh_right_roll
        self._knee_pr_roll = knee_right_roll     ########### this probably will have different name

        if self.current_state == self.swing:
            self._update_motors_reference()

        # print(f"Lth_gyr: {thigh_left_gyr:.2f}, Lth_roll: {thigh_left_roll:.2f}, LKn_roll: {knee_left_roll:.2f}, leading leg: {self._leading_leg}")
        # print(self.current_state)
        # Push `phase` estimate into the upstream queue.
        # TODO: add phase estimate logic.
        # self._phase_estimate_queue.put(PhaseEstimate(timestamp=get_time(), phase=self._phase))

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
