"""
Filename: hermes/aidwear/prosthesis/state_machines/hurdle.py
Description: AidWear-specific state machine for the hierarchical control
    of the sit-to-stand ambulation mode.
"""

from dataclasses import asdict
import numpy as np
from statemachine import Event, State, StateMachine

from hermes.utils.time_utils import get_time

from .base import ProsthesisStateMachine
from ..utils.types import (
    ModeContext,
    PhaseEstimate,
    HurdlesParameters,
    StateEnum,
    StateTransition,
    MotorId,
)


class Hurdle(StateMachine, ProsthesisStateMachine):
    # States.
    stance = State(
        value=StateEnum.Hurdle.STANCE.value,
        initial=True,
    )
    swing = State(
        value=StateEnum.Hurdle.SWING.value,
    )

    # Transitions.
    cycle = (
        stance.to(stance, unless="is_stance_to_swing")
        | stance.to(swing, cond="is_stance_to_swing")  # T1
        | swing.to(swing, unless="is_swing_to_stance")
        | swing.to(stance, cond="is_swing_to_stance")  # T2
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_pr_gyr = 0          # θ̇_thigh,pr
        self._thigh_pr_roll = 0         # θ_thigh,pr
        self._knee_pr_roll = 0          # θ_knee,pr
        self._knee_reference = 0        # θ_knee,pr reference
        self._ankle_reference = 0       # θ_ankle,pr reference
        self._thigh_swing_start = 20

        self._knee_swing_start = 0

        # Positive gain means that the knee follows the thigh in the same
        # direction. Tune this from recorded thigh/knee data.
        self._knee_thigh_gain = 1.3

        self._K = ctx.K
        self._motor_latest_data = ctx.motor_latest_data
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # In a real scenario, we would load personalized parameters from the configuration file.
        # Personalized parameters.
        #        thresholds = ctx.config_manager.get_section("hurdles")["thresholds"]
        #        timings = ctx.config_manager.get_section("hurdles")["timings"]

        # Personalized parameters.
        #        self._param = HurdlesParameters(
        #            stance_to_swing_th_roll_pr = thresholds["stance_to_swing_th_roll_pr"],
        #            stance_to_swing_th_gyr_pr = thresholds[
        #                "stance_to_swing_th_gyr_pr"
        #            ],
        #            swing_to_stance_th_roll_pr = thresholds[
        #                "swing_to_stance_th_roll_pr"
        #            ],
        #        )

        # Personalized parameters.
        self._param = HurdlesParameters(
            # --------------------------- Stance -> Swing (T1) ---------------------------
            stance_to_swing_th_roll_pr=20,           # θ_thigh,pr > 20 deg
            stance_to_swing_th_gyr_pr=500,           # θ̇_thigh,pr > 30 deg/s
            # --------------------------- Swing -> Stance (T2) ---------------------------
            swing_to_stance_th_roll_pr=20,           # θ_thigh,pr < 20 deg
        )

        super(Hurdle, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )

    # T1: Stance -> Swing
    def is_stance_to_swing(self):
        return (
            self._thigh_pr_roll > self._param.stance_to_swing_th_roll_pr
            and self._thigh_pr_gyr > self._param.stance_to_swing_th_gyr_pr
        )

    # T2: Swing -> Stance
    def is_swing_to_stance(self):
        return self._thigh_pr_roll < self._param.swing_to_stance_th_roll_pr

    # Actions.
    def on_enter_stance(self):
        self._knee_reference = self._knee_swing_start
        print("hurdle: stance", flush=True)
        # TODO: add motor control logic for stance.

    def on_enter_swing(self):
        # Start the reference from the measured knee angle at swing onset.
        print("hurdle: swing", flush=True)
        # TODO: add motor control logic for swing.

    def _update_motors_reference(self):
        """Make the knee reference follow changes in the thigh angle."""
        thigh_change = self._thigh_pr_roll - self._thigh_swing_start
        self._knee_reference = (
            self._knee_swing_start + self._knee_thigh_gain * thigh_change
        )
        self._ankle_reference = 0

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
        self._thigh_left_gyr = thigh_left_gyr
        self._thigh_left_roll = thigh_left_roll
        self._knee_left_roll = knee_left_roll
        self._thigh_right_gyr = thigh_right_gyr
        self._thigh_right_roll = thigh_right_roll
        self._knee_right_roll = knee_right_roll
        self._torso_roll = torso_angle

        # The right leg is currently treated as the prosthetic leg.
        self._thigh_pr_gyr = thigh_right_gyr
        self._thigh_pr_roll = thigh_right_roll
        self._knee_pr_roll = knee_right_roll  #! This should be the encoder measurement of the prosthetic knee or the IMU measurement of the thigh.

        if self.current_state == self.swing:
            self._update_motors_reference()

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
