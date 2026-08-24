"""
Filename: hermes/revalexo/exo/state_machines/stair_descent.py
Author: Elias Thiery <ethiery@vub.be>
Date: 2025-12-10
Version: 1.0
Description: Revalexo-specific state machine for the hierarchical control
    of the stair descent ambulation mode.
"""

import numpy as np
from statemachine import Event, State, StateMachine

from hermes.utils.time_utils import get_time

from .base import ProsthesisStateMachine
from ..can_control.motor_cubemars import can_set_torque
from ..utils.types import (
    ModeContext,
    ServoMotorEnum,
    StairDescentParameters,
    StairLeadingLegEnum,
    StateEnum,
    StateTransition,
)


class StairDescent(StateMachine, ProsthesisStateMachine):
    # States.
    double_support = State(
        value=StateEnum.StairDescent.DOUBLE_SUPPORT.value,
        initial=True
    )
    swing = State(
        value=StateEnum.StairDescent.SWING.value,
    )
    stance = State(
        value=StateEnum.StairDescent.STANCE.value,
    )

    # Transitions.
    start_descent = (
        double_support.to(swing, cond="double_to_swing")
        | double_support.to(double_support, unless="double_to_swing")
    )

    continue_descent = (
        stance.to(stance, unless=["stance_to_swing", "stance_to_double"])
        | stance.to(swing, cond="stance_to_swing")
        | swing.to(swing, unless=["swing_to_stance", "swing_to_double"])
        | swing.to(stance, cond="swing_to_stance")
        | swing.to(double_support, cond="swing_to_double", on="reset")
        | stance.to(double_support, cond="stance_to_double", on="reset")
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_left_gyr = 0
        self._thigh_left_roll = 0
        self._knee_left_roll = 0
        self._thigh_o_gyr = 0
        self._thigh_o_roll = 0
        self._knee_o_roll = 0
        self._inactivity_timer = 0
        self._leading_leg = StairLeadingLegEnum.NONE
        self._tot_movement = [[], []]
        self._swing_dur = 0
        self._stance_dur = 0

        self._K = ctx.K
        self._bus = ctx.bus
        self._motor_latest_data = ctx.motor_latest_data
        self._fatigue = ctx.fatigue
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # Personalized parameters.
        # TODO: source from `watchdog` observed YAML config file.
        self._param = StairDescentParameters(
            # Transition thresholds.
            double_to_swing_th_gyr=500,
            stance_th_gyr_min=-200,
            stance_th_gyr_max=200,
            stance_kn_roll=30,
            stance_to_swing_th_gyr=500,
            stance_to_swing_kn_roll=30,
            swing_to_double_gyr_min=-200,
            swing_to_double_gyr_max=200,
            swing_to_double_th_roll=10,
            swing_to_double_inactive_dur=2,
            stance_to_double_gyr_min=-200,
            stance_to_double_gyr_max=200,
            stance_to_double_th_roll=10,
            stance_to_double_inactive_dur=2,
            # Action parameters.
            angle_activation_threshold=5,
            torque_knee=10,
            torque_hip=1.5,
            ramp_time=0.2,
            gain_step=0.01,
            # Inactivity detection.
            inactivity_gyr_threshold=200,
            inactivity_time_step=0.01,
            # Movement detection.
            movement_angle_threshold=5,
            movement_gyr_threshold=300,
            movement_sum_threshold=10,
        )

        super(StairDescent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )
        # print(f"Completed transition to {state.name}", flush=True)

    def double_to_swing(self):
        return self._thigh_left_gyr > self._param.double_to_swing_th_gyr

    def swing_to_stance(self):
        return (
            self._param.stance_th_gyr_min
            < self._thigh_left_gyr
            < self._param.stance_th_gyr_max
            and self._param.stance_th_gyr_min
            < self._thigh_o_gyr
            < self._param.stance_th_gyr_max
            and self._knee_o_roll > self._param.stance_kn_roll
        )

    def stance_to_swing(self):
        return (
            self._thigh_left_gyr > self._param.stance_to_swing_th_gyr
            and self._knee_left_roll > self._param.stance_to_swing_kn_roll
        )

    def swing_to_double(self):
        return (
            self._param.swing_to_double_gyr_min
            < self._thigh_o_gyr
            < self._param.swing_to_double_gyr_max
            and self._thigh_o_roll < self._param.swing_to_double_th_roll
            and self._inactivity_timer > self._param.swing_to_double_inactive_dur
        )

    def stance_to_double(self):
        return (
            self._param.stance_to_double_gyr_min
            < self._thigh_left_gyr
            < self._param.stance_to_double_gyr_max
            and self._thigh_left_roll < self._param.stance_to_double_th_roll
            and self._inactivity_timer > self._param.stance_to_double_inactive_dur
        )

    # Actions.
    def reset(self):
        pass
    def on_enter_double_support(self):
        pass

    def on_enter_swing(self):
        pass

    def on_enter_stance(self):
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
        # print(self._leading_leg)
        # Inactivity detection.
        if (
            np.array([abs(thigh_right_gyr), abs(thigh_left_gyr)])
            < self._param.inactivity_gyr_threshold
        ).all():
            self._inactivity_timer += self._param.inactivity_time_step
        else:
            self._inactivity_timer = 0

        # Map leading and other leg.
        if self._leading_leg == StairLeadingLegEnum.RIGHT:
            self._thigh_left_gyr, self._thigh_left_roll, self._knee_left_roll = (
                thigh_right_gyr,
                thigh_right_roll,
                knee_right_roll,
            )
            self._thigh_o_gyr, self._thigh_o_roll, self._knee_o_roll = (
                thigh_left_gyr,
                thigh_left_roll,
                knee_left_roll,
            )
        elif self._leading_leg == StairLeadingLegEnum.LEFT:
            self._thigh_left_gyr, self._thigh_left_roll, self._knee_left_roll = (
                thigh_left_gyr,
                thigh_left_roll,
                knee_left_roll,
            )
            self._thigh_o_gyr, self._thigh_o_roll, self._knee_o_roll = (
                thigh_right_gyr,
                thigh_right_roll,
                knee_right_roll,
            )
        else:
            if (
                sum(self._tot_movement[0]) < self._param.movement_sum_threshold
                and sum(self._tot_movement[1]) < self._param.movement_sum_threshold
            ):
                if (
                    thigh_left_roll > self._param.movement_angle_threshold
                    and abs(thigh_left_gyr) > self._param.movement_gyr_threshold
                ):
                    self._tot_movement[0].append(thigh_left_roll)
                if (
                    thigh_right_roll > self._param.movement_angle_threshold
                    and abs(thigh_right_gyr) > self._param.movement_gyr_threshold
                ):
                    self._tot_movement[1].append(thigh_right_roll)
            else:
                self._leading_leg = (
                    StairLeadingLegEnum.LEFT
                    if sum(self._tot_movement[0]) > sum(self._tot_movement[1])
                    else StairLeadingLegEnum.RIGHT
                )

        # print(f"Lth_gyr: {thigh_left_gyr:.2f}, Lth_roll: {thigh_left_roll:.2f}, Lkn_roll: {self._knee_left_roll:.2f}, leading leg: {self._leading_leg}, timeout: {self._inactivity_timer:.2f}")
        # Push `phase` estimate into the upstream queue.
        # TODO: add phase estimate logic.
        # self._phase_estimate_queue.put(PhaseEstimate(timestamp=get_time(), phase=self._phase))

    def step(self) -> None:
        if self.current_state == self.double_support:
            self.send("start_descent")
        else:
            self.send("continue_descent")

    def send_data(self) -> dict:
        data = {
            "Lth_gyr": self._thigh_left_gyr,
            "Lth_roll": self._thigh_left_roll,
            "torso_roll": self._knee_left_roll,
            "phase": self._knee_o_roll,
            "state": str(self.current_state),
        }
        return data
