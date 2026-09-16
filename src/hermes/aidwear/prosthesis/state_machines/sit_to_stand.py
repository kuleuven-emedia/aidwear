"""
Filename: hermes/aidwear/prosthesis/state_machines/sit_to_stand.py
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
    ModeEnum,
    PhaseEstimate,
    ServoImpedanceGains,
    ServoMotorEnum,
    ServoReference,
    SitToStandParameters,
    StateEnum,
    StateTransition,
    MotorId,
)


class SitToStand(StateMachine, ProsthesisStateMachine):
    # States.
    stance = State(value=StateEnum.SitToStand.STANCE.value, initial=True)
    lowering = State(
        value=StateEnum.SitToStand.LOWERING.value,
    )
    sitting = State(
        value=StateEnum.SitToStand.SITTING.value,
    )
    rising = State(
        value=StateEnum.SitToStand.RISING.value,
    )

    # Transitions.
    cycle = (
        stance.to(stance, unless="is_stance_to_lowering")
        | stance.to(lowering, cond="is_stance_to_lowering")
        | lowering.to(lowering, unless="is_lowering_to_sitting")
        | lowering.to(sitting, cond="is_lowering_to_sitting")
        | sitting.to(sitting, unless="is_sitting_to_rising")
        | sitting.to(rising, cond="is_sitting_to_rising")
        | rising.to(rising, unless="is_rising_to_stance")
        | rising.to(stance, cond="is_rising_to_stance")
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_left_gyr = 0
        self._thigh_left_roll = 0
        self._knee_left_roll = 0
        self._thigh_right_gyr = 0
        self._thigh_right_roll = 0
        self._knee_right_roll = 0
        self._torso_roll = 0
        self._phase = 0
        self._idle_dur = 0.00000001
        self._active_dur = 0
        self._is_safe = False

        self._ctx = ctx

        # # Personalized parameters.
        # thresholds = ctx.config_manager.get_section("sit_to_stand")["thresholds"]
        # timings = ctx.config_manager.get_section("sit_to_stand")["timings"]
        # phase = ctx.config_manager.get_section("sit_to_stand")["phase"]
        # self._is_biodex = ctx.config_manager.get_section("biodex")["enabled"]

        # self._param = SitToStandParameters(
        #     # Transition thresholds.
        #     stance_to_lowering_th_gyr=thresholds["stance_to_lowering_th_gyr"],
        #     stance_to_lowering_th_roll=thresholds["stance_to_lowering_th_roll"],
        #     stance_to_lowering_torso_roll=thresholds["stance_to_lowering_torso_roll"],
        #     stance_to_lowering_kn_roll=thresholds["stance_to_lowering_kn_roll"],
        #     lowering_to_sitting_th_gyr_min=thresholds["lowering_to_sitting_th_gyr_min"],
        #     lowering_to_sitting_th_gyr_max=thresholds["lowering_to_sitting_th_gyr_max"],
        #     lowering_to_sitting_phase_threshold=thresholds[
        #         "lowering_to_sitting_phase_threshold"
        #     ],
        #     sitting_to_rising_th_gyr=thresholds["sitting_to_rising_th_gyr"],
        #     sitting_to_rising_phase_threshold=thresholds[
        #         "sitting_to_rising_phase_threshold"
        #     ],
        #     sitting_to_rising_idle_dur_min=thresholds["sitting_to_rising_idle_dur_min"],
        #     rising_to_stance_th_gyr_min=thresholds["rising_to_stance_th_gyr_min"],
        #     rising_to_stance_th_gyr_max=thresholds["rising_to_stance_th_gyr_max"],
        #     rising_to_stance_kn_roll=thresholds["rising_to_stance_kn_roll"],
        #     rising_to_stance_phase_threshold=thresholds[
        #         "rising_to_stance_phase_threshold"
        #     ],
        #     reset_phase_threshold=thresholds["reset_phase_threshold"],
        #     inactivity_angle_threshold=thresholds["inactivity_angle_threshold"],
        #     # Impedance gains and timing.
        #     imp_gain_rise_time=timings["imp_gain_rise_time"],
        #     gain_step=timings["gain_step"],
        #     active_ramp_time=timings["active_ramp_time"],
        #     # Phase calculation parameters.
        #     phase_start_angle=phase["phase_start_angle"],
        #     phase_end_angle=phase["phase_end_angle"],
        # )

        super(SitToStand, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        if not self._is_stop_new_data_event.is_set():
            self._state_changed_queue.put(
                StateTransition(timestamp=get_time(), state=state.value)
            )

    def is_stance_to_lowering(self):
        return (
            self._thigh_right_gyr > self._param.stance_to_lowering_th_gyr
            and self._thigh_left_gyr > self._param.stance_to_lowering_th_gyr
            and self._thigh_right_roll > self._param.stance_to_lowering_th_roll
            and self._thigh_left_roll > self._param.stance_to_lowering_th_roll
            and self._torso_roll > self._param.stance_to_lowering_torso_roll
            and self._knee_left_roll > self._param.stance_to_lowering_kn_roll
            and self._knee_right_roll > self._param.stance_to_lowering_kn_roll
        )

    def is_lowering_to_sitting(self):
        return (
            self._param.lowering_to_sitting_th_gyr_min
            < self._thigh_right_gyr
            < self._param.lowering_to_sitting_th_gyr_max
            and self._param.lowering_to_sitting_th_gyr_min
            < self._thigh_left_gyr
            < self._param.lowering_to_sitting_th_gyr_max
            and self._phase > self._param.lowering_to_sitting_phase_threshold
        )

    def is_sitting_to_rising(self):
        return (
            self._thigh_right_gyr > self._param.sitting_to_rising_th_gyr
            and self._thigh_left_gyr > self._param.sitting_to_rising_th_gyr
            and self._phase > self._param.sitting_to_rising_phase_threshold
            and self._idle_dur >= self._param.sitting_to_rising_idle_dur_min
        )

    def is_rising_to_stance(self):
        return (
            # self._param.rising_to_stance_th_gyr_min
            # < self._thigh_right_gyr
            # < self._param.rising_to_stance_th_gyr_max
            # and self._param.rising_to_stance_th_gyr_min
            # < self._thigh_left_gyr
            # < self._param.rising_to_stance_th_gyr_max and
            self._knee_left_roll < self._param.rising_to_stance_kn_roll
            and self._knee_right_roll < self._param.rising_to_stance_kn_roll
            and self._phase < self._param.rising_to_stance_phase_threshold
        )

    # Actions.
    def on_enter_stance(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value
        # TODO: add motor control logic for stance.

    def on_enter_lowering(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value
        # TODO: add motor control logic for lowering.

    def on_enter_sitting(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value
        # TODO: add motor control logic for sitting.

    def on_enter_rising(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value
        # TODO: add motor control logic for rising.

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
        knee_left_gyr: int = 0,
        knee_right_gyr: int = 0,
        dt: float = 0.01,
    ):
        self._thigh_left_gyr = thigh_left_gyr
        self._thigh_left_roll = thigh_left_roll
        self._knee_left_roll = knee_left_roll
        self._thigh_right_gyr = thigh_right_gyr
        self._thigh_right_roll = thigh_right_roll
        self._knee_right_roll = knee_right_roll
        self._torso_roll = torso_angle

        # Phase now uses configurable start and end angles.
        start_angle = self._param.phase_start_angle
        end_angle = self._param.phase_end_angle
        avg_roll = (thigh_left_roll + thigh_right_roll) / 2
        self._phase = (start_angle - avg_roll) / (start_angle - end_angle) * 100

        # Push `phase` estimate into the upstream queue.
        if not self._is_stop_new_data_event.is_set():
            self._phase_estimate_queue.put(
                PhaseEstimate(timestamp=get_time(), phase=self._phase)
            )

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
