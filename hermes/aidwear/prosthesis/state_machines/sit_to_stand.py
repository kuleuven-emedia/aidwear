"""
Filename: hermes/revalexo/exo/state_machines/sit_to_stand.py
Author: Elias Thiery <ethiery@vub.be>
Date: 2025-12-10
Version: 1.0
Description: Revalexo-specific state machine for the hierarchical control
    of the sit-to-stand ambulation mode.
"""

from dataclasses import asdict
import numpy as np
from statemachine import Event, State, StateMachine

from hermes.utils.time_utils import get_time

from .base import ProsthesisStateMachine
from ..can_control.motor_cubemars import can_set_position_impedance
from ..utils.types import (
    ModeContext,
    PhaseEstimate,
    ServoImpedanceGains,
    ServoMotorEnum,
    ServoReference,
    SitToStandParameters,
    StateEnum,
    StateTransition,
)


class SitToStand(StateMachine, ProsthesisStateMachine):
    # States.
    stance = State(
        value=StateEnum.SitToStand.STANCE.value,
        initial=True
    )
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
        stance.to(stance, unless="stance_to_lowering")
        | stance.to(lowering, cond="stance_to_lowering")
        | lowering.to(lowering, unless="lowering_to_sitting")
        | lowering.to(sitting, cond="lowering_to_sitting")
        | sitting.to(sitting, unless="sitting_to_rising")
        | sitting.to(rising, cond="sitting_to_rising")
        | rising.to(rising, unless="rising_to_stance")
        | rising.to(stance, cond="rising_to_stance")
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
        self._idle_dur = 0
        self._active_dur = 0

        self._K = ctx.K
        self._bus = ctx.bus
        self._motor_latest_data = ctx.motor_latest_data
        self._fatigue = ctx.fatigue
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # Personalized parameters.
        # TODO: source from `watchdog` observed YAML config file.
        self._param = SitToStandParameters(
            # Transition thresholds.
            stance_to_lowering_th_gyr=500,
            stance_to_lowering_th_roll=20,
            stance_to_lowering_torso_roll=5,
            stance_to_lowering_kn_roll=10,
            lowering_to_sitting_th_gyr_min=-200,
            lowering_to_sitting_th_gyr_max=200,
            lowering_to_sitting_phase_threshold=85,
            sitting_to_rising_th_gyr=500,
            sitting_to_rising_phase_threshold=80,
            sitting_to_rising_idle_dur_min=0.5,
            rising_to_stance_th_gyr_min=-200,
            rising_to_stance_th_gyr_max=200,
            rising_to_stance_kn_roll=20,
            rising_to_stance_phase_threshold=50,
            reset_phase_threshold=95,
            inactivity_angle_threshold=2,
            # Impedance gains and timing.
            imp_gain_rise_time=0.2,
            gain_step=0.01,
            active_ramp_time=0.2,
            # Phase calculation parameters.
            phase_start_angle=10,
            phase_end_angle=80,
        )

        super(SitToStand, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )
        # print(f"Completed transition to {state.name}", flush=True)

    def stance_to_lowering(self):
        return (
            self._thigh_right_gyr > self._param.stance_to_lowering_th_gyr
            and self._thigh_left_gyr > self._param.stance_to_lowering_th_gyr
            and self._thigh_right_roll > self._param.stance_to_lowering_th_roll
            and self._thigh_left_roll > self._param.stance_to_lowering_th_roll
            and self._torso_roll > self._param.stance_to_lowering_torso_roll
            and self._knee_left_roll > self._param.stance_to_lowering_kn_roll
            and self._knee_right_roll > self._param.stance_to_lowering_kn_roll
        )

    def lowering_to_sitting(self):
        return (
            self._param.lowering_to_sitting_th_gyr_min
            < self._thigh_right_gyr
            < self._param.lowering_to_sitting_th_gyr_max
            and self._param.lowering_to_sitting_th_gyr_min
            < self._thigh_left_gyr
            < self._param.lowering_to_sitting_th_gyr_max
            and self._phase > self._param.lowering_to_sitting_phase_threshold
        )

    def sitting_to_rising(self):
        return (
            self._thigh_right_gyr > self._param.sitting_to_rising_th_gyr
            and self._thigh_left_gyr > self._param.sitting_to_rising_th_gyr
            and self._phase > self._param.sitting_to_rising_phase_threshold
            and self._idle_dur >= self._param.sitting_to_rising_idle_dur_min
        )

    def rising_to_stance(self):
        return (
            self._param.rising_to_stance_th_gyr_min
            < self._thigh_right_gyr
            < self._param.rising_to_stance_th_gyr_max
            and self._param.rising_to_stance_th_gyr_min
            < self._thigh_left_gyr
            < self._param.rising_to_stance_th_gyr_max
            and self._knee_left_roll < self._param.rising_to_stance_kn_roll
            and self._knee_right_roll < self._param.rising_to_stance_kn_roll
            and self._phase < self._param.rising_to_stance_phase_threshold
        )

    # Actions.
    def on_enter_stance(self):
        imp_rise_time = self._param.imp_gain_rise_time
        can_set_position_impedance(
            bus=self._bus,
            controller_id=1,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=2,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=3,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=4,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )

        self._idle_dur += self._param.gain_step
        self._active_dur = 0

    def on_enter_lowering(self):
        ramp_time = self._param.active_ramp_time
        can_set_position_impedance(
            bus=self._bus,
            controller_id=1,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=2,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=3,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=4,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )

        self._active_dur += self._param.gain_step
        self._idle_dur = 0

    def on_enter_sitting(self):
        imp_rise_time = self._param.imp_gain_rise_time
        can_set_position_impedance(
            bus=self._bus,
            controller_id=1,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=2,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=3,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=4,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )

        self._idle_dur += self._param.gain_step
        self._active_dur = 0

    def on_enter_rising(self):
        ramp_time = self._param.active_ramp_time
        can_set_position_impedance(
            bus=self._bus,
            controller_id=1,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=2,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=3,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK80_8,
            motor_command_queue=self._motor_command_queue,
        )
        can_set_position_impedance(
            bus=self._bus,
            controller_id=4,
            motor_latest_data=self._motor_latest_data,
            ref=ServoReference(
                position=0,
                velocity=0,
                acceleration=0,
            ),
            K=ServoImpedanceGains(
                **{
                    k: min(v * self._active_dur / ramp_time, v)
                    for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                }
            ),
            motor_type=ServoMotorEnum.AK10_9,
            motor_command_queue=self._motor_command_queue,
        )

        self._active_dur += self._param.gain_step
        self._idle_dur = 0

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

        # Phase now uses configurable start and end angles.
        start_angle = self._param.phase_start_angle
        end_angle = self._param.phase_end_angle
        avg_roll = (thigh_left_roll + thigh_right_roll) / 2
        self._phase = (start_angle - avg_roll) / (start_angle - end_angle) * 100

        # Push `phase` estimate into the upstream queue.
        self._phase_estimate_queue.put(
            PhaseEstimate(timestamp=get_time(), phase=self._phase)
        )

        # print(f"Lth_gyr: {self._thigh_left_gyr:.2f}, Lth_roll: {self._thigh_left_roll:.2f}, torso_roll: {self._torso_roll:.2f}, Lknee_roll: {self._knee_left_roll:.2f}, phase: {self._phase:.2f}")

    def step(self) -> None:
        self.send("cycle")

    def send_data(self) -> dict:
        data = {
            "Lth_gyr": self._thigh_left_gyr,
            "Lth_roll": self._thigh_left_roll,
            "torso_roll": self._torso_roll,
            "phase": self._phase,
            "state": str(self.current_state),
        }
        return data
