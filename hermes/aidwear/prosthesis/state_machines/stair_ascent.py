"""
Filename: hermes/revalexo/exo/state_machines/stair_ascent.py
Author: Elias Thiery <ethiery@vub.be>
Date: 2025-12-10
Version: 1.0
Description: Revalexo-specific state machine for the hierarchical control
    of the stair ascent ambulation mode.
"""

from dataclasses import asdict
import numpy as np
from statemachine import Event, State, StateMachine

from hermes.utils.time_utils import get_time

from .base import ProsthesisStateMachine
from ..can_control.motor_cubemars import can_set_position_impedance, can_set_torque
from ..utils.types import (
    ModeContext,
    ServoImpedanceGains,
    ServoMotorEnum,
    ServoReference,
    StairAscentParameters,
    StairLeadingLegEnum,
    StateEnum,
    StateTransition,
)


class StairAscent(StateMachine, ProsthesisStateMachine):
    # States.
    stance = State(
        value=StateEnum.StairAscent.STANCE.value,
        initial=True
    )
    swing = State(
        value=StateEnum.StairAscent.SWING.value,
    )
    pushoff = State(
        value=StateEnum.StairAscent.PUSH_OFF.value,
    )

    # Transitions.
    cycle = (
        stance.to(stance, unless="stance_to_swing")
        | stance.to(swing, cond="stance_to_swing")
        | swing.to(swing, unless="swing_to_pushoff")
        | swing.to(pushoff, cond="swing_to_pushoff")
        | pushoff.to(pushoff, unless="pushoff_to_stance")
        | pushoff.to(stance, cond="pushoff_to_stance")
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

        self._idle_dur = 0
        self._swing_dur = 0
        self._pushoff_dur = 0

        self._K = ctx.K
        self._bus = ctx.bus
        self._motor_latest_data = ctx.motor_latest_data
        self._fatigue = ctx.fatigue
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # Personalized parameters.
        # TODO: source from `watchdog` observed YAML config file.
        self._param = StairAscentParameters(
            # Transition thresholds.
            stance_to_swing_th_gyr=500,
            stance_to_swing_th_roll=10,
            stance_to_swing_kn_roll=20,
            swing_to_pushoff_gyr_min=-200,
            swing_to_pushoff_gyr_max=200,
            swing_to_pushoff_th_roll=30,
            pushoff_trigger_delay=0.3,
            pushoff_to_stance_gyr_min=-200,
            pushoff_to_stance_gyr_max=200,
            pushoff_to_stance_th_roll=35,
            pushoff_to_stance_roll_small=5,
            pushoff_to_stance_kn_roll=20,
            # Action timing and impedance control.
            imp_gain_rise_time=0.2,
            gain_step=0.01,
            pushoff_kn_threshold=20,
            # Torque scaling.
            torque_scale=5,
            torque_norm_angle=110,
            # Inactivity detection.
            inactivity_gyr_threshold=200,
            inactivity_time_step=0.01,
            # Movement detection.
            movement_angle_threshold=5,
            movement_gyr_threshold=300,
            movement_sum_threshold=10,
        )

        super(StairAscent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )
        # print(f"Completed transition to {state.name}", flush=True)

    def stance_to_swing(self):
        # print(f"Lth_gyr: {self._thigh_left_gyr:.2f}, Lth_roll: {self._thigh_left_roll:.2f}, LKn_roll: {self._knee_left_roll:.2f}")
        return (
            self._thigh_left_gyr > self._param.stance_to_swing_th_gyr
            and self._thigh_left_roll > self._param.stance_to_swing_th_roll
            and self._knee_left_roll > self._param.stance_to_swing_kn_roll
        )

    def swing_to_pushoff(self):
        return (
            self._param.swing_to_pushoff_gyr_min
            < self._thigh_left_gyr
            < self._param.swing_to_pushoff_gyr_max
            and self._thigh_left_roll > self._param.swing_to_pushoff_th_roll
            and self._inactivity_timer > self._param.pushoff_trigger_delay
        )

    def pushoff_to_stance(self):
        return (
            self._param.pushoff_to_stance_gyr_min
            < self._thigh_o_gyr
            < self._param.pushoff_to_stance_gyr_max
            and (
                (
                    self._thigh_left_roll < self._param.pushoff_to_stance_th_roll
                    and self._knee_o_roll > self._param.pushoff_to_stance_kn_roll
                )
                or (
                    self._thigh_left_roll < self._param.pushoff_to_stance_roll_small
                    and self._thigh_o_roll < self._param.pushoff_to_stance_roll_small
                    and self._knee_o_roll < self._param.pushoff_to_stance_roll_small
                )
            )
        )

    # Actions.
    def on_enter_stance(self):
        imp_rise_time = self._param.imp_gain_rise_time

        if self._leading_leg == StairLeadingLegEnum.RIGHT:
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
                        k: min(v * self._idle_dur / imp_rise_time, v)
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
                        k: min(v * self._idle_dur / imp_rise_time, v)
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
            )
        else:
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
                        k: min(v * self._idle_dur / imp_rise_time, v)
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
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
                        k: min(v * self._idle_dur / imp_rise_time, v)
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
            )
        self._idle_dur += self._param.gain_step
        self._swing_dur = 0
        self._pushoff_dur = 0

    def on_enter_swing(self):
        self._idle_dur = 0
        self._pushoff_dur = 0

    def on_enter_pushoff(self):
        imp_rise_time = self._param.imp_gain_rise_time
        torque_scale = self._param.torque_scale
        norm_angle = self._param.torque_norm_angle

        if self._leading_leg == StairLeadingLegEnum.RIGHT:
            if self._thigh_left_roll > self._param.pushoff_kn_threshold:
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
                            k: min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
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
                            k: min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                )
            else:
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
                            k: max(
                                v * (imp_rise_time - self._pushoff_dur) / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
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
                            k: max(
                                v * (imp_rise_time - self._pushoff_dur) / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                )
            can_set_torque(
                bus=self._bus,
                controller_id=2,
                torque=torque_scale,
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
            )
            can_set_torque(
                bus=self._bus,
                controller_id=4,
                torque=-(norm_angle - self._thigh_o_roll) / norm_angle * torque_scale,
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
            )
        else:
            if self._thigh_left_roll > self._param.pushoff_kn_threshold:
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
                            k: min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
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
                            k: min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                )
            else:
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
                            k: max(
                                v * (imp_rise_time - self._pushoff_dur) / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
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
                            k: max(
                                v * (imp_rise_time - self._pushoff_dur) / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                )
            can_set_torque(
                bus=self._bus,
                controller_id=3,
                torque=-torque_scale,
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
            )
            can_set_torque(
                bus=self._bus,
                controller_id=1,
                torque=(norm_angle - self._thigh_o_roll) / norm_angle * torque_scale,
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
            )

        self._pushoff_dur += self._param.gain_step
        self._idle_dur = 0
        self._swing_dur = 0

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
        if (
            np.array([abs(thigh_right_gyr), abs(thigh_left_gyr)])
            < self._param.inactivity_gyr_threshold
        ).all():
            self._inactivity_timer += self._param.inactivity_time_step
        else:
            self._inactivity_timer = 0

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

        # print(f"Lth_gyr: {thigh_left_gyr:.2f}, Lth_roll: {thigh_left_roll:.2f}, LKn_roll: {knee_left_roll:.2f}, leading leg: {self._leading_leg}")
        # print(self.current_state)
        # Push `phase` estimate into the upstream queue.
        # TODO: add phase estimate logic.
        # self._phase_estimate_queue.put(PhaseEstimate(timestamp=get_time(), phase=self._phase))

    def step(self) -> None:
        self.send("cycle")

    def send_data(self) -> dict:
        data = {
            "Lth_gyr": self._thigh_left_gyr,
            "Lth_roll": self._thigh_left_roll,
            "Oth_gyr": self._thigh_o_gyr,
            "Lkn_roll": self._knee_left_roll,
            "state": str(self.current_state),
        }
        return data
