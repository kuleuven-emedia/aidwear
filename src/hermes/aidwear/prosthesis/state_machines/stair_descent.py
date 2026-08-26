"""
Filename: hermes/aidwear/prosthesis/state_machines/stair_descent.py
Description: AidWear-specific state machine for the hierarchical control
    of the stair descent ambulation mode.
"""

import numpy as np
from statemachine import Event, State, StateMachine
from dataclasses import asdict
from hermes.utils.time_utils import get_time
from scipy.interpolate import CubicHermiteSpline

from .base import ProsthesisStateMachine
from ..can_control.motor_epos import can_set_position_impedance, can_set_torque
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
    ModeEnum,
)


class StairDescent(StateMachine, ProsthesisStateMachine):
    # States.
    double_support = State(
        value=StateEnum.StairDescent.DOUBLE_SUPPORT.value, initial=True
    )
    swing = State(
        value=StateEnum.StairDescent.SWING.value,
    )
    stance = State(
        value=StateEnum.StairDescent.STANCE.value,
    )

    # Transitions.
    start_descent = double_support.to(
        swing, cond="double_to_swing"
    ) | double_support.to(double_support, unless="double_to_swing")

    continue_descent = (
        stance.to(stance, unless=["stance_to_swing", "stance_to_double"])
        | stance.to(swing, cond="stance_to_swing")
        | swing.to(swing, unless=["swing_to_stance", "swing_to_double"])
        | swing.to(stance, cond="swing_to_stance")
        | swing.to(double_support, cond="swing_to_double", on="reset")
        | stance.to(double_support, cond="stance_to_double", on="reset")
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_leading_leg_gyr = 0
        self._thigh_leading_leg_roll = 0
        self._knee_leading_leg_roll = 0
        self._knee_leading_leg_gyr = 0
        self._thigh_lagging_leg_gyr = 0
        self._thigh_lagging_leg_roll = 0
        self._knee_lagging_leg_roll = 0
        self._knee_lagging_leg_gyr = 0

        self._inactivity_timer = 0
        self._leading_leg = StairLeadingLegEnum.NONE
        self._tot_movement = [[], []]
        self._swing_dur = 0
        self._stance_dur = 0
        self._transition_dur = 0

        self._is_toe_off = True
        self._is_heel_strike = True
        self._is_switch = False
        self._is_safe = False
        self._is_biodex = ctx.config_manager.get_section("biodex")["enabled"]

        self._ctx = ctx

        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value

        self._kp_hip = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.26
                for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
            }
        ).position
        self._kd_hip = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.26
                for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
            }
        ).velocity
        self._kp_knee = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.19
                for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
            }
        ).position
        self._kd_knee = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.19
                for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
            }
        ).velocity

        # Personalized parameters.
        params = ctx.config_manager.get_section("stair_descent")
        thresholds = params["thresholds"]
        action = params["action"]
        inactivity = params["inactivity"]
        movement = params["movement"]

        self._param = StairDescentParameters(
            # Transition thresholds.
            double_to_swing_th_gyr=thresholds["double_to_swing_th_gyr"],
            stance_th_gyr_min=thresholds["stance_th_gyr_min"],
            stance_th_gyr_max=thresholds["stance_th_gyr_max"],
            stance_kn_roll=thresholds["stance_kn_roll"],
            stance_to_swing_th_gyr=thresholds["stance_to_swing_th_gyr"],
            stance_to_swing_kn_roll=thresholds["stance_to_swing_kn_roll"],
            swing_to_double_gyr_min=thresholds["swing_to_double_gyr_min"],
            swing_to_double_gyr_max=thresholds["swing_to_double_gyr_max"],
            swing_to_double_th_roll=thresholds["swing_to_double_th_roll"],
            swing_to_double_inactive_dur=thresholds["swing_to_double_inactive_dur"],
            stance_to_double_gyr_min=thresholds["stance_to_double_gyr_min"],
            stance_to_double_gyr_max=thresholds["stance_to_double_gyr_max"],
            stance_to_double_th_roll=thresholds["stance_to_double_th_roll"],
            stance_to_double_inactive_dur=thresholds["stance_to_double_inactive_dur"],
            # Action parameters.
            angle_activation_threshold=action["angle_activation_threshold"],
            torque_knee=action["torque_knee"],
            torque_hip=action["torque_hip"],
            ramp_time=action["ramp_time"],
            gain_step=action["gain_step"],
            # Inactivity detection.
            inactivity_gyr_threshold=inactivity["inactivity_gyr_threshold"],
            inactivity_time_step=inactivity["inactivity_time_step"],
            # Movement detection.
            movement_angle_threshold=movement["movement_angle_threshold"],
            movement_gyr_threshold=movement["movement_gyr_threshold"],
            movement_sum_threshold=movement["movement_sum_threshold"],
        )

        super(StairDescent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        if not self._is_stop_new_data_event.is_set():
            self._state_changed_queue.put(
                StateTransition(timestamp=get_time(), state=state.value)
            )

    def double_to_swing(self):
        return self._thigh_leading_leg_gyr > self._param.double_to_swing_th_gyr

    def swing_to_stance(self):
        return (
            self._thigh_lagging_leg_gyr > self._param.stance_to_swing_th_gyr
            and self._knee_lagging_leg_roll > self._param.stance_kn_roll
            and self._knee_leading_leg_roll < self._param.stance_to_swing_kn_roll
        ) and not self._is_switch

    def stance_to_swing(self):
        return (
            self._thigh_leading_leg_gyr > self._param.stance_to_swing_th_gyr
            and self._knee_leading_leg_roll > self._param.stance_to_swing_kn_roll
            and self._knee_lagging_leg_roll < self._param.stance_kn_roll
        ) and not self._is_switch

    def swing_to_double(self):
        return (
            self._param.swing_to_double_gyr_min
            < self._thigh_lagging_leg_gyr
            < self._param.swing_to_double_gyr_max
            and self._thigh_lagging_leg_roll < self._param.swing_to_double_th_roll
            and self._inactivity_timer > self._param.swing_to_double_inactive_dur
        )

    def stance_to_double(self):
        return (
            self._param.stance_to_double_gyr_min
            < self._thigh_leading_leg_gyr
            < self._param.stance_to_double_gyr_max
            and self._thigh_leading_leg_roll < self._param.stance_to_double_th_roll
            and self._inactivity_timer > self._param.stance_to_double_inactive_dur
        )

    # Actions.
    def reset(self):
        self._leading_leg = StairLeadingLegEnum.NONE
        self._tot_movement = [[], []]

    def on_enter_double_support(self):
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value

        if next_mode != ModeEnum.STAIR_DESCENT.value.id:
            self._is_safe = True

    def on_enter_swing(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value
        self._is_heel_strike = True

        if next_mode == ModeEnum.WALKING.value and self._is_toe_off:
            motor_data_hip_right = self._motor_latest_data[MotorId.HIP_RIGHT][-1]
            motor_data_hip_left = self._motor_latest_data[MotorId.HIP_LEFT][-1]
            motor_data_knee_right = self._motor_latest_data[MotorId.KNEE_RIGHT][-1]
            motor_data_knee_left = self._motor_latest_data[MotorId.KNEE_LEFT][-1]

            ref_position_hip_right = (
                motor_data_hip_right.current * (8 * 0.69 * 0.199)
                + self._kd_hip * motor_data_hip_right.velocity
            ) / self._kp_hip + (motor_data_hip_right.position)
            ref_position_hip_left = (
                motor_data_hip_left.current * (8 * 0.69 * 0.199)
                + self._kd_hip * motor_data_hip_left.velocity
            ) / self._kp_hip + (motor_data_hip_left.position)
            ref_position_knee_right = (
                motor_data_knee_right.current * (9 * 0.921914 * 0.16)
                + self._kd_knee * motor_data_knee_right.velocity
            ) / self._kp_knee + (motor_data_knee_right.position)
            ref_position_knee_left = (
                motor_data_knee_left.current * (9 * 0.921914 * 0.16)
                + self._kd_knee * motor_data_knee_left.velocity
            ) / self._kp_knee + (motor_data_knee_left.position)

            if self._leading_leg == StairLeadingLegEnum.RIGHT:
                self._spline_hip_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_right, -25],
                    [motor_data_hip_right.velocity, 0],
                )
                self._spline_hip_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_left, -5],
                    [motor_data_hip_left.velocity, 0],
                )
                self._spline_knee_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_right, 5 / 2],
                    [motor_data_knee_right.velocity, 0],
                )
                self._spline_knee_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_left, -30 / 2],
                    [motor_data_knee_left.velocity, 0],
                )

                self._spline_len = 0
                self._token.value = 1
                self._is_switch = True
            elif self._leading_leg == StairLeadingLegEnum.LEFT:
                self._spline_hip_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_right, 5],
                    [motor_data_hip_right.velocity, 0],
                )
                self._spline_hip_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_left, 25],
                    [motor_data_hip_left.velocity, 0],
                )
                self._spline_knee_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_right, 30 / 2],
                    [motor_data_knee_right.velocity, 0],
                )
                self._spline_knee_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_left, -5 / 2],
                    [motor_data_knee_left.velocity, 0],
                )
                self._spline_len = 0
                self._token.value = 2
                self._is_switch = True

        if self._is_switch:
            transition_dur = 0.6
            n = int(transition_dur / self._param.gain_step)
            x = np.linspace(0, 1, n)

            angle_hip_right = self._spline_hip_right(x[self._spline_len])
            velocity_hip_right = self._spline_hip_right.derivative()(
                x[self._spline_len]
            )
            angle_hip_left = self._spline_hip_left(x[self._spline_len])
            velocity_hip_left = self._spline_hip_left.derivative()(x[self._spline_len])
            angle_knee_right = self._spline_knee_right(x[self._spline_len])
            velocity_knee_right = self._spline_knee_right.derivative()(
                x[self._spline_len]
            )
            angle_knee_left = self._spline_knee_left(x[self._spline_len])
            velocity_knee_left = self._spline_knee_left.derivative()(
                x[self._spline_len]
            )

            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.HIP_RIGHT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_hip_right,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.HIP_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.HIP_LEFT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_hip_left,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.HIP_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.KNEE_RIGHT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_knee_right,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.KNEE_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.KNEE_LEFT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_knee_left,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.KNEE_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            self._spline_len += 1
            if self._spline_len == n - 2:
                self._is_switch = False
                self._is_safe = True
        else:
            if self._knee_lagging_leg_roll > self._param.angle_activation_threshold:
                factor = min(self._swing_dur / self._param.ramp_time, 1)
                if self._leading_leg == StairLeadingLegEnum.RIGHT:
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_LEFT,
                        torque=-1.125
                        * self.performance_factor(MotorId.HIP_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * factor,
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_LEFT,
                        torque=1.125
                        * self.performance_factor(MotorId.KNEE_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * factor,
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_RIGHT,
                        torque=-1.125
                        * self.performance_factor(MotorId.KNEE_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * max(
                            (self._param.ramp_time - self._swing_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_RIGHT,
                        torque=1.125
                        * self.performance_factor(MotorId.HIP_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * max(
                            (self._param.ramp_time - self._swing_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                else:
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_RIGHT,
                        torque=1.125
                        * self.performance_factor(MotorId.HIP_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * factor,
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_RIGHT,
                        torque=-1.125
                        * self.performance_factor(MotorId.KNEE_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * factor,
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_LEFT,
                        torque=-1.125
                        * self.performance_factor(MotorId.HIP_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * max(
                            (self._param.ramp_time - self._swing_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_LEFT,
                        torque=1.125
                        * self.performance_factor(MotorId.KNEE_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * max(
                            (self._param.ramp_time - self._swing_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                self._swing_dur += self._param.gain_step
            self._stance_dur = 0
        self._is_toe_off = False

    def on_enter_stance(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value

        imp_rise_time = self._param.ramp_time / 2
        if next_mode == ModeEnum.SIT_TO_STAND.value.id:
            self._token.value = 0
            factor = min(self._stance_dur / self._param.ramp_time, 1) * max(
                (imp_rise_time - self._transition_dur) / imp_rise_time, 0
            )
            self._transition_dur += self._param.gain_step
            self._is_safe = True
        else:
            factor = min(self._stance_dur / self._param.ramp_time, 1)
            self._transition_dur = 0
            if not self._is_switch:
                self._stance_dur += self._param.gain_step
        self._is_toe_off = True

        if next_mode == ModeEnum.WALKING.value.id and self._is_heel_strike:
            motor_data_hip_right = self._motor_latest_data[MotorId.HIP_RIGHT][-1]
            motor_data_hip_left = self._motor_latest_data[MotorId.HIP_LEFT][-1]
            motor_data_knee_right = self._motor_latest_data[MotorId.KNEE_RIGHT][-1]
            motor_data_knee_left = self._motor_latest_data[MotorId.KNEE_LEFT][-1]

            ref_position_hip_right = (
                motor_data_hip_right.current * (8 * 0.69 * 0.199)
                + self._kd_hip * motor_data_hip_right.velocity
            ) / self._kp_hip + (motor_data_hip_right.position)
            ref_position_hip_left = (
                motor_data_hip_left.current * (8 * 0.69 * 0.199)
                + self._kd_hip * motor_data_hip_left.velocity
            ) / self._kp_hip + (motor_data_hip_left.position)
            ref_position_knee_right = (
                motor_data_knee_right.current * (9 * 0.921914 * 0.16)
                + self._kd_knee * motor_data_knee_right.velocity
            ) / self._kp_knee + (motor_data_knee_right.position)
            ref_position_knee_left = (
                motor_data_knee_left.current * (9 * 0.921914 * 0.16)
                + self._kd_knee * motor_data_knee_left.velocity
            ) / self._kp_knee + (motor_data_knee_left.position)

            if self._leading_leg == StairLeadingLegEnum.RIGHT:
                self._spline_hip_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_right, 5],
                    [motor_data_hip_right.velocity, 0],
                )
                self._spline_hip_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_left, 25],
                    [motor_data_hip_left.velocity, 0],
                )
                self._spline_knee_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_right, 30 / 2],
                    [motor_data_knee_right.velocity, 0],
                )
                self._spline_knee_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_left, -5 / 2],
                    [motor_data_knee_left.velocity, 0],
                )
                self._is_switch = True
                self._token.value = 2
                self._spline_len = 0
            elif self._leading_leg == StairLeadingLegEnum.LEFT:
                self._spline_hip_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_right, -25],
                    [motor_data_hip_right.velocity, 0],
                )
                self._spline_hip_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_hip_left, -5],
                    [motor_data_hip_left.velocity, 0],
                )
                self._spline_knee_right = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_right, 5 / 2],
                    [motor_data_knee_right.velocity, 0],
                )
                self._spline_knee_left = CubicHermiteSpline(
                    [0, 1],
                    [ref_position_knee_left, -30 / 2],
                    [motor_data_knee_left.velocity, 0],
                )
                self._is_switch = True
                self._token.value = 1
                self._spline_len = 0

        if self._is_switch:
            transition_dur = 0.6
            n = int(transition_dur / self._param.gain_step)
            x = np.linspace(0, 1, n)

            angle_hip_right = self._spline_hip_right(x[self._spline_len])
            velocity_hip_right = self._spline_hip_right.derivative()(
                x[self._spline_len]
            )
            angle_hip_left = self._spline_hip_left(x[self._spline_len])
            velocity_hip_left = self._spline_hip_left.derivative()(x[self._spline_len])
            angle_knee_right = self._spline_knee_right(x[self._spline_len])
            velocity_knee_right = self._spline_knee_right.derivative()(
                x[self._spline_len]
            )
            angle_knee_left = self._spline_knee_left(x[self._spline_len])
            velocity_knee_left = self._spline_knee_left.derivative()(
                x[self._spline_len]
            )

            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.HIP_RIGHT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_hip_right,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.HIP_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.HIP_LEFT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_hip_left,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.HIP_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.KNEE_RIGHT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_knee_right,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.KNEE_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.KNEE_LEFT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=angle_knee_left,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: self.performance_factor(MotorId.KNEE_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * v
                        * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            self._spline_len += 1
            if self._spline_len == n - 1:
                self._is_switch = False
                self._is_safe = True
        else:
            if self._knee_leading_leg_roll > self._param.angle_activation_threshold:
                if self._stance_dur >= self._param.ramp_time:
                    self._token.value = 0

                if self._leading_leg == StairLeadingLegEnum.RIGHT:
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_RIGHT,
                        torque=-1.125
                        * self.performance_factor(MotorId.KNEE_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * factor,
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_RIGHT,
                        torque=1.125
                        * self.performance_factor(MotorId.HIP_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * factor,
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_LEFT,
                        torque=-1.125
                        * self.performance_factor(MotorId.HIP_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * max(
                            (self._param.ramp_time - self._stance_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_LEFT,
                        torque=1.125
                        * self.performance_factor(MotorId.KNEE_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * max(
                            (self._param.ramp_time - self._stance_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                else:
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_LEFT,
                        torque=-1.125
                        * self.performance_factor(MotorId.HIP_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * factor,
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_LEFT,
                        torque=1.125
                        * self.performance_factor(MotorId.KNEE_LEFT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * factor,
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.HIP_RIGHT,
                        torque=1.125
                        * self.performance_factor(MotorId.HIP_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_hip
                        * max(
                            (self._param.ramp_time - self._stance_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK80_8,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                    can_set_torque(
                        bus=self._bus,
                        motor_id=MotorId.KNEE_RIGHT,
                        torque=-1.125
                        * self.performance_factor(MotorId.KNEE_RIGHT)
                        * max(0.2, int(next_fatigue) / 100)
                        * self._param.torque_knee
                        * max(
                            (self._param.ramp_time - self._stance_dur)
                            / self._param.ramp_time,
                            0,
                        ),
                        motor_type=ServoMotorEnum.AK10_9,
                        motor_command_queue=self._motor_command_queue,
                        is_keep_data_event=self._is_keep_data_event,
                    )
                self._stance_dur += self._param.gain_step
            self._swing_dur = 0
        self._is_heel_strike = False

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
            (
                self._thigh_leading_leg_gyr,
                self._thigh_leading_leg_roll,
                self._knee_leading_leg_roll,
                self._knee_leading_leg_gyr,
            ) = (
                thigh_right_gyr,
                thigh_right_roll,
                knee_right_roll,
                knee_right_gyr,
            )
            (
                self._thigh_lagging_leg_gyr,
                self._thigh_lagging_leg_roll,
                self._knee_lagging_leg_roll,
                self._knee_lagging_leg_gyr,
            ) = (thigh_left_gyr, thigh_left_roll, knee_left_roll, knee_left_gyr)
        elif self._leading_leg == StairLeadingLegEnum.LEFT:
            (
                self._thigh_leading_leg_gyr,
                self._thigh_leading_leg_roll,
                self._knee_leading_leg_roll,
                self._knee_leading_leg_gyr,
            ) = (
                thigh_left_gyr,
                thigh_left_roll,
                knee_left_roll,
                knee_left_gyr,
            )
            (
                self._thigh_lagging_leg_gyr,
                self._thigh_lagging_leg_roll,
                self._knee_lagging_leg_roll,
                self._knee_lagging_leg_gyr,
            ) = (
                thigh_right_gyr,
                thigh_right_roll,
                knee_right_roll,
                knee_right_gyr,
            )
        else:
            if self._token.value == 1:
                self._leading_leg = StairLeadingLegEnum.RIGHT
            elif self._token.value == 2:
                self._leading_leg = StairLeadingLegEnum.LEFT
            else:
                if (
                    sum(self._tot_movement[0]) < self._param.movement_sum_threshold
                    and sum(self._tot_movement[1]) < self._param.movement_sum_threshold
                ):
                    if abs(thigh_left_gyr) > self._param.movement_gyr_threshold:
                        self._tot_movement[0].append(thigh_left_roll)
                    if abs(thigh_right_gyr) > self._param.movement_gyr_threshold:
                        self._tot_movement[1].append(thigh_right_roll)
                else:
                    self._leading_leg = (
                        StairLeadingLegEnum.LEFT
                        if sum(self._tot_movement[0]) > sum(self._tot_movement[1])
                        else StairLeadingLegEnum.RIGHT
                    )

        # Push `phase` estimate into the upstream queue.
        # TODO: add phase estimate logic.
        # if not self._is_stop_new_data_event.is_set():
        #   self._phase_estimate_queue.put(PhaseEstimate(timestamp=get_time(), phase=self._phase))

    def step(self) -> None:
        if self.current_state == self.double_support:
            self.send("start_descent")
        else:
            self.send("continue_descent")

    def is_safe_to_switch(self) -> bool:
        return self._is_safe

    def send_data(self) -> dict:
        data = {
            "Lth_gyr": self._thigh_leading_leg_gyr,
            "Lth_roll": self._thigh_leading_leg_roll,
            "torso_roll": self._knee_leading_leg_roll,
            "phase": self._knee_lagging_leg_roll,
            "state": str(self.current_state),
        }
        return data
