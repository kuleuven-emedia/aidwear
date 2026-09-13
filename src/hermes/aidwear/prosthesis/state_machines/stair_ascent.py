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
    stance = State(value=StateEnum.StairAscent.STANCE.value, initial=True)
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
        self._is_toe_off = True
        self._is_heel_strike = True
        self._is_switch = False
        self._is_safe = False
        self._is_biodex = ctx.config_manager.get_section("biodex")["enabled"]

        self._idle_dur = 0
        self._swing_dur = 0
        self._pushoff_dur = 0
        self._pushoff_dur_init = 0
        self._transition_dur = 0

        self._ctx = ctx
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value

        self._kp_hip = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.35
                for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
            }
        ).position
        self._kd_hip = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.35
                for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
            }
        ).velocity
        self._kp_knee = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.25
                for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
            }
        ).position
        self._kd_knee = ServoImpedanceGains(
            **{
                k: max(0.2, int(next_fatigue) / 100) * v * 0.25
                for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
            }
        ).velocity

        # Personalized parameters.
        params = ctx.config_manager.get_section("stair_ascent")
        thresholds = params["thresholds"]
        action = params["action"]
        torque = params["torque"]
        inactivity = params["inactivity"]
        movement = params["movement"]

        self._param = StairAscentParameters(
            # Transition thresholds.
            stance_to_swing_th_gyr=thresholds["stance_to_swing_th_gyr"],
            stance_to_swing_th_roll=thresholds["stance_to_swing_th_roll"],
            stance_to_swing_kn_roll=thresholds["stance_to_swing_kn_roll"],
            swing_to_pushoff_gyr_min=thresholds["swing_to_pushoff_gyr_min"],
            swing_to_pushoff_gyr_max=thresholds["swing_to_pushoff_gyr_max"],
            swing_to_pushoff_th_roll=thresholds["swing_to_pushoff_th_roll"],
            pushoff_trigger_delay=thresholds["pushoff_trigger_delay"],
            pushoff_to_stance_gyr_min=thresholds["pushoff_to_stance_gyr_min"],
            pushoff_to_stance_gyr_max=thresholds["pushoff_to_stance_gyr_max"],
            pushoff_to_stance_th_roll=thresholds["pushoff_to_stance_th_roll"],
            pushoff_to_stance_roll_small=thresholds["pushoff_to_stance_roll_small"],
            pushoff_to_stance_kn_roll=thresholds["pushoff_to_stance_kn_roll"],
            # Action timing and impedance control.
            imp_gain_rise_time=action["imp_gain_rise_time"]
            * max(0.2, int(next_fatigue) / 100),
            gain_step=action["gain_step"],
            pushoff_kn_threshold=action["pushoff_kn_threshold"],
            # Torque scaling.
            torque_scale=torque["torque_scale"],
            torque_norm_angle=torque["torque_norm_angle"],
            # Inactivity detection.
            inactivity_gyr_threshold=inactivity["inactivity_gyr_threshold"],
            inactivity_time_step=inactivity["inactivity_time_step"],
            # Movement detection.
            movement_angle_threshold=movement["movement_angle_threshold"],
            movement_gyr_threshold=movement["movement_gyr_threshold"],
            movement_sum_threshold=movement["movement_sum_threshold"],
        )

        super(StairAscent, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        if not self._is_stop_new_data_event.is_set():
            self._state_changed_queue.put(
                StateTransition(timestamp=get_time(), state=state.value)
            )

    def stance_to_swing(self):
        return (
            self._thigh_leading_leg_gyr > self._param.stance_to_swing_th_gyr
            and self._thigh_leading_leg_roll > self._param.stance_to_swing_th_roll
            and self._knee_leading_leg_roll > self._param.stance_to_swing_kn_roll
        ) and not self._is_switch

    def swing_to_pushoff(self):
        return (
            self._param.swing_to_pushoff_gyr_min
            < self._thigh_leading_leg_gyr
            < self._param.swing_to_pushoff_gyr_max
            and self._thigh_leading_leg_roll > self._param.swing_to_pushoff_th_roll
            and self._inactivity_timer >= self._param.pushoff_trigger_delay
        ) and not self._is_switch

    def pushoff_to_stance(self):
        return (
            self._param.pushoff_to_stance_gyr_min
            < self._thigh_lagging_leg_gyr
            < self._param.pushoff_to_stance_gyr_max
            and (
                (
                    self._thigh_leading_leg_roll < self._param.pushoff_to_stance_th_roll
                    and self._knee_lagging_leg_roll
                    > self._param.pushoff_to_stance_kn_roll
                )
                or (
                    self._thigh_leading_leg_roll
                    < self._param.pushoff_to_stance_roll_small
                    and self._thigh_lagging_leg_roll
                    < self._param.pushoff_to_stance_roll_small
                    and self._knee_lagging_leg_roll
                    < self._param.pushoff_to_stance_roll_small
                )
            )
        )

    # Actions.
    def on_enter_stance(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value

        imp_rise_time = self._param.imp_gain_rise_time
        if not self._is_switch:
            if self._token.value == 0:
                factor = 0
                factor2 = 0
            else:
                factor = 1
                factor2 = 1

        if (
            next_mode == ModeEnum.SIT_TO_STAND.value.id
            and self._token.value == 0
        ):
            self._is_safe = True

        elif (
            next_mode == ModeEnum.WALKING.value.id and self._is_heel_strike
        ):
            motor_data_hip_right = self._motor_latest_data[MotorId.HIP_RIGHT][-1]
            motor_data_hip_left = self._motor_latest_data[MotorId.HIP_LEFT][-1]
            motor_data_knee_right = self._motor_latest_data[MotorId.KNEE_RIGHT][-1]
            motor_data_knee_left = self._motor_latest_data[MotorId.KNEE_LEFT][-1]

            if self._leading_leg == StairLeadingLegEnum.RIGHT:
                ref_position_hip_right = (
                    motor_data_hip_right.current * (8 * 0.69 * 0.199)
                    + self._kd_hip * motor_data_hip_right.velocity
                ) / (self._kp_hip) + (
                    motor_data_hip_right.position
                )  # these offsets are position offsets just like in cubemars.py
                ref_position_hip_left = motor_data_hip_left.position
                ref_position_knee_right = (
                    motor_data_knee_right.current * (9 * 0.921914 * 0.16)
                    + self._kd_knee * motor_data_knee_right.velocity
                ) / (self._kp_knee) + (motor_data_knee_right.position)
                ref_position_knee_left = motor_data_knee_left.position

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
                self._is_heel_strike = False
                self._is_switch = True

            elif self._leading_leg == StairLeadingLegEnum.LEFT:
                ref_position_hip_right = (
                    motor_data_hip_right.position
                )  # these offsets are position offsets just like in cubemars.py
                ref_position_hip_left = (
                    motor_data_hip_left.current * (8 * 0.69 * 0.199)
                    + self._kd_hip * motor_data_hip_left.velocity
                ) / (self._kp_hip) + (motor_data_hip_left.position)
                ref_position_knee_right = motor_data_knee_right.position
                ref_position_knee_left = (
                    motor_data_knee_left.current * (9 * 0.921914 * 0.16)
                    + self._kd_knee * motor_data_knee_left.velocity
                ) / (self._kp_knee) + (motor_data_knee_left.position)

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
                self._is_heel_strike = False
                self._is_switch = True

        if self._is_switch:
            transition_smoothing_dur = 0.6
            n = int(transition_smoothing_dur / self._param.gain_step)
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
            if self._spline_len >= n - 2:
                self._is_switch = False
                self._is_safe = True

        else:
            if self._leading_leg == StairLeadingLegEnum.RIGHT:
                if factor != 0:
                    factor = max((imp_rise_time - self._idle_dur) / imp_rise_time, 0)

                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )

                if factor2 != 0:
                    factor2 = min(self._idle_dur + 0.01 / imp_rise_time, 1)
                if next_mode == ModeEnum.SIT_TO_STAND.value.id:
                    factor2 = max(
                        min((self._idle_dur - self._transition_dur) / imp_rise_time, 1)
                        - self._transition_dur / imp_rise_time,
                        0,
                    )
                    self._transition_dur += self._param.gain_step
                    if factor2 == 0:
                        self._token.value = 0
                        self._is_safe = True

                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor2
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor2
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
            else:
                if factor != 0:
                    factor = max((imp_rise_time - self._idle_dur) / imp_rise_time, 0)

                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )

                if factor2 != 0:
                    factor2 = min(self._idle_dur + 0.01 / imp_rise_time, 1)
                if next_mode == ModeEnum.SIT_TO_STAND.value.id:
                    factor2 = max(
                        min((self._idle_dur - self._transition_dur) / imp_rise_time, 1)
                        - self._transition_dur / imp_rise_time,
                        0,
                    )
                    self._transition_dur += self._param.gain_step
                    self._idle_dur -= self._param.gain_step
                    if factor2 == 0:
                        self._token.value = 0
                        self._is_safe = True

                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor2
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor2
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )

            self._idle_dur += self._param.gain_step
            self._swing_dur = 0
            self._pushoff_dur = 0
            self._pushoff_dur_init = 0
            self._is_heel_strike = False

    def on_enter_swing(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value

        self._is_heel_strike = True
        self._idle_dur = 0
        self._pushoff_dur = 0
        imp_rise_time = self._param.imp_gain_rise_time

        if self._token.value == 2 and not self._is_switch:
            self._token.value = 3

        if next_mode == ModeEnum.WALKING.value.id and self._is_toe_off:
            motor_data_hip_right = self._motor_latest_data[MotorId.HIP_RIGHT][-1]
            motor_data_hip_left = self._motor_latest_data[MotorId.HIP_LEFT][-1]
            motor_data_knee_right = self._motor_latest_data[MotorId.KNEE_RIGHT][-1]
            motor_data_knee_left = self._motor_latest_data[MotorId.KNEE_LEFT][-1]

            if self._leading_leg == StairLeadingLegEnum.RIGHT:
                ref_position_hip_right = motor_data_hip_right.position
                ref_position_hip_left = (
                    motor_data_hip_left.current * (8 * 0.69 * 0.199)
                    + self._kd_hip * motor_data_hip_left.velocity
                ) / self._kp_hip + (motor_data_hip_left.position)
                ref_position_knee_right = motor_data_knee_right.position
                ref_position_knee_left = (
                    motor_data_knee_left.current * (9 * 0.921914 * 0.16)
                    + self._kd_knee * motor_data_knee_left.velocity
                ) / self._kp_knee + (motor_data_knee_left.position)

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
                ref_position_hip_right = (
                    motor_data_hip_right.current * (8 * 0.69 * 0.199)
                    + self._kd_hip * motor_data_hip_right.velocity
                ) / self._kp_hip + (motor_data_hip_right.position)
                ref_position_hip_left = motor_data_hip_left.position
                ref_position_knee_right = (
                    motor_data_knee_right.current * (9 * 0.921914 * 0.16)
                    + self._kd_knee * motor_data_knee_right.velocity
                ) / self._kp_knee + (motor_data_knee_right.position)
                ref_position_knee_left = motor_data_knee_left.position

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
            transition_due = 0.6
            n = int(transition_due / self._param.gain_step)
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
            if self._spline_len >= n - 2:
                self._is_switch = False
                self._is_safe = True
        else:
            factor = max(imp_rise_time - self._swing_dur / imp_rise_time, 0)
            if self._leading_leg == StairLeadingLegEnum.RIGHT:
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
            else:
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
        self._is_toe_off = False
        self._swing_dur += self._param.gain_step

    def on_enter_pushoff(self):
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value

        self._is_toe_off = True
        imp_rise_time = self._param.imp_gain_rise_time
        torque_scale = self._param.torque_scale
        norm_angle = self._param.torque_norm_angle
        if self._leading_leg == StairLeadingLegEnum.RIGHT:
            if self._thigh_leading_leg_roll > self._param.pushoff_kn_threshold:
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                self._pushoff_dur += self._param.gain_step
                self._pushoff_dur_init = max(imp_rise_time - self._pushoff_dur, 0)
            else:
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * max(
                                v
                                * (imp_rise_time - self._pushoff_dur_init)
                                / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=0,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_RIGHT)
                            * max(0.2, int(next_fatigue) / 100)
                            * max(
                                v
                                * (imp_rise_time - self._pushoff_dur_init)
                                / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                self._pushoff_dur_init += self._param.gain_step
                self._pushoff_dur = min(imp_rise_time - self._pushoff_dur_init, 0)
            can_set_torque(
                bus=self._bus,
                motor_id=MotorId.HIP_LEFT,
                torque=-self.performance_factor(MotorId.HIP_LEFT)
                * max(0.2, int(next_fatigue) / 100)
                * torque_scale,
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_torque(
                bus=self._bus,
                motor_id=MotorId.KNEE_LEFT,
                torque=self.performance_factor(MotorId.KNEE_LEFT)
                * max(0.2, int(next_fatigue) / 100)
                * (norm_angle - self._thigh_lagging_leg_roll)
                / norm_angle
                * torque_scale,
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
        else:
            if self._thigh_leading_leg_roll > self._param.pushoff_kn_threshold:
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(position=0, velocity=0, acceleration=0),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(position=0, velocity=0, acceleration=0),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * min(v * self._pushoff_dur / imp_rise_time, v)
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                self._pushoff_dur += self._param.gain_step
                self._pushoff_dur_init = max(imp_rise_time - self._pushoff_dur, 0)
            else:
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.KNEE_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(position=0, velocity=0, acceleration=0),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.KNEE_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * max(
                                v
                                * (imp_rise_time - self._pushoff_dur_init)
                                / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(position=0, velocity=0, acceleration=0),
                    K=ServoImpedanceGains(
                        **{
                            k: self.performance_factor(MotorId.HIP_LEFT)
                            * max(0.2, int(next_fatigue) / 100)
                            * max(
                                v
                                * (imp_rise_time - self._pushoff_dur_init)
                                / imp_rise_time,
                                0,
                            )
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK80_8.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                self._pushoff_dur_init += self._param.gain_step
                self._pushoff_dur = min(imp_rise_time - self._pushoff_dur_init, 0)
            can_set_torque(
                bus=self._bus,
                motor_id=MotorId.HIP_RIGHT,
                torque=self.performance_factor(MotorId.HIP_RIGHT)
                * max(0.2, int(next_fatigue) / 100)
                * torque_scale,
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_torque(
                bus=self._bus,
                motor_id=MotorId.KNEE_RIGHT,
                torque=-self.performance_factor(MotorId.KNEE_RIGHT)
                * max(0.2, int(next_fatigue) / 100)
                * (norm_angle - self._thigh_lagging_leg_roll)
                / norm_angle
                * torque_scale,
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
        self._idle_dur = 0
        self._swing_dur = 0
        self._token.value = 3

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
        if (
            np.array([abs(thigh_right_gyr), abs(thigh_left_gyr)])
            < self._param.inactivity_gyr_threshold
        ).all():
            self._inactivity_timer += self._param.inactivity_time_step
        else:
            self._inactivity_timer = 0

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
            ) = (
                thigh_left_gyr,
                thigh_left_roll,
                knee_left_roll,
                knee_left_gyr,
            )
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
            if (
                sum(self._tot_movement[0]) < self._param.movement_sum_threshold
                and sum(self._tot_movement[1]) < self._param.movement_sum_threshold
            ):
                if (
                    # thigh_left_roll > self._param.movement_angle_threshold and
                    abs(thigh_left_gyr) > self._param.movement_gyr_threshold
                ):
                    self._tot_movement[0].append(thigh_left_roll)
                if (
                    # thigh_right_roll > self._param.movement_angle_threshold and
                    abs(thigh_right_gyr) > self._param.movement_gyr_threshold
                ):
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
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return self._is_safe

    def send_data(self) -> dict:
        data = {
            "Lth_gyr": self._thigh_leading_leg_gyr,
            "Lth_roll": self._thigh_leading_leg_roll,
            "Oth_gyr": self._thigh_lagging_leg_gyr,
            "Lkn_roll": self._knee_leading_leg_roll,
            "state": str(self.current_state),
        }
        return data
