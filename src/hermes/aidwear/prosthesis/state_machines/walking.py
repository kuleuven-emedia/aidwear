"""
Filename: hermes/aidwear/prosthesis/state_machines/walking.py
Description: AidWear-specific state machine for the hierarchical control
    of the walking ambulation mode.
"""

from dataclasses import asdict
import numpy as np
from statemachine import Event, State, StateMachine

from hermes.utils.time_utils import get_time
from scipy.interpolate import CubicHermiteSpline

from .base import ExoStateMachine
from ..can_control.motor_epos import can_set_position_impedance, can_set_torque
from ..utils.types import (
    ModeContext,
    ModeEnum,
    PhaseEstimate,
    ServoImpedanceGains,
    ServoMotorEnum,
    ServoReference,
    StateEnum,
    StateTransition,
    WalkingFirstStrideEnum,
    WalkingParameters,
    MotorId,
)


class Walking(StateMachine, ExoStateMachine):
    # States.
    idle = State(value=StateEnum.Walking.IDLE.value, initial=True)
    walking = State(
        value=StateEnum.Walking.WALKING.value,
    )

    # Transitions.
    cycle = (
        idle.to(idle, unless="idle_to_walking")
        | idle.to(walking, cond="idle_to_walking")
        | walking.to(idle, cond="walking_to_idle")
        | walking.to(walking, unless="walking_to_idle")
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_roll = [0]
        self._prev_thigh_roll = [0]
        self._velocity = [0]
        self._prev_velocity = [0]
        self._int = [0]
        self._prev_int = [0]
        self._phase = 0
        self._max_min = [0, 0, 0, 0]
        self._first_stride = WalkingFirstStrideEnum.NONE
        self._inactivity_timer = 0
        self._walking_dur = 0
        self._idle_dur = 0
        self._transition_dur = 0

        self._thigh_left_gyr = 0
        self._thigh_left_angle = 0
        self._thigh_left_roll = 0
        self._knee_left_roll = 0
        self._thigh_right_gyr = 0
        self._thigh_right_angle = 0
        self._thigh_right_roll = 0
        self._knee_right_roll = 0
        self._torso_roll = 0

        self._is_walk = False
        self._is_switch = False
        self._is_safe = False
        self._gain_step = 0.01

        self._ctx = ctx

        # Personalized parameters.
        params = ctx.config_manager.get_section("walking")
        self._param = WalkingParameters(
            inactivity_gyr_threshold=params["inactivity_gyr_threshold"],
            inactivity_idle_transition_time=params["inactivity_idle_transition_time"],
            traj_shift={MotorId[k]: v for k, v in params["traj_shift"].items()},
            inactivity_time_step=params["inactivity_time_step"],
            first_stride_end_gyr=params["first_stride_end_gyr"],
            reset_phase_threshold=params["reset_phase_threshold"],
            inactivity_angle_threshold=params["inactivity_angle_threshold"],
            ramp_time=params["ramp_time"],
        )
        self._is_biodex = ctx.config_manager.get_section("biodex")["enabled"]

        # Target positions for each gait phase between 0 and 100%.
        self._trajectory: dict[MotorId, list[float]] = {
            MotorId[k]: v for k, v in params["trajectory"].items()
        }

        # Kalman filter variables.
        self.kalman_prediction_horizon = (
            0.015  # prediction horizon / latency [s] (optional)
        )
        self.kalman_wrap_threshold = 95  # detect wrap when drop >50%
        self.kalman_cycle_len = 100.0  # phase range per cycle (percent)
        # Tuning.
        self.kalman_process_noise = np.diag([1e-6, 1e-3])  # [phase, rate]
        self.kalman_measurement_noise_variance = 5e-1
        self.kalman_initial_covariance = np.diag([0.1, 1.0])
        self.kalman_initial_state = np.array([0.0, 1.0])  # [phase_unwrapped, rate]
        self.kalman_num_cycles = 0
        self.kalman_prev_raw = 0.0
        super(Walking, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        if not self._is_stop_new_data_event.is_set():
            self._state_changed_queue.put(
                StateTransition(timestamp=get_time(), state=state.value)
            )

    def idle_to_walking(self):
        return self._thigh_right_gyr > self._param.inactivity_gyr_threshold + 100

    def walking_to_idle(self):
        return self._inactivity_timer > self._param.inactivity_idle_transition_time

    # Actions.
    def on_enter_idle(self):
        # Reset.
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value

        if next_mode != ModeEnum.WALKING.value.id:
            self._ctx.token.value = 0
            self._is_safe = True

        self._first_stride = WalkingFirstStrideEnum.NONE
        self._thigh_roll = [0]
        self._velocity = [0]
        self._phase = 0
        self._max_min = [0, 0, 0, 0]
        self._prev_thigh_roll = [0]
        self._prev_velocity = [0]
        self._walking_dur = 0

        shift = self._param.traj_shift
        imp_rise_time = self._param.ramp_time
        performance_factor_knee = self.performance_factor(MotorId.KNEE, self._factor_prev)
        performance_factor_ankle = self.performance_factor(MotorId.ANKLE, self._factor_prev)

        if self._token.value == 1:
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.HIP_RIGHT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=-25,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorHR
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
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
                    position=-5,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorHL
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
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
                    position=5,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorKR
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
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
                    position=-30,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorKL
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
        elif self._token.value == 2:
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.HIP_RIGHT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=5,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorHR
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
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
                    position=25,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorHL
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
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
                    position=30,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorKR
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
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
                    position=-5,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorKL
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.075
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
        elif self._is_walk:
            imp_rise_time = self._param.ramp_time
            can_set_position_impedance(
                bus=self._bus,
                motor_id=MotorId.HIP_RIGHT,
                motor_latest_data=self._motor_latest_data,
                ref=ServoReference(
                    position=-self._trajectory[MotorId.HIP_RIGHT][
                        (min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100
                    ],
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorHR
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
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
                    position=self._trajectory[MotorId.HIP_LEFT][
                        (min(int(self._phase), 99) + shift[MotorId.HIP_LEFT]) % 100
                    ],
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorHL
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
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
                    position=self._trajectory[MotorId.KNEE_RIGHT][
                        (min(int(self._phase), 99) + shift[MotorId.KNEE_RIGHT]) % 100
                    ]
                    / 2,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorKR
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
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
                    position=-self._trajectory[MotorId.KNEE_LEFT][
                        (min(int(self._phase), 99) + shift[MotorId.KNEE_LEFT]) % 100
                    ]
                    / 2,
                    velocity=0,
                    acceleration=0,
                ),
                K=ServoImpedanceGains(
                    **{
                        k: Performance_factorKL
                        * max(0.2, int(next_fatigue) / 100)
                        * max(v * (imp_rise_time - self._idle_dur) / imp_rise_time, 0)
                        * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ),
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
        else:
            can_set_torque(
                motor_id=MotorId.HIP_RIGHT,
                torque=0,
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_torque(
                motor_id=MotorId.HIP_LEFT,
                torque=0,
                motor_type=ServoMotorEnum.AK80_8,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_torque(
                motor_id=MotorId.KNEE_RIGHT,
                torque=0,
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            can_set_torque(
                motor_id=MotorId.KNEE_LEFT,
                torque=0,
                motor_type=ServoMotorEnum.AK10_9,
                motor_command_queue=self._motor_command_queue,
                is_keep_data_event=self._is_keep_data_event,
            )
            self._idle_dur += self._gain_step

    def on_enter_walking(self):
        # Reset.
        with self._ctx.next_fatigue.lock():
            next_fatigue = self._ctx.next_fatigue.next_value.value
        with self._ctx.next_mode.lock():
            next_mode = self._ctx.next_mode.next_value.value

        # Phase angle based impedance/torque trajectory control.
        self._idle_dur = 0
        self._ctx.token.value = 4
        performance_factor_knee = self.performance_factor(MotorId.KNEE, self._factor_prev)
        performance_factor_ankle = self.performance_factor(MotorId.ANKLE, self._factor_prev)

        if self._first_stride == WalkingFirstStrideEnum.DEACTIVATED:
            shift = self._param.traj_shift
            if (
                next_mode == ModeEnum.STAIR_ASCENT.value.id
                and (
                    (
                        65
                        <= (
                            (min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100
                        )
                        < 70
                    )
                    or (
                        15
                        <= (
                            (min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100
                        )
                        < 20
                    )
                )
                and not self._is_switch
            ):
                kp_hip = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ).position
                kd_hip = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ).velocity
                kp_knee = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ).position
                kd_knee = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ).velocity

                torque_adjusted_hip_right = kp_hip * Performance_factorHR * (
                    -self._trajectory[MotorId.HIP_RIGHT][
                        (min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.HIP_RIGHT][-1].position)
                ) + kd_hip * Performance_factorHR * (
                    0 - self._motor_latest_data[MotorId.HIP_RIGHT][-1].velocity
                )
                torque_adjusted_hip_left = kp_hip * Performance_factorHL * (
                    self._trajectory[MotorId.HIP_LEFT][
                        (min(int(self._phase), 99) + shift[MotorId.HIP_LEFT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.HIP_LEFT][-1].position )
                ) + kd_hip * Performance_factorHL * (
                    0 - self._motor_latest_data[MotorId.HIP_LEFT][-1].velocity
                )
                torque_adjusted_knee_right = kp_knee * Performance_factorKR * (
                    self._trajectory[MotorId.KNEE_RIGHT][
                        (min(int(self._phase), 99) + shift[MotorId.KNEE_RIGHT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.KNEE_RIGHT][-1].position)
                ) + kd_knee * Performance_factorKR * (
                    0 - self._motor_latest_data[MotorId.KNEE_RIGHT][-1].velocity
                )
                torque_adjusted_knee_left = kp_knee * Performance_factorKL * (
                    -self._trajectory[MotorId.KNEE_LEFT][
                        (min(int(self._phase), 99) + shift[MotorId.KNEE_LEFT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.KNEE_LEFT][-1].position)
                ) + kd_knee * Performance_factorKL * (
                    0 - self._motor_latest_data[MotorId.KNEE_LEFT][-1].velocity
                )

                self._spline_hip_right = CubicHermiteSpline(
                    [0, 1], [torque_adjusted_hip_right, 0], [0, 0]
                )
                self._spline_hip_left = CubicHermiteSpline(
                    [0, 1], [torque_adjusted_hip_left, 0], [0, 0]
                )
                self._spline_knee_right = CubicHermiteSpline(
                    [0, 1], [torque_adjusted_knee_right, 0], [0, 0]
                )
                self._spline_knee_left = CubicHermiteSpline(
                    [0, 1], [torque_adjusted_knee_left, 0], [0, 0]
                )

                self._spline_len = 0
                self._transition_smoothing_dur = 0.7
                self._token.value = 2
                self._is_switch = True

            if next_mode == ModeEnum.STAIR_DESCENT.value.id and (
                (
                    90
                    <= ((min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100)
                    < 95
                )
                or (
                    40
                    <= ((min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100)
                    < 45
                )
            ):
                kp_hip = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ).position
                kd_hip = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.35
                        for k, v in asdict(self._K[ServoMotorEnum.AK80_8.name]).items()
                    }
                ).velocity
                kp_knee = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ).position
                kd_knee = ServoImpedanceGains(
                    **{
                        k: max(0.2, int(next_fatigue) / 100) * v * 0.25
                        for k, v in asdict(self._K[ServoMotorEnum.AK10_9.name]).items()
                    }
                ).velocity

                torque_adjusted_hip_right = kp_hip * Performance_factorHR * (
                    -self._trajectory[MotorId.HIP_RIGHT][
                        (min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.HIP_RIGHT][-1].position)
                ) + kd_hip * Performance_factorHR* (
                    0 - self._motor_latest_data[MotorId.HIP_RIGHT][-1].velocity
                )
                torque_adjusted_hip_left = kp_hip *-Performance_factorHL * (
                    self._trajectory[MotorId.HIP_LEFT][
                        (min(int(self._phase), 99) + shift[MotorId.HIP_LEFT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.HIP_LEFT][-1].position)
                ) + kd_hip * Performance_factorHL * (
                    0 - self._motor_latest_data[MotorId.HIP_LEFT][-1].velocity
                )
                torque_adjusted_knee_right = kp_knee * Performance_factorKR* (
                    self._trajectory[MotorId.KNEE_RIGHT][
                        (min(int(self._phase), 99) + shift[MotorId.KNEE_RIGHT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.KNEE_RIGHT][-1].position)
                ) + kd_knee * Performance_factorKR* (
                    0 - self._motor_latest_data[MotorId.KNEE_RIGHT][-1].velocity
                )
                torque_adjusted_knee_left = kp_knee * Performance_factorKL* (
                    -self._trajectory[MotorId.KNEE_LEFT][
                        (min(int(self._phase), 99) + shift[MotorId.KNEE_LEFT]) % 100
                    ]
                    - (self._motor_latest_data[MotorId.KNEE_LEFT][-1].position)
                ) + kd_knee * Performance_factorKL * (
                    0 - self._motor_latest_data[MotorId.KNEE_LEFT][-1].velocity
                )

                self._transition_smoothing_dur = 0.3
                if (
                    90
                    <= ((min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100)
                    < 95
                ):
                    self._spline_hip_right = CubicHermiteSpline(
                        [0, 1],
                        [
                            torque_adjusted_hip_right,
                            1.125
                            * Performance_factorHR
                            * max(0.2, int(next_fatigue) / 100)*1.5,
                        ],
                        [0, 0],
                    )
                    self._spline_hip_left = CubicHermiteSpline(
                        [0, 1], [torque_adjusted_hip_left, 0], [0, 0]
                    )
                    self._spline_knee_right = CubicHermiteSpline(
                        [0, 1],
                        [
                            torque_adjusted_knee_right,
                            -1.125
                            * Performance_factorKR
                            * max(0.2, int(next_fatigue) / 100)
                            * 10,
                        ],
                        [0, 0],
                    )
                    self._spline_knee_left = CubicHermiteSpline(
                        [0, 1], [torque_adjusted_knee_left, 0], [0, 0]
                    )

                    self._spline_len = 0
                    self._is_switch = True
                    self._token.value = 1
                elif (
                    40
                    <= ((min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100)
                    < 45
                ):
                    self._spline_hip_right = CubicHermiteSpline(
                        [0, 1], [torque_adjusted_hip_right, 0], [0, 0]
                    )
                    self._spline_hip_left = CubicHermiteSpline(
                        [0, 1],
                        [
                            torque_adjusted_hip_left,
                            -1.125
                            * Performance_factorHL
                            * max(0.2, int(next_fatigue) / 100)
                            * 1.5,
                        ],
                        [0, 0],
                    )
                    self._spline_knee_right = CubicHermiteSpline(
                        [0, 1], [torque_adjusted_knee_right, 0], [0, 0]
                    )
                    self._spline_knee_left = CubicHermiteSpline(
                        [0, 1],
                        [
                            torque_adjusted_knee_left,
                            1.125
                            * Performance_factorKL
                            * max(0.2, int(next_fatigue) / 100)
                            * 10,
                        ],
                        [0, 0],
                    )

                    self._spline_len = 0
                    self._is_switch = True
                    self._token.value = 2

            if self._is_switch:
                n = int(self._transition_smoothing_dur / self._gain_step)
                x = np.linspace(0, 1, n)
                torque_hip_right = self._spline_hip_right(x[self._spline_len])
                torque_hip_left = self._spline_hip_left(x[self._spline_len])
                torque_knee_right = self._spline_knee_right(x[self._spline_len])
                torque_knee_left = self._spline_knee_left(x[self._spline_len])

                can_set_torque(
                    motor_id=MotorId.HIP_RIGHT,
                    torque=torque_hip_right,
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_torque(
                    motor_id=MotorId.HIP_LEFT,
                    torque=torque_hip_left,
                    motor_type=ServoMotorEnum.AK80_8,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_torque(
                    motor_id=MotorId.KNEE_RIGHT,
                    torque=torque_knee_right,
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                can_set_torque(
                    motor_id=MotorId.KNEE_LEFT,
                    torque=torque_knee_left,
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )
                self._spline_len += 1
                if self._spline_len == n - 2:
                    self._is_switch = False
                    self._is_safe = True
            else:
                self._is_walk = True
                imp_rise_time = self._param.ramp_time
                if next_mode == ModeEnum.SIT_TO_STAND.value.id:
                    self._token.value = 0
                    factor = max(
                        (imp_rise_time - self._transition_dur) / imp_rise_time, 0
                    ) * min(self._walking_dur / imp_rise_time, 1)
                    self._transition_dur += self._gain_step
                    if factor == 0:
                        self._is_safe = True
                else:
                    factor = min(self._walking_dur / imp_rise_time, 1)
                    self._walking_dur += self._gain_step

                can_set_position_impedance(
                    bus=self._bus,
                    motor_id=MotorId.HIP_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=-self._trajectory[MotorId.HIP_RIGHT][
                            (min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100
                        ],
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: Performance_factorHR
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            * 0.35
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
                    motor_id=MotorId.HIP_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=self._trajectory[MotorId.HIP_LEFT][
                            (min(int(self._phase), 99) + shift[MotorId.HIP_LEFT]) % 100
                        ],
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: Performance_factorHL
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            * 0.35
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
                    motor_id=MotorId.KNEE_RIGHT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=self._trajectory[MotorId.KNEE_RIGHT][
                            (min(int(self._phase), 99) + shift[MotorId.KNEE_RIGHT])
                            % 100
                        ]
                        / 2,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: Performance_factorKR
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            * 0.25
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
                    motor_id=MotorId.KNEE_LEFT,
                    motor_latest_data=self._motor_latest_data,
                    ref=ServoReference(
                        position=-self._trajectory[MotorId.KNEE_LEFT][
                            (min(int(self._phase), 99) + shift[MotorId.KNEE_LEFT]) % 100
                        ]
                        / 2,
                        velocity=0,
                        acceleration=0,
                    ),
                    K=ServoImpedanceGains(
                        **{
                            k: Performance_factorKL
                            * max(0.2, int(next_fatigue) / 100)
                            * v
                            * factor
                            * 0.25
                            for k, v in asdict(
                                self._K[ServoMotorEnum.AK10_9.name]
                            ).items()
                        }
                    ),
                    motor_type=ServoMotorEnum.AK10_9,
                    motor_command_queue=self._motor_command_queue,
                    is_keep_data_event=self._is_keep_data_event,
                )

    def _kalman_phase_update(self, phase_raw):
        dt = 0.01  # sample period [s]
        # matrices (constant velocity model)
        A = np.array([[1, dt], [0, 1]])
        H = np.array([[1, 0]])
        I = np.eye(2)

        # ---- 1. predict ----
        x_pred = A @ self.kalman_initial_state
        P_pred = A @ self.kalman_initial_covariance @ A.T + self.kalman_process_noise

        # ---- 2. unwrap ----
        n = int(round((x_pred[0] - phase_raw) / self.kalman_cycle_len))
        phase_unwrapped = phase_raw + self.kalman_cycle_len * n

        # ---- 3. update ----
        z = np.array([phase_unwrapped])
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + self.kalman_measurement_noise_variance
        K = P_pred @ H.T / S
        self.kalman_initial_state = x_pred + (K @ y).ravel()
        self.kalman_initial_covariance = (I - K @ H) @ P_pred

        # ---- 4. optional: latency compensation ----
        phi_pred = (
            self.kalman_initial_state[0]
            + self.kalman_initial_state[1] * self.kalman_prediction_horizon
        )

        # ---- 5. rewrap for output ----
        phi_out = phi_pred % self.kalman_cycle_len

        return phi_out, self.kalman_initial_state[
            1
        ]  # smoothed phase [0; 100), and estimated rate

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
        self._thigh_left_angle = -thigh_left_angle
        self._thigh_left_roll = thigh_left_roll
        self._knee_left_roll = knee_left_roll
        self._thigh_right_gyr = thigh_right_gyr
        self._thigh_right_angle = -thigh_right_angle
        self._thigh_right_roll = thigh_right_roll
        self._knee_right_roll = knee_right_roll
        self._knee_right_gyr = knee_right_gyr
        self._knee_left_gyr = knee_left_gyr

        # Check if sensor values remain more or less constant.
        if abs(thigh_right_gyr) < self._param.inactivity_gyr_threshold:
            self._inactivity_timer += self._param.inactivity_time_step
        else:
            self._inactivity_timer = 0

        if (
            self._first_stride == WalkingFirstStrideEnum.NONE
            and thigh_right_gyr > self._param.inactivity_gyr_threshold
        ):
            self._first_stride = WalkingFirstStrideEnum.ONGOING
        elif self._first_stride == WalkingFirstStrideEnum.ONGOING:
            if thigh_right_gyr < self._param.first_stride_end_gyr:
                self._first_stride = WalkingFirstStrideEnum.TAKEN

        # Using velocity.
        self._thigh_roll.append(thigh_right_angle)
        self._velocity.append(thigh_right_gyr)

        Gamma = -(self._max_min[1] + self._max_min[3]) / 2
        gamma = -(self._max_min[0] + self._max_min[2]) / 2

        if self._max_min[1] - self._max_min[3] != 0:
            z = np.abs(self._max_min[0] - self._max_min[2]) / np.abs(
                self._max_min[1] - self._max_min[3]
            )
        else:
            z = 0

        phase, _ = self._kalman_phase_update(
            (
                np.arctan2(
                    (self._velocity[-1] + Gamma) * z, self._thigh_roll[-1] + gamma
                )
                + np.pi
            )
            / (2 * np.pi)
            * 100
        )
        if (
            self._first_stride == WalkingFirstStrideEnum.TAKEN
            or self._first_stride == WalkingFirstStrideEnum.DEACTIVATED
        ):
            self._phase = phase
        else:
            self._phase = (
                (
                    np.arctan2(
                        (self._velocity[-1] + Gamma) * z, self._thigh_roll[-1] + gamma
                    )
                    + np.pi
                )
                / (2 * np.pi)
                * 100
            )

        # Push `phase` estimate into the upstream queue.
        if not self._is_stop_new_data_event.is_set():
            self._phase_estimate_queue.put(
                PhaseEstimate(timestamp=get_time(), phase=self._phase)
            )

        if self._first_stride == WalkingFirstStrideEnum.TAKEN or (
            self._first_stride == WalkingFirstStrideEnum.DEACTIVATED
            and (
                self._phase >= self._param.reset_phase_threshold
                or abs(thigh_right_roll - max(self._prev_thigh_roll))
                <= self._param.inactivity_angle_threshold
            )
        ):
            self._first_stride = WalkingFirstStrideEnum.DEACTIVATED
            self._prev_thigh_roll = self._thigh_roll
            self._prev_velocity = self._velocity
            self._thigh_roll = [0]
            self._velocity = [0]

            self._max_min = [
                max(self._prev_thigh_roll),
                max(self._prev_velocity),
                min(self._prev_thigh_roll),
                min(self._prev_velocity),
            ]

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return self._is_safe

    def send_data(self) -> dict:
        shift = self._param.traj_shift
        data = {
            "Lth_roll": self._thigh_left_angle,
            "Rth_roll": self._thigh_right_angle,
            "Lkn_roll": self._knee_left_roll,
            "Rkn_roll": self._knee_right_roll,
            "Rth_gyr": self._thigh_right_gyr,
            "phase": self._phase,
            "Rth_traj": float(
                self._trajectory[MotorId.HIP_RIGHT][
                    (min(int(self._phase), 99) + shift[MotorId.HIP_RIGHT]) % 100
                ]
            ),
            "Lth_traj": float(
                self._trajectory[MotorId.HIP_LEFT][
                    (min(int(self._phase), 99) + shift[MotorId.HIP_LEFT]) % 100
                ]
            ),
            "Rkn_traj": float(
                self._trajectory[MotorId.KNEE_RIGHT][
                    (min(int(self._phase), 99) + shift[MotorId.KNEE_RIGHT]) % 100
                ]
            ),
            "Lkn_traj": float(
                self._trajectory[MotorId.KNEE_LEFT][
                    (min(int(self._phase), 99) + shift[MotorId.KNEE_LEFT]) % 100
                ]
            ),
        }
        return data
