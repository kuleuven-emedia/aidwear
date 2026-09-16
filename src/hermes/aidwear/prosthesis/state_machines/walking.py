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

from .base import ProsthesisStateMachine
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


class Walking(StateMachine, ProsthesisStateMachine):
    # States.
    idle = State(
        value=StateEnum.Walking.IDLE.value,
        initial=True,
    )
    one_step = State(
        value=StateEnum.Walking.ONE_STEP.value,
    )
    stance = State(
        value=StateEnum.Walking.STANCE.value,
    )
    swing = State(
        value=StateEnum.Walking.SWING.value,
    )
    #    swing_ext = State(
    #        value=StateEnum.Walking.SWING_EXT.value,
    #    )

    # Transitions.
    cycle = (
        idle.to(idle, unless="is_idle_to_one_step")
        | idle.to(one_step, cond="is_idle_to_one_step")  # T1
        | one_step.to(idle, cond="is_one_step_to_idle")  # T2
        | one_step.to(one_step, unless="is_one_step_to_stance or is_one_step_to_idle")
        | one_step.to(stance, cond="is_one_step_to_stance")  # T3
        | stance.to(idle, cond="is_stance_to_idle")  # T2
        | stance.to(swing, cond="is_stance_to_swing")  # T4
        | stance.to(stance, unless="is_stance_to_swing or is_stance_to_idle")
        | swing.to(swing, unless="is_swing_to_stance")
        | swing.to(stance, cond="is_swing_to_stance")  # T5
        # | swing.to(swing, unless="swing_to_swing_ext")
        # | swing.to(swing_ext, cond="swing_to_swing_ext")
        # | swing_ext.to(swing_ext, unless="swing_ext_to_stance")
        # | swing_ext.to(stance, cond="swing_ext_to_stance")                                                                  # T5
    )

    def __init__(self, ctx: ModeContext):
        self._thigh_intact_gyr = 0      # θ̇_thigh,intact
        self._thigh_pr_gyr = 0          # θ̇_thigh,pr  (prosthetic side)
        self._phase = 0                 # ϑ_walking
        self._inactivity_dur = 0        # counter_inactivity
        self._bending_dur = 0           # time_bending
        self._first_stride = WalkingFirstStrideEnum.NONE

        self._th_roll = [0]
        self._int = [0]
        self._vel = [0]
        self._max_min = [0, 0, 0, 0]
        self._prev_th_roll = [0]
        self._prev_int = [0]
        self._prev_vel = [0]

        self._K = ctx.K
        self._bus = ctx.bus
        self._motor_latest_data = ctx.motor_latest_data
        self._fatigue = ctx.fatigue
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # Personalized parameters.
        self._param = WalkingParameters(
            # --------------------------- Idle -> One Step (T1) ---------------------------
            idle_to_one_step_th_gyr=-600,
            # --------------------------- One Step / Stance -> Idle (T2) ---------------------------
            to_idle_inactivity_dur=1.0,
            # --------------------------- One Step -> Stance (T3) ---------------------------
            one_step_to_stance_th_gyr=-40,
            # --------------------------- Stance -> Swing (T4) ---------------------------
            stance_to_swing_th_gyr=600,
            stance_to_swing_phase_threshold=47,
            # --------------------------- Swing -> Stance (T5) ---------------------------
            swing_to_stance_th_gyr=600,
            swing_to_stance_phase_threshold=70,
            swing_to_stance_bending_dur=0.5,
            inactivity_gyr_threshold=250,
            inactivity_idle_transition_time=2,
            inactivity_time_step=0.01,
            first_stride_end_gyr=-700,
            reset_phase_threshold=95,
            inactivity_angle_threshold=2,
        )

        # Kalman filter variables.
        self.L = 0.015  # prediction horizon / latency [s] (optional)
        self.wrap_threshold = 95  # detect wrap when drop >50%
        self.cycle_len = 100.0  # phase range per cycle (percent)
        # Tuning.
        self.Q = np.diag([1e-5, 1e-3])  # process noise: [phase, rate]
        self.R = 5e-3  # measurement noise variance
        self.P = np.diag([0.1, 1.0])  # initial covariance
        self.x = np.array([0.0, 1.0])  # initial state [phase_unwrapped, rate]
        self.cycles = 0
        self.prev_raw = 0.0

        super(Walking, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )

    # T1: Idle -> One Step
    def is_idle_to_one_step(self):
        return self._thigh_intact_gyr < self._param.idle_to_one_step_th_gyr

    # T2: One Step -> Idle
    def is_one_step_to_idle(self):
        return self._inactivity_dur > self._param.to_idle_inactivity_dur

    # T3: One Step -> Stance
    def is_one_step_to_stance(self):
        return self._thigh_pr_gyr < self._param.one_step_to_stance_th_gyr

    # T2: Stance -> Idle
    def is_stance_to_idle(self):
        return self._inactivity_dur > self._param.to_idle_inactivity_dur

    # T4: Stance -> Swing
    def is_stance_to_swing(self):
        return (
            self._thigh_pr_gyr > self._param.stance_to_swing_th_gyr
            and self._phase > self._param.stance_to_swing_phase_threshold
        )

    # T5: Swing -> Stance
    def is_swing_to_stance(self):
        return (
            self._thigh_pr_gyr > self._param.swing_to_stance_th_gyr
            and self._phase > self._param.swing_to_stance_phase_threshold
        ) or self._bending_dur > self._param.swing_to_stance_bending_dur

    # Actions.    ########## for the moment i'm Using a random value for the servo
    def on_enter_idle(self):
        # TODO: add motor control logic for idle.
        pass

    def on_enter_one_step(self):
        # TODO: add motor control logic for one step.
        pass

    def on_enter_stance(self):
        # TODO: add motor control logic for stance.
        pass

    def on_enter_swing(self):
        # TODO: add motor control logic for swing.
        pass

    def _kalman_phase_update(self, phase_raw):
        dt = 0.01  # sample period [s]
        # matrices (constant velocity model)
        A = np.array([[1, dt], [0, 1]])
        H = np.array([[1, 0]])
        I = np.eye(2)

        # ---- 1. predict ----
        x_pred = A @ self.x
        P_pred = A @ self.P @ A.T + self.Q

        # ---- 2. unwrap ----
        n = int(round((x_pred[0] - phase_raw) / self.cycle_len))
        # if phase_raw < self.prev_raw - self.wrap_threshold:
        #     self.cycles += 1
        # self.prev_raw = phase_raw
        phase_unwrapped = phase_raw + self.cycle_len * n

        # ---- 3. update ----
        z = np.array([phase_unwrapped])
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T / S
        self.x = x_pred + (K @ y).ravel()
        self.P = (I - K @ H) @ P_pred

        # ---- 4. optional: latency compensation ----
        phi_pred = self.x[0] + self.x[1] * self.L

        # ---- 5. rewrap for output ----
        phi_out = phi_pred % self.cycle_len

        return phi_out, self.x[1]  # smoothed phase [0?100), and estimated rate

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
        self._thigh_left_angle = thigh_left_angle
        self._thigh_left_roll = thigh_left_roll
        self._knee_left_roll = knee_left_roll
        self._thigh_right_gyr = thigh_right_gyr
        self._thigh_right_angle = thigh_right_angle
        self._thigh_right_roll = thigh_right_roll
        self._knee_right_roll = knee_right_roll

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

        # Using integral.
        # self.th_roll.append(th_roll)
        # self.int.append(self.int[-1] + self.th_roll[-1]*dt)

        # Gamma = -(self.max_min[1] + self.max_min[3])/2
        # gamma = -(self.max_min[0] + self.max_min[2])/2
        # z = np.abs(self.max_min[0] - self.max_min[2])/np.abs(self.max_min[1] - self.max_min[3])

        # self.phase = (np.arctan2((self.int[-1]+Gamma)*z,self.th_roll[-1]+gamma) + np.pi)/(2*np.pi)*100

        # print(f"{Gamma}, {gamma}, {z}, {self.phase}")

        # if self.phase >= 95 or self.first_stride == WalkingFirstStrideEnum.TAKEN:
        #     self.first_stride = WalkingFirstStrideEnum.DEACTIVATED
        #     self.prev_th_roll = self.th_roll
        #     self.prev_int = self.int
        #     self.th_roll = [0]
        #     self.int = [0]
        #     self.max_min = [max(self.prev_th_roll), max(self.prev_int), min(self.prev_th_roll), min(self.prev_int)]

        # Using velocity.
        self._th_roll.append(thigh_right_angle)
        self._vel.append(thigh_right_gyr)

        Gamma = -(self._max_min[1] + self._max_min[3]) / 2
        gamma = -(self._max_min[0] + self._max_min[2]) / 2

        if self._max_min[1] - self._max_min[3] != 0:
            z = np.abs(self._max_min[0] - self._max_min[2]) / np.abs(
                self._max_min[1] - self._max_min[3]
            )
        else:
            z = 0

        phase, _ = self._kalman_phase_update(
            (np.arctan2((self._vel[-1] + Gamma) * z, self._th_roll[-1] + gamma) + np.pi)
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
                    np.arctan2((self._vel[-1] + Gamma) * z, self._th_roll[-1] + gamma)
                    + np.pi
                )
                / (2 * np.pi)
                * 100
            )

        # Push `phase` estimate into the upstream queue.
        self._phase_estimate_queue.put(
            PhaseEstimate(timestamp=get_time(), phase=self._phase)
        )

        if self._first_stride == WalkingFirstStrideEnum.TAKEN or (
            self._first_stride == WalkingFirstStrideEnum.DEACTIVATED
            and (
                self._phase >= self._param.reset_phase_threshold
                or abs(thigh_right_roll - max(self._prev_th_roll))
                <= self._param.inactivity_angle_threshold
            )
        ):
            self._first_stride = WalkingFirstStrideEnum.DEACTIVATED
            self._prev_th_roll = self._th_roll
            self._prev_vel = self._vel
            self._th_roll = [0]
            self._vel = [0]

            # if not self._prev_th_roll or not self._prev_vel:
            #     print("empty", flush=True)
            #     pass

            self._max_min = [
                max(self._prev_th_roll),
                max(self._prev_vel),
                min(self._prev_th_roll),
                min(self._prev_vel),
            ]

        # print(f"th_roll: {self._th_roll[-1]:.2f}, th_gyr: {self._vel[-1]:.2f}, kn_roll: {knee_right_roll:.2f}, phase: {self._phase:.2f}")

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
