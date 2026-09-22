"""
Filename: hermes/aidwear/prosthesis/state_machines/hurdle.py
Description: AidWear-specific state machine for the hierarchical control
    of the sit-to-stand ambulation mode.
"""

import numpy as np
from hermes.aidwear.prosthesis.utils.types import (
    NiclaSamples,
    ServoMotorData,
    EncoderData,
)
from statemachine import Event, State, StateMachine
from hermes.utils.time_utils import get_time
from .base import ProsthesisStateMachine

from ..motor_control.epos_commands import (
    activate_position_mode,
    pm_set_position_must,
    activate_current_mode,
    cm_set_current_must,
)

from ..utils.types import (
    ModeContext,
    PhaseEstimate,
    HurdlesParameters,
    StateEnum,
    StateTransition,
    MotorId,
    EncoderId,
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
        | stance.to(swing, cond="is_stance_to_swing", on="stance_to_swing")  # T1
        | swing.to(swing, unless="is_swing_to_stance")
        | swing.to(stance, cond="is_swing_to_stance", on="swing_to_stance")  # T2
    )

    def __init__(self, ctx: ModeContext):
        # Prime the motors for the initial idle/stance state.

        self.is_new_target = True

        # Store local values for reference trajectory generation. 
        self._thigh_pr_gyr = 0  # θ̇_thigh,pr
        self._thigh_pr_roll = 0  # θ_thigh,pr
        self._knee_pr_roll = 0  # θ_knee,pr

        self._knee_reference = 0  # θ_knee,pr reference
        self._ankle_reference = 0  # θ_ankle,pr reference
        self._thigh_swing_start = 20
        self._knee_encoder_prev_time = None
        self._knee_encoder_prev_angle = None

        self._knee_swing_start = 0

        # Positive gain means that the knee follows the thigh in the same
        # direction. Tune this from recorded thigh/knee data.
        self._knee_thigh_gain = 1.3

        # Store reference to upstream variables for syncing state machine with the rest
        #   of the mid-level controller. 
        self._ctx = ctx
        self._K = ctx.K
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # Personalized parameters.
        self._param = HurdlesParameters(
            # --------------------------- Stance -> Swing (T1) ---------------------------
            stance_to_swing_th_roll_pr=20,  # θ_thigh,pr > 20 deg
            stance_to_swing_th_gyr_pr=30,# put 200  # θ̇_thigh,pr > 30 deg/s
            # --------------------------- Swing -> Stance (T2) ---------------------------
            swing_to_stance_th_roll_pr=20,  # θ_thigh,pr < 20 deg
        )

        # Parameters for motor control
        self._swing_stiffness = 0.1
        self._swing_damping = 0.05
        self._swing_current_limit_ma = 1000
        self._knee_velocity = 0.0

        activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        activate_position_mode(self._ctx.handle, MotorId.KNEE)

        super(Hurdle, self).__init__()

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

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )

    def swing_to_stance(self):
        activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        activate_position_mode(self._ctx.handle, MotorId.KNEE)
        self.is_new_target = True

    def stance_to_swing(self):
        activate_position_mode(self._ctx.handle, MotorId.ANKLE)
        activate_current_mode(self._ctx.handle, MotorId.KNEE)
        self.is_new_target = False

    # T1: Stance -> Swing
    def is_stance_to_swing(self):
        if(
            self._thigh_pr_roll > self._param.stance_to_swing_th_roll_pr
            and self._thigh_pr_gyr > self._param.stance_to_swing_th_gyr_pr
        ):
            self._knee_swing_start = self._knee_pr_roll
            self._thigh_swing_start = self._thigh_pr_roll

        return (
            self._thigh_pr_roll > self._param.stance_to_swing_th_roll_pr
            and self._thigh_pr_gyr > self._param.stance_to_swing_th_gyr_pr
        )

    # T2: Swing -> Stance
    def is_swing_to_stance(self):
        if (
            self._thigh_pr_roll < self._param.swing_to_stance_th_roll_pr
        ):
            self._knee_reference = 0  # self._knee_swing_start
            self._ankle_reference = 0

        return self._thigh_pr_roll < self._param.swing_to_stance_th_roll_pr

    # Actions.
    def on_enter_stance(self):
        # Trigger new command writing if the reference changes.
        if self.is_new_target:
            pm_set_position_must(self._ctx.handle, MotorId.ANKLE, int(0))
            pm_set_position_must(self._ctx.handle, MotorId.KNEE, int(0))
            self.is_new_target = False

    def on_enter_swing(self):
        # Trigger new command writing if the reference changes.
        if self.is_new_target:
            # KNEE: impedance control from absolute encoder angle/velocity.
            knee_error = self._knee_reference - self._knee_pr_roll
            #print(self._knee_pr_roll)
            # TODO: Convert torque to current using a simple linear model.
            knee_torque = (
                self._swing_stiffness * knee_error
                - self._swing_damping * self._knee_velocity
            )
    
            knee_current = ((knee_torque * 8)*1000) / ((5 / 9) * self._knee_pr_roll + 10)
            knee_current = int(
                np.clip(knee_current, -self._swing_current_limit_ma, self._swing_current_limit_ma)
            )
            self._knee_current = knee_current

            cm_set_current_must(self._ctx.handle, MotorId.KNEE, knee_current)
            # ANKLE: stays in position mode at reference.
            pm_set_position_must(self._ctx.handle, MotorId.ANKLE, int(self._ankle_reference))
            self.is_new_target = False

            #print("[swing] thigh_roll={:.2f} knee_error={:.2f}  knee_ref={:.2f} knee_current={:.2f} knee_torque={:.2f}".format(self._thigh_pr_roll,knee_error,self._knee_reference,knee_current,knee_torque,),flush=True,)

    def _update_motors_reference(self):
        """
        Update the motor reference trajectories and indicate to driver to send it.

        Uses the absolute knee encoder as the actual joint angle in swing.
        """
        thigh_change = self._thigh_pr_roll - self._thigh_swing_start
        self._knee_reference = (
            self._knee_swing_start + self._knee_thigh_gain * thigh_change
        )
        self._ankle_reference = 0
        self.is_new_target = True

    def update_sensor_values(
        self,
        nicla_samples: NiclaSamples,
        #nicla_euler_samples: NiclaSamples,
        encoder_samples: dict[EncoderId, EncoderData],
        motor_samples: dict[MotorId, ServoMotorData],
        dt: float = 0.01,
    ):
        # NOTE: get called on each loop iteration of the mid-level controller.
        #   Then, transition is invoked and corresponding `on_<state>` event is triggered.

        # TODO: save the variables of interest to the self._*, to use in the next "step()".
        self._thigh_left_gyr = nicla_samples.thigh_left_gyr
        #print(nicla_samples)
        self._thigh_left_roll = nicla_samples.thigh_left_roll
        self._shank_left_gyr = nicla_samples.knee_left_gyr
        self._shank_left_roll = nicla_samples.knee_left_roll
        # I'm using the right leg as prosthetic one
        self._thigh_pr_gyr = nicla_samples.thigh_right_gyr
        self._shank_pr_gyr = nicla_samples.knee_right_gyr
        self._shank_pr_roll = nicla_samples.knee_right_roll

        self._torso_roll = nicla_samples.torso_angle
        self._thigh_pr_roll = nicla_samples.thigh_right_roll
        self._knee_pr_roll = encoder_samples[EncoderId.KNEE].angle #-180
        #self._knee_pr_roll_timestamp = encoder_samples[EncoderId.KNEE].timestamp

        if self._knee_pr_roll is not None:
            if self._knee_encoder_prev_angle is not None:
                self._knee_velocity = (self._knee_pr_roll - self._knee_encoder_prev_angle) / dt
            else:
                self._knee_velocity = 0.0
            #self._knee_encoder_prev_time = self._knee_pr_roll_timestamp
            self._knee_encoder_prev_angle = self._knee_pr_roll
        
        # Update the reference trajectory for both motors.
        self._update_motors_reference()

    def step(self) -> None:
        self.send("cycle")

    def is_safe_to_switch(self) -> bool:
        return True
