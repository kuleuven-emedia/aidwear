"""
Filename: hermes/aidwear/prosthesis/state_machines/base.py
Description: Abstract class for the hierarchical exoskeleton controller
    of a predetermined ambulation mode.
"""

from hermes.aidwear.prosthesis.utils.types import MotorId
from abc import abstractmethod
import numpy as np


class ProsthesisStateMachine:
    @abstractmethod
    def step(self) -> None:
        """Cycle the state machine."""
        pass

    @abstractmethod
    def update_sensor_values(
        self,
        torso_angle: float = np.nan,
        thigh_left_angle: float = np.nan,
        thigh_right_angle: float = np.nan,
        thigh_left_roll: float = np.nan,
        thigh_right_roll: float = np.nan,
        knee_left_roll: float = np.nan,
        knee_right_roll: float = np.nan,
        thigh_left_gyr: float = np.nan,
        thigh_right_gyr: float = np.nan,
        knee_left_gyr: float = np.nan,
        knee_right_gyr: float = np.nan,
        knee_enc_angle: float = np.nan,
        ankle_enc_angle: float = np.nan,
        dt: float = 0.01,
    ) -> None:
        """Does state machine dependent update with new sensor readings."""
        pass

    @abstractmethod
    def send_data(self) -> dict:
        """Gets data from the state machine to send over MQTT.

        Returns:
            dict: Data for MQTT client to plot for visualization.
        """
        pass

    @abstractmethod
    def is_safe_to_switch(self) -> bool:
        """Checks if switching to the next FSM is allowed.

        Returns:
            bool: Predicate whether switching to the next FSM is allowed.
        """
        pass

    def performance_factor(self, motor_id: MotorId, factor_prev: tuple):
        alpha = 0.15
        if not self._is_biodex:
            factor = 1
        else:
            if motor_id in [MotorId.HIP_LEFT, MotorId.HIP_RIGHT]:
                if (
                    self._motor_latest_data[motor_id][-1].current <= 0
                    and motor_id == MotorId.HIP_LEFT
                ) or (
                    self._motor_latest_data[motor_id][-1].current >= 0
                    and motor_id == MotorId.HIP_RIGHT
                ):
                    CONSTANT_HEALTHY = [2.5, 1.04, 1.09, 0.74, 1.93, 0.15]
                    CONSTANT_SARCOPENIA = [1.36, 0.78, 1.13, 0.27, 0.64, 2]
                else:
                    CONSTANT_HEALTHY = [1.92, 1.17, 0.58, 0.74, 2.02, 0.012]
                    CONSTANT_SARCOPENIA = [0.81, 0.87, 0.11, 0.29, 0.77, 1.12]
            else:
                if (
                    self._motor_latest_data[motor_id][-1].current >= 0
                    and motor_id == MotorId.KNEE_LEFT
                ) or (
                    self._motor_latest_data[motor_id][-1].current <= 0
                    and motor_id == MotorId.KNEE_RIGHT
                ):
                    CONSTANT_HEALTHY = [2.81, 1.37, 1.42, 0.68, 2.29, 0.14]
                    CONSTANT_SARCOPENIA = [1.06, 1.42, 1.48, 0.54, 1.83, 0.77]
                else:
                    CONSTANT_HEALTHY = [1.49, 0.92, 0.84, 1.56, 5, 0.045]
                    CONSTANT_SARCOPENIA = [0.6, 0.95, 0.72, 0.59, 1.39, 1.58]


            if motor_id in [MotorId.HIP_LEFT, MotorId.KNEE_RIGHT]:
                angle = (
                    (self._motor_latest_data[motor_id][-1].position)
                    / 180
                    * np.pi
                )
                velocity = (
                    self._motor_latest_data[motor_id][-1].velocity * 2 * np.pi / 60
                )
            else:
                angle = (
                    -(self._motor_latest_data[motor_id][-1].position)
                    / 180
                    * np.pi
                )
                velocity = (
                    -self._motor_latest_data[motor_id][-1].velocity * 2 * np.pi / 60
                )

            if motor_id in [MotorId.KNEE_LEFT, MotorId.KNEE_RIGHT]:
                angle = 2*angle

            if velocity >= 0:
                TORQUE_HEALTHY = (
                    CONSTANT_HEALTHY[0]
                    * np.cos(CONSTANT_HEALTHY[1] * (angle - CONSTANT_HEALTHY[2]))
                    * (
                        (
                            2 * CONSTANT_HEALTHY[3] * CONSTANT_HEALTHY[4]
                            + velocity * (CONSTANT_HEALTHY[4] - 3 * CONSTANT_HEALTHY[3])
                        )
                        / (
                            2 * CONSTANT_HEALTHY[3] * CONSTANT_HEALTHY[4]
                            + velocity
                            * (2 * CONSTANT_HEALTHY[4] - 4 * CONSTANT_HEALTHY[3])
                        )
                    )
                )
                TORQUE_SARCOPENIA = (
                    CONSTANT_SARCOPENIA[0]
                    * np.cos(CONSTANT_HEALTHY[1] * (angle - CONSTANT_SARCOPENIA[2]))
                    * (
                        (
                            2 * CONSTANT_SARCOPENIA[3] * CONSTANT_SARCOPENIA[4]
                            + velocity
                            * (CONSTANT_SARCOPENIA[4] - 3 * CONSTANT_SARCOPENIA[3])
                        )
                        / (
                            2 * CONSTANT_SARCOPENIA[3] * CONSTANT_SARCOPENIA[4]
                            + velocity
                            * (2 * CONSTANT_SARCOPENIA[4] - 4 * CONSTANT_SARCOPENIA[3])
                        )
                    )
                )
            else:
                TORQUE_HEALTHY = (
                    CONSTANT_HEALTHY[0]
                    * np.cos(CONSTANT_HEALTHY[1] * (angle - CONSTANT_HEALTHY[2]))
                    * (
                        (
                            2 * CONSTANT_HEALTHY[3] * CONSTANT_HEALTHY[4]
                            + velocity * (CONSTANT_HEALTHY[4] - 3 * CONSTANT_HEALTHY[3])
                        )
                        / (
                            2 * CONSTANT_HEALTHY[3] * CONSTANT_HEALTHY[4]
                            - velocity
                            * (2 * CONSTANT_HEALTHY[4] - 4 * CONSTANT_HEALTHY[3])
                        )
                    )
                    * (1 - CONSTANT_HEALTHY[5] * velocity)
                )
                TORQUE_SARCOPENIA = (
                    CONSTANT_SARCOPENIA[0]
                    * np.cos(CONSTANT_HEALTHY[1] * (angle - CONSTANT_SARCOPENIA[2]))
                    * (
                        (
                            2 * CONSTANT_SARCOPENIA[3] * CONSTANT_SARCOPENIA[4]
                            + velocity
                            * (CONSTANT_SARCOPENIA[4] - 3 * CONSTANT_SARCOPENIA[3])
                        )
                        / (
                            2 * CONSTANT_SARCOPENIA[3] * CONSTANT_SARCOPENIA[4]
                            - velocity
                            * (2 * CONSTANT_SARCOPENIA[4] - 4 * CONSTANT_SARCOPENIA[3])
                        )
                    )
                    * (1 - CONSTANT_SARCOPENIA[5] * velocity)
                )

            if TORQUE_SARCOPENIA < 0:
                TORQUE_SARCOPENIA = 0
            if TORQUE_SARCOPENIA > TORQUE_HEALTHY:
                TORQUE_SARCOPENIA = TORQUE_HEALTHY

            if TORQUE_HEALTHY <= 0:
                factor = 0
            else:
                factor = (TORQUE_HEALTHY - TORQUE_SARCOPENIA) / TORQUE_HEALTHY
            factor = max(0.2, factor)
            factor = alpha * factor + (1 - alpha) * factor_prev[motor_id]
            self._factor_prev[motor_id] = factor

        return factor
