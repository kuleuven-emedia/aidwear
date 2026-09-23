from enum import Enum
from typing import Union
from dataclasses import dataclass

from hermes.aidwear.prosthesis.utils.types import (
    HomingMethod,
    EposDevice,
    EposProtocolStack,
)


class RecoveryStrategy(Enum):
    """Strategy for applying motor setpoints following an automatic fault recovery."""

    BUMPLESS_HOLD = (
        "bumpless_hold"  # Latch actual position upon re-enabling to prevent jerk
    )
    LATEST_TARGET = "latest_target"  # Immediately dispatch latest upstream setpoint
    RAMP_TO_TARGET = (
        "ramp_to_target"  # Smoothly interpolate between actual and target setpoint
    )


@dataclass
class EposRecoveryConfig:
    """Parameters controlling drive fault detection and autonomous recovery.

    Args:
        max_retries: Number of recovery retries. Set to -1 (default) to retry
            indefinitely until program termination.
        retry_delay_s: Delay in seconds between recovery attempts.
        strategy: Recovery strategy for restoring setpoints (BUMPLESS_HOLD, LATEST_TARGET, RAMP_TO_TARGET).
        auto_recover: If True, automatically clears faults and recovers the drive.
        rate_limit: Rate limit when ramping setpoints.
    """

    max_retries: int = -1
    retry_delay_s: float = 0.05
    strategy: RecoveryStrategy = RecoveryStrategy.BUMPLESS_HOLD
    auto_recover: bool = True
    rate_limit: float = 0.0


@dataclass
class EposDeviceConfig:
    device: Union[EposDevice, str]
    protocol: Union[EposProtocolStack, str]
    interface: str
    port: str
    baudrate: int
    timeout_ms: int

    def __init__(
        self,
        device: Union[EposDevice, str] = EposDevice.EPOS4,
        protocol: Union[EposProtocolStack, str] = EposProtocolStack.CAN_OPEN,
        interface: str = "CAN_mcp251xfd 0",
        port: str = "CAN0",
        baudrate: int = 500_000,
        timeout_ms: int = 500,
    ):
        self.device = device if isinstance(device, EposDevice) else EposDevice[device]
        self.protocol = (
            protocol
            if isinstance(protocol, EposProtocolStack)
            else EposProtocolStack[protocol]
        )
        self.interface = interface
        self.port = port
        self.baudrate = baudrate
        self.timeout_ms = timeout_ms


@dataclass
class HomingConfig:
    """
    Configuration parameters for current-threshold homing calibration.

    Args:
        homing_method (HomingMethod): Method used for homing mode; ankle plantar flexion (-4), knee extension (...). Defaults to `CURRENT_THRESHOLD_NEGATIVE_SPEED`.
        acceleration (int): Homing acceleration in rpm/s (0x609A). Defaults to `1_000`.
        speed_switch (int): Crawl velocity toward hardstop in rpm (0x6099-01). Defaults to `900`.
        speed_index (int): Speed for movement to home position in rpm (0x6099-02). Defaults to `900`.
        current_threshold_ma (int): Stall current limit in mA (0x2080 / 0x30B1). Defaults to `1_000`.
        home_offset_enc_ticks (int): Distance in encoder counts from hardstop (0x607C). Defaults to `-100_000`.
        home_position_coordinate (int): Coordinate assigned to home position (0x30B0). Defaults to `0`.
    """

    homing_method: Union[HomingMethod, str]
    acceleration: int
    speed_switch: int
    speed_index: int
    current_threshold_ma: int
    home_offset_enc_ticks: int
    home_position_coordinate: int

    def __init__(
        self,
        homing_method: Union[
            HomingMethod, str
        ] = HomingMethod.CURRENT_THRESHOLD_NEGATIVE_SPEED,
        acceleration: int = 1_000,
        speed_switch: int = 900,
        speed_index: int = 900,
        current_threshold_ma: int = 1_000,
        home_offset_enc_ticks: int = -100_000,
        home_position_coordinate: int = 0,
    ):
        self.homing_method = (
            homing_method
            if isinstance(homing_method, HomingMethod)
            else HomingMethod[homing_method]
        )
        self.acceleration = acceleration
        self.speed_switch = speed_switch
        self.speed_index = speed_index
        self.current_threshold_ma = current_threshold_ma
        self.home_offset_enc_ticks = home_offset_enc_ticks
        self.home_position_coordinate = home_position_coordinate
