"""
Filename: hermes/aidwear/prosthesis/utils/types.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-09-12
Version: 1.0
Description: AidWear-specific data types.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
from multiprocessing import Queue, Lock, Value
from multiprocessing.synchronize import Event as _Event, Lock as _Lock
from multiprocessing.sharedctypes import Synchronized
from typing import TypeAlias, TYPE_CHECKING

if TYPE_CHECKING:
    from hermes.aidwear.prosthesis.utils.config_manager import ConfigManager

from hermes.aidwear.utils.types import NiclaData, NiclaLocation

epos_handle: TypeAlias = ctypes.c_void_p


class MotorId(Enum):
    KNEE = 1
    ANKLE = 2


class EncoderId(Enum):
    KNEE = 0x201
    ANKLE = 0x202


@dataclass
class ServoReference:
    position: float
    velocity: float
    acceleration: float


@dataclass
class ServoImpedanceGains:
    position: float
    velocity: float
    acceleration: float


@dataclass
class ServoParameters:
    position_min: float
    position_max: float
    velocity_min: float
    velocity_max: float
    current_min: float
    current_max: float
    torque_min: float
    torque_max: float
    Kt_nominal: float  # from the TMotor website (actually 1/Kvll)
    current_factor: float
    Kt_actual: float
    gear_ratio: float
    num_pole_pairs: int
    is_use_derived_torque_constants: bool


class ServoMotorEnum(Enum):
    AK10_9 = ServoParameters(
        position_min=-36000,  # -36000 deg
        position_max=36000,  # 36000 deg
        velocity_min=-100000,  # -100000 rpm electrical speed, -235 rpm rated, -320 rpm peak
        velocity_max=100000,  # 100000 rpm electrical speed, 235 rpm rated, 320 rpm peak
        current_min=-15,  # -60A is the firmware limit, -31.9A is the rated peak, but set to -15A for current battery
        current_max=15,  # 60A is the firmware limit, 31.9A is the rated peak, but set to 15A for current battery
        torque_min=-53,  # Peak torque in Nm, -18 Nm rated
        torque_max=53,  # Peak torque in Nm, 18 Nm rated
        Kt_nominal=0.16,  # from TMotor website (actually 1/Kvll)
        current_factor=0.59,  # ! UNTESTED CONSTANT
        Kt_actual=0.921914 * 0.16,  # ! TESTED CONSTANT adjusted using torque sensor
        gear_ratio=9.0,
        num_pole_pairs=21,
        is_use_derived_torque_constants=False,  # `true` if you have a better model
    )
    AK80_9 = ServoParameters(
        position_min=-36000,  # -36000 deg
        position_max=36000,  # 36000 deg
        velocity_min=-100000,  # -100000 rpm electrical speed, -390 rpm rated, -570 rpm peak
        velocity_max=100000,  # 100000 rpm electrical speed, 390 rpm rated, 570 rpm peak
        current_min=-15,  # -60A is the firmware limit, -28A is the rated peak, but set to -15A for current battery
        current_max=15,  # 60A is the firmware limit, 28A is the rated peak, but set to 15A for current battery
        torque_min=-22,  # Peak torque in Nm, -9 Nm rated
        torque_max=22,  # Peak torque in Nm, 9 Nm rated
        Kt_nominal=0.091,  # from TMotor website (actually 1/Kvll)
        current_factor=0.59,
        Kt_actual=0.115,
        gear_ratio=9.0,
        num_pole_pairs=21,
        is_use_derived_torque_constants=False,  # `true` if you have a better model
    )
    AK80_8 = ServoParameters(
        position_min=-36000,  # -36000 deg
        position_max=36000,  # 36000 deg
        velocity_min=-100000,  # -100000 rpm electrical speed, -243 rpm rated, -298 rpm peak
        velocity_max=100000,  # 100000 rpm electrical speed, 243 rpm rated, 298 rpm peak
        current_min=-15,  # -60A is the firmware limit, -21A is the rated peak, but set to -15A for current battery
        current_max=15,  # 60A is the firmware limit, 21A is the rated peak, but set to 15A for current battery
        torque_min=-25,  # Peak torque in Nm, -10 Nm rated
        torque_max=25,  # Peak torque in Nm, 10 Nm rated
        Kt_nominal=0.199,  # from TMotor website (actually 1/Kvll)
        current_factor=0.59,
        Kt_actual=0.69 * 0.199,
        gear_ratio=8.0,
        num_pole_pairs=21,
        is_use_derived_torque_constants=False,  # `true` if you have a better model
    )


class EposDevice(Enum):
    EPOS = b"EPOS"
    EPOS2 = b"EPOS2"
    EPOS4 = b"EPOS4"


class EposProtocolStack(Enum):
    MAXON_RS232 = b"MAXON_RS232"
    MAXON_SERIAL_V2 = b"MAXON SERIAL V2"
    CAN_OPEN = b"CANopen"


# [Refer to the EPOS docs](https://www.maxongroup.com/medias/sys_master/root/9444047912990/EPOS4-Firmware-Specification-En.pdf#G5.2444977)
class EposOperationMode(Enum):
    PROFILE_POSITION = 1
    PROFILE_VELOCITY = 3
    HOMING = 6
    INTERPOLATED_POSITION = 7
    POSITION = -1               # Cyclic synchronous position mode (8)
    VELOCITY = -2               # Cyclic synchronous velocity mode (9)
    CURRENT = -3                # Cyclic synchronous current mode (10)
    MASTER_ENCODER = -5
    STEP_DIRECTION = -6


class EposState(Enum):
    DISABLED = 0
    ENABLED = 1
    QUICKSTOP = 2
    FAULT = 3


class HomingMethod(Enum):
    ACTUAL_POSITION = 35
    INDEX_POSITIVE_SPEED = 34
    INDEX_NEGATIVE_SPEED = 33
    HOME_SWITCH_NEGATIVE_SPEED = 27
    HOME_SWITCH_POSITIVE_SPEED = 23
    POSITIVE_LIMIT_SWITCH = 18
    NEGATIVE_LIMIT_SWITCH = 17
    HOME_SWITCH_NEGATIVE_SPEED_AND_INDEX = 11
    HOME_SWITCH_POSITIVE_SPEED_AND_INDEX = 7
    POSITIVE_LIMIT_SWITCH_AND_INDEX = 2
    NEGATIVE_LIMIT_SWITCH_AND_INDEX = 1
    NO_HOMING = 0
    CURRENT_THRESHOLD_POSITIVE_SPEED_AND_INDEX = -1
    CURRENT_THRESHOLD_NEGATIVE_SPEED_AND_INDEX = -2
    CURRENT_THRESHOLD_POSITIVE_SPEED = -3
    CURRENT_THRESHOLD_NEGATIVE_SPEED = -4


# TODO: used (used to be for CubeMars)
class ServoErrorCode(Enum):
    NO_ERR = 0  # No Error
    OVER_TEMP = 1  # Over temperature fault
    OVER_CURR = 2  # Over current fault
    OVER_VOLT = 3  # Over voltage fault
    UNDER_VOLT = 4  # Under voltage fault
    ENCODER_ERR = 5  # Encoder fault
    PHASE_IMBALANCE_ERR = 6  # Phase current unbalanced fault (The hardware may be damaged)


# TODO: update (used to be for CubeMars)
class ServoCanPacketEnum(Enum):
    DUTY_CYCLE_MODE = 0  # Motor is driven by a square wave voltage of specified duty cycle
    CURRENT_LOOP_MODE = 1  # Motor operates in torque loop mode
    CURRENT_BRAKE_MODE = 2  # Motor holds current position at specified braking current
    VELOCITY_MODE = 3  # Motor operates at specified target speed
    POSITION_MODE = 4  # Motor reaches specified target position at maximum speed
    SET_ORIGIN_MODE = 5  # Motor calibrates homing position
    POSITION_VELOCITY_MODE = 6  # Motor operates at specified position, velocity, and acceleration
    MOTOR_DISABLE_MODE = 15  # Motor gets disabled
    FEEDBACK_MESSAGE_CONFIG = 16  # Motor feedback data contents are updated
    VIRTUAL_IMPEDANCE_MODE = 17  # Motor operates in impedance mode, through torque loop mode as proxy


@dataclass
class EncoderData:
    timestamp: float
    angle: float
    is_error: bool


@dataclass
class ServoMotorData:
    timestamp: float
    position: float
    velocity: float
    current: float
    error: bool


class StateEnum:
    class Idle(Enum):
        IDLE = 0x0

    class Walking(Enum):
        IDLE = 0x10
        ONE_STEP = 0x11
        STANCE = 0x12
        SWING = 0x13

    class SitToStand(Enum):
        STANCE = 0x20
        LOWERING = 0x21
        SITTING = 0x22
        RISING = 0x23

    class StairAscent(Enum):
        STANCE = 0x30
        PUSH_OFF = 0x31
        SWING = 0x32
        STEP_UP = 0x33

    class StairDescent(Enum):
        IDLE = 0x40

    class Hurdle(Enum):
        STANCE = 0x50
        SWING = 0x51


class IntentCommandSource(Enum):
    CLI = 0
    GUI = 1
    AI = 2


class FatigueCommandSource(Enum):
    CLI = 0
    GUI = 1
    AI = 2


@dataclass
class StateTransition:
    timestamp: float
    state: int


@dataclass
class ModeTransition:
    timestamp: float
    mode: int
    sequence_id: int
    source: int


@dataclass
class PhaseEstimate:
    timestamp: float
    phase: float


@dataclass
class MotorCommand:
    motor_id: str
    timestamp: float
    command_data: bytes
    control_mode: int
    log_data: bytes


class CalibrationEventType(Enum):
    NICLA = "nicla"
    ENCODER = "encoder"


@dataclass
class CalibrationEvent:
    timestamp: float
    sensor_type: CalibrationEventType
    offsets: dict[str, float]


@dataclass
class AbsoluteEncoderOffset:
    reference: float
    offset: float


@dataclass
class NextModeSynchronized:
    lock: _Lock = field(init=False)
    next_value: "Synchronized[int]" = field(init=False)
    sequence_id: "Synchronized[int]" = field(init=False)
    source: "Synchronized[int]" = field(init=False)

    def __post_init__(self):
        self.lock = Lock()
        self.next_value = Value("i", lock=False)
        self.sequence_id = Value("i", lock=False)
        self.source = Value("i", lock=False)


@dataclass
class NextFatigueSynchronized:
    lock: _Lock = field(init=False)
    next_value: "Synchronized[float]" = field(init=False)
    sequence_id: "Synchronized[int]" = field(init=False)
    source: "Synchronized[int]" = field(init=False)

    def __post_init__(self):
        self.lock = Lock()
        self.next_value = Value("f", lock=False)
        self.sequence_id = Value("i", lock=False)
        self.source = Value("i", lock=False)


@dataclass
class NextIsPauseSynchronized:
    lock: _Lock = field(init=False)
    next_value: "Synchronized[bool]" = field(init=False)
    sequence_id: "Synchronized[int]" = field(init=False)

    def __post_init__(self):
        self.lock = Lock()
        self.next_value = Value("b", lock=False)
        self.sequence_id = Value("i", lock=False)


@dataclass
class ModeContext:
    handle: epos_handle
    K: dict[str, ServoImpedanceGains]
    _nicla_latest_data: dict[str, deque[NiclaData]]
    _encoder_latest_data: dict[MotorId, deque[EncoderData]]
    _motor_latest_data: dict[MotorId, deque[ServoMotorData]]
    next_mode: NextModeSynchronized
    next_fatigue: NextFatigueSynchronized
    mode_changed_queue: "Queue[ModeTransition]"
    state_changed_queue: "Queue[StateTransition]"
    phase_estimate_queue: "Queue[PhaseEstimate]"
    motor_command_queue: "Queue[MotorCommand]"
    is_stop_new_data_event: _Event
    is_keep_data_event: _Event
    config_manager: ConfigManager


@dataclass
class ModeTuple:
    id: int
    text: str


class ModeEnum(Enum):
    IDLE = ModeTuple(id=0, text="idle")
    WALKING = ModeTuple(id=1, text="walking")
    SIT_TO_STAND = ModeTuple(id=2, text="sit_to_stand")
    STAIR_ASCENT = ModeTuple(id=3, text="stair_ascent")
    STAIR_DESCENT = ModeTuple(id=4, text="stair_descent")
    HURDLE = ModeTuple(id=5, text="hurdle")


CLASS_TO_MODE = {
    0: ModeEnum.WALKING,
    1: ModeEnum.SIT_TO_STAND,
    2: ModeEnum.SIT_TO_STAND,
    3: ModeEnum.SIT_TO_STAND,
    4: ModeEnum.STAIR_ASCENT,
    5: ModeEnum.STAIR_DESCENT,
    6: ModeEnum.WALKING,
    7: ModeEnum.WALKING,
    8: ModeEnum.WALKING,
    9: ModeEnum.WALKING,
    10: ModeEnum.WALKING,
}


class StairLeadingLegEnum(Enum):
    NONE = 0
    RIGHT = 1
    LEFT = 2


class WalkingFirstStrideEnum(Enum):
    NONE = 0
    ONGOING = 1
    TAKEN = 2
    DEACTIVATED = 3


@dataclass
class StairAscentParameters2:
    # Transition thresholds
    stance_to_swing_th_gyr: float
    stance_to_swing_th_roll: float
    stance_to_swing_kn_roll: float
    swing_to_pushoff_gyr_min: float
    swing_to_pushoff_gyr_max: float
    swing_to_pushoff_th_roll: float
    pushoff_trigger_delay: float
    pushoff_to_stance_gyr_min: float
    pushoff_to_stance_gyr_max: float
    pushoff_to_stance_th_roll: float
    pushoff_to_stance_roll_small: float
    pushoff_to_stance_kn_roll: float

    # Action timing and impedance control
    imp_gain_rise_time: float
    gain_step: float
    pushoff_kn_threshold: float

    # Torque scaling
    torque_scale: float
    torque_norm_angle: float

    # Inactivity detection
    inactivity_gyr_threshold: float
    inactivity_time_step: float

    # Movement detection
    movement_angle_threshold: float
    movement_gyr_threshold: float
    movement_sum_threshold: float


@dataclass
class StairAscentParameters:
    # transitions threshold
    stance_to_push_off_th_gyr: float
    stance_to_push_off_th_roll: float
    push_off_to_swing_th_gyr: float
    push_off_to_swing_th_roll: float
    push_off_to_swing_pr_roll: float
    swing_to_step_up_th_gyr: float
    swing_to_step_up_th_roll: float
    swing_to_step_up_inactivity_dur: float
    step_up_to_stance_th_gyr_range: float
    step_up_to_stance_th_roll: float
    # for the torque rising
    risetime: float


@dataclass
class StairDescentParameters:
    # Transition thresholds
    double_to_swing_th_gyr: float
    stance_th_gyr_min: float
    stance_th_gyr_max: float
    stance_kn_roll: float
    stance_to_swing_th_gyr: float
    stance_to_swing_kn_roll: float
    swing_to_double_gyr_min: float
    swing_to_double_gyr_max: float
    swing_to_double_th_roll: float
    swing_to_double_inactive_dur: float
    stance_to_double_gyr_min: float
    stance_to_double_gyr_max: float
    stance_to_double_th_roll: float
    stance_to_double_inactive_dur: float

    # Action parameters
    angle_activation_threshold: float
    torque_knee: float
    torque_hip: float
    ramp_time: float
    gain_step: float

    # Inactivity detection
    inactivity_gyr_threshold: float
    inactivity_time_step: float

    # Movement detection
    movement_angle_threshold: float
    movement_gyr_threshold: float
    movement_sum_threshold: float


@dataclass
class SitToStandParameters:
    # Transition thresholds
    stance_to_lowering_th_gyr: float
    stance_to_lowering_th_roll: float
    stance_to_lowering_torso_roll: float
    stance_to_lowering_kn_roll: float
    lowering_to_sitting_th_gyr_min: float
    lowering_to_sitting_th_gyr_max: float
    lowering_to_sitting_phase_threshold: float
    sitting_to_rising_th_gyr: float
    sitting_to_rising_phase_threshold: float
    sitting_to_rising_idle_dur_min: float
    rising_to_stance_th_gyr_min: float
    rising_to_stance_th_gyr_max: float
    rising_to_stance_kn_roll: float
    rising_to_stance_phase_threshold: float
    reset_phase_threshold: float
    inactivity_angle_threshold: float

    # Impedance gains and timing
    imp_gain_rise_time: float
    gain_step: float
    active_ramp_time: float

    # Phase calculation parameters
    phase_start_angle: float
    phase_end_angle: float


@dataclass
class WalkingParameters2:
    inactivity_gyr_threshold: float
    inactivity_idle_transition_time: float
    traj_shift: dict[MotorId, float]
    inactivity_time_step: float
    first_stride_end_gyr: float
    reset_phase_threshold: float
    inactivity_angle_threshold: float
    ramp_time: float


@dataclass
class WalkingParameters:
    # transition thresholds
    idle_to_one_step_th_gyr: float
    to_idle_inactivity_dur: float
    one_step_to_stance_th_gyr: float
    stance_to_swing_th_gyr: float
    stance_to_swing_phase_threshold: float
    swing_to_stance_th_gyr: float
    swing_to_stance_phase_threshold: float
    swing_to_stance_bending_dur: float

    inactivity_gyr_threshold: float
    inactivity_idle_transition_time: float
    inactivity_time_step: float
    first_stride_end_gyr: float
    reset_phase_threshold: float
    inactivity_angle_threshold: float


@dataclass
class HurdlesParameters:
    stance_to_swing_th_roll_pr: float
    stance_to_swing_th_gyr_pr: float
    swing_to_stance_th_roll_pr: float


@dataclass
class ProsthesisMotorMapping:
    knee: str
    ankle: str


@dataclass
class NiclaSamples:
    torso_angle: float
    thigh_left_angle: float
    thigh_right_angle: float
    thigh_left_roll: float
    thigh_right_roll: float
    knee_left_roll: float
    knee_right_roll: float
    thigh_left_gyr: float
    thigh_right_gyr: float
    knee_right_gyr: float
    knee_left_gyr: float

    def __init__(
        self,
        euler: dict[NiclaLocation, float],
        gyroscope: dict[NiclaLocation, float]
    ):
        self.torso_angle = euler[NiclaLocation.TORSO]
        self.thigh_left_angle = euler[NiclaLocation.THIGH_LEFT]
        self.thigh_right_angle = euler[NiclaLocation.THIGH_RIGHT]

        self.thigh_left_roll = (
            self.torso_angle
            - self.thigh_left_angle
        )
        self.thigh_right_roll = (
            self.torso_angle
            - self.thigh_right_angle
        )
        self.knee_left_roll = (
            euler[NiclaLocation.SHANK_LEFT]
            - self.thigh_left_angle
        )
        self.knee_right_roll = (
            euler[NiclaLocation.SHANK_RIGHT]
            - self.thigh_right_angle
        )

        self.thigh_left_gyr = (
            gyroscope[NiclaLocation.THIGH_LEFT]
            - gyroscope[NiclaLocation.TORSO]
        )
        self.thigh_right_gyr = (
            gyroscope[NiclaLocation.THIGH_RIGHT]
            - gyroscope[NiclaLocation.TORSO]
        )
        self.knee_right_gyr = (
            gyroscope[NiclaLocation.SHANK_RIGHT]
            - gyroscope[NiclaLocation.THIGH_RIGHT]
        )
        self.knee_left_gyr = (
            gyroscope[NiclaLocation.SHANK_LEFT]
            - gyroscope[NiclaLocation.THIGH_LEFT]
        )