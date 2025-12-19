"""
CANOpen commands for Epos motor drivers controlling Maxon motors.
"""

from collections import deque
import can
import numpy as np

from ..utils.types import (
    ServoCanPacketEnum,
    ServoImpedanceGains,
    ServoMotorState,
    ServoMotorEnum,
    ServoReference
)


def buffer_append_int64(buffer: list[int | bytes], number: np.int64) -> list[int | bytes]:
    """Buffer allocation for 64-bit signed integer.

    Args:
        buffer (`list[int | bytes]`): Memory allocated to store data.
        number (`int`): Number in the corresponding bit range to pack into bytes.
    """
    buffer.append((number >> 56) & (0x00000000000000FF))
    buffer.append((number >> 48) & (0x00000000000000FF))
    buffer.append((number >> 40) & (0x00000000000000FF))
    buffer.append((number >> 31) & (0x00000000000000FF))
    buffer.append((number >> 24) & (0x00000000000000FF))
    buffer.append((number >> 16) & (0x00000000000000FF))
    buffer.append((number >> 8) & (0x00000000000000FF))
    buffer.append((number) & (0x00000000000000FF))
    return buffer


def buffer_append_int32(buffer: list[int | bytes], number: np.int32) -> list[int | bytes]:
    """Buffer allocation for 32-bit signed integer.

    Args:
        buffer (`list[int | bytes]`): Memory allocated to store data.
        number (`int`): Number in the corresponding bit range to pack into bytes.
    """
    buffer.append((number >> 24) & (0x000000FF))
    buffer.append((number >> 16) & (0x000000FF))
    buffer.append((number >> 8) & (0x000000FF))
    buffer.append((number) & (0x000000FF))
    return buffer


def buffer_append_int16(buffer: list[int | bytes], number: np.int16) -> list[int | bytes]:
    """Buffer allocation for 16-bit signed integer.

    Args:
        buffer (`list[int | bytes]`): Memory allocated to store data.
        number (`int`): Number in the corresponding bit range to pack into bytes.
    """
    buffer.append((number >> 8) & (0x00FF))
    buffer.append((number) & (0x00FF))
    return buffer


def buffer_append_uint8(buffer: list[int | bytes], number: np.uint8) -> list[int | bytes]:
    """Buffer allocation for 8-bit unsigned integer.

    Args:
        buffer (`list[int | bytes]`): Memory allocated to store data.
        number (`int`): Number in the corresponding bit range to pack into bytes.
    """
    buffer.append((number) & (0xFF))
    return buffer


def send_can_message(bus: can.BusABC, motor_id: int, data: list[int | bytes]) -> None:
    """Sends a message to the motor, with a header of `motor_id` and data array of data.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        motor_id (`int`): The CAN ID of the motor to send to.
        data (`list[int | bytes]`): An array of integers or bytes of data to send.
    """
    message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

    try:
        bus.send(message)
    except can.CanError as e:
        print("[CAN]: Message NOT sent: ", e, flush=True)


def can_set_duty(bus: can.BusABC, controller_id: int, duty: float) -> None:
    """Sends Servo control message for duty cycle control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        duty (`float`): Desired driving duty cycle [0, 1].
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(duty * 100000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_DUTY << 8),
        buffer,
    )


def can_set_torque(bus: can.BusABC, controller_id: int, torque: float, motor_type: ServoMotorEnum) -> None:
    """Sends Servo control message for current loop mode.

    Current loop mode: given the Iq current specified by the motor,
    the motor output torque = Iq *KT, so it can be used as a torque loop.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        torque (`float`): Desired current in Amps [-60.0, 60.0].
        motor_type: (`ServoMotorEnum`): Motor type, e.g. `ServoMotorEnum.AK10-9`.
    """
    buffer = []
    # NOTE: converts requested torque, accounting for gearbox, to select correct current.
    # linear fit
    # Kt_adjusted = 0.921914 * motor_type.value.Kt_nominal
    current = (
        torque
        / motor_type.value.Kt_actual
        / motor_type.value.gear_ratio
    )
    # quadratic fit
    # torque_adjusted = -0.00465 * torque**2 + 1.189967 * torque + -0.649202
    # current = (torque_adjusted / motor_type.value.Kt_nominal / motor_type.value.gear_ratio)

    if current > 30:
        current = 30
    if current < -30:
        current = -30

    buffer = buffer_append_int32(buffer, np.int32(current * 1000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_CURRENT << 8),
        buffer,
    )


def can_set_break(bus: can.BusABC, controller_id: int, torque: float) -> None:
    """Sends Servo control message for current break mode.

    Current break mode: hold the motor in the current position with the
    specified breaking torque.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        torque (`float`): Desired current in Amps [0.0 to 60.0].
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(torque * 1000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_CURRENT_BRAKE << 8),
        buffer,
    )


def can_set_speed(bus: can.BusABC, controller_id: int, speed: float) -> None:
    """Sends Servo control message for velocity loop control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        speed (`float`): Desired motor speed [-100000.0, 100000.0].
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(speed))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_RPM << 8),
        buffer,
    )


def can_set_speed(bus: can.BusABC, controller_id: int, position: float) -> None:
    """Sends Servo control message for position loop control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        position (`float`): Desired position in degrees [-36000.0, 36000.0].
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(position * 10000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_POS << 8),
        buffer,
    )


def can_set_origin(bus: can.BusABC, controller_id: int, mode: int) -> None:
    """Sends Servo control message for origin setting.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        mode (`float`): Desired type of origin to set - temporary or permanent, [0, 1], respectively.
    """
    buffer = []
    buffer = buffer_append_uint8(buffer, np.uint8(mode))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_ORIGIN_HERE << 8),
        buffer,
    )


def can_set_position_speed(bus: can.BusABC, controller_id: int, position: float, speed: int, acceleration: int) -> None:
    """Sends Servo control message for position-velocity loop control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        position (`float`): Desired position in degrees [-36000.0, 36000.0].
        speed (`int`): Desired speed in ERPM [-327680, 327680].
        acceleration (`int`): Desired position in ERPM/s^2 [0, 327670].
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(position * 10000.0))
    buffer = buffer_append_int16(buffer, np.int16(speed / 10))
    buffer = buffer_append_int16(buffer, np.int16(acceleration / 10))

    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_POS_SPD << 8),
        buffer,
    )


def can_set_position_impedance(
    bus: can.BusABC,
    controller_id: int,
    motor_latest_state: dict[int, deque[ServoMotorState | None]],
    ref: ServoReference,
    K: ServoImpedanceGains,
    motor_type: ServoMotorEnum,
    vel_prev: float | None = None
) -> None:
    """Send a servo control message for impedance control.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        motor_latest_state (dict[int, deque[ServoMotorState | None]]): Register of the most recent motor state.
        ref (`ServoReference`): Reference position, velocity and acceleration.
        K (`ServoImpedanceGains`): Impedance gains ([Kp, Kd, Ka]).
        motor_type (`ServoMotorEnum`): Motor type, e.g. AK10-9.
        vel_prev (`float`, optional): . Defaults to `None`.
    """
    tsample = 0.02
    motor_state = motor_latest_state[controller_id][-1]

    if motor_state is None:
        return

    if vel_prev is not None:
        torque = (
            K.position * (ref.position - motor_state.position)
            + K.velocity * (ref.velocity - motor_state.velocity)
            + K.acceleration * (ref.acceleration - (motor_state.velocity - vel_prev) / tsample)
        )
    else:
        torque = K.position * (ref.position - motor_state.position) + K.velocity * (ref.velocity - motor_state.velocity)

    can_set_torque(bus, controller_id, torque, motor_type)


def parse_servo_message(timestamp: float, data: bytes, motor_type: ServoMotorEnum) -> ServoMotorState:
    """Parses the CAN frame message into a `ServoMotorState` object.

    Args:
        timestamp (`float`): Time since epoch when the CAN message was received, where possible, stamped in hardware.
        data (`bytes`): Decoded bytes from the CAN frame of a motor.
        motor_type (`ServoMotorEnum`): Motor identifier with specific parameters.

    Returns:
        A `ServoMotorState` object representing the motor's state from the received data.
    """
    # Convert signed/unsigned integers using Numpy.
    pos_int = np.array(data[0] << 8 | data[1]).astype("int16")  # [-3200, 3200] degrees
    spd_int = np.array(data[2] << 8 | data[3]).astype("int16")  # [-320000, 320000] RPM
    cur_int = np.array(data[4] << 8 | data[5]).astype("int16")  # [-60, 60] A
    motor_position = float(pos_int * 0.1)
    motor_speed = float(
        spd_int
        * 10.0
        / (
            motor_type.value.gear_ratio
            * motor_type.value.num_pole_pairs
        )
    )
    motor_current = float(cur_int * 0.01)
    motor_temperature = np.array(data[6]).astype("int8")    # [-20, 127] Celsius
    motor_error = np.array(data[7]).astype("uint8")         # [0, 7] codes

    return ServoMotorState(
        timestamp=timestamp,
        position=motor_position,
        velocity=motor_speed,
        current=motor_current,
        temperature=motor_temperature,
        error=motor_error
    )
