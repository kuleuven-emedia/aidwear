"""
CubeMars motors can operate in:
1. Duty cycle mode
    Duty cycle voltage is given, similar to square wave driving.
2. Current/torque loop mode
    Torque is given. NOTE: torque constant must be referenced from docs.
3. Current break mode
    Breaking torque is given, to resist movement. NOTE: must monitor temperature.
4. Velocity mode
    Speed is given. Moves at maximum acceleration.
5. Position mode
    Position is given. Moves at maximum speed and acceleration.
6. Position-velocity loop mode
    Position, speed and acceleration are given.

TODO: add the MIT force mode controller as described in the CubeMars documentation.
"""

from collections import deque
from queue import Queue
import struct
import can
import numpy as np

from hermes.utils.time_utils import get_time

from ..utils.types import (
    MotorCommand,
    ServoCanPacketEnum,
    ServoImpedanceGains,
    ServoMotorData,
    ServoMotorEnum,
    ServoReference,
)


def buffer_append_int64(
    buffer: list[int | bytes], number: np.int64
) -> list[int | bytes]:
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


def buffer_append_int32(
    buffer: list[int | bytes], number: np.int32
) -> list[int | bytes]:
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


def buffer_append_int16(
    buffer: list[int | bytes], number: np.int16
) -> list[int | bytes]:
    """Buffer allocation for 16-bit signed integer.

    Args:
        buffer (`list[int | bytes]`): Memory allocated to store data.
        number (`int`): Number in the corresponding bit range to pack into bytes.
    """
    buffer.append((number >> 8) & (0x00FF))
    buffer.append((number) & (0x00FF))
    return buffer


def buffer_append_uint8(
    buffer: list[int | bytes], number: np.uint8
) -> list[int | bytes]:
    """Buffer allocation for 8-bit unsigned integer.

    Args:
        buffer (`list[int | bytes]`): Memory allocated to store data.
        number (`int`): Number in the corresponding bit range to pack into bytes.
    """
    buffer.append((number) & (0xFF))
    return buffer


def send_can_message(
    bus: can.BusABC,
    motor_id: int,
    data: list[int | bytes],
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends a message to the motor, with a header of `motor_id` and data array of data.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        motor_id (`int`): The CAN ID of the motor to send to.
        data (`list[int | bytes]`): An array of integers or bytes of data to send.
    """
    message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

    try:
        bus.send(message)
        if motor_command_queue is not None:
            motor_command_queue.put(
                MotorCommand(
                    motor_id=(motor_id & 0xFF),
                    timestamp=get_time(),
                    data=data,
                    control_mode=(motor_id >> 8),
                )
            )
    except can.CanError as e:
        print("[CAN]: Message NOT sent: ", e, flush=True)


def can_set_duty(
    bus: can.BusABC,
    controller_id: int,
    duty: float,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends Servo control message for duty cycle control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        duty (`float`): Desired driving duty cycle [0, 1].
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(duty * 100000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_DUTY.value << 8),
        buffer,
        motor_command_queue,
    )


def can_set_torque(
    bus: can.BusABC,
    controller_id: int,
    torque: float,
    motor_type: ServoMotorEnum,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends Servo control message for current loop mode.

    Current loop mode: given the Iq current specified by the motor,
    the motor output torque = Iq *KT, so it can be used as a torque loop.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        torque (`float`): Desired current in Amps [-60.0, 60.0].
        motor_type: (`ServoMotorEnum`): Motor type, e.g. `ServoMotorEnum.AK10-9`.
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    buffer = []
    # NOTE: converts requested torque, accounting for gearbox, to select correct current.
    # linear fit
    # Kt_adjusted = 0.921914 * motor_type.value.Kt_nominal
    current = torque / motor_type.value.Kt_actual / motor_type.value.gear_ratio
    # quadratic fit
    # torque_adjusted = -0.00465 * torque**2 + 1.189967 * torque + -0.649202
    # current = (torque_adjusted / motor_type.value.Kt_nominal / motor_type.value.gear_ratio)

    if current > 30:
        current = 30
    elif current < -30:
        current = -30

    buffer = buffer_append_int32(buffer, np.int32(current * 1000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_CURRENT.value << 8),
        buffer,
        motor_command_queue,
    )


def can_set_break(
    bus: can.BusABC,
    controller_id: int,
    torque: float,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends Servo control message for current break mode.

    Current break mode: hold the motor in the current position with the
    specified breaking torque.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        torque (`float`): Desired current in Amps [0.0 to 60.0].
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(torque * 1000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_CURRENT_BRAKE.value << 8),
        buffer,
        motor_command_queue,
    )


def can_set_speed(
    bus: can.BusABC,
    controller_id: int,
    speed: float,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends Servo control message for velocity loop control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        speed (`float`): Desired motor speed [-100000.0, 100000.0].
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(speed))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_RPM.value << 8),
        buffer,
        motor_command_queue,
    )


def can_set_position(
    bus: can.BusABC,
    controller_id: int,
    position: float,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends Servo control message for position loop control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        position (`float`): Desired position in degrees [-36000.0, 36000.0].
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(position * 10000.0))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_POS.value << 8),
        buffer,
        motor_command_queue,
    )


def can_set_origin(
    bus: can.BusABC,
    controller_id: int,
    mode: int,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends Servo control message for origin setting.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        mode (`float`): Desired type of origin to set - temporary or permanent, [0, 1], respectively.
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    buffer = []
    buffer = buffer_append_uint8(buffer, np.uint8(mode))
    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_ORIGIN_HERE.value << 8),
        buffer,
        motor_command_queue,
    )


def can_set_position_speed(
    bus: can.BusABC,
    controller_id: int,
    position: float,
    speed: int,
    acceleration: int,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Sends Servo control message for position-velocity loop control mode.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        position (`float`): Desired position in degrees [-36000.0, 36000.0].
        speed (`int`): Desired speed in ERPM [-327680, 327680].
        acceleration (`int`): Desired position in ERPM/s^2 [0, 327670].
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    buffer = []
    buffer = buffer_append_int32(buffer, np.int32(position * 10000.0))
    buffer = buffer_append_int16(buffer, np.int16(speed / 10))
    buffer = buffer_append_int16(buffer, np.int16(acceleration / 10))

    send_can_message(
        bus,
        controller_id | (ServoCanPacketEnum.CAN_PACKET_SET_POS_SPD.value << 8),
        buffer,
        motor_command_queue,
    )


def can_set_position_impedance(
    bus: can.BusABC,
    controller_id: int,
    motor_latest_data: dict[int, deque[ServoMotorData | None]],
    ref: ServoReference,
    K: ServoImpedanceGains,
    motor_type: ServoMotorEnum,
    vel_prev: float | None = None,
    motor_command_queue: Queue[MotorCommand] | None = None,
) -> None:
    """Send a servo control message for impedance control.

    Supports position-velocity based impedance control, or position-velocity-acceleration if `vel_prev` is provided.

    Args:
        bus (`can.BusABC`): Reference to the CAN bus on which to send the data.
        controller_id (`int`): CAN ID of the motor to send the message to.
        motor_latest_data (dict[int, deque[ServoMotorState | None]]): Register of the most recent motor state.
        ref (`ServoReference`): Reference position, velocity and acceleration.
        K (`ServoImpedanceGains`): Impedance gains ([Kp, Kd, Ka]).
        motor_type (`ServoMotorEnum`): Motor type, e.g. AK10-9.
        vel_prev (`float`, optional): Previous velocity to use for modulating the de/acceleration component of the impedance controller. Defaults to `None`.
        motor_command_queue (`Queue[MotorCommand] | None`): Multiprocessing queue to pass upstream outgoing motor CAN commands. Defaults to `None`.
    """
    tsample = 0.02
    motor_data = motor_latest_data[controller_id][-1]

    if motor_data is None:
        return

    if vel_prev is not None:
        torque = (
            K.position * (ref.position - motor_data.position)
            + K.velocity * (ref.velocity - motor_data.velocity)
            + K.acceleration
            * (ref.acceleration - (motor_data.velocity - vel_prev) / tsample)
        )
    else:
        torque = K.position * (ref.position - motor_data.position) + K.velocity * (
            ref.velocity - motor_data.velocity
        )

    can_set_torque(bus, controller_id, torque, motor_type, motor_command_queue)


def parse_servo_message(
    timestamp: float, data: bytes, motor_type: ServoMotorEnum
) -> ServoMotorData:
    """Parses the CAN frame message into a `ServoMotorData` object.

    Bytes from CubeMars motors arrive in big-endian order.
    [0-1]: Motor position [-3200, 3200] degrees (int16).
    [2-3]: Motor speed [-320000, 320000] RPM (int16).
    [4-5]: Motor current [-60, 60] A (int16).
    [6]: Motor temperature [-20, 127] Celsius (int8).
    [7]: Motor error [0, 7] codes (uint8).

    Args:
        timestamp (`float`): Time since epoch when the CAN message was received, where possible, stamped in hardware.
        data (`bytes`): Decoded bytes from the CAN frame of a motor.
        motor_type (`ServoMotorEnum`): Motor identifier with specific parameters.

    Returns:
        A ServoMotorData object representing the motor's state from the received data.
    """
    pos_int16, spd_int16, cur_int16, temp_int8, err_uint8 = struct.unpack(
        ">hhhbB", data
    )
    motor_position = float(pos_int16 * 0.1)
    motor_speed = float(
        spd_int16
        * 10.0
        / (motor_type.value.gear_ratio * motor_type.value.num_pole_pairs)
    )
    motor_current = float(cur_int16 * 0.01)

    return ServoMotorData(
        timestamp=timestamp,
        position=motor_position,
        velocity=motor_speed,
        current=motor_current,
        temperature=temp_int8,
        error=err_uint8,
    )
