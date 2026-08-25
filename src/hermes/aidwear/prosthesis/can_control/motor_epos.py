"""
Filename: hermes/aidwear/prosthesis/can_control/motor_epos.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-07-02
Version: 1.0
Description: Python wrapper for Maxon EPOS4 Compact motor drives via the EposCmd
    C library, serving as a plug-and-play replacement for CubeMars interfaces.

    NOTE: Only most relevant EPOS commands are implemented here, and only the CANopen protocol is supported.

    Maxon EPOS4 motors can operate in various modes, mapped here to match prior logic:
    1. Duty cycle mode -> Not natively supported. Replaced by Torque/Current mode.
    2. Current/torque loop mode -> Mapped to EPOS Current Mode (CM).
    3. Current break mode -> Mapped to EPOS Quick Stop / Halt commands.
    4. Velocity mode -> Mapped to EPOS Profile Velocity Mode (PVM).
    5. Position mode -> Mapped to EPOS Profile Position Mode (PPM).

    [Command Library](https://www.maxongroup.com/medias/sys_master/root/9157360353310/EPOS-Command-Library-En.pdf)
    [Firmware Specification](https://www.maxongroup.com/medias/sys_master/root/9444047912990/EPOS4-Firmware-Specification-En.pdf)
"""

import time
import ctypes
import ctypes.util
from collections import deque
from queue import Queue
from multiprocessing.synchronize import Event as _Event
from typing import Tuple, TypeAlias

from hermes.utils.time_utils import get_time

from ..utils.types import (
    EposDevice,
    EposOperationMode,
    EposProtocolStack,
    EposState,
    MotorCommand,
    ServoCanPacketEnum,
    ServoErrorCode,
    ServoImpedanceGains,
    ServoMotorData,
    ServoMotorEnum,
    ServoReference,
    MotorId,
)

# ============================================================================
# CTYPES BINDINGS FOR EPOS COMMAND LIBRARY
# ============================================================================
name = ctypes.util.find_library("EposCmd")
if not name:
    raise OSError(
        "libEposCmd.so / EposCmd.dll not found. Please install the EPOS Command Library."
    )
epos = ctypes.CDLL(name)

# Type Aliases for readability
epos_char_p: TypeAlias = ctypes.c_byte
epos_int8: TypeAlias = ctypes.c_byte
epos_uint8: TypeAlias = ctypes.c_ubyte
epos_int16: TypeAlias = ctypes.c_short
epos_uint16: TypeAlias = ctypes.c_ushort
epos_int32: TypeAlias = ctypes.c_long
epos_uint32: TypeAlias = ctypes.c_ulong
epos_bool: TypeAlias = ctypes.c_long
epos_handle: TypeAlias = ctypes.c_void_p

# ============================================================================
# Device General C-specifiers
# ============================================================================
epos.VCS_OpenDevice.restype = epos_handle
epos.VCS_OpenDevice.argtypes = [
    epos_char_p,
    epos_char_p,
    epos_char_p,
    epos_char_p,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_CloseDevice.restype = epos_bool
epos.VCS_CloseDevice.argtypes = [epos_handle, ctypes.POINTER(epos_uint32)]
epos.VCS_CloseAllDevices.restype = epos_bool
epos.VCS_CloseAllDevices.argtypes = [ctypes.POINTER(epos_uint32)]
epos.VCS_SetProtocolStackSettings.restype = epos_bool
epos.VCS_SetProtocolStackSettings.argtypes = [
    epos_handle,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetProtocolStackSettings.restype = epos_bool
epos.VCS_GetProtocolStackSettings.argtypes = [
    epos_handle,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetErrorInfo.restype = epos_bool
epos.VCS_GetErrorInfo.argtypes = [epos_uint32, ctypes.POINTER(epos_char_p), epos_uint16]

# ============================================================================
# Operation Mode C-specifiers
# ============================================================================
epos.VCS_SetOperationMode.restype = epos_bool
epos.VCS_SetOperationMode.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int8,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetOperationMode.restype = epos_bool
epos.VCS_GetOperationMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int8),
    ctypes.POINTER(epos_uint32),
]

# ============================================================================
# State Machine C-specifiers
# ============================================================================
epos.VCS_ResetDevice.restype = epos_bool
epos.VCS_ResetDevice.restype = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
epos.VCS_SetState.restype = epos_bool
epos.VCS_SetState.restype = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetState.restype = epos_bool
epos.VCS_GetState.restype = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ClearFault.restype = epos_bool
epos.VCS_ClearFault.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
epos.VCS_GetFaultState.restype = epos_bool
epos.VCS_GetFaultState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]

# ============================================================================
# Getters C-specifiers
# ============================================================================
epos.VSC_GetMovementState.restype = epos_bool
epos.VSC_GetMovementState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetPositionIs.restype = epos_bool
epos.VCS_GetPositionIs.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetVelocityIs.restype = epos_bool
epos.VCS_GetVelocityIs.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetVelocityIsAveraged.restype = epos_bool
epos.VCS_GetVelocityIsAveraged.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetCurrentIsEx.restype = epos_bool
epos.VCS_GetCurrentIsEx.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetCurrentIsAveragedEx.restype = epos_bool
epos.VCS_GetCurrentIsAveragedEx.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_WaitForTargetReached.restype = epos_bool
epos.VCS_WaitForTargetReached.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]

# ============================================================================
# PPM (Profile Position Mode) C-specifiers
# ============================================================================
epos.VSC_ActivateProfilePositionMode.restype = epos_bool
epos.VSC_ActivateProfilePositionMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_SetPositionProfile.restype = epos_bool
epos.VSC_SetPositionProfile.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_GetPositionProfile.restype = epos_bool
epos.VSC_GetPositionProfile.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VSC_MoveToPosition.restype = epos_bool
epos.VSC_MoveToPosition.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    epos_bool,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_GetTargetPosition.restype = epos_bool
epos.VSC_GetTargetPosition.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_HaltPositionMovement.restype = epos_bool
epos.VCS_HaltPositionMovement.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_EnablePositionWindow.restype = epos_bool
epos.VCS_EnablePositionWindow.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisablePositionWindow.restype = epos_bool
epos.VCS_DisablePositionWindow.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ============================================================================
# PVM (Profile Velocity Mode) C-specifiers
# ============================================================================
epos.VSC_ActivateProfileVelocityMode.restype = epos_bool
epos.VSC_ActivateProfileVelocityMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_SetVelocityProfile.restype = epos_bool
epos.VSC_SetVelocityProfile.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_GetVelocityProfile.restype = epos_bool
epos.VSC_GetVelocityProfile.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_MoveWithVelocity.restype = epos_bool
epos.VCS_MoveWithVelocity.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_GetTargetVelocity.restype = epos_bool
epos.VSC_GetTargetVelocity.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_HaltVelocityMovement.restype = epos_bool
epos.VCS_HaltVelocityMovement.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_EnableVelocityWindow.restype = epos_bool
epos.VCS_EnableVelocityWindow.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisableVelocityWindow.restype = epos_bool
epos.VCS_DisableVelocityWindow.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# Homing mode
epos.VSC_ActivateHomingMode.restype = epos_bool
epos.VSC_ActivateHomingMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_SetHomingParameter.restype = epos_bool
epos.VSC_SetHomingParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    epos_uint32,
    epos_int32,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VSC_GetHomingParameter.restype = epos_bool
epos.VSC_GetHomingParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]

epos.VCS_FindHome.restype = epos_bool
epos.VCS_FindHome.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int8,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_StopHoming.restype = epos_bool
epos.VCS_StopHoming.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_DefinePosition.restype = epos_bool
epos.VCS_DefinePosition.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_GetHomingState.restype = epos_bool
epos.VCS_GetHomingState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_WaitForHomingAttained.restype = epos_bool
epos.VCS_WaitForHomingAttained.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]

# ============================================================================
# Interpolated Position Mode (IPM) C-specifiers
# ============================================================================
epos.VCS_ActivateInterpolatedPositionMode.restype = epos_bool
epos.VCS_ActivateInterpolatedPositionMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_SetIpmBufferParameter.restype = epos_bool
epos.VCS_SetIpmBufferParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_GetIpmBufferParameter.restype = epos_bool
epos.VCS_GetIpmBufferParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]

epos.VCS_ClearIpmBuffer.restype = epos_bool
epos.VCS_ClearIpmBuffer.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_GetFreeIpmBufferSize.restype = epos_bool
epos.VCS_GetFreeIpmBufferSize.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]

epos.VCS_AddPvtValueToIpmBuffer.restype = epos_bool
epos.VCS_AddPvtValueToIpmBuffer.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    epos_int32,
    epos_uint8,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_StartIpmTrajectory.restype = epos_bool
epos.VCS_StartIpmTrajectory.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_StopIpmTrajectory.restype = epos_bool
epos.VCS_StopIpmTrajectory.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

epos.VCS_GetIpmStatus.restype = epos_bool
epos.VCS_GetIpmStatus.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]


# ============================================================================
# Position Mode (PM) C-specifiers
# ============================================================================

# ============================================================================
# HELPER UTILITIES
# ============================================================================
def get_error_info(error_code: epos_uint32) -> str:
    """Returns a human-readable error message for a given EPOS error code."""
    buf = ctypes.create_string_buffer(1024)
    epos.VCS_GetErrorInfo(error_code.value, buf, 1024)
    return buf.value.decode(errors="ignore")


def check_error(success: bool, error_code: ctypes.c_uint):
    """Parses and throws an error if an EPOS command fails."""
    if not success and error_code.value != 0:
        print(f"[MAXON ERROR] 0x{error_code.value:08X}: {get_error_info(error_code)}")


def open_device(
    device_name: EposDevice,
    protocol_stack_name: EposProtocolStack,
    interfac_name: str,
    port_name: str,
) -> epos_handle:
    error_code = epos_uint32()
    handle = epos.VCS_OpenDevice(
        device_name.value,
        protocol_stack_name.value,
        interfac_name,
        port_name,
        ctypes.byref(error_code),
    )
    if not handle:
        raise RuntimeError(
            f"[MAXON ERR] Failed to open {device_name.value} device: ",
            hex(error_code.value),
        )
    return handle


def set_protocol_stack_settings(
    handle: epos_handle,
    baudrate: int,
    timeout_ms: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_SetProtocolStackSettings(
        handle, baudrate, timeout_ms, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to set protocol settings: ", hex(error_code.value)
        )
    return True


def get_protocol_stack_settings(handle: epos_handle) -> Tuple[int, int]:
    baudrate = epos_uint32()
    timeout_ms = epos_uint32()
    error_code = epos_uint32()
    status = epos.VCS_GetProtocolStackSettings(
        handle,
        ctypes.byref(baudrate),
        ctypes.byref(timeout_ms),
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get protocol settings: ", hex(error_code.value)
        )
    return baudrate.value, timeout_ms.value


def close_all_devices() -> bool:
    error_code = epos_uint32()
    status = epos.VCS_CloseAllDevices(ctypes.byref(error_code))
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to close all devices: ", hex(error_code.value)
        )
    return True


def close_device(
    handle: epos_handle,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_CloseDevice(handle, ctypes.byref(error_code))
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to close device: ", hex(error_code.value)
        )
    return True


def set_operation_mode(
    handle: epos_handle,
    motor_id: MotorId,
    mode: EposOperationMode,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_SetOperationMode(
        handle, motor_id.value, mode.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to set operation mode: ", hex(error_code.value)
        )
    return True


def get_operation_mode(
    handle: epos_handle,
    motor_id: MotorId,
) -> EposOperationMode:
    error_code = epos_uint32()
    mode = epos_int8()
    status = epos.VCS_GetOperationMode(
        handle, motor_id.value, ctypes.byref(mode), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get operation mode: ", hex(error_code.value)
        )
    return EposOperationMode(mode.value)


def reset_device(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_ResetDevice(handle, motor_id.value, ctypes.byref(error_code))
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to reset device {motor_id}: ", hex(error_code.value)
        )
    return True


def set_state(
    handle: epos_handle,
    motor_id: MotorId,
    state: EposState,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_SetState(
        handle, motor_id.value, state.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to set EPOS state machine of {motor_id} to {state}: ",
            hex(error_code.value),
        )
    return True


def clear_fault(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    """Changes the device state from `EposState.FAULT` to `EposState.DISABLED`."""
    error_code = epos_uint32()
    status = epos.VCS_ClearFault(handle, motor_id.value, ctypes.byref(error_code))
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to clear the fault of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def get_state(
    handle: epos_handle,
    motor_id: MotorId,
) -> EposState:
    error_code = epos_uint32()
    state = epos_uint16()
    status = epos.VCS_GetState(
        handle, motor_id.value, ctypes.byref(state), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get the of the {motor_id}'s state machine: ",
            hex(error_code.value),
        )
    return EposState(state.value)


def is_fault(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    is_fault = epos_bool()
    status = epos.VCS_GetFaultState(
        handle, motor_id.value, ctypes.byref(is_fault), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to read the fault state of {motor_id}: ",
            hex(error_code.value),
        )
    return is_fault


def is_target_reached(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    is_reached = epos_bool()
    status = epos.VCS_GetMovementState(
        handle, motor_id.value, ctypes.byref(is_reached), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to check whether {motor_id} reached its target: ",
            hex(error_code.value),
        )
    return is_reached


def get_position(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    position = epos_int32()
    status = epos.VCS_GetPositionIs(
        handle, motor_id.value, ctypes.byref(position), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get position of {motor_id}: ", hex(error_code.value)
        )
    return position


def get_velocity(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    velocity = epos_int32()
    status = epos.VCS_GetVelocityIs(
        handle, motor_id.value, ctypes.byref(velocity), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get velocity of {motor_id}: ", hex(error_code.value)
        )
    return velocity


def get_velocity_avg(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    velocity = epos_int32()
    status = epos.VCS_GetVelocityIsAveraged(
        handle, motor_id.value, ctypes.byref(velocity), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get averaged velocity of {motor_id}: ",
            hex(error_code.value),
        )
    return velocity


def get_current(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    current = epos_int32()
    status = epos.VCS_GetCurrentIsEx(
        handle, motor_id.value, ctypes.byref(current), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get current of {motor_id}: ", hex(error_code.value)
        )
    return current


def get_current_avg(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    current = epos_int32()
    status = epos.VCS_GetCurrentIsAveragedEx(
        handle, motor_id.value, ctypes.byref(current), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get averaged current of {motor_id}: ",
            hex(error_code.value),
        )
    return current


def wait_target_reached(
    handle: epos_handle,
    motor_id: MotorId,
    timeout_ms: int,
) -> bool:
    """Waits until the state is changed to target reached or until the time is up."""
    error_code = epos_uint32()
    status = epos.VCS_WaitForTargetReached(
        handle, motor_id.value, timeout_ms, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to wait for target reached of {motor_id}: ",
            hex(error_code.value),
        )
    return True


# ============================================================================
# PPM (Profile Position Mode) specific commands
# ============================================================================
def activate_profile_position_mode(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_ActivateProfilePositionMode(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to activate profile position mode of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ppm_set_position_profile(
    handle: epos_handle,
    motor_id: MotorId,
    velocity: int,
    acceleration: int,
    deceleration: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VSC_SetPositionProfile(
        handle,
        motor_id.value,
        velocity,
        acceleration,
        deceleration,
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to set position profile parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ppm_get_position_profile(
    handle: epos_handle,
    motor_id: MotorId,
) -> Tuple[int, int, int]:
    error_code = epos_uint32()
    velocity = epos_uint32()
    acceleration = epos_uint32()
    deceleration = epos_uint32()
    status = epos.VSC_GetPositionProfile(
        handle,
        motor_id.value,
        ctypes.byref(velocity),
        ctypes.byref(acceleration),
        ctypes.byref(deceleration),
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get position profile parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return velocity.value, acceleration.value, deceleration.value


def ppm_move_to_position(
    handle: epos_handle,
    motor_id: MotorId,
    position: int,
    is_absolute: bool,
    is_immediately: bool,
) -> bool:
    error_code = epos_uint32()
    status = epos.VSC_MoveToPosition(
        handle,
        motor_id.value,
        position,
        is_absolute,
        is_immediately,
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to move {motor_id} to position {position}: ",
            hex(error_code.value),
        )
    return True


def ppm_get_target_position(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    target_position = epos_int32()
    status = epos.VSC_GetTargetPosition(
        handle, motor_id.value, ctypes.byref(target_position), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get target position of {motor_id}: ",
            hex(error_code.value),
        )
    return target_position.value


def ppm_halt_position_movement(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    """Stops the movement with profile deceleration."""
    error_code = epos_uint32()
    status = epos.VCS_HaltPositionMovement(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to halt position of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ppm_enable_position_window(
    handle: epos_handle,
    motor_id: MotorId,
    window_size: int,
    window_time: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_EnablePositionWindow(
        handle, motor_id.value, window_size, window_time, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to activate position window of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ppm_disable_position_window(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_DisablePositionWindow(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to activate position window of {motor_id}: ",
            hex(error_code.value),
        )
    return True


# ============================================================================
# PVM (Profile Velocity Mode) specific commands
# ============================================================================
def activate_profile_velocity_mode(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_ActivateProfileVelocityMode(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to activate profile velocity mode of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def pvm_set_velocity_profile(
    handle: epos_handle,
    motor_id: MotorId,
    acceleration: int,
    deceleration: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VSC_SetVelocityProfile(
        handle, motor_id.value, acceleration, deceleration, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to set velocity profile parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def pvm_get_velocity_profile(
    handle: epos_handle,
    motor_id: MotorId,
) -> Tuple[int, int]:
    error_code = epos_uint32()
    acceleration = epos_uint32()
    deceleration = epos_uint32()
    status = epos.VSC_GetVelocityProfile(
        handle,
        motor_id.value,
        ctypes.byref(acceleration),
        ctypes.byref(deceleration),
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get velocity profile parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return acceleration.value, deceleration.value


def pvm_move_with_velocity(
    handle: epos_handle,
    motor_id: MotorId,
    velocity: int,
) -> bool:
    """Starts the movement with velocity profile to target velocity. The velocity is interpreted according to the currently configured velocity unit."""
    error_code = epos_uint32()
    status = epos.VCS_MoveWithVelocity(
        handle, motor_id.value, velocity, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to move {motor_id} with velocity {velocity}: ",
            hex(error_code.value),
        )
    return True


def pvm_get_target_velocity(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    target_velocity = epos_int32()
    status = epos.VSC_GetTargetVelocity(
        handle, motor_id.value, ctypes.byref(target_velocity), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get target velocity of {motor_id}: ",
            hex(error_code.value),
        )
    return target_velocity.value


def pvm_halt_velocity_movement(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    """Stops the movement with profile deceleration."""
    error_code = epos_uint32()
    status = epos.VCS_HaltVelocityMovement(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to halt velocity of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def pvm_enable_velocity_window(
    handle: epos_handle,
    motor_id: MotorId,
    window_size: int,
    window_time: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_EnableVelocityWindow(
        handle, motor_id.value, window_size, window_time, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to activate velocity window of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def pvm_disable_velocity_window(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_DisableVelocityWindow(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to deactivate velocity window of {motor_id}: ",
            hex(error_code.value),
        )
    return True


# ============================================================================
# HM (Homing Mode) specific commands
# ============================================================================
def activate_homing_mode(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VSC_ActivateHomingMode(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to activate homing mode of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def hm_set_homing_parameter(
    handle: epos_handle,
    motor_id: MotorId,
    acceleration: int,
    speed_switch: int,
    speed_index: int,
    offset: int,
    threshold: int,
    position: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VSC_SetHomingParameter(
        handle,
        motor_id.value,
        acceleration,
        speed_switch,
        speed_index,
        offset,
        threshold,
        position,
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to set homing parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def hm_get_homing_parameter(
    handle: epos_handle,
    motor_id: MotorId,
) -> Tuple[int, int, int, int, int, int]:
    error_code = epos_uint32()
    acceleration = epos_uint32()
    speed_switch = epos_uint32()
    speed_index = epos_uint32()
    offset = epos_int32()
    threshold = epos_uint16()
    position = epos_int32()
    status = epos.VSC_GetHomingParameter(
        handle,
        motor_id.value,
        ctypes.byref(acceleration),
        ctypes.byref(speed_switch),
        ctypes.byref(speed_index),
        ctypes.byref(offset),
        ctypes.byref(threshold),
        ctypes.byref(position),
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get homing parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return (
        acceleration.value,
        speed_switch.value,
        speed_index.value,
        offset.value,
        threshold.value,
        position.value,
    )


def hm_start_homing(
    handle: epos_handle,
    motor_id: MotorId,
    homing_method: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_FindHome(
        handle, motor_id.value, homing_method, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to find home of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def hm_stop_homing(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_StopHoming(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to stop homing of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def hm_set_home(
    handle: epos_handle,
    motor_id: MotorId,
    position: int,
) -> bool:
    """Sets current position as home."""
    error_code = epos_uint32()
    status = epos.VCS_DefinePosition(
        handle, motor_id.value, position, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to define home position of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def hm_get_state(
    handle: epos_handle,
    motor_id: MotorId,
) -> Tuple[bool, bool]:
    error_code = epos_uint32()
    is_homed = epos_bool()
    is_error = epos_bool()
    status = epos.VCS_GetHomingState(
        handle, motor_id.value, ctypes.byref(is_homed), ctypes.byref(is_error), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get homing state of {motor_id}: ",
            hex(error_code.value),
        )
    return is_homed.value, is_error.value


def hm_wait_for_homing(
    handle: epos_handle,
    motor_id: MotorId,
    timeout_ms: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_WaitForHomingAttained(
        handle, motor_id.value, timeout_ms, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to wait for homing completion of {motor_id}: ",
            hex(error_code.value),
        )
    return True


# ============================================================================
# IPM (Interpolated Position Mode) specific commands
# ============================================================================
def activate_interpolated_position_mode(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_ActivateInterpolatedPositionMode(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to activate interpolated position mode of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ipm_set_buffer_parameter(
    handle: epos_handle,
    motor_id: MotorId,
    underflow_limit: int,
    overflow_limit: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_SetIpmBufferParameter(
        handle, motor_id.value, underflow_limit, overflow_limit, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to set interpolated position buffer parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ipm_get_buffer_parameter(
    handle: epos_handle,
    motor_id: MotorId,
) -> Tuple[int, int, int]:
    underflow_limit = epos_uint16()
    overflow_limit = epos_uint16()
    max_buffer_size = epos_uint32()
    error_code = epos_uint32()
    status = epos.VCS_GetIpmBufferParameter(
        handle,
        motor_id.value,
        ctypes.byref(underflow_limit),
        ctypes.byref(overflow_limit),
        ctypes.byref(max_buffer_size),
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get interpolated position buffer parameters of {motor_id}: ",
            hex(error_code.value),
        )
    return underflow_limit.value, overflow_limit.value, max_buffer_size.value


def ipm_clear_buffer(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_ClearIpmBuffer(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to clear interpolated position buffer of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ipm_get_free_buffer_size(
    handle: epos_handle,
    motor_id: MotorId,
) -> int:
    error_code = epos_uint32()
    free_size = epos_uint32()
    status = epos.VCS_GetFreeIpmBufferSize(
        handle, motor_id.value, ctypes.byref(free_size), ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get free interpolated position buffer size of {motor_id}: ",
            hex(error_code.value),
        )
    return free_size.value


def ipm_add_pvt_value(
    handle: epos_handle,
    motor_id: MotorId,
    position: int,
    velocity: int,
    ref_time: int,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_AddPvtValueToIpmBuffer(
        handle, motor_id.value, position, velocity, ref_time, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to add PVT value to interpolated position buffer of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ipm_start_trajectory(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_StartIpmTrajectory(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to start interpolated position trajectory of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ipm_stop_trajectory(
    handle: epos_handle,
    motor_id: MotorId,
) -> bool:
    error_code = epos_uint32()
    status = epos.VCS_StopIpmTrajectory(
        handle, motor_id.value, ctypes.byref(error_code)
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to stop interpolated position trajectory of {motor_id}: ",
            hex(error_code.value),
        )
    return True


def ipm_get_status(
    handle: epos_handle,
    motor_id: MotorId,
) -> Tuple[bool, bool, bool, bool, bool, bool, bool, bool, bool]:
    error_code = epos_uint32()
    is_trajectory_running = epos_bool()
    is_underflow_warning = epos_bool()
    is_overflow_warning = epos_bool()
    is_velocity_warning = epos_bool()
    is_acceleration_warning = epos_bool()
    is_underflow_error = epos_bool()
    is_overflow_error = epos_bool()
    is_velocity_error = epos_bool()
    is_acceleration_error = epos_bool()
    status = epos.VCS_GetIpmStatus(
        handle,
        motor_id.value,
        ctypes.byref(is_trajectory_running),
        ctypes.byref(is_underflow_warning),
        ctypes.byref(is_overflow_warning),
        ctypes.byref(is_velocity_warning),
        ctypes.byref(is_acceleration_warning),
        ctypes.byref(is_underflow_error),
        ctypes.byref(is_overflow_error),
        ctypes.byref(is_velocity_error),
        ctypes.byref(is_acceleration_error),
        ctypes.byref(error_code),
    )
    if not status:
        raise RuntimeError(
            f"[MAXON ERR] Failed to get interpolated position trajectory status of {motor_id}: ",
            hex(error_code.value),
        )
    return (
        is_trajectory_running.value,
        is_underflow_warning.value,
        is_overflow_warning.value,
        is_velocity_warning.value,
        is_acceleration_warning.value,
        is_underflow_error.value,
        is_overflow_error.value,
        is_velocity_error.value,
        is_acceleration_error.value,
    )


# ============================================================================
# PM (Position Mode) specific commands
# ============================================================================

# ============================================================================
# CONTROL INTERFACES (CUBEMARS COMPATIBLE)
# ============================================================================
def can_set_duty(
    key_handle: int,
    motor_id: MotorId,
    duty: float,
    motor_command_queue: Queue[MotorCommand],
    is_keep_data_event: _Event,
) -> None:
    """
    Maxon EPOS does not natively support an open-loop duty cycle mode in CANopen standard.
    This safely defaults to applying a 0 torque to prevent unwanted behavior.
    """
    print("[WARNING] Duty cycle mode not supported by EPOS4. Defaulting to 0 Current.")
    can_set_torque(
        key_handle, motor_id, 0.0, None, motor_command_queue, is_keep_data_event
    )


def can_set_torque(
    key_handle: int,
    motor_id: MotorId,
    torque: float,
    motor_type: ServoMotorEnum,
    motor_command_queue: Queue[MotorCommand],
    is_keep_data_event: _Event,
) -> None:
    """Sends Servo control message for Current Mode (Torque)."""
    current_amps = torque / motor_type.value.Kt_actual / motor_type.value.gear_ratio

    if current_amps > motor_type.value.current_max:
        current_amps = motor_type.value.current_max
    elif current_amps < motor_type.value.current_min:
        current_amps = motor_type.value.current_min

    current_ma = int(current_amps * 1000.0)
    p_err = epos_uint32()

    # Enable mode & send target
    epos.VCS_ActivateCurrentMode(key_handle, motor_id.value, ctypes.byref(p_err))
    success = epos.VCS_SetCurrentMustEx(
        key_handle, motor_id.value, current_ma, ctypes.byref(p_err)
    )
    check_error(success, p_err)

    if is_keep_data_event.is_set():
        motor_command_queue.put(
            MotorCommand(
                motor_id=motor_id.value,
                timestamp=get_time(),
                command_data=current_ma.to_bytes(4, byteorder="little", signed=True),
                control_mode=ServoCanPacketEnum.CURRENT_LOOP_MODE.value,
                log_data=b"",
            )
        )


def can_set_break(
    key_handle: int,
    motor_id: MotorId,
    torque: float,
    motor_type: ServoMotorEnum,
    motor_command_queue: Queue[MotorCommand],
    is_keep_data_event: _Event,
) -> None:
    """Sets the EPOS drive into a Quick Stop state to resist movement."""
    p_err = epos_uint32()
    success = epos.VCS_SetQuickStopState(
        key_handle, motor_id.value, ctypes.byref(p_err)
    )
    check_error(success, p_err)

    if is_keep_data_event.is_set():
        motor_command_queue.put(
            MotorCommand(
                motor_id=motor_id.value,
                timestamp=get_time(),
                command_data=b"\x00",
                control_mode=ServoCanPacketEnum.CURRENT_BRAKE_MODE.value,
                log_data=b"",
            )
        )


def can_set_speed(
    key_handle: int,
    motor_id: MotorId,
    speed: float,
    motor_type: ServoMotorEnum,
    motor_command_queue: Queue[MotorCommand],
    is_keep_data_event: _Event,
) -> None:
    """Sends Servo control message for Profile Velocity Mode."""
    # Convert incoming speed to RPM based on motor definitions
    rpm = int(speed * motor_type.value.gear_ratio * 9.549296596425384)
    p_err = epos_uint32()

    epos.VCS_ActivateProfileVelocityMode(
        key_handle, motor_id.value, ctypes.byref(p_err)
    )
    success = epos.VCS_MoveWithVelocity(
        key_handle, motor_id.value, rpm, ctypes.byref(p_err)
    )
    check_error(success, p_err)

    if is_keep_data_event.is_set():
        motor_command_queue.put(
            MotorCommand(
                motor_id=motor_id.value,
                timestamp=get_time(),
                command_data=rpm.to_bytes(4, byteorder="little", signed=True),
                control_mode=ServoCanPacketEnum.VELOCITY_MODE.value,
                log_data=b"",
            )
        )


def can_set_position(
    key_handle: int,
    motor_id: MotorId,
    position: float,
    motor_type: ServoMotorEnum,
    motor_command_queue: Queue[MotorCommand],
    is_keep_data_event: _Event,
) -> None:
    """Sends Servo control message for Profile Position Mode."""
    # Conversion from requested positional rads to QC (quadrature counts)
    qc_pos = int(
        (position / (2 * 3.14159265359))
        * motor_type.value.encoder_resolution
        * motor_type.value.gear_ratio
    )
    p_err = epos_uint32()

    epos.VCS_ActivateProfilePositionMode(
        key_handle, motor_id.value, ctypes.byref(p_err)
    )
    # True = Absolute positioning, True = Start immediately
    success = epos.VCS_MoveToPosition(
        key_handle, motor_id.value, qc_pos, True, True, ctypes.byref(p_err)
    )
    check_error(success, p_err)

    if is_keep_data_event.is_set():
        motor_command_queue.put(
            MotorCommand(
                motor_id=motor_id.value,
                timestamp=get_time(),
                command_data=qc_pos.to_bytes(4, byteorder="little", signed=True),
                control_mode=ServoCanPacketEnum.POSITION_MODE.value,
                log_data=b"",
            )
        )
