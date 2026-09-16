"""
Filename: hermes/aidwear/prosthesis/motor_control/epos_commands.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-09-11
Version: 1.0
Description: Python ctypes wrapper and communication primitives for Maxon EPOS motor controllers.
    Implements complete communication primitives, configuration, operation, and low-layer CANopen
    interfaces extracted from the official Maxon EPOS Command Library Documentation.

    Command Library Manual: https://www.maxongroup.com/medias/sys_master/root/9157360353310/EPOS-Command-Library-En.pdf
    Firmware Specification: https://www.maxongroup.com/medias/sys_master/root/9444047912990/EPOS4-Firmware-Specification-En.pdf
    EPOS4 Communication Guide: https://www.maxongroup.co.kr/medias/sys_master/8822983458846.pdf
    Staging Knowledge Reference: docs/epos_command_library.md
"""

from __future__ import annotations

import ctypes
import ctypes.util
from typing import Tuple, TypeAlias, Union

from ..utils.types import (
    EposDevice,
    EposOperationMode,
    EposProtocolStack,
    EposState,
    HomingMethod,
    MotorId,
)

# ============================================================================
# EPOS COMMAND LIBRARY DATA TYPE DEFINITIONS (Section 2.5 of Manual)
# ============================================================================
# Mapping C types to Python ctypes
epos_char_p: TypeAlias = ctypes.c_char_p       # char* (null-terminated string)
epos_int8: TypeAlias = ctypes.c_int8           # char, __int8 (8-bit signed integer)
epos_uint8: TypeAlias = ctypes.c_uint8         # BYTE (8-bit unsigned integer)
epos_int16: TypeAlias = ctypes.c_int16         # short (16-bit signed integer)
epos_uint16: TypeAlias = ctypes.c_uint16       # WORD (16-bit unsigned integer)
epos_int32: TypeAlias = ctypes.c_int32         # long, int (32-bit signed integer)
epos_uint32: TypeAlias = ctypes.c_uint32       # DWORD (32-bit unsigned integer)
epos_uint64: TypeAlias = ctypes.c_uint64       # DWORD64 (64-bit unsigned integer)
epos_bool: TypeAlias = ctypes.c_int32          # BOOL (32-bit signed integer: 1=TRUE, 0=FALSE)
epos_handle: TypeAlias = ctypes.c_void_p       # HANDLE (Object/device pointer, 32 or 64-bit)

# ============================================================================
# SHARED LIBRARY LOADER WITH LOCAL DEVELOPMENT PROXY
# ============================================================================
_lib_name = ctypes.util.find_library("EposCmd") or ctypes.util.find_library("EposCmd64")
epos = None
if _lib_name:
    try:
        epos = ctypes.CDLL(_lib_name)
    except OSError:
        pass

if not epos:
    for _candidate in ("EposCmd64.dll", "EposCmd.dll", "libEposCmd.so"):
        try:
            epos = ctypes.CDLL(_candidate)
            break
        except OSError:
            pass


class _EposCmdProxy:
    """Proxy object allowing inspection and signature binding when EposCmd DLL is not present locally."""

    def __init__(self):
        self._funcs = {}

    def __getattr__(self, name: str):
        if name not in self._funcs:

            class _FuncHolder:
                def __init__(self, func_name: str):
                    self.func_name = func_name
                    self.argtypes = []
                    self.restype = None

                def __call__(self, *args, **kwargs):
                    raise OSError(
                        f"Cannot execute {self.func_name}: libEposCmd.so / EposCmd.dll not found. "
                        f"Please install the Maxon EPOS Command Library."
                    )

            self._funcs[name] = _FuncHolder(name)
        return self._funcs[name]


if epos is None:
    epos = _EposCmdProxy()

# ============================================================================
# CTYPES BINDINGS FOR OFFICIAL EPOS COMMAND LIBRARY (201 FUNCTIONS)
# ============================================================================

# ----------------------------------------------------------------------------
# Chapter 3.1: Communication Initialization & Port Management
# ----------------------------------------------------------------------------
epos.VCS_OpenDevice.restype = epos_handle
epos.VCS_OpenDevice.argtypes = [
    epos_char_p,
    epos_char_p,
    epos_char_p,
    epos_char_p,
    ctypes.POINTER(epos_uint32),
]
# epos.VCS_OpenDeviceDlg.restype = epos_handle
# epos.VCS_OpenDeviceDlg.argtypes = [ctypes.POINTER(epos_uint32)]
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
# epos.VCS_FindDeviceCommunicationSettings.restype = epos_bool
# epos.VCS_FindDeviceCommunicationSettings.argtypes = [ctypes.POINTER(epos_handle), epos_char_p, epos_char_p, epos_char_p, epos_char_p, epos_uint16, ctypes.POINTER(epos_uint32), ctypes.POINTER(epos_uint32), ctypes.POINTER(epos_uint16), epos_int32, ctypes.POINTER(epos_uint32)]
epos.VCS_CloseAllDevices.restype = epos_bool
epos.VCS_CloseAllDevices.argtypes = [ctypes.POINTER(epos_uint32)]
epos.VCS_CloseDevice.restype = epos_bool
epos.VCS_CloseDevice.argtypes = [epos_handle, ctypes.POINTER(epos_uint32)]
epos.VCS_OpenSubDevice.restype = epos_handle
epos.VCS_OpenSubDevice.argtypes = [
    epos_handle,
    epos_char_p,
    epos_char_p,
    ctypes.POINTER(epos_uint32),
]
# epos.VCS_OpenSubDeviceDlg.restype = epos_handle
# epos.VCS_OpenSubDeviceDlg.argtypes = [epos_handle, ctypes.POINTER(epos_uint32)]
epos.VCS_SetGatewaySettings.restype = epos_bool
epos.VCS_SetGatewaySettings.argtypes = [
    epos_handle,
    epos_uint32,
    ctypes.POINTER(epos_uint16),
]
epos.VCS_GetGatewaySettings.restype = epos_bool
epos.VCS_GetGatewaySettings.argtypes = [
    epos_handle,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
# epos.VCS_FindSubDeviceCommunicationSettings.restype = epos_bool
# epos.VCS_FindSubDeviceCommunicationSettings.argtypes = [epos_handle, ctypes.POINTER(epos_handle), epos_char_p, epos_char_p, epos_uint16, ctypes.POINTER(epos_uint32), ctypes.POINTER(epos_uint16), epos_int32, ctypes.POINTER(epos_uint32)]
epos.VCS_CloseAllSubDevices.restype = epos_bool
epos.VCS_CloseAllSubDevices.argtypes = [epos_handle, ctypes.POINTER(epos_uint32)]
epos.VCS_CloseSubDevice.restype = epos_bool
epos.VCS_CloseSubDevice.argtypes = [epos_handle, ctypes.POINTER(epos_uint32)]

# ----------------------------------------------------------------------------
# Chapter 3.2: Library & Device Info
# ----------------------------------------------------------------------------
epos.VCS_GetErrorInfo.restype = epos_bool
epos.VCS_GetErrorInfo.argtypes = [epos_uint32, epos_char_p, epos_uint16]
epos.VCS_GetDriverInfo.restype = epos_bool
epos.VCS_GetDriverInfo.argtypes = [
    epos_char_p,
    epos_uint16,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetVersion.restype = epos_bool
epos.VCS_GetVersion.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 3.3: Advanced Device & Interface Selection
# ----------------------------------------------------------------------------
epos.VCS_GetDeviceNameSelection.restype = epos_bool
epos.VCS_GetDeviceNameSelection.argtypes = [
    epos_bool,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetProtocolStackNameSelection.restype = epos_bool
epos.VCS_GetProtocolStackNameSelection.argtypes = [
    epos_char_p,
    epos_bool,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetInterfaceNameSelection.restype = epos_bool
epos.VCS_GetInterfaceNameSelection.argtypes = [
    epos_char_p,
    epos_char_p,
    epos_bool,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetPortNameSelection.restype = epos_bool
epos.VCS_GetPortNameSelection.argtypes = [
    epos_char_p,
    epos_char_p,
    epos_char_p,
    epos_bool,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ResetPortNameSelection.restype = epos_bool
epos.VCS_ResetPortNameSelection.argtypes = [
    epos_char_p,
    epos_char_p,
    epos_char_p,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetBaudrateSelection.restype = epos_bool
epos.VCS_GetBaudrateSelection.argtypes = [
    epos_char_p,
    epos_char_p,
    epos_char_p,
    epos_char_p,
    epos_bool,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetKeyHandle.restype = epos_bool
epos.VCS_GetKeyHandle.argtypes = [
    epos_char_p,
    epos_char_p,
    epos_char_p,
    epos_char_p,
    ctypes.POINTER(epos_handle),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetDeviceName.restype = epos_bool
epos.VCS_GetDeviceName.argtypes = [
    epos_handle,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetProtocolStackName.restype = epos_bool
epos.VCS_GetProtocolStackName.argtypes = [
    epos_handle,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetInterfaceName.restype = epos_bool
epos.VCS_GetInterfaceName.argtypes = [
    epos_handle,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetPortName.restype = epos_bool
epos.VCS_GetPortName.argtypes = [
    epos_handle,
    epos_char_p,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 4.1: General Configuration & Object Dictionary Access
# ----------------------------------------------------------------------------
# epos.VCS_ImportParameter.restype = epos_bool
# epos.VCS_ImportParameter.argtypes = [epos_handle, epos_uint16, epos_char_p, epos_bool, epos_bool, ctypes.POINTER(epos_uint32)]
# epos.VCS_ExportParameter.restype = epos_bool
# epos.VCS_ExportParameter.argtypes = [epos_handle, epos_uint16, epos_char_p, epos_char_p, epos_char_p, epos_char_p, epos_bool, epos_bool, ctypes.POINTER(epos_uint32)]
epos.VCS_SetObject.restype = epos_bool
epos.VCS_SetObject.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint8,
    ctypes.c_void_p,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetObject.restype = epos_bool
epos.VCS_GetObject.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint8,
    ctypes.c_void_p,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_Restore.restype = epos_bool
epos.VCS_Restore.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
epos.VCS_Store.restype = epos_bool
epos.VCS_Store.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
# epos.VCS_UpdateFirmware.restype = epos_bool
# epos.VCS_UpdateFirmware.argtypes = [epos_handle, epos_uint16, epos_char_p, epos_bool, epos_bool, epos_bool, ctypes.POINTER(epos_uint32)]

# ----------------------------------------------------------------------------
# Chapter 4.2: Advanced Motor, Sensor, Safety & Controller Configuration
# ----------------------------------------------------------------------------
epos.VCS_SetMotorType.restype = epos_bool
epos.VCS_SetMotorType.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetDcMotorParameter.restype = epos_bool
epos.VCS_SetDcMotorParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetDcMotorParameterEx.restype = epos_bool
epos.VCS_SetDcMotorParameterEx.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetEcMotorParameter.restype = epos_bool
epos.VCS_SetEcMotorParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint8,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetEcMotorParameterEx.restype = epos_bool
epos.VCS_SetEcMotorParameterEx.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    epos_uint16,
    epos_uint8,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetMotorType.restype = epos_bool
epos.VCS_GetMotorType.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetDcMotorParameter.restype = epos_bool
epos.VCS_GetDcMotorParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetDcMotorParameterEx.restype = epos_bool
epos.VCS_GetDcMotorParameterEx.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetEcMotorParameter.restype = epos_bool
epos.VCS_GetEcMotorParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetEcMotorParameterEx.restype = epos_bool
epos.VCS_GetEcMotorParameterEx.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetSensorType.restype = epos_bool
epos.VCS_SetSensorType.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetIncEncoderParameter.restype = epos_bool
epos.VCS_SetIncEncoderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetHallSensorParameter.restype = epos_bool
epos.VCS_SetHallSensorParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetSsiAbsEncoderParameter.restype = epos_bool
epos.VCS_SetSsiAbsEncoderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetSsiAbsEncoderParameterEx.restype = epos_bool
epos.VCS_SetSsiAbsEncoderParameterEx.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_bool,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetSsiAbsEncoderParameterEx2.restype = epos_bool
epos.VCS_SetSsiAbsEncoderParameterEx2.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_bool,
    epos_uint16,
    epos_uint16,
    epos_bool,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetSensorType.restype = epos_bool
epos.VCS_GetSensorType.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetIncEncoderParameter.restype = epos_bool
epos.VCS_GetIncEncoderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetHallSensorParameter.restype = epos_bool
epos.VCS_GetHallSensorParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetSsiAbsEncoderParameter.restype = epos_bool
epos.VCS_GetSsiAbsEncoderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetSsiAbsEncoderParameterEx.restype = epos_bool
epos.VCS_GetSsiAbsEncoderParameterEx.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetSsiAbsEncoderParameterEx2.restype = epos_bool
epos.VCS_GetSsiAbsEncoderParameterEx2.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetMaxFollowingError.restype = epos_bool
epos.VCS_SetMaxFollowingError.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetMaxFollowingError.restype = epos_bool
epos.VCS_GetMaxFollowingError.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetMaxProfileVelocity.restype = epos_bool
epos.VCS_SetMaxProfileVelocity.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetMaxProfileVelocity.restype = epos_bool
epos.VCS_GetMaxProfileVelocity.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetMaxAcceleration.restype = epos_bool
epos.VCS_SetMaxAcceleration.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetMaxAcceleration.restype = epos_bool
epos.VCS_GetMaxAcceleration.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetControllerGain.restype = epos_bool
epos.VCS_SetControllerGain.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint64,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetControllerGain.restype = epos_bool
epos.VCS_GetControllerGain.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint64),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DigitalInputConfiguration.restype = epos_bool
epos.VCS_DigitalInputConfiguration.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_bool,
    epos_bool,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DigitalOutputConfiguration.restype = epos_bool
epos.VCS_DigitalOutputConfiguration.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_bool,
    epos_bool,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_AnalogInputConfiguration.restype = epos_bool
epos.VCS_AnalogInputConfiguration.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_AnalogOutputConfiguration.restype = epos_bool
epos.VCS_AnalogOutputConfiguration.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetVelocityUnits.restype = epos_bool
epos.VCS_SetVelocityUnits.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    epos_int8,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetVelocityUnits.restype = epos_bool
epos.VCS_GetVelocityUnits.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint8),
    epos_char_p,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.1: Operation Mode Selection
# ----------------------------------------------------------------------------
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

# ----------------------------------------------------------------------------
# Chapter 5.2: State Machine Control
# ----------------------------------------------------------------------------
epos.VCS_ResetDevice.restype = epos_bool
epos.VCS_ResetDevice.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
epos.VCS_SetState.restype = epos_bool
epos.VCS_SetState.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetEnableState.restype = epos_bool
epos.VCS_SetEnableState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetDisableState.restype = epos_bool
epos.VCS_SetDisableState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetQuickStopState.restype = epos_bool
epos.VCS_SetQuickStopState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ClearFault.restype = epos_bool
epos.VCS_ClearFault.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
epos.VCS_GetState.restype = epos_bool
epos.VCS_GetState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetEnableState.restype = epos_bool
epos.VCS_GetEnableState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetDisableState.restype = epos_bool
epos.VCS_GetDisableState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetQuickStopState.restype = epos_bool
epos.VCS_GetQuickStopState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetFaultState.restype = epos_bool
epos.VCS_GetFaultState.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.3: Device Error Handling
# ----------------------------------------------------------------------------
epos.VCS_GetNbOfDeviceError.restype = epos_bool
epos.VCS_GetNbOfDeviceError.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetDeviceErrorCode.restype = epos_bool
epos.VCS_GetDeviceErrorCode.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.4: Movement & Actual Value Sensor Acquisition
# ----------------------------------------------------------------------------
epos.VCS_GetMovementState.restype = epos_bool
epos.VCS_GetMovementState.argtypes = [
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
epos.VCS_GetCurrentIs.restype = epos_bool
epos.VCS_GetCurrentIs.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetCurrentIsEx.restype = epos_bool
epos.VCS_GetCurrentIsEx.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetCurrentIsAveraged.restype = epos_bool
epos.VCS_GetCurrentIsAveraged.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int16),
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

# ----------------------------------------------------------------------------
# Chapter 5.5: Profile Position Mode (PPM)
# ----------------------------------------------------------------------------
epos.VCS_ActivateProfilePositionMode.restype = epos_bool
epos.VCS_ActivateProfilePositionMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetPositionProfile.restype = epos_bool
epos.VCS_SetPositionProfile.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetPositionProfile.restype = epos_bool
epos.VCS_GetPositionProfile.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_MoveToPosition.restype = epos_bool
epos.VCS_MoveToPosition.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    epos_bool,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetTargetPosition.restype = epos_bool
epos.VCS_GetTargetPosition.argtypes = [
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
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisablePositionWindow.restype = epos_bool
epos.VCS_DisablePositionWindow.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.6: Profile Velocity Mode (PVM)
# ----------------------------------------------------------------------------
epos.VCS_ActivateProfileVelocityMode.restype = epos_bool
epos.VCS_ActivateProfileVelocityMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetVelocityProfile.restype = epos_bool
epos.VCS_SetVelocityProfile.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetVelocityProfile.restype = epos_bool
epos.VCS_GetVelocityProfile.argtypes = [
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
epos.VCS_GetTargetVelocity.restype = epos_bool
epos.VCS_GetTargetVelocity.argtypes = [
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
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisableVelocityWindow.restype = epos_bool
epos.VCS_DisableVelocityWindow.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.7: Homing Mode (HM)
# ----------------------------------------------------------------------------
epos.VCS_ActivateHomingMode.restype = epos_bool
epos.VCS_ActivateHomingMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetHomingParameter.restype = epos_bool
epos.VCS_SetHomingParameter.argtypes = [
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
epos.VCS_GetHomingParameter.restype = epos_bool
epos.VCS_GetHomingParameter.argtypes = [
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
epos.VCS_StopHoming.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
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
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_WaitForHomingAttained.restype = epos_bool
epos.VCS_WaitForHomingAttained.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.8: Interpolated Position Mode (IPM)
# ----------------------------------------------------------------------------
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

# ----------------------------------------------------------------------------
# Chapter 5.9: Position Mode (PM)
# ----------------------------------------------------------------------------
epos.VCS_ActivatePositionMode.restype = epos_bool
epos.VCS_ActivatePositionMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetPositionMust.restype = epos_bool
epos.VCS_SetPositionMust.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetPositionMust.restype = epos_bool
epos.VCS_GetPositionMust.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ActivateAnalogPositionSetpoint.restype = epos_bool
epos.VCS_ActivateAnalogPositionSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.c_float,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DeactivateAnalogPositionSetpoint.restype = epos_bool
epos.VCS_DeactivateAnalogPositionSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_EnableAnalogPositionSetpoint.restype = epos_bool
epos.VCS_EnableAnalogPositionSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisableAnalogPositionSetpoint.restype = epos_bool
epos.VCS_DisableAnalogPositionSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.10: Velocity Mode (VM)
# ----------------------------------------------------------------------------
epos.VCS_ActivateVelocityMode.restype = epos_bool
epos.VCS_ActivateVelocityMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetVelocityMust.restype = epos_bool
epos.VCS_SetVelocityMust.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetVelocityMust.restype = epos_bool
epos.VCS_GetVelocityMust.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ActivateAnalogVelocitySetpoint.restype = epos_bool
epos.VCS_ActivateAnalogVelocitySetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.c_float,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DeactivateAnalogVelocitySetpoint.restype = epos_bool
epos.VCS_DeactivateAnalogVelocitySetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_EnableAnalogVelocitySetpoint.restype = epos_bool
epos.VCS_EnableAnalogVelocitySetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisableAnalogVelocitySetpoint.restype = epos_bool
epos.VCS_DisableAnalogVelocitySetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.11: Current Mode (CM)
# ----------------------------------------------------------------------------
epos.VCS_ActivateCurrentMode.restype = epos_bool
epos.VCS_ActivateCurrentMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetCurrentMust.restype = epos_bool
epos.VCS_GetCurrentMust.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetCurrentMustEx.restype = epos_bool
epos.VCS_GetCurrentMustEx.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetCurrentMust.restype = epos_bool
epos.VCS_SetCurrentMust.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetCurrentMustEx.restype = epos_bool
epos.VCS_SetCurrentMustEx.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ActivateAnalogCurrentSetpoint.restype = epos_bool
epos.VCS_ActivateAnalogCurrentSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.c_float,
    epos_int16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DeactivateAnalogCurrentSetpoint.restype = epos_bool
epos.VCS_DeactivateAnalogCurrentSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_EnableAnalogCurrentSetpoint.restype = epos_bool
epos.VCS_EnableAnalogCurrentSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisableAnalogCurrentSetpoint.restype = epos_bool
epos.VCS_DisableAnalogCurrentSetpoint.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.12: Master Encoder Mode (MEM)
# ----------------------------------------------------------------------------
epos.VCS_ActivateMasterEncoderMode.restype = epos_bool
epos.VCS_ActivateMasterEncoderMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetMasterEncoderParameter.restype = epos_bool
epos.VCS_SetMasterEncoderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint8,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetMasterEncoderParameter.restype = epos_bool
epos.VCS_GetMasterEncoderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.13: Step Direction Mode (SDM)
# ----------------------------------------------------------------------------
epos.VCS_ActivateStepDirectionMode.restype = epos_bool
epos.VCS_ActivateStepDirectionMode.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetStepDirectionParameter.restype = epos_bool
epos.VCS_SetStepDirectionParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    epos_uint8,
    epos_uint32,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetStepDirectionParameter.restype = epos_bool
epos.VCS_GetStepDirectionParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 5.14: Digital & Analog Inputs/Outputs & Position Compare/Marker
# ----------------------------------------------------------------------------
epos.VCS_GetAllDigitalInputs.restype = epos_bool
epos.VCS_GetAllDigitalInputs.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetAllDigitalOutputs.restype = epos_bool
epos.VCS_GetAllDigitalOutputs.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetAllDigitalOutputs.restype = epos_bool
epos.VCS_SetAllDigitalOutputs.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetAnalogInput.restype = epos_bool
epos.VCS_GetAnalogInput.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetAnalogInputVoltage.restype = epos_bool
epos.VCS_GetAnalogInputVoltage.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetAnalogInputState.restype = epos_bool
epos.VCS_GetAnalogInputState.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetAnalogOutput.restype = epos_bool
epos.VCS_SetAnalogOutput.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetAnalogOutputVoltage.restype = epos_bool
epos.VCS_SetAnalogOutputVoltage.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetAnalogOutputState.restype = epos_bool
epos.VCS_SetAnalogOutputState.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetPositionCompareParameter.restype = epos_bool
epos.VCS_SetPositionCompareParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    epos_uint8,
    epos_uint8,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetPositionCompareParameter.restype = epos_bool
epos.VCS_GetPositionCompareParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ActivatePositionCompare.restype = epos_bool
epos.VCS_ActivatePositionCompare.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DeactivatePositionCompare.restype = epos_bool
epos.VCS_DeactivatePositionCompare.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_EnablePositionCompare.restype = epos_bool
epos.VCS_EnablePositionCompare.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisablePositionCompare.restype = epos_bool
epos.VCS_DisablePositionCompare.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetPositionCompareReferencePosition.restype = epos_bool
epos.VCS_SetPositionCompareReferencePosition.argtypes = [
    epos_handle,
    epos_uint16,
    epos_int32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SetPositionMarkerParameter.restype = epos_bool
epos.VCS_SetPositionMarkerParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    epos_uint8,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetPositionMarkerParameter.restype = epos_bool
epos.VCS_GetPositionMarkerParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint8),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ActivatePositionMarker.restype = epos_bool
epos.VCS_ActivatePositionMarker.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_bool,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DeactivatePositionMarker.restype = epos_bool
epos.VCS_DeactivatePositionMarker.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ReadPositionMarkerCounter.restype = epos_bool
epos.VCS_ReadPositionMarkerCounter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ReadPositionMarkerCapturedPosition.restype = epos_bool
epos.VCS_ReadPositionMarkerCapturedPosition.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_int32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ResetPositionMarkerCounter.restype = epos_bool
epos.VCS_ResetPositionMarkerCounter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 6.1: Recorder Configuration & Trigger Setup
# ----------------------------------------------------------------------------
epos.VCS_SetRecorderParameter.restype = epos_bool
epos.VCS_SetRecorderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_GetRecorderParameter.restype = epos_bool
epos.VCS_GetRecorderParameter.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_EnableTrigger.restype = epos_bool
epos.VCS_EnableTrigger.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DisableAllTriggers.restype = epos_bool
epos.VCS_DisableAllTriggers.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ActivateChannel.restype = epos_bool
epos.VCS_ActivateChannel.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    epos_uint16,
    epos_uint8,
    epos_uint8,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_DeactivateAllChannels.restype = epos_bool
epos.VCS_DeactivateAllChannels.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 6.2: Recorder Execution & Status
# ----------------------------------------------------------------------------
epos.VCS_StartRecorder.restype = epos_bool
epos.VCS_StartRecorder.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_StopRecorder.restype = epos_bool
epos.VCS_StopRecorder.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
epos.VCS_ForceTrigger.restype = epos_bool
epos.VCS_ForceTrigger.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
epos.VCS_IsRecorderRunning.restype = epos_bool
epos.VCS_IsRecorderRunning.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_IsRecorderTriggered.restype = epos_bool
epos.VCS_IsRecorderTriggered.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_bool),
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 6.3: Data Extraction & File Export
# ----------------------------------------------------------------------------
epos.VCS_ReadChannelVectorSize.restype = epos_bool
epos.VCS_ReadChannelVectorSize.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ReadChannelDataVector.restype = epos_bool
epos.VCS_ReadChannelDataVector.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    ctypes.POINTER(epos_uint8),
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
# epos.VCS_ShowChannelDataDlg.restype = epos_bool
# epos.VCS_ShowChannelDataDlg.argtypes = [epos_handle, epos_uint16, ctypes.POINTER(epos_uint32)]
# epos.VCS_ExportChannelDataToFile.restype = epos_bool
# epos.VCS_ExportChannelDataToFile.argtypes = [epos_handle, epos_uint16, epos_char_p, ctypes.POINTER(epos_uint32)]

# ----------------------------------------------------------------------------
# Chapter 6.4: Advanced Buffer Functions
# ----------------------------------------------------------------------------
epos.VCS_ReadDataBuffer.restype = epos_bool
epos.VCS_ReadDataBuffer.argtypes = [
    epos_handle,
    epos_uint16,
    ctypes.POINTER(epos_uint8),
    epos_uint32,
    ctypes.POINTER(epos_uint32),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint16),
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ExtractChannelDataVector.restype = epos_bool
epos.VCS_ExtractChannelDataVector.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint8,
    ctypes.POINTER(epos_uint8),
    epos_uint32,
    ctypes.POINTER(epos_uint8),
    epos_uint32,
    epos_uint16,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]

# ----------------------------------------------------------------------------
# Chapter 7.1: Raw CAN Framing & NMT Services
# ----------------------------------------------------------------------------
epos.VCS_SendCANFrame.restype = epos_bool
epos.VCS_SendCANFrame.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.c_void_p,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_ReadCANFrame.restype = epos_bool
epos.VCS_ReadCANFrame.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.c_void_p,
    epos_uint32,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_RequestCANFrame.restype = epos_bool
epos.VCS_RequestCANFrame.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.c_void_p,
    ctypes.POINTER(epos_uint32),
]
epos.VCS_SendNMTService.restype = epos_bool
epos.VCS_SendNMTService.argtypes = [
    epos_handle,
    epos_uint16,
    epos_uint16,
    ctypes.POINTER(epos_uint32),
]


# HIGH-LEVEL PYTHON WRAPPERS & DIAGNOSTIC UTILITIES
# ============================================================================


def _to_bytes(val: Union[str, bytes]) -> bytes:
    if isinstance(val, str):
        return val.encode("utf-8")
    return val


def _node(motor_id: Union[MotorId, int]) -> int:
    return motor_id.value if isinstance(motor_id, MotorId) else motor_id


def get_error_info(error_code: Union[int, epos_uint32]) -> str:
    """Returns a human-readable error description for a given EPOS error code."""
    err_val = error_code.value if hasattr(error_code, "value") else int(error_code)
    buf = ctypes.create_string_buffer(1024)
    epos.VCS_GetErrorInfo(err_val, buf, 1024)
    return buf.value.decode("utf-8", errors="ignore")


def check_error(
    success: Union[bool, int], error_code: epos_uint32, context_msg: str = ""
) -> None:
    """Parses error code and raises a RuntimeError if the EPOS command failed."""
    if not success and error_code.value != 0:
        err_msg = get_error_info(error_code)
        prefix = f"[{context_msg}] " if context_msg else ""
        raise RuntimeError(f"{prefix}[MAXON ERROR 0x{error_code.value:08X}]: {err_msg}")


# ----------------------------------------------------------------------------
# Communication Initialization & Port Management (Chapter 3.1)
# ----------------------------------------------------------------------------


def open_device(
    device: Union[EposDevice, str, bytes],
    protocol: Union[EposProtocolStack, str, bytes],
    interface: Union[str, bytes],
    port: Union[str, bytes],
) -> epos_handle:
    """Opens the communication port to send and receive commands to/from EPOS."""
    d_name = device.value if isinstance(device, EposDevice) else _to_bytes(device)
    p_name = (
        protocol.value
        if isinstance(protocol, EposProtocolStack)
        else _to_bytes(protocol)
    )
    i_name = _to_bytes(interface)
    port = _to_bytes(port)

    err = epos_uint32()
    handle = epos.VCS_OpenDevice(d_name, p_name, i_name, port, ctypes.byref(err))
    if not handle:
        raise RuntimeError(
            f"[MAXON ERR] Failed to open device {d_name!r}: 0x{err.value:08X} - {get_error_info(err)}"
        )
    return handle


def close_device(handle: epos_handle) -> bool:
    """Closes the communication port of the given device handle."""
    err = epos_uint32()
    status = epos.VCS_CloseDevice(handle, ctypes.byref(err))
    check_error(status, err, "VCS_CloseDevice")
    return bool(status)


def close_all_devices() -> bool:
    """Closes all opened communication ports."""
    err = epos_uint32()
    status = epos.VCS_CloseAllDevices(ctypes.byref(err))
    check_error(status, err, "VCS_CloseAllDevices")
    return bool(status)


def set_protocol_stack_settings(
    handle: epos_handle, baudrate: int, timeout_ms: int
) -> bool:
    """Sets the communication baud rate and timeout."""
    err = epos_uint32()
    status = epos.VCS_SetProtocolStackSettings(
        handle, baudrate, timeout_ms, ctypes.byref(err)
    )
    check_error(status, err, "VCS_SetProtocolStackSettings")
    return bool(status)


def get_protocol_stack_settings(handle: epos_handle) -> Tuple[int, int]:
    """Returns the current baud rate and timeout (baudrate, timeout_ms)."""
    baudrate = epos_uint32()
    timeout = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_GetProtocolStackSettings(
        handle, ctypes.byref(baudrate), ctypes.byref(timeout), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetProtocolStackSettings")
    return baudrate.value, timeout.value


def open_sub_device(
    device_handle: epos_handle,
    device_name: Union[EposDevice, str, bytes],
    protocol_stack_name: Union[EposProtocolStack, str, bytes],
) -> epos_handle:
    """Opens a subdevice behind a gateway device (e.g., CANopen sub-drive connected to EPOS USB gateway)."""
    d_name = (
        device_name.value
        if isinstance(device_name, EposDevice)
        else _to_bytes(device_name)
    )
    p_name = (
        protocol_stack_name.value
        if isinstance(protocol_stack_name, EposProtocolStack)
        else _to_bytes(protocol_stack_name)
    )
    err = epos_uint32()
    sub_handle = epos.VCS_OpenSubDevice(
        device_handle, d_name, p_name, ctypes.byref(err)
    )
    if not sub_handle:
        raise RuntimeError(
            f"[MAXON ERR] Failed to open sub-device {d_name!r}: 0x{err.value:08X} - {get_error_info(err)}"
        )
    return sub_handle


def close_sub_device(sub_device_handle: epos_handle) -> bool:
    """Closes an opened subdevice communication channel."""
    err = epos_uint32()
    status = epos.VCS_CloseSubDevice(sub_device_handle, ctypes.byref(err))
    check_error(status, err, "VCS_CloseSubDevice")
    return bool(status)


def close_all_sub_devices(device_handle: epos_handle) -> bool:
    """Closes all subdevices opened under the specified gateway device."""
    err = epos_uint32()
    status = epos.VCS_CloseAllSubDevices(device_handle, ctypes.byref(err))
    check_error(status, err, "VCS_CloseAllSubDevices")
    return bool(status)


def set_gateway_settings(device_handle: epos_handle, baudrate: int) -> bool:
    """Sets the gateway communication baud rate."""
    err = epos_uint32()
    status = epos.VCS_SetGatewaySettings(device_handle, baudrate, ctypes.byref(err))
    check_error(status, err, "VCS_SetGatewaySettings")
    return bool(status)


def get_gateway_settings(device_handle: epos_handle) -> int:
    """Gets the gateway communication baud rate."""
    baudrate = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_GetGatewaySettings(
        device_handle, ctypes.byref(baudrate), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetGatewaySettings")
    return baudrate.value


# ----------------------------------------------------------------------------
# Library & Device Info (Chapter 3.2)
# ----------------------------------------------------------------------------


def get_driver_info() -> Tuple[str, str]:
    """Returns the library name and library version of the EPOS Command Library."""
    lib_name = ctypes.create_string_buffer(256)
    lib_version = ctypes.create_string_buffer(256)
    err = epos_uint32()
    status = epos.VCS_GetDriverInfo(lib_name, 256, lib_version, 256, ctypes.byref(err))
    check_error(status, err, "VCS_GetDriverInfo")
    return lib_name.value.decode("utf-8", errors="ignore"), lib_version.value.decode(
        "utf-8", errors="ignore"
    )


def get_version(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> Tuple[int, int, int, int]:
    """Returns (hardware_version, software_version, application_number, application_version)."""
    hw = epos_uint16()
    sw = epos_uint16()
    app_num = epos_uint16()
    app_ver = epos_uint16()
    err = epos_uint32()
    status = epos.VCS_GetVersion(
        handle,
        _node(motor_id),
        ctypes.byref(hw),
        ctypes.byref(sw),
        ctypes.byref(app_num),
        ctypes.byref(app_ver),
        ctypes.byref(err),
    )
    check_error(status, err, "VCS_GetVersion")
    return hw.value, sw.value, app_num.value, app_ver.value


# ----------------------------------------------------------------------------
# Object Dictionary Access (Chapter 4.1)
# ----------------------------------------------------------------------------


def get_object(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    index: int,
    sub_index: int,
    max_bytes: int = 64,
) -> bytes:
    """Reads an object entry directly from the CANopen object dictionary."""
    buf = (ctypes.c_uint8 * max_bytes)()
    bytes_read = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_GetObject(
        handle,
        _node(motor_id),
        index,
        sub_index,
        buf,
        max_bytes,
        ctypes.byref(bytes_read),
        ctypes.byref(err),
    )
    check_error(status, err, f"VCS_GetObject [0x{index:04X}:{sub_index:02X}]")
    return bytes(buf[: bytes_read.value])


def set_object(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    index: int,
    sub_index: int,
    data: bytes,
) -> bool:
    """Writes an object entry directly to the CANopen object dictionary."""
    buf = (ctypes.c_uint8 * len(data))(*data)
    bytes_written = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_SetObject(
        handle,
        _node(motor_id),
        index,
        sub_index,
        buf,
        len(data),
        ctypes.byref(bytes_written),
        ctypes.byref(err),
    )
    check_error(status, err, f"VCS_SetObject [0x{index:04X}:{sub_index:02X}]")
    return bool(status)


def store(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Stores all configurable parameters into non-volatile EEPROM memory."""
    err = epos_uint32()
    status = epos.VCS_Store(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_Store")
    return bool(status)


def restore(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Restores all parameters to factory default values from non-volatile memory."""
    err = epos_uint32()
    status = epos.VCS_Restore(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_Restore")
    return bool(status)


# ----------------------------------------------------------------------------
# Operation Mode (Chapter 5.1)
# ----------------------------------------------------------------------------


def set_operation_mode(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    mode: Union[EposOperationMode, int],
) -> bool:
    """Configures the drive operational mode (PPM, PVM, CM, HM, IPM, etc.)."""
    mode_val = mode.value if isinstance(mode, EposOperationMode) else int(mode)
    err = epos_uint32()
    status = epos.VCS_SetOperationMode(
        handle, _node(motor_id), mode_val, ctypes.byref(err)
    )
    check_error(status, err, "VCS_SetOperationMode")
    return bool(status)


def get_operation_mode(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> EposOperationMode:
    """Returns the active operational mode."""
    mode_val = epos_int8()
    err = epos_uint32()
    status = epos.VCS_GetOperationMode(
        handle, _node(motor_id), ctypes.byref(mode_val), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetOperationMode")
    return EposOperationMode(mode_val.value)


# ----------------------------------------------------------------------------
# State Machine (Chapter 5.2)
# ----------------------------------------------------------------------------


def reset_device(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Resets the EPOS controller."""
    err = epos_uint32()
    status = epos.VCS_ResetDevice(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_ResetDevice")
    return bool(status)


def set_state(
    handle: epos_handle, motor_id: Union[MotorId, int], state: Union[EposState, int]
) -> bool:
    """Transitions the EPOS state machine to the given state."""
    state_val = state.value if isinstance(state, EposState) else int(state)
    err = epos_uint32()
    status = epos.VCS_SetState(handle, _node(motor_id), state_val, ctypes.byref(err))
    check_error(status, err, f"VCS_SetState({state_val})")
    return bool(status)


def set_enable_state(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Enables power stage and closes control loops (switches to 'Operation Enable')."""
    err = epos_uint32()
    status = epos.VCS_SetEnableState(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_SetEnableState")
    return bool(status)


def set_disable_state(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Disables the motor power stage (switches to 'Switch On Disabled')."""
    err = epos_uint32()
    status = epos.VCS_SetDisableState(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_SetDisableState")
    return bool(status)


def set_quick_stop_state(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Executes a quick stop deceleration."""
    err = epos_uint32()
    status = epos.VCS_SetQuickStopState(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_SetQuickStopState")
    return bool(status)


def clear_fault(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Clears active fault condition and transitions drive from Fault to Switch On Disabled."""
    err = epos_uint32()
    status = epos.VCS_ClearFault(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_ClearFault")
    return bool(status)


def get_state(handle: epos_handle, motor_id: Union[MotorId, int]) -> EposState:
    """Returns the current state machine state."""
    state = epos_uint16()
    err = epos_uint32()
    status = epos.VCS_GetState(
        handle, _node(motor_id), ctypes.byref(state), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetState")
    return EposState(state.value)


def is_fault(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Returns True if the drive is in a fault state."""
    fault = epos_bool()
    err = epos_uint32()
    status = epos.VCS_GetFaultState(
        handle, _node(motor_id), ctypes.byref(fault), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetFaultState")
    return bool(fault.value)


def is_target_reached(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Returns True if target position or velocity has been reached."""
    reached = epos_bool()
    err = epos_uint32()
    status = epos.VCS_GetMovementState(
        handle, _node(motor_id), ctypes.byref(reached), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetMovementState")
    return bool(reached.value)


# ----------------------------------------------------------------------------
# Sensor Readouts & Movement Getters (Chapter 5.4)
# ----------------------------------------------------------------------------


def get_position(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Returns current actual position in position units (QC / counts)."""
    pos = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetPositionIs(
        handle, _node(motor_id), ctypes.byref(pos), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetPositionIs")
    return pos.value


def get_velocity(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Returns current actual velocity in velocity units (rpm)."""
    vel = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetVelocityIs(
        handle, _node(motor_id), ctypes.byref(vel), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetVelocityIs")
    return vel.value


def get_velocity_avg(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Returns averaged actual velocity in velocity units (rpm)."""
    vel = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetVelocityIsAveraged(
        handle, _node(motor_id), ctypes.byref(vel), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetVelocityIsAveraged")
    return vel.value


def get_current(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Returns current actual value in milliamperes (mA)."""
    curr = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetCurrentIsEx(
        handle, _node(motor_id), ctypes.byref(curr), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetCurrentIsEx")
    return curr.value


def get_current_avg(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Returns averaged actual current in milliamperes (mA)."""
    curr = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetCurrentIsAveragedEx(
        handle, _node(motor_id), ctypes.byref(curr), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetCurrentIsAveragedEx")
    return curr.value


def wait_target_reached(
    handle: epos_handle, motor_id: Union[MotorId, int], timeout_ms: int
) -> bool:
    """Blocks until target reached bit is set or timeout occurs."""
    err = epos_uint32()
    status = epos.VCS_WaitForTargetReached(
        handle, _node(motor_id), timeout_ms, ctypes.byref(err)
    )
    check_error(status, err, "VCS_WaitForTargetReached")
    return bool(status)


# ----------------------------------------------------------------------------
# Profile Position Mode (PPM) (Chapter 5.5)
# ----------------------------------------------------------------------------


def activate_profile_position_mode(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> bool:
    """Activates Profile Position Mode."""
    err = epos_uint32()
    status = epos.VCS_ActivateProfilePositionMode(
        handle, _node(motor_id), ctypes.byref(err)
    )
    check_error(status, err, "VCS_ActivateProfilePositionMode")
    return bool(status)


def ppm_set_position_profile(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    velocity: int,
    acceleration: int,
    deceleration: int,
) -> bool:
    """Sets trajectory profile velocity, acceleration, and deceleration for PPM."""
    err = epos_uint32()
    status = epos.VCS_SetPositionProfile(
        handle, _node(motor_id), velocity, acceleration, deceleration, ctypes.byref(err)
    )
    check_error(status, err, "VCS_SetPositionProfile")
    return bool(status)


def ppm_get_position_profile(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> Tuple[int, int, int]:
    """Returns (profile_velocity, profile_acceleration, profile_deceleration)."""
    vel = epos_uint32()
    acc = epos_uint32()
    dec = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_GetPositionProfile(
        handle,
        _node(motor_id),
        ctypes.byref(vel),
        ctypes.byref(acc),
        ctypes.byref(dec),
        ctypes.byref(err),
    )
    check_error(status, err, "VCS_GetPositionProfile")
    return vel.value, acc.value, dec.value


def ppm_move_to_position(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    position: int,
    is_absolute: bool = True,
    is_immediately: bool = True,
) -> bool:
    """Commands movement to a target position using profile position mode."""
    err = epos_uint32()
    status = epos.VCS_MoveToPosition(
        handle,
        _node(motor_id),
        position,
        int(is_absolute),
        int(is_immediately),
        ctypes.byref(err),
    )
    check_error(status, err, f"VCS_MoveToPosition({position})")
    return bool(status)


def ppm_get_target_position(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Reads the current target position setpoint."""
    pos = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetTargetPosition(
        handle, _node(motor_id), ctypes.byref(pos), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetTargetPosition")
    return pos.value


def ppm_halt_position_movement(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> bool:
    """Stops position movement using profile deceleration."""
    err = epos_uint32()
    status = epos.VCS_HaltPositionMovement(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_HaltPositionMovement")
    return bool(status)


def ppm_enable_position_window(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    window_size: int,
    window_time: int,
) -> bool:
    """Enables position window monitoring for target reached status."""
    err = epos_uint32()
    status = epos.VCS_EnablePositionWindow(
        handle, _node(motor_id), window_size, window_time, ctypes.byref(err)
    )
    check_error(status, err, "VCS_EnablePositionWindow")
    return bool(status)


def ppm_disable_position_window(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> bool:
    """Disables position window monitoring."""
    err = epos_uint32()
    status = epos.VCS_DisablePositionWindow(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_DisablePositionWindow")
    return bool(status)


# ----------------------------------------------------------------------------
# Profile Velocity Mode (PVM) (Chapter 5.6)
# ----------------------------------------------------------------------------


def activate_profile_velocity_mode(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> bool:
    """Activates Profile Velocity Mode."""
    err = epos_uint32()
    status = epos.VCS_ActivateProfileVelocityMode(
        handle, _node(motor_id), ctypes.byref(err)
    )
    check_error(status, err, "VCS_ActivateProfileVelocityMode")
    return bool(status)


def pvm_set_velocity_profile(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    acceleration: int,
    deceleration: int,
) -> bool:
    """Sets acceleration and deceleration slopes for velocity profile."""
    err = epos_uint32()
    status = epos.VCS_SetVelocityProfile(
        handle, _node(motor_id), acceleration, deceleration, ctypes.byref(err)
    )
    check_error(status, err, "VCS_SetVelocityProfile")
    return bool(status)


def pvm_get_velocity_profile(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> Tuple[int, int]:
    """Returns (profile_acceleration, profile_deceleration)."""
    acc = epos_uint32()
    dec = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_GetVelocityProfile(
        handle, _node(motor_id), ctypes.byref(acc), ctypes.byref(dec), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetVelocityProfile")
    return acc.value, dec.value


def pvm_move_with_velocity(
    handle: epos_handle, motor_id: Union[MotorId, int], velocity: int
) -> bool:
    """Starts movement with profile velocity to target velocity setpoint."""
    err = epos_uint32()
    status = epos.VCS_MoveWithVelocity(
        handle, _node(motor_id), velocity, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_MoveWithVelocity({velocity})")
    return bool(status)


def pvm_get_target_velocity(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Reads the target velocity setpoint."""
    vel = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetTargetVelocity(
        handle, _node(motor_id), ctypes.byref(vel), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetTargetVelocity")
    return vel.value


def pvm_halt_velocity_movement(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> bool:
    """Halts movement with profile deceleration."""
    err = epos_uint32()
    status = epos.VCS_HaltVelocityMovement(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_HaltVelocityMovement")
    return bool(status)


def pvm_enable_velocity_window(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    window_size: int,
    window_time: int,
) -> bool:
    """Enables velocity window monitoring."""
    err = epos_uint32()
    status = epos.VCS_EnableVelocityWindow(
        handle, _node(motor_id), window_size, window_time, ctypes.byref(err)
    )
    check_error(status, err, "VCS_EnableVelocityWindow")
    return bool(status)


def pvm_disable_velocity_window(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> bool:
    """Disables velocity window monitoring."""
    err = epos_uint32()
    status = epos.VCS_DisableVelocityWindow(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_DisableVelocityWindow")
    return bool(status)


# ----------------------------------------------------------------------------
# Homing Mode (HM) (Chapter 5.7)
# ----------------------------------------------------------------------------


def activate_homing_mode(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Activates Homing Mode."""
    err = epos_uint32()
    status = epos.VCS_ActivateHomingMode(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_ActivateHomingMode")
    return bool(status)


def hm_set_homing_parameter(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    acceleration: int,
    speed_switch: int,
    speed_index: int,
    offset: int,
    current_threshold: int,
    home_position: int,
) -> bool:
    """Sets homing acceleration, search speeds, home offset, current threshold, and position."""
    err = epos_uint32()
    status = epos.VCS_SetHomingParameter(
        handle,
        _node(motor_id),
        acceleration,
        speed_switch,
        speed_index,
        offset,
        current_threshold,
        home_position,
        ctypes.byref(err),
    )
    check_error(status, err, "VCS_SetHomingParameter")
    return bool(status)


def hm_get_homing_parameter(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> Tuple[int, int, int, int, int, int]:
    """Returns (acceleration, speed_switch, speed_index, offset, current_threshold, home_position)."""
    acc = epos_uint32()
    speed_sw = epos_uint32()
    speed_idx = epos_uint32()
    offset = epos_int32()
    thresh = epos_uint16()
    pos = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetHomingParameter(
        handle,
        _node(motor_id),
        ctypes.byref(acc),
        ctypes.byref(speed_sw),
        ctypes.byref(speed_idx),
        ctypes.byref(offset),
        ctypes.byref(thresh),
        ctypes.byref(pos),
        ctypes.byref(err),
    )
    check_error(status, err, "VCS_GetHomingParameter")
    return (
        acc.value,
        speed_sw.value,
        speed_idx.value,
        offset.value,
        thresh.value,
        pos.value,
    )


def hm_find_home(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    homing_method: Union[HomingMethod, int],
) -> bool:
    """Starts the homing search according to the specified homing method."""
    method_val = (
        homing_method.value
        if isinstance(homing_method, HomingMethod)
        else int(homing_method)
    )
    err = epos_uint32()
    status = epos.VCS_FindHome(handle, _node(motor_id), method_val, ctypes.byref(err))
    check_error(status, err, f"VCS_FindHome({method_val})")
    return bool(status)


def hm_stop_homing(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Stops the active homing procedure."""
    err = epos_uint32()
    status = epos.VCS_StopHoming(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_StopHoming")
    return bool(status)


def hm_define_position(
    handle: epos_handle, motor_id: Union[MotorId, int], position: int
) -> bool:
    """Sets the current mechanical position as the specified reference position value."""
    err = epos_uint32()
    status = epos.VCS_DefinePosition(
        handle, _node(motor_id), position, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_DefinePosition({position})")
    return bool(status)


def hm_get_state(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> Tuple[bool, bool]:
    """Returns (is_homing_attained, is_homing_error)."""
    attained = epos_bool()
    error = epos_bool()
    err = epos_uint32()
    status = epos.VCS_GetHomingState(
        handle,
        _node(motor_id),
        ctypes.byref(attained),
        ctypes.byref(error),
        ctypes.byref(err),
    )
    check_error(status, err, "VCS_GetHomingState")
    return bool(attained.value), bool(error.value)


def hm_wait_for_homing(
    handle: epos_handle, motor_id: Union[MotorId, int], timeout_ms: int
) -> bool:
    """Waits until homing attained bit is set or timeout expires."""
    err = epos_uint32()
    status = epos.VCS_WaitForHomingAttained(
        handle, _node(motor_id), timeout_ms, ctypes.byref(err)
    )
    check_error(status, err, "VCS_WaitForHomingAttained")
    return bool(status)


# ----------------------------------------------------------------------------
# Current Mode (CM) / Torque Control (Chapter 5.11)
# ----------------------------------------------------------------------------


def activate_current_mode(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Activates Current Mode (Torque loop)."""
    err = epos_uint32()
    status = epos.VCS_ActivateCurrentMode(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_ActivateCurrentMode")
    return bool(status)


def cm_set_current_must(
    handle: epos_handle, motor_id: Union[MotorId, int], current_ma: int
) -> bool:
    """Writes target current setpoint in milliamperes (mA)."""
    err = epos_uint32()
    status = epos.VCS_SetCurrentMustEx(
        handle, _node(motor_id), current_ma, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_SetCurrentMustEx({current_ma})")
    return bool(status)


def cm_get_current_must(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Reads target current setpoint in milliamperes (mA)."""
    curr = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetCurrentMustEx(
        handle, _node(motor_id), ctypes.byref(curr), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetCurrentMustEx")
    return curr.value


# ----------------------------------------------------------------------------
# Position Mode (PM) & Velocity Mode (VM) (Chapter 5.9, 5.10)
# ----------------------------------------------------------------------------


def activate_position_mode(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Activates direct Position Mode."""
    err = epos_uint32()
    status = epos.VCS_ActivatePositionMode(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_ActivatePositionMode")
    return bool(status)


def pm_set_position_must(
    handle: epos_handle, motor_id: Union[MotorId, int], position: int
) -> bool:
    """Writes direct position setpoint in position units."""
    err = epos_uint32()
    status = epos.VCS_SetPositionMust(
        handle, _node(motor_id), position, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_SetPositionMust({position})")
    return bool(status)


def pm_get_position_must(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Reads direct position setpoint."""
    pos = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetPositionMust(
        handle, _node(motor_id), ctypes.byref(pos), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetPositionMust")
    return pos.value


def activate_velocity_mode(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Activates direct Velocity Mode."""
    err = epos_uint32()
    status = epos.VCS_ActivateVelocityMode(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_ActivateVelocityMode")
    return bool(status)


def vm_set_velocity_must(
    handle: epos_handle, motor_id: Union[MotorId, int], velocity: int
) -> bool:
    """Writes direct velocity setpoint in velocity units."""
    err = epos_uint32()
    status = epos.VCS_SetVelocityMust(
        handle, _node(motor_id), velocity, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_SetVelocityMust({velocity})")
    return bool(status)


def vm_get_velocity_must(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Reads direct velocity setpoint."""
    vel = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetVelocityMust(
        handle, _node(motor_id), ctypes.byref(vel), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetVelocityMust")
    return vel.value


# ----------------------------------------------------------------------------
# Interpolated Position Mode (IPM) (Chapter 5.8)
# ----------------------------------------------------------------------------


def activate_interpolated_position_mode(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> bool:
    """Activates Interpolated Position Mode."""
    err = epos_uint32()
    status = epos.VCS_ActivateInterpolatedPositionMode(
        handle, _node(motor_id), ctypes.byref(err)
    )
    check_error(status, err, "VCS_ActivateInterpolatedPositionMode")
    return bool(status)


def ipm_set_buffer_parameter(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    underflow_limit: int,
    overflow_limit: int,
) -> bool:
    """Sets underflow and overflow sample warning thresholds in the IPM buffer."""
    err = epos_uint32()
    status = epos.VCS_SetIpmBufferParameter(
        handle, _node(motor_id), underflow_limit, overflow_limit, ctypes.byref(err)
    )
    check_error(status, err, "VCS_SetIpmBufferParameter")
    return bool(status)


def ipm_get_buffer_parameter(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> Tuple[int, int, int]:
    """Returns (underflow_limit, overflow_limit, max_buffer_size)."""
    underflow = epos_uint16()
    overflow = epos_uint16()
    max_size = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_GetIpmBufferParameter(
        handle,
        _node(motor_id),
        ctypes.byref(underflow),
        ctypes.byref(overflow),
        ctypes.byref(max_size),
        ctypes.byref(err),
    )
    check_error(status, err, "VCS_GetIpmBufferParameter")
    return underflow.value, overflow.value, max_size.value


def ipm_clear_buffer(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Clears the trajectory buffer of the Interpolated Position Mode."""
    err = epos_uint32()
    status = epos.VCS_ClearIpmBuffer(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_ClearIpmBuffer")
    return bool(status)


def ipm_get_free_buffer_size(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Returns the number of free PVT buffer slots available."""
    free_size = epos_uint32()
    err = epos_uint32()
    status = epos.VCS_GetFreeIpmBufferSize(
        handle, _node(motor_id), ctypes.byref(free_size), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetFreeIpmBufferSize")
    return free_size.value


def ipm_add_pvt_value(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    position: int,
    velocity: int,
    time_delta: int,
) -> bool:
    """Appends a PVT (Position, Velocity, Time interval) point to the IPM buffer."""
    err = epos_uint32()
    status = epos.VCS_AddPvtValueToIpmBuffer(
        handle, _node(motor_id), position, velocity, time_delta, ctypes.byref(err)
    )
    check_error(status, err, "VCS_AddPvtValueToIpmBuffer")
    return bool(status)


def ipm_start_trajectory(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Starts trajectory execution from the Interpolated Position buffer."""
    err = epos_uint32()
    status = epos.VCS_StartIpmTrajectory(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_StartIpmTrajectory")
    return bool(status)


def ipm_stop_trajectory(handle: epos_handle, motor_id: Union[MotorId, int]) -> bool:
    """Stops IPM trajectory execution."""
    err = epos_uint32()
    status = epos.VCS_StopIpmTrajectory(handle, _node(motor_id), ctypes.byref(err))
    check_error(status, err, "VCS_StopIpmTrajectory")
    return bool(status)


def ipm_get_status(
    handle: epos_handle, motor_id: Union[MotorId, int]
) -> Tuple[bool, bool, bool, bool, bool, bool, bool, bool, bool]:
    """
    Returns IPM status flags:
    (is_running, underflow_warn, overflow_warn, vel_warn, acc_warn, underflow_err, overflow_err, vel_err, acc_err)
    """
    is_running = epos_bool()
    u_warn = epos_bool()
    o_warn = epos_bool()
    v_warn = epos_bool()
    a_warn = epos_bool()
    u_err = epos_bool()
    o_err = epos_bool()
    v_err = epos_bool()
    a_err = epos_bool()
    err = epos_uint32()
    status = epos.VCS_GetIpmStatus(
        handle,
        _node(motor_id),
        ctypes.byref(is_running),
        ctypes.byref(u_warn),
        ctypes.byref(o_warn),
        ctypes.byref(v_warn),
        ctypes.byref(a_warn),
        ctypes.byref(u_err),
        ctypes.byref(o_err),
        ctypes.byref(v_err),
        ctypes.byref(a_err),
        ctypes.byref(err),
    )
    check_error(status, err, "VCS_GetIpmStatus")
    return (
        bool(is_running.value),
        bool(u_warn.value),
        bool(o_warn.value),
        bool(v_warn.value),
        bool(a_warn.value),
        bool(u_err.value),
        bool(o_err.value),
        bool(v_err.value),
        bool(a_err.value),
    )


# ----------------------------------------------------------------------------
# Digital & Analog Inputs/Outputs (Chapter 5.14)
# ----------------------------------------------------------------------------


def get_all_digital_inputs(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Reads bitmask of all digital inputs."""
    inputs = epos_uint16()
    err = epos_uint32()
    status = epos.VCS_GetAllDigitalInputs(
        handle, _node(motor_id), ctypes.byref(inputs), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetAllDigitalInputs")
    return inputs.value


def get_all_digital_outputs(handle: epos_handle, motor_id: Union[MotorId, int]) -> int:
    """Reads bitmask of all digital outputs."""
    outputs = epos_uint16()
    err = epos_uint32()
    status = epos.VCS_GetAllDigitalOutputs(
        handle, _node(motor_id), ctypes.byref(outputs), ctypes.byref(err)
    )
    check_error(status, err, "VCS_GetAllDigitalOutputs")
    return outputs.value


def set_all_digital_outputs(
    handle: epos_handle, motor_id: Union[MotorId, int], outputs: int
) -> bool:
    """Sets states of all digital outputs."""
    err = epos_uint32()
    status = epos.VCS_SetAllDigitalOutputs(
        handle, _node(motor_id), outputs, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_SetAllDigitalOutputs({outputs})")
    return bool(status)


def get_analog_input(
    handle: epos_handle, motor_id: Union[MotorId, int], input_number: int
) -> int:
    """Reads analog input raw digital ADC value."""
    val = epos_uint16()
    err = epos_uint32()
    status = epos.VCS_GetAnalogInput(
        handle, _node(motor_id), input_number, ctypes.byref(val), ctypes.byref(err)
    )
    check_error(status, err, f"VCS_GetAnalogInput({input_number})")
    return val.value


def get_analog_input_voltage(
    handle: epos_handle, motor_id: Union[MotorId, int], input_number: int
) -> int:
    """Reads analog input voltage in millivolts (mV)."""
    voltage = epos_int32()
    err = epos_uint32()
    status = epos.VCS_GetAnalogInputVoltage(
        handle, _node(motor_id), input_number, ctypes.byref(voltage), ctypes.byref(err)
    )
    check_error(status, err, f"VCS_GetAnalogInputVoltage({input_number})")
    return voltage.value


def set_analog_output(
    handle: epos_handle, motor_id: Union[MotorId, int], output_number: int, value: int
) -> bool:
    """Sets raw DAC value on analog output."""
    err = epos_uint32()
    status = epos.VCS_SetAnalogOutput(
        handle, _node(motor_id), output_number, value, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_SetAnalogOutput({output_number})")
    return bool(status)


def set_analog_output_voltage(
    handle: epos_handle,
    motor_id: Union[MotorId, int],
    output_number: int,
    voltage_mv: int,
) -> bool:
    """Sets output voltage on analog output in millivolts (mV)."""
    err = epos_uint32()
    status = epos.VCS_SetAnalogOutputVoltage(
        handle, _node(motor_id), output_number, voltage_mv, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_SetAnalogOutputVoltage({output_number})")
    return bool(status)


# ----------------------------------------------------------------------------
# Low-Layer CAN Functions (Chapter 7.1)
# ----------------------------------------------------------------------------


def can_send_frame(handle: epos_handle, cob_id: int, length: int, data: bytes) -> bool:
    """Sends a raw CAN frame over the open CAN interface."""
    buf = (ctypes.c_uint8 * 8)(*data[:8])
    err = epos_uint32()
    status = epos.VCS_SendCANFrame(handle, cob_id, length, buf, ctypes.byref(err))
    check_error(status, err, f"VCS_SendCANFrame(0x{cob_id:X})")
    return bool(status)


def can_read_frame(
    handle: epos_handle, cob_id: int, length: int, timeout_ms: int
) -> bytes:
    """Reads a raw CAN frame from the open CAN interface."""
    buf = (ctypes.c_uint8 * 8)()
    err = epos_uint32()
    status = epos.VCS_ReadCANFrame(
        handle, cob_id, length, buf, timeout_ms, ctypes.byref(err)
    )
    check_error(status, err, f"VCS_ReadCANFrame(0x{cob_id:X})")
    return bytes(buf[:length])


def can_request_frame(handle: epos_handle, cob_id: int, length: int) -> bool:
    """Sends a remote transmission request (RTR) frame over CAN."""
    err = epos_uint32()
    status = epos.VCS_RequestCANFrame(handle, cob_id, length, ctypes.byref(err))
    check_error(status, err, f"VCS_RequestCANFrame(0x{cob_id:X})")
    return bool(status)


def can_send_nmt_service(
    handle: epos_handle, node_id: int, command_specifier: int
) -> bool:
    """Sends a CANopen NMT (Network Management) service command."""
    err = epos_uint32()
    status = epos.VCS_SendNMTService(
        handle, node_id, command_specifier, ctypes.byref(err)
    )
    check_error(
        status, err, f"VCS_SendNMTService(Node {node_id}, CS {command_specifier})"
    )
    return bool(status)
