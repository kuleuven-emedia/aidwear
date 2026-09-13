"""
Filename: hermes/aidwear/prosthesis/motor_control/epos_facade.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-09-13
Version: 1.0
Description: 
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
import sys
import time
from typing import Optional, Dict, Any, Tuple

from hermes.utils.time_utils import get_time

from .types import (
    EposDeviceConfig,
    HomingConfig,
)

from hermes.aidwear.prosthesis.utils.types import (
    EposDevice,
    EposProtocolStack,
    HomingMethod,
    MotorId,
    ServoMotorData,
)

from hermes.aidwear.prosthesis.motor_control.epos_commands import (
    open_device,
    close_device,
    set_protocol_stack_settings,
    clear_fault,
    set_enable_state,
    set_disable_state,
    set_quick_stop_state,
    activate_homing_mode,
    hm_set_homing_parameter,
    hm_get_homing_parameter,
    hm_find_home,
    hm_stop_homing,
    hm_get_state,
    hm_wait_for_homing,
    get_position,
    get_velocity,
    get_velocity_avg,
    get_current,
    get_current_avg,
    get_state,
    is_fault,
    epos_handle,
)

__all__ = [
    'init',
    'shutdown',
    'connect',
    'enable',
    'disable',
    'start_homing',
    'stop_homing',
    'quick_stop',
    'get_motor_data',
    'get_homing_state',
    'wait_for_homing',
]

def init(
    config: EposDeviceConfig,
) -> epos_handle:
    print(f"Opening communication channel to {config.device.name} devices...", flush=True)

    handle = open_device(
        device=config.device,
        protocol=config.protocol,
        interface=config.interface,
        port=config.port,
    )
    set_protocol_stack_settings(handle, config.baudrate, config.timeout_ms)
    print(f"EPOS CAN driver connected successfully with handle: {handle}", flush=True)
    return handle


def shutdown(
    handle: epos_handle,
    motor_ids: list[MotorId],
):
    print("Disabling motor power stage and closing communication...", flush=True)
    for motor_id in motor_ids:
        try:
            set_disable_state(handle, motor_id)
        except Exception as e:
            print(f"Warning: Failed to set disable on {motor_id.name}: {e}", flush=True)

    try:
        close_device(handle)
        print("EPOS device closed safely.", flush=True)
    except Exception as e:
        print(f"Warning: Failed to close device: {e}", flush=True)


def connect(
    handle: epos_handle,
    motor_id: MotorId,
):
    print(f"Checking drive status on Node {motor_id}...", flush=True)
    if is_fault(handle, motor_id):
        print("Active fault detected on drive. Clearing fault...", flush=True)
        clear_fault(handle, motor_id)
        time.sleep(0.05)

    start_pos = get_position(handle, motor_id)
    start_curr = get_current(handle, motor_id)
    print(f"Initial State: Position = {start_pos} QC, Current = {start_curr} mA", flush=True)


def enable(
    handle: epos_handle,
    motor_id: MotorId,
):
    set_enable_state(handle, motor_id)


def disable(
    handle: epos_handle,
    motor_id: MotorId,
):
    set_disable_state(handle, motor_id)


def start_homing(
    handle: epos_handle,
    motor_id: MotorId,
    config: HomingConfig,
):
    hm_set_homing_parameter(
        handle=handle,
        motor_id=motor_id,
        acceleration=config.acceleration,
        speed_switch=config.speed_switch,
        speed_index=config.speed_index,
        offset=config.home_offset_enc_ticks,
        current_threshold=config.current_threshold_ma,
        home_position=config.home_position_coordinate,
    )

    activate_homing_mode(handle, motor_id)

    print(f"Initiating hardstop search ({config.homing_method.name})...", flush=True)

    hm_find_home(handle, motor_id, config.homing_method)


def stop_homing(
    handle: epos_handle,
    motor_id: MotorId,
):
    hm_stop_homing(handle, motor_id)


def quick_stop(
    handle: epos_handle,
    motor_id: MotorId,
):
    set_quick_stop_state(handle, motor_id)


def get_motor_data(
    handle: epos_handle,
    motor_id: MotorId,
) -> ServoMotorData:
    """Captures the current motor state: position, velocity, and current."""
    timestamp = get_time()
    cur_pos = get_position(handle, motor_id)
    cur_vel = get_velocity(handle, motor_id)
    cur_curr = get_current(handle, motor_id)
    fault = is_fault(handle, motor_id)

    return ServoMotorData(
        timestamp=timestamp,
        position=float(cur_pos),
        velocity=float(cur_vel),
        current=float(cur_curr),
        error=fault,
    )


def get_homing_state(
    handle: epos_handle,
    motor_id: MotorId,
) -> tuple[bool, bool]:
    """Returns (is_homing_attained, is_homing_error)."""
    return hm_get_state(handle, motor_id)


def wait_for_homing(
    handle: epos_handle,
    motor_id: MotorId,
    timeout_ms: int = 30_000,
) -> bool:
    """Blocks until homing attained bit is set or timeout occurs."""
    return hm_wait_for_homing(handle, motor_id, timeout_ms)
