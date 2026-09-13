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
    pass

    # TODO: wait until it reached home.


    # TODO: wrap into a subprocess or scheduled AsyncIO routine that continuously probes and captures the motors data.
    #   Move into data reading subprocess / coroutine.
    # # Step 6: Real-time telemetry monitoring loop
    # peak_current = 0
    # hardstop_pos = None
    # start_pos = 0
    # t0 = get_time()
    # telemetry_samples = []
    # threshold_detected = False
    # t0 = get_time()
    # while True:
    #     elapsed = get_time() - t0
    #     cur_pos = get_position(handle, config.node_id)
    #     cur_vel = get_velocity(handle, config.node_id)
    #     cur_curr = get_current(handle, config.node_id)
    #     peak_current = max(peak_current, abs(cur_curr))

    #     attained, homing_err = hm_get_state(handle, config.node_id)

    #     if homing_err:
    #         raise RuntimeError(
    #             f"EPOS reported Homing Error flag! Position: {cur_pos}, Current: {cur_curr} mA"
    #         )

    #     # Detect threshold event
    #     if abs(cur_curr) >= config.current_threshold and not threshold_detected:
    #         threshold_detected = True
    #         hardstop_pos = cur_pos

    #     status_msg = "Searching (Crawl)"
    #     if threshold_detected and not attained:
    #         status_msg = f"Threshold Hit ({cur_curr} mA) -> Moving to Home Pos"
    #     elif attained:
    #         status_msg = "HOMING ATTAINED"

    #     if config.log_telemetry:
    #         print(
    #             f"{elapsed:8.2f} | {cur_pos:14d} | {cur_vel:14d} | {cur_curr:12d} | {status_msg}"
    #         )

    #     telemetry_samples.append(
    #         {
    #             "time": elapsed,
    #             "pos": cur_pos,
    #             "vel": cur_vel,
    #             "curr": cur_curr,
    #         }
    #     )

    #     if attained:
    #         print("-" * 72)
    #         final_pos = get_position(handle, config.node_id)
    #         final_curr = get_current(handle, config.node_id)
    #         print(f"[SUCCESS] Homing procedure completed in {elapsed:.2f} s.")
    #         print(f"          Starting Position : {start_pos} QC")
    #         if hardstop_pos is not None:
    #             print(f"          Hardstop Contact  : {hardstop_pos} QC")
    #         print(
    #             f"          Final Position    : {final_pos} QC (Target: {config.home_position})"
    #         )
    #         print(f"          Final Current     : {final_curr} mA (Idle)")
    #         print(
    #             f"          Peak Sensed Curr  : {peak_current} mA (Threshold: {config.current_threshold} mA)"
    #         )

    #         return {
    #             "success": True,
    #             "start_position": start_pos,
    #             "hardstop_position": hardstop_pos,
    #             "final_position": final_pos,
    #             "peak_current_ma": peak_current,
    #             "elapsed_time_s": elapsed,
    #             "samples_count": len(telemetry_samples),
    #         }

    #     time.sleep(config.poll_interval_s)


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
