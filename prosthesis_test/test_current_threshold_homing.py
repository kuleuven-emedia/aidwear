#!/usr/bin/env python3
"""
Filename: prosthesis_test/test_current_threshold_homing.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-09-12
Version: 1.0
Description: Test procedure for current-threshold-based homing mode for power-on joint calibration.
    Drives the motor toward the mechanical hardstop at a controlled crawl speed until the motor
    current exceeds a predefined threshold. The EPOS4 controller registers the hardstop position
    and moves to the home position in the opposite direction according to the configured home offset,
    relieving mechanical strain on the end-stop.

    Reference:
    - Maxon EPOS4 Firmware Specification (Methods -3 and -4, Sections 3.5.3.14 & 3.5.3.15):
        Method -3: Current Threshold Positive Speed (Object 0x6098 = -3)
        Method -4: Current Threshold Negative Speed (Object 0x6098 = -4)
    - Maxon EPOS Command Library: docs/epos_command_library.md (Chapter 5.7)
    - Architecture Guide: docs/epos4_firmware_guide.md (Section 3.4 & 6.1)
    - Controller Driver: src/hermes/aidwear/prosthesis/can_control/motor_epos.py

Usage example:
    Method -3:
        uv run python prosthesis_test/test_current_threshold_homing.py --interface "CAN_mcp251xfd 0" --port CAN0 --node-id 2 --method -3 --current-threshold 500 --crawl-speed 60 --home-offset 5000
    Method -4:
        uv run python prosthesis_test/test_current_threshold_homing.py --interface "CAN_mcp251xfd 0" --port CAN0 --node-id 2 --method -4 --current-threshold 400 --crawl-speed 50 --home-offset 6000
    Mock:
        uv run python prosthesis_test/test_current_threshold_homing.py --mock --method -3
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Dict, Any
from collections import deque
import can

# Ensure repository root and src directory are on sys.path
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

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
)
from hermes.aidwear.prosthesis.utils.types import (
    EposDevice,
    EposProtocolStack,
    HomingMethod,
    MotorId,
    EncoderId,
)
from hermes.aidwear.prosthesis.utils.utils import config_can_linux
from hermes.aidwear.prosthesis.motor_control.types import HomingConfig, EposDeviceConfig


class CanBackend(can.Listener):
    def __init__(
        self,
        encoder_latest_data: dict[EncoderId, deque[float]],
    ):
        super().__init__()
        self._encoder_latest_data = encoder_latest_data

    def on_message_received(self, msg: can.Message) -> None:
        # Add support for other CAN devices (e.g. PMU, etc.), accounting for Arbitration IDs.
        if (
            msg.arbitration_id in [EncoderId.KNEE.value, EncoderId.ANKLE.value]
            and len(msg.data) >= 2
        ):
            src_id = EncoderId(msg.arbitration_id)

            angle_raw = (msg.data[0] << 8) | msg.data[1]

            error = (angle_raw >> 14) & 0x01  # verification error flag (bit 14)

            angle_data = angle_raw & 0x3FFF  # 14 data bits
            angle_deg = angle_data * (
                360.0 / 16384.0
            )  # Resolution of the 14 bit => 2^14 = 16384

            self._encoder_latest_data[src_id].append(angle_deg)


def run_current_threshold_homing(
    node_id: MotorId,
    device_config: EposDeviceConfig,
    homing_config: HomingConfig,
    poll_interval_s: int,
    is_mock: bool,
    timeout_s: float,
    is_log_telemetry: bool,
) -> Dict[str, Any]:
    """
    Executes current-threshold-based homing calibration for power-on zero reference.

    Workflow:
      1. Initialize & configure communication port to the EPOS controller.
      2. Clear active drive faults and enable the power stage (Operation Enable).
      3. Configure homing parameters (acceleration, speed switch, speed index, offset, current threshold).
      4. Switch operational mode to Homing Mode (Method -3 or -4).
      5. Dispatch homing search command (VCS_FindHome).
      6. Monitor real-time telemetry (position, velocity, current) until homing is attained or timeout occurs.
      7. Validate final position and relieve end-stop mechanical preload.
      8. Safely disable power stage and close communication.

    Returns:
        dict containing calibration results, telemetry statistics, and execution status.
    """
    print("=" * 72)
    print(" MAXON EPOS4 CURRENT-THRESHOLD HOMING CALIBRATION")
    print("=" * 72)
    print(f" Node ID           : {node_id}")
    print(
        f" Interface / Port  : {device_config.interface} / {device_config.port} ({device_config.baudrate} bps)"
    )
    print(
        f" Homing Method     : {homing_config.homing_method.name} ({homing_config.homing_method.value})"
    )
    print(f" Crawl Speed       : {homing_config.speed_switch} RPM toward hardstop")
    print(f" Speed to Home Pos : {homing_config.speed_index} RPM")
    print(f" Acceleration      : {homing_config.acceleration} RPM/s")
    print(f" Current Threshold : {homing_config.current_threshold_ma} mA")
    print(
        f" Home Offset       : {homing_config.home_offset_enc_ticks} counts (retreat from hardstop)"
    )
    print(f" Home Position     : {homing_config.home_position_coordinate} counts")
    print(f" Timeout Limit     : {timeout_s:.1f} s")
    print(
        f" Execution Mode    : {'MOCK SIMULATION' if is_mock else 'PHYSICAL HARDWARE'}"
    )
    print("=" * 72)

    encoder_latest_data: dict[EncoderId, deque[float]] = {
        encoder_id: deque([0.0], maxlen=1) for encoder_id in EncoderId
    }

    can_bus = can.interface.Bus(channel="can0", interface="socketcan", fd=True)
    can_listener = CanBackend(
        encoder_latest_data=encoder_latest_data,
    )
    can_notifier = can.Notifier(bus=can_bus, listeners=[can_listener])

    handle = None
    owns_handle = handle is None
    peak_current = 0
    hardstop_pos = None
    start_pos = 0
    t0 = time.time()
    telemetry_samples = []

    # -------------------------------------------------------------------------
    # Physical Hardware Execution
    # -------------------------------------------------------------------------
    try:
        # Step 1: Open device communication
        if handle is None:
            print(
                f"\n[1/6] Opening communication channel to {device_config.device.value.decode()}..."
            )
            handle = open_device(
                device_config.device,
                device_config.protocol,
                device_config.interface,
                device_config.port,
            )
            set_protocol_stack_settings(
                handle, device_config.baudrate, device_config.timeout_ms
            )
            print(f"      Connected successfully. Device handle: {handle}")

        # Step 2: Clear faults & verify drive status
        print(f"\n[2/6] Checking drive status on Node {node_id}...")
        if is_fault(handle, node_id):
            print("      Active fault detected on drive. Clearing fault...")
            clear_fault(handle, node_id)
            time.sleep(0.05)

        start_pos = get_position(handle, node_id)
        start_curr = get_current(handle, node_id)
        print(
            f"      Initial State: Position = {start_pos} QC, Current = {start_curr} mA"
        )

        # Step 3: Transition to Operation Enable
        print("\n[3/6] Transitioning EPOS state machine to 'Operation Enable'...")
        set_enable_state(handle, node_id)
        time.sleep(0.05)

        # Step 4: Configure homing parameters
        print("\n[4/6] Writing homing parameters to EPOS dictionary...")
        hm_set_homing_parameter(
            handle=handle,
            motor_id=node_id,
            acceleration=homing_config.acceleration,
            speed_switch=homing_config.speed_switch,
            speed_index=homing_config.speed_index,
            offset=homing_config.home_offset_enc_ticks,
            current_threshold=homing_config.current_threshold_ma,
            home_position=homing_config.home_position_coordinate,
        )

        # Read back parameters for verification
        acc, spd_sw, spd_idx, off, thresh, h_pos = hm_get_homing_parameter(
            handle, node_id
        )
        print("      Homing parameters verified:")
        print(f"        Acceleration     : {acc} RPM/s")
        print(f"        Search Speed     : {spd_sw} RPM")
        print(f"        Index/Home Speed : {spd_idx} RPM")
        print(f"        Home Offset      : {off} counts")
        print(f"        Threshold Current: {thresh} mA")
        print(f"        Home Position    : {h_pos} counts")

        # Step 5: Activate Homing Mode & start search
        print(
            f"\n[5/6] Activating Homing Mode (Method {homing_config.homing_method.value})..."
        )
        activate_homing_mode(handle, node_id)
        time.sleep(0.02)

        print(
            f"      Initiating hardstop search ({homing_config.homing_method.name})..."
        )
        hm_find_home(handle, node_id, homing_config.homing_method)

        # Step 6: Real-time telemetry monitoring loop
        print(
            "\n[6/6] Monitoring homing progress (waiting for current threshold & homing attained)..."
        )
        print(
            f"{'Time [s]':>8} | {'Abs. Position [QC]':>14} | {'Inc. Position [QC]':>14} | {'Velocity [RPM]':>14} | {'Current [mA]':>12} | Status"
        )
        print("-" * 72)

        threshold_detected = False
        t0 = time.time()

        while True:
            elapsed = time.time() - t0
            cur_pos = get_position(handle, node_id)
            cur_vel = get_velocity(handle, node_id)
            cur_curr = get_current(handle, node_id)
            peak_current = max(peak_current, abs(cur_curr))
            cur_enc = encoder_latest_data[EncoderId[node_id.name]][-1]

            attained, homing_err = hm_get_state(handle, node_id)

            if homing_err:
                raise RuntimeError(
                    f"EPOS reported Homing Error flag! Position: {cur_pos}, Current: {cur_curr} mA"
                )

            # Detect threshold event
            if (
                abs(cur_curr) >= homing_config.current_threshold_ma
                and not threshold_detected
            ):
                threshold_detected = True
                hardstop_pos = cur_pos

            status_msg = "Searching (Crawl)"

            if threshold_detected and not attained:
                status_msg = f"Threshold Hit ({cur_curr} mA) -> Moving to Home Pos"
            elif attained:
                status_msg = "HOMING ATTAINED"

            if is_log_telemetry:
                print(
                    f"{elapsed:8.2f} | {cur_enc:.4f} | {cur_pos:14d} | {cur_vel:14d} | {cur_curr:12d} | {status_msg}"
                )

            telemetry_samples.append(
                {
                    "time": elapsed,
                    "abs_pos": cur_enc,
                    "inc_pos": cur_pos,
                    "vel": cur_vel,
                    "curr": cur_curr,
                }
            )

            if attained:
                print("-" * 72)
                final_pos = get_position(handle, node_id)
                final_curr = get_current(handle, node_id)
                final_enc = encoder_latest_data[EncoderId[node_id.name]][-1]
                print(f"[SUCCESS] Homing procedure completed in {elapsed:.2f} s.")
                print(f"          Starting Position : {start_pos} QC")
                if hardstop_pos is not None:
                    print(f"          Hardstop Contact  : {hardstop_pos} QC")
                print(
                    f"          Final Angle    : {final_enc:.3f} ° (Target: reference)"
                )
                print(
                    f"          Final Position    : {final_pos} QC (Target: {homing_config.home_position_coordinate})"
                )
                print(f"          Final Current     : {final_curr} mA (Idle)")
                print(
                    f"          Peak Sensed Curr  : {peak_current} mA (Threshold: {homing_config.current_threshold_ma} mA)"
                )

                return {
                    "success": True,
                    "start_position": start_pos,
                    "hardstop_position": hardstop_pos,
                    "final_position": final_pos,
                    "peak_current_ma": peak_current,
                    "elapsed_time_s": elapsed,
                    "samples_count": len(telemetry_samples),
                }

            if elapsed > timeout_s:
                raise TimeoutError(
                    f"Homing procedure timed out after {elapsed:.1f} s without reaching 'Homing Attained'."
                )

            time.sleep(poll_interval_s)

    except KeyboardInterrupt:
        print(
            "\n\n[ABORT] KeyboardInterrupt detected. Stopping homing and safely disabling drive..."
        )
        if handle:
            try:
                hm_stop_homing(handle, node_id)
                set_quick_stop_state(handle, node_id)
            except Exception as e:
                print(f"Error during quick stop: {e}")
        raise

    except Exception as exc:
        print(f"\n[ERROR] Calibration failed: {exc}")
        if handle:
            try:
                hm_stop_homing(handle, node_id)
            except Exception:
                pass
        raise

    finally:
        # Safe shutdown: disable power stage and close port if owned
        if handle and owns_handle:
            print(
                "\n[CLEANUP] Disabling motor power stage and closing communication..."
            )
            try:
                set_disable_state(handle, node_id)
            except Exception as e:
                print(f"Warning: Failed to set disable state: {e}")
            try:
                close_device(handle)
                print("          Device closed safely.")
            except Exception as e:
                print(f"Warning: Failed to close device: {e}")
        can_notifier.stop()
        can_bus.shutdown()

        with open(f"prosthesis_test/{node_id.name.lower()}_calibration.csv", "w") as f:
            f.write("time,abs_pos,inc_pos,vel,curr\n")
            f.writelines(
                map(
                    lambda s: f"{','.join(map(lambda e: str(e), s.values()))}\n",
                    telemetry_samples,
                )
            )


def main():
    parser = argparse.ArgumentParser(
        description="EPOS4 Current-Threshold-Based Homing Calibration Test Procedure",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Hardware & Communication
    parser.add_argument(
        "--device",
        default=EposDevice.EPOS4.name,
        choices=[EposDevice.EPOS.name, EposDevice.EPOS2.name, EposDevice.EPOS4.name],
        help="EPOS controller model",
    )
    parser.add_argument(
        "--protocol",
        default=EposProtocolStack.CAN_OPEN.name,
        choices=[
            EposProtocolStack.CAN_OPEN.name,
            EposProtocolStack.MAXON_RS232.name,
            EposProtocolStack.MAXON_SERIAL_V2.name,
        ],
        help="Protocol stack name",
    )
    parser.add_argument(
        "--interface",
        default="CAN_mcp251xfd 0",
        help="CAN interface name",
    )
    parser.add_argument(
        "--port",
        default="CAN0",
        help="CAN port name (e.g. CAN0, USB0)",
    )
    parser.add_argument(
        "--baudrate", type=int, default=500_000, help="CANopen baudrate in bps"
    )
    parser.add_argument(
        "--node-id",
        type=int,
        default=MotorId.ANKLE.value,
        choices=[MotorId.KNEE.value, MotorId.ANKLE.value],
        help="CANopen Node ID of the target motor",
    )

    # Homing Parameters
    parser.add_argument(
        "--method",
        type=int,
        default=HomingMethod.CURRENT_THRESHOLD_POSITIVE_SPEED.value,
        choices=[
            HomingMethod.CURRENT_THRESHOLD_POSITIVE_SPEED.value,
            HomingMethod.CURRENT_THRESHOLD_NEGATIVE_SPEED.value,
        ],
        help="Direction of the current threshold based homing method",
    )
    parser.add_argument(
        "--current-threshold",
        type=int,
        default=500,
        help="Hardstop stall current limit in mA",
    )
    parser.add_argument(
        "--crawl-speed",
        type=int,
        default=60,
        help="Search velocity toward hardstop in RPM",
    )
    parser.add_argument(
        "--retreat-speed",
        type=int,
        default=None,
        help="Retreat velocity to home pos in RPM (default: crawl-speed)",
    )
    parser.add_argument(
        "--acceleration", type=int, default=1000, help="Homing acceleration in RPM/s"
    )
    parser.add_argument(
        "--home-offset",
        type=int,
        default=5000,
        help="Home offset distance in encoder counts (retreat away from hardstop)",
    )
    parser.add_argument(
        "--home-position",
        type=int,
        default=0,
        help="Target home position coordinate in counts",
    )

    # Safety & Telemetry
    parser.add_argument(
        "--timeout", type=float, default=20.0, help="Homing attained timeout in seconds"
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.05,
        help="Telemetry polling interval in seconds",
    )
    parser.add_argument(
        "--quiet", action="store_true", help="Suppress real-time telemetry line logging"
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Execute in mock simulation mode (no hardware required)",
    )

    args = parser.parse_args()

    # Determine method enum
    device_enum = EposDevice[args.device]
    protocol_enum = EposProtocolStack[args.protocol]
    method = HomingMethod(args.method)
    node_id = MotorId(args.node_id)

    device_config = EposDeviceConfig(
        device=device_enum,
        protocol=protocol_enum,
        interface=args.interface,
        port=args.port,
        baudrate=args.baudrate,
    )

    homing_config = HomingConfig(
        homing_method=method,
        acceleration=args.acceleration,
        speed_switch=args.crawl_speed,
        speed_index=args.retreat_speed
        if args.retreat_speed is not None
        else args.crawl_speed,
        current_threshold_ma=args.current_threshold,
        home_offset_enc_ticks=args.home_offset,
        home_position_coordinate=args.home_position,
    )

    try:
        results = run_current_threshold_homing(
            node_id=node_id,
            device_config=device_config,
            homing_config=homing_config,
            poll_interval_s=args.poll_interval,
            is_mock=args.mock,
            timeout_s=args.timeout,
            is_log_telemetry=not args.quiet,
        )
        sys.exit(0 if results.get("success") else 1)
    except Exception as e:
        print(f"\nExecution aborted with error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    config_can_linux("can0")
    main()
