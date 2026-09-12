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
        uv run python prosthesis_test/test_current_threshold_homing.py --interface CAN0 --port CAN0 --node-id 1 --method -3 --current-threshold 500 --crawl-speed 60 --home-offset 5000
    Method -4:
        uv run python prosthesis_test/test_current_threshold_homing.py --interface CAN0 --port CAN0 --node-id 1 --method -4 --current-threshold 400 --crawl-speed 50 --home-offset 6000
    Mock:
        uv run python prosthesis_test/test_current_threshold_homing.py --mock --method -3
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
import sys
import time
from typing import Optional, Dict, Any, Tuple

# Ensure repository root and src directory are on sys.path
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from hermes.aidwear.prosthesis.can_control.motor_epos import (
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
    EposDevice,
    EposProtocolStack,
    HomingMethod,
    MotorId,
)


@dataclass
class HomingConfig:
    """Configuration parameters for current-threshold homing calibration."""

    device_name: EposDevice = EposDevice.EPOS4
    protocol_stack_name: EposProtocolStack = EposProtocolStack.CAN_OPEN
    interface_name: str = "CAN_mcp251xfd 0"
    port_name: str = "CAN0"
    baudrate: int = 500_000
    timeout_ms: int = 500
    node_id: int = MotorId.ANKLE.value  # 1: Knee, 2: Ankle

    # Homing method: -3 (positive speed) or -4 (negative speed)
    homing_method: HomingMethod = HomingMethod.CURRENT_THRESHOLD_POSITIVE_SPEED
    acceleration: int = 1000  # Homing acceleration in rpm/s (0x609A)
    speed_switch: int = 60  # Crawl velocity toward hardstop in rpm (0x6099-01)
    speed_index: int = 60  # Speed for movement to home position in rpm (0x6099-02)
    current_threshold: int = 500  # Stall current limit in mA (0x2080 / 0x30B1)
    home_offset: int = 5000  # Distance in encoder counts from hardstop (0x607C)
    home_position: int = 0  # Coordinate assigned to home position (0x30B0)

    # Execution safeguards
    timeout_s: float = 20.0  # Max timeout waiting for homing attained
    poll_interval_s: float = 0.05  # Telemetry sampling period during homing (20 Hz)
    log_telemetry: bool = True  # Print live telemetry during homing
    mock_mode: bool = False  # Mock simulation mode for testing without hardware


class MockHomingExecutor:
    """Simulates EPOS4 current threshold homing when running in mock mode."""

    def __init__(self, config: HomingConfig):
        self.config = config
        self.pos = 0
        self.vel = 0
        self.curr = 50
        self.attained = False
        self.error = False
        self._start_time = 0.0
        self._hardstop_hit = False

    def start(self):
        self._start_time = time.time()
        self._hardstop_hit = False
        self.attained = False
        self.error = False
        self.pos = 10000

    def step(self) -> Tuple[int, int, int, bool, bool]:
        elapsed = time.time() - self._start_time
        direction = (
            1
            if self.config.homing_method
            == HomingMethod.CURRENT_THRESHOLD_POSITIVE_SPEED
            else -1
        )

        if elapsed < 2.0:
            # Crawling toward hardstop
            self.vel = direction * self.config.speed_switch
            self.pos += int(direction * 1500 * self.config.poll_interval_s)
            self.curr = 80 + int(elapsed * 40)
        elif elapsed < 2.5:
            # Hitting hardstop: velocity stops, current rises above threshold
            self._hardstop_hit = True
            self.vel = 0
            self.curr = self.config.current_threshold + 120
        elif elapsed < 4.0:
            # Drive detected hardstop and moves to home_position via home_offset
            # Moving in opposite direction to relieve load
            retreat_dir = -direction
            self.vel = retreat_dir * self.config.speed_index
            self.pos += int(retreat_dir * 1200 * self.config.poll_interval_s)
            self.curr = 120
        else:
            # Homing attained at home_position
            self.vel = 0
            self.curr = 45
            self.pos = self.config.home_position
            self.attained = True

        return self.pos, self.vel, self.curr, self.attained, self.error


def run_current_threshold_homing(
    config: HomingConfig,
    existing_handle: Optional[Any] = None,
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
    print(f" Node ID           : {config.node_id}")
    print(
        f" Interface / Port  : {config.interface_name} / {config.port_name} ({config.baudrate} bps)"
    )
    print(
        f" Homing Method     : {config.homing_method.name} ({config.homing_method.value})"
    )
    print(f" Crawl Speed       : {config.speed_switch} RPM toward hardstop")
    print(f" Speed to Home Pos : {config.speed_index} RPM")
    print(f" Acceleration      : {config.acceleration} RPM/s")
    print(f" Current Threshold : {config.current_threshold} mA")
    print(f" Home Offset       : {config.home_offset} counts (retreat from hardstop)")
    print(f" Home Position     : {config.home_position} counts")
    print(f" Timeout Limit     : {config.timeout_s:.1f} s")
    print(
        f" Execution Mode    : {'MOCK SIMULATION' if config.mock_mode else 'PHYSICAL HARDWARE'}"
    )
    print("=" * 72)

    handle = existing_handle
    owns_handle = existing_handle is None
    peak_current = 0
    hardstop_pos = None
    start_pos = 0
    t0 = time.time()
    telemetry_samples = []

    if config.mock_mode:
        mock = MockHomingExecutor(config)
        mock.start()
        print("\n[MOCK] Initialized mock simulation executor.")
        print(
            "[MOCK] Simulating drive initialization, fault clearing, and homing search..."
        )
        time.sleep(0.2)
        print(
            f"[MOCK] Active Homing Mode: Method {config.homing_method.value} started.\n"
        )

        print(
            f"{'Time [s]':>8} | {'Position [QC]':>14} | {'Velocity [RPM]':>14} | {'Current [mA]':>12} | Status"
        )
        print("-" * 72)

        while True:
            elapsed = time.time() - t0
            pos, vel, curr, attained, error = mock.step()
            peak_current = max(peak_current, abs(curr))

            if config.log_telemetry:
                status_str = "Homing Search (Crawl)"
                if mock._hardstop_hit and not attained:
                    status_str = f"Threshold Hit! Retreating ({config.speed_index} RPM)"
                elif attained:
                    status_str = "HOMING ATTAINED"
                print(
                    f"{elapsed:8.2f} | {pos:14d} | {vel:14d} | {curr:12d} | {status_str}"
                )

            telemetry_samples.append(
                {"time": elapsed, "pos": pos, "vel": vel, "curr": curr}
            )

            if attained:
                print("-" * 72)
                print("[MOCK] Homing attained successfully!")
                return {
                    "success": True,
                    "final_position": pos,
                    "peak_current_ma": peak_current,
                    "elapsed_time_s": elapsed,
                    "samples_count": len(telemetry_samples),
                }

            if elapsed > config.timeout_s:
                raise TimeoutError(
                    f"[MOCK] Homing timed out after {elapsed:.1f} s without attaining home."
                )

            time.sleep(config.poll_interval_s)

    # -------------------------------------------------------------------------
    # Physical Hardware Execution
    # -------------------------------------------------------------------------
    try:
        # Step 1: Open device communication
        if handle is None:
            print(
                f"\n[1/6] Opening communication channel to {config.device_name.value.decode()}..."
            )
            handle = open_device(
                config.device_name,
                config.protocol_stack_name,
                config.interface_name,
                config.port_name,
            )
            set_protocol_stack_settings(handle, config.baudrate, config.timeout_ms)
            print(f"      Connected successfully. Device handle: {handle}")

        # Step 2: Clear faults & verify drive status
        print(f"\n[2/6] Checking drive status on Node {config.node_id}...")
        if is_fault(handle, config.node_id):
            print("      Active fault detected on drive. Clearing fault...")
            clear_fault(handle, config.node_id)
            time.sleep(0.05)

        start_pos = get_position(handle, config.node_id)
        start_curr = get_current(handle, config.node_id)
        print(
            f"      Initial State: Position = {start_pos} QC, Current = {start_curr} mA"
        )

        # Step 3: Transition to Operation Enable
        print("\n[3/6] Transitioning EPOS state machine to 'Operation Enable'...")
        set_enable_state(handle, config.node_id)
        time.sleep(0.05)

        # Step 4: Configure homing parameters
        print("\n[4/6] Writing homing parameters to EPOS dictionary...")
        hm_set_homing_parameter(
            handle=handle,
            motor_id=config.node_id,
            acceleration=config.acceleration,
            speed_switch=config.speed_switch,
            speed_index=config.speed_index,
            offset=config.home_offset,
            current_threshold=config.current_threshold,
            home_position=config.home_position,
        )

        # Read back parameters for verification
        acc, spd_sw, spd_idx, off, thresh, h_pos = hm_get_homing_parameter(
            handle, config.node_id
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
            f"\n[5/6] Activating Homing Mode (Method {config.homing_method.value})..."
        )
        activate_homing_mode(handle, config.node_id)
        time.sleep(0.02)

        print(f"      Initiating hardstop search ({config.homing_method.name})...")
        hm_find_home(handle, config.node_id, config.homing_method)

        # Step 6: Real-time telemetry monitoring loop
        print(
            "\n[6/6] Monitoring homing progress (waiting for current threshold & homing attained)..."
        )
        print(
            f"{'Time [s]':>8} | {'Position [QC]':>14} | {'Velocity [RPM]':>14} | {'Current [mA]':>12} | Status"
        )
        print("-" * 72)

        threshold_detected = False
        t0 = time.time()

        while True:
            elapsed = time.time() - t0
            cur_pos = get_position(handle, config.node_id)
            cur_vel = get_velocity(handle, config.node_id)
            cur_curr = get_current(handle, config.node_id)
            peak_current = max(peak_current, abs(cur_curr))

            attained, homing_err = hm_get_state(handle, config.node_id)

            if homing_err:
                raise RuntimeError(
                    f"EPOS reported Homing Error flag! Position: {cur_pos}, Current: {cur_curr} mA"
                )

            # Detect threshold event
            if abs(cur_curr) >= config.current_threshold and not threshold_detected:
                threshold_detected = True
                hardstop_pos = cur_pos

            status_msg = "Searching (Crawl)"
            if threshold_detected and not attained:
                status_msg = f"Threshold Hit ({cur_curr} mA) -> Moving to Home Pos"
            elif attained:
                status_msg = "HOMING ATTAINED"

            if config.log_telemetry:
                print(
                    f"{elapsed:8.2f} | {cur_pos:14d} | {cur_vel:14d} | {cur_curr:12d} | {status_msg}"
                )

            telemetry_samples.append(
                {
                    "time": elapsed,
                    "pos": cur_pos,
                    "vel": cur_vel,
                    "curr": cur_curr,
                }
            )

            if attained:
                print("-" * 72)
                final_pos = get_position(handle, config.node_id)
                final_curr = get_current(handle, config.node_id)
                print(f"[SUCCESS] Homing procedure completed in {elapsed:.2f} s.")
                print(f"          Starting Position : {start_pos} QC")
                if hardstop_pos is not None:
                    print(f"          Hardstop Contact  : {hardstop_pos} QC")
                print(
                    f"          Final Position    : {final_pos} QC (Target: {config.home_position})"
                )
                print(f"          Final Current     : {final_curr} mA (Idle)")
                print(
                    f"          Peak Sensed Curr  : {peak_current} mA (Threshold: {config.current_threshold} mA)"
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

            if elapsed > config.timeout_s:
                raise TimeoutError(
                    f"Homing procedure timed out after {elapsed:.1f} s without reaching 'Homing Attained'."
                )

            time.sleep(config.poll_interval_s)

    except KeyboardInterrupt:
        print(
            "\n\n[ABORT] KeyboardInterrupt detected. Stopping homing and safely disabling drive..."
        )
        if handle:
            try:
                hm_stop_homing(handle, config.node_id)
                set_quick_stop_state(handle, config.node_id)
            except Exception as e:
                print(f"Error during quick stop: {e}")
        raise

    except Exception as exc:
        print(f"\n[ERROR] Calibration failed: {exc}")
        if handle:
            try:
                hm_stop_homing(handle, config.node_id)
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
                set_disable_state(handle, config.node_id)
            except Exception as e:
                print(f"Warning: Failed to set disable state: {e}")
            try:
                close_device(handle)
                print("          Device closed safely.")
            except Exception as e:
                print(f"Warning: Failed to close device: {e}")


def parse_args() -> HomingConfig:
    """Parses command line arguments and constructs HomingConfig."""
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
        choices=[HomingMethod.CURRENT_THRESHOLD_POSITIVE_SPEED.value, HomingMethod.CURRENT_THRESHOLD_NEGATIVE_SPEED.value],
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

    retreat_spd = (
        args.retreat_speed if args.retreat_speed is not None else args.crawl_speed
    )

    return HomingConfig(
        device_name=device_enum,
        protocol_stack_name=protocol_enum,
        interface_name=args.interface,
        port_name=args.port,
        baudrate=args.baudrate,
        node_id=args.node_id,
        homing_method=method,
        acceleration=args.acceleration,
        speed_switch=args.crawl_speed,
        speed_index=retreat_spd,
        current_threshold=args.current_threshold,
        home_offset=args.home_offset,
        home_position=args.home_position,
        timeout_s=args.timeout,
        poll_interval_s=args.poll_interval,
        log_telemetry=not args.quiet,
        mock_mode=args.mock,
    )


def main():
    config = parse_args()
    try:
        results = run_current_threshold_homing(config)
        sys.exit(0 if results.get("success") else 1)
    except Exception as e:
        print(f"\nExecution aborted with error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
