"""
Filename: hermes/aidwear/prosthesis/motor_control/epos_facade.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-09-21
Version: 2.0
Description: EPOS Facade and CiA 402 drive state machine manager implemented with python-statemachine.
    Provides autonomous fault detection, fault clearing, graceful recovery, and setpoint buffering
    to allow upstream real-time control loops (e.g., gait trajectory generators, impedance controllers)
    to command the motor drives continuously without handling low-level CANopen state transitions.
"""

from __future__ import annotations

import ctypes
import struct
import time
from multiprocessing import Queue
from typing import Optional, Dict, Tuple, List, Union, Any
from statemachine import StateMachine, State

from hermes.utils.time_utils import get_time

from .types import (
    EposDeviceConfig,
    HomingConfig,
    RecoveryStrategy,
    EposRecoveryConfig,
)

from hermes.aidwear.prosthesis.utils.types import (
    EposOperationMode,
    MotorId,
    ServoMotorData,
    epos_handle,
    MotorCommand,
    ServoCanPacketEnum,
)

from hermes.aidwear.prosthesis.motor_control import epos_commands as cmd

__all__ = [
    "EposMotorFacade",
    "EposFacade",
]


class EposMotorFacade(StateMachine):
    """CiA 402 drive state machine manager and facade for a single EPOS motor drive.

    Manages low-level CANopen drive transitions, monitors drive status, intercepts
    drive fault trips, autonomously clears faults, and restores desired operating modes
    while buffering incoming setpoints from upstream real-time control loops.
    """

    # ------------------------------------------------------------------------
    # State Machine States
    # ------------------------------------------------------------------------
    uninitialized = State(name="Uninitialized", value=0, initial=True)
    disabled = State(name="Disabled", value=1)
    position_mode = State(name="PositionMode", value=2)
    current_mode = State(name="CurrentMode", value=3)
    velocity_mode = State(name="VelocityMode", value=4)
    homing = State(name="Homing", value=5)
    quick_stop = State(name="QuickStop", value=6)
    faulted = State(name="Faulted", value=7)
    recovering = State(name="Recovering", value=8)

    allow_event_without_transition: bool = True

    # ------------------------------------------------------------------------
    # State Machine Transitions
    # ------------------------------------------------------------------------
    to_connect = disabled.from_(uninitialized)
    to_disabled = disabled.from_.any()
    to_position_mode = position_mode.from_.any()
    to_current_mode = current_mode.from_.any()
    to_velocity_mode = velocity_mode.from_.any()
    to_homing = homing.from_.any()
    to_quick_stop = quick_stop.from_.any()
    to_faulted = faulted.from_.any()
    to_recovering = recovering.from_(faulted)

    to_recovery_succeeded = (
        recovering.to(position_mode, cond="is_target_position_mode")
        | recovering.to(current_mode, cond="is_target_current_mode")
        | recovering.to(velocity_mode, cond="is_target_velocity_mode")
        | recovering.to(disabled, cond="is_target_disabled")
    )

    to_recovery_failed = recovering.to(faulted)

    def __init__(
        self,
        handle: epos_handle,
        motor_id: MotorId,
        command_queue: "Queue[MotorCommand]",
        recovery_config: Optional[EposRecoveryConfig] = None,
    ):
        self._handle = handle
        self._motor_id = motor_id
        self._recovery_config = recovery_config or EposRecoveryConfig()
        self._command_queue: Queue[MotorCommand] = command_queue

        # Desired operating mode tracking for seamless recovery
        self._desired_mode: Optional[EposOperationMode] = None

        # Targets & setpoint buffers
        self._target_position: Optional[int] = None
        self._target_current: Optional[int] = None
        self._target_velocity: Optional[int] = None

        self._pending_target_position: Optional[int] = None
        self._pending_target_current: Optional[int] = None
        self._pending_target_velocity: Optional[int] = None

        # Telemetry cache
        self._last_telemetry: Optional[ServoMotorData] = None
        self._last_fault_time: float = 0.0
        self._recovery_attempts: int = 0
        self._last_error_codes: List[int] = []

        super().__init__()

    @property
    def current_state(self) -> State:
        """Returns the currently active State object."""
        return next(iter(self.configuration))

    @property
    def motor_id(self) -> MotorId:
        return self._motor_id

    @property
    def handle(self) -> epos_handle:
        return self._handle

    @property
    def desired_mode(self) -> Optional[EposOperationMode]:
        return self._desired_mode

    @property
    def recovery_attempts(self) -> int:
        return self._recovery_attempts

    @property
    def last_error_codes(self) -> List[int]:
        return self._last_error_codes

    @property
    def command_queue(self) -> "Queue[MotorCommand]":
        return self._command_queue

    @command_queue.setter
    def command_queue(self, queue: "Queue[MotorCommand]") -> None:
        self._command_queue = queue

    def _record_command(self, control_mode: int, value: Union[int, float], label: str) -> None:
        """Records commanded target setpoint into multiprocessing queue for upstream telemetry."""
        if self._command_queue is not None:
            try:
                # TODO: update how the motor data is stored.
                cmd_item = MotorCommand(
                    motor_id=self._motor_id,
                    timestamp=get_time(),
                    command_data=struct.pack("<q", int(round(value))),
                    control_mode=int(control_mode) & 0xFF,
                    log_data=f"{label}:{value}".encode("utf-8").ljust(20, b"\x00")[:20],
                )
                self._command_queue.put_nowait(cmd_item)
            except Exception:
                # Real-time safety: queue overflow must never disrupt motor control loop
                pass

    # ------------------------------------------------------------------------
    # State Machine Transition Conditions
    # ------------------------------------------------------------------------
    def is_target_position_mode(self) -> bool:
        return self._desired_mode == EposOperationMode.POSITION

    def is_target_current_mode(self) -> bool:
        return self._desired_mode == EposOperationMode.CURRENT

    def is_target_velocity_mode(self) -> bool:
        return self._desired_mode == EposOperationMode.VELOCITY

    def is_target_disabled(self) -> bool:
        return self._desired_mode is None

    # ------------------------------------------------------------------------
    # State Machine Callbacks
    # ------------------------------------------------------------------------
    def on_enter_disabled(self):
        self._desired_mode = None
        try:
            cmd.set_disable_state(self._handle, self._motor_id)
        except Exception as e:
            print(f"[{self._motor_id.name}] Warning setting disable state: {e}", flush=True)

    def on_enter_position_mode(self):
        self._desired_mode = EposOperationMode.POSITION
        self._recovery_attempts = 0
        try:
            cmd.set_enable_state(self._handle, self._motor_id)
            cmd.activate_position_mode(self._handle, self._motor_id)
            if self._pending_target_position is not None:
                target = self._pending_target_position
                self._pending_target_position = None
                self._target_position = target
                cmd.pm_set_position_must(self._handle, self._motor_id, target)
        except Exception as e:
            print(f"[{self._motor_id.name}] Failed to activate Position Mode: {e}", flush=True)
            self._handle_comm_exception()

    def on_enter_current_mode(self):
        self._desired_mode = EposOperationMode.CURRENT
        self._recovery_attempts = 0
        try:
            cmd.set_enable_state(self._handle, self._motor_id)
            cmd.activate_current_mode(self._handle, self._motor_id)
            if self._pending_target_current is not None:
                target = self._pending_target_current
                self._pending_target_current = None
                self._target_current = target
                cmd.cm_set_current_must(self._handle, self._motor_id, target)
        except Exception as e:
            print(f"[{self._motor_id.name}] Failed to activate Current Mode: {e}", flush=True)
            self._handle_comm_exception()

    def on_enter_velocity_mode(self):
        self._desired_mode = EposOperationMode.VELOCITY
        self._recovery_attempts = 0
        try:
            cmd.set_enable_state(self._handle, self._motor_id)
            cmd.activate_velocity_mode(self._handle, self._motor_id)
            if self._pending_target_velocity is not None:
                target = self._pending_target_velocity
                self._pending_target_velocity = None
                self._target_velocity = target
                cmd.vm_set_velocity_must(self._handle, self._motor_id, target)
        except Exception as e:
            print(f"[{self._motor_id.name}] Failed to activate Velocity Mode: {e}", flush=True)
            self._handle_comm_exception()

    def on_enter_homing(self):
        self._desired_mode = EposOperationMode.HOMING
        self._recovery_attempts = 0
        try:
            cmd.set_enable_state(self._handle, self._motor_id)
            cmd.activate_homing_mode(self._handle, self._motor_id)
        except Exception as e:
            print(f"[{self._motor_id.name}] Failed to activate Homing Mode: {e}", flush=True)
            self._handle_comm_exception()

    def on_enter_quick_stop(self):
        try:
            cmd.set_quick_stop_state(self._handle, self._motor_id)
        except Exception as e:
            print(f"[{self._motor_id.name}] Warning executing Quick Stop: {e}", flush=True)

    def on_enter_faulted(self):
        self._last_fault_time = get_time()
        self._read_device_errors()
        print(
            f"[{self._motor_id.name}] FAULT DETECTED on drive! "
            f"Device error codes: {[hex(c) for c in self._last_error_codes]}",
            flush=True,
        )

        if self._recovery_config.auto_recover:
            is_infinite = self._recovery_config.max_retries <= 0
            if is_infinite or self._recovery_attempts < self._recovery_config.max_retries:
                # Trigger recovery sequence
                self.start_recovery()
            else:
                print(
                    f"[{self._motor_id.name}] ERROR: Max recovery attempts "
                    f"({self._recovery_config.max_retries}) exceeded. Drive remains faulted.",
                    flush=True,
                )

    def on_enter_recovering(self):
        self._recovery_attempts += 1
        retries_str = "inf" if self._recovery_config.max_retries <= 0 else str(self._recovery_config.max_retries)
        print(
            f"[{self._motor_id.name}] Executing recovery attempt {self._recovery_attempts}/"
            f"{retries_str}...",
            flush=True,
        )

        success = self._execute_recovery_routine()
        if success:
            print(
                f"[{self._motor_id.name}] Recovery SUCCEEDED. Restoring to mode: {self._desired_mode}",
                flush=True,
            )
            self.recovery_succeeded()
        else:
            print(f"[{self._motor_id.name}] Recovery FAILED on attempt {self._recovery_attempts}.", flush=True)
            if self._recovery_config.retry_delay_s > 0:
                time.sleep(self._recovery_config.retry_delay_s)
            self.recovery_failed()

    # ------------------------------------------------------------------------
    # Fault Clearance and Recovery Execution
    # ------------------------------------------------------------------------
    def _read_device_errors(self) -> List[int]:
        """Queries and records device error codes from the EPOS device."""
        codes = []
        try:
            nb_errors = cmd.epos_uint8()
            err = cmd.epos_uint32()
            status = cmd.epos.VCS_GetNbOfDeviceError(
                self._handle, self._motor_id.value, ctypes.byref(nb_errors), ctypes.byref(err)
            )
            if status and nb_errors.value > 0:
                for idx in range(1, nb_errors.value + 1):
                    err_code = cmd.epos_uint32()
                    cmd.epos.VCS_GetDeviceErrorCode(
                        self._handle,
                        self._motor_id.value,
                        idx,
                        ctypes.byref(err_code),
                        ctypes.byref(err),
                    )
                    codes.append(err_code.value)
        except Exception:
            pass
        self._last_error_codes = codes
        return codes

    def _execute_recovery_routine(self) -> bool:
        """Attempts to clear fault and restore drive to operational state."""
        try:
            # 1. Clear Fault via VCS_ClearFault
            cmd.clear_fault(self._handle, self._motor_id)
            if self._recovery_config.retry_delay_s > 0:
                time.sleep(self._recovery_config.retry_delay_s)

            # 2. Verify fault bit is cleared
            if cmd.is_fault(self._handle, self._motor_id):
                return False

            # 3. Re-enable power stage
            if self._desired_mode is not None:
                cmd.set_enable_state(self._handle, self._motor_id)

                # 4. Re-activate desired mode
                if self._desired_mode == EposOperationMode.POSITION:
                    cmd.activate_position_mode(self._handle, self._motor_id)
                    self._apply_position_recovery_strategy()
                elif self._desired_mode == EposOperationMode.CURRENT:
                    cmd.activate_current_mode(self._handle, self._motor_id)
                    target = (
                        self._pending_target_current
                        if self._pending_target_current is not None
                        else 0
                    )
                    cmd.cm_set_current_must(self._handle, self._motor_id, target)
                elif self._desired_mode == EposOperationMode.VELOCITY:
                    cmd.activate_velocity_mode(self._handle, self._motor_id)
                    target = (
                        self._pending_target_velocity
                        if self._pending_target_velocity is not None
                        else 0
                    )
                    cmd.vm_set_velocity_must(self._handle, self._motor_id, target)

            return True
        except Exception as e:
            print(f"[{self._motor_id.name}] Exception during recovery routine: {e}", flush=True)
            return False

    def _apply_position_recovery_strategy(self):
        """Applies position setpoint following recovery based on configured strategy."""
        strategy = self._recovery_config.strategy
        try:
            cur_pos = cmd.get_position(self._handle, self._motor_id)
        except Exception:
            cur_pos = 0

        if strategy == RecoveryStrategy.BUMPLESS_HOLD:
            # Latch current physical position to prevent sudden mechanical jumps
            cmd.pm_set_position_must(self._handle, self._motor_id, cur_pos)
            self._target_position = cur_pos
            self._pending_target_position = None
        elif strategy == RecoveryStrategy.LATEST_TARGET:
            target = (
                self._pending_target_position
                if self._pending_target_position is not None
                else cur_pos
            )
            cmd.pm_set_position_must(self._handle, self._motor_id, target)
            self._target_position = target
            self._pending_target_position = None
        elif strategy == RecoveryStrategy.RAMP_TO_TARGET:
            # Hold current position first; subsequent commands will ramp smoothly
            cmd.pm_set_position_must(self._handle, self._motor_id, cur_pos)
            self._target_position = cur_pos

    def _handle_comm_exception(self):
        """Catches CAN / drive communication exceptions and checks for faults."""
        try:
            if cmd.is_fault(self._handle, self._motor_id):
                if not (self.current_state == self.faulted or self.current_state == self.recovering):
                    self.fault_detected()
                return
        except Exception:
            pass

        if not (self.current_state == self.faulted or self.current_state == self.recovering):
            self.fault_detected()

    # ------------------------------------------------------------------------
    # Public Upstream API: Mode Selection & State Control
    # ------------------------------------------------------------------------
    def connect(self):
        """Transitions motor from uninitialized to disabled (ready for enablement)."""
        if self.current_state == self.uninitialized:
            self.to_connect()

    def set_position_mode(self):
        """Switches drive into closed-loop Position Mode."""
        self._desired_mode = EposOperationMode.POSITION
        if self.current_state != self.position_mode:
            self.to_position_mode()

    def set_current_mode(self):
        """Switches drive into closed-loop Current / Torque Mode."""
        self._desired_mode = EposOperationMode.CURRENT
        if self.current_state != self.current_mode:
            self.to_current_mode()

    def set_velocity_mode(self):
        """Switches drive into closed-loop Velocity Mode."""
        self._desired_mode = EposOperationMode.VELOCITY
        if self.current_state != self.velocity_mode:
            self.to_velocity_mode()

    def set_disabled(self):
        """Disables the drive power stage."""
        self.to_disabled()

    def disable(self):
        """Alias for set_disabled()."""
        self.set_disabled()

    def quick_stop(self):
        """Commands quick stop on the drive."""
        self.to_quick_stop()

    def fault_detected(self):
        """Reports a detected fault on the drive."""
        self.to_faulted()

    def start_recovery(self):
        """Triggers recovery routine from faulted state."""
        self.to_recovering()

    def recovery_succeeded(self):
        """Transitions from recovering back to desired operational mode."""
        self.to_recovery_succeeded()

    def recovery_failed(self):
        """Transitions from recovering back to faulted."""
        self.to_recovery_failed()

    # ------------------------------------------------------------------------
    # Public Upstream API: Setpoint Emitting
    # ------------------------------------------------------------------------
    def set_target_position(self, position: int) -> bool:
        """Commands a target position in position units (encoder ticks / QC).

        If the drive is currently faulted or undergoing recovery, the setpoint is buffered
        seamlessly without raising an error or crashing upstream control loops.
        """
        self._target_position = position
        self._record_command(ServoCanPacketEnum.POSITION_MODE.value, position, "pos")

        # Buffer setpoint if currently faulted or recovering
        if self.current_state in (self.faulted, self.recovering):
            self._pending_target_position = position
            return False

        # If not yet in position mode, automatically switch to position mode
        if self.current_state != self.position_mode:
            self._pending_target_position = position
            self.set_position_mode()
            return True

        try:
            cmd.pm_set_position_must(self._handle, self._motor_id, position)
            return True
        except Exception as e:
            print(f"[{self._motor_id.name}] Error in set_target_position: {e}", flush=True)
            self._pending_target_position = position
            self._handle_comm_exception()
            return False

    def set_target_current(self, current_ma: int) -> bool:
        """Commands a target current setpoint in milliamperes (mA)."""
        self._target_current = current_ma
        self._record_command(ServoCanPacketEnum.CURRENT_LOOP_MODE.value, current_ma, "cur")

        if self.current_state in (self.faulted, self.recovering):
            self._pending_target_current = current_ma
            return False

        if self.current_state != self.current_mode:
            self._pending_target_current = current_ma
            self.set_current_mode()
            return True

        try:
            cmd.cm_set_current_must(self._handle, self._motor_id, current_ma)
            return True
        except Exception as e:
            print(f"[{self._motor_id.name}] Error in set_target_current: {e}", flush=True)
            self._pending_target_current = current_ma
            self._handle_comm_exception()
            return False

    def set_target_velocity(self, velocity_rpm: int) -> bool:
        """Commands a target velocity setpoint in rpm."""
        self._target_velocity = velocity_rpm
        self._record_command(ServoCanPacketEnum.VELOCITY_MODE.value, velocity_rpm, "vel")

        if self.current_state in (self.faulted, self.recovering):
            self._pending_target_velocity = velocity_rpm
            return False

        if self.current_state != self.velocity_mode:
            self._pending_target_velocity = velocity_rpm
            self.set_velocity_mode()
            return True

        try:
            cmd.vm_set_velocity_must(self._handle, self._motor_id, velocity_rpm)
            return True
        except Exception as e:
            print(f"[{self._motor_id.name}] Error in set_target_velocity: {e}", flush=True)
            self._pending_target_velocity = velocity_rpm
            self._handle_comm_exception()
            return False

    # ------------------------------------------------------------------------
    # Public Upstream API: Telemetry & State Inquiry
    # ------------------------------------------------------------------------
    def get_motor_data(self) -> ServoMotorData:
        """Acquires current telemetry (position, velocity, current, error flag).

        Catches low-level faults cleanly: if the drive tripped, triggers the state machine's
        fault detection and returns the last known position/velocity/current with error=True,
        ensuring upstream telemetry polling threads never encounter uncaught exceptions.
        """
        timestamp = get_time()
        try:
            fault = cmd.is_fault(self._handle, self._motor_id)
            if fault:
                if not (self.current_state == self.faulted or self.current_state == self.recovering):
                    self.fault_detected()
                return ServoMotorData(
                    timestamp=timestamp,
                    position=float(self._last_telemetry.position if self._last_telemetry else 0.0),
                    velocity=float(self._last_telemetry.velocity if self._last_telemetry else 0.0),
                    current=float(self._last_telemetry.current if self._last_telemetry else 0.0),
                    error=True,
                )

            cur_pos = cmd.get_position(self._handle, self._motor_id)
            cur_vel = cmd.get_velocity(self._handle, self._motor_id)
            cur_curr = cmd.get_current(self._handle, self._motor_id)

            data = ServoMotorData(
                timestamp=timestamp,
                position=float(cur_pos),
                velocity=float(cur_vel),
                current=float(cur_curr),
                error=False,
            )
            self._last_telemetry = data
            return data

        except Exception as e:
            self._handle_comm_exception()
            return ServoMotorData(
                timestamp=timestamp,
                position=float(self._last_telemetry.position if self._last_telemetry else 0.0),
                velocity=float(self._last_telemetry.velocity if self._last_telemetry else 0.0),
                current=float(self._last_telemetry.current if self._last_telemetry else 0.0),
                error=True,
            )

    # ------------------------------------------------------------------------
    # Public Upstream API: Homing Calibration
    # ------------------------------------------------------------------------
    def start_homing(self, config: HomingConfig):
        """Configures homing parameters and triggers the homing routine."""
        self.to_homing()
        cmd.hm_set_homing_parameter(
            handle=self._handle,
            motor_id=self._motor_id,
            acceleration=config.acceleration,
            speed_switch=config.speed_switch,
            speed_index=config.speed_index,
            offset=config.home_offset_enc_ticks,
            current_threshold=config.current_threshold_ma,
            home_position=config.home_position_coordinate,
        )
        cmd.activate_homing_mode(self._handle, self._motor_id)
        cmd.hm_find_home(self._handle, self._motor_id, config.homing_method)

    def stop_homing(self):
        """Halts active homing procedure."""
        cmd.hm_stop_homing(self._handle, self._motor_id)
        self.disable()

    def get_homing_state(self) -> Tuple[bool, bool]:
        """Returns (is_homing_attained, is_homing_error)."""
        return cmd.hm_get_state(self._handle, self._motor_id)

    def wait_for_homing(self, timeout_ms: int = 30_000) -> bool:
        """Blocks until homing attained bit is set or timeout occurs."""
        return cmd.hm_wait_for_homing(self._handle, self._motor_id, timeout_ms)


class EposFacade:
    """System-level EPOS coordinator managing CANopen communications and motor state machines."""

    def __init__(
        self,
        command_queue: "Queue[MotorCommand]",
        config: Optional[EposDeviceConfig] = None,
        recovery_config: Optional[EposRecoveryConfig] = None,
    ):
        self._config = config
        self._recovery_config = recovery_config or EposRecoveryConfig()
        self._command_queue: Queue[MotorCommand] = command_queue
        self._handle: Optional[epos_handle] = None
        self._motors: Dict[MotorId, EposMotorFacade] = {}

    @property
    def handle(self) -> Optional[epos_handle]:
        return self._handle

    @property
    def motors(self) -> Dict[MotorId, EposMotorFacade]:
        return self._motors

    @property
    def command_queue(self) -> "Queue[MotorCommand]":
        return self._command_queue

    @command_queue.setter
    def command_queue(self, queue: "Queue[MotorCommand]") -> None:
        self._command_queue = queue
        for motor in self._motors.values():
            motor.command_queue = queue

    def __getitem__(self, motor_id: MotorId) -> EposMotorFacade:
        return self.get_motor(motor_id)

    def init_device(self, config: EposDeviceConfig) -> epos_handle:
        """Opens CANopen communication channel and configures protocol stack."""
        self._config = config
        print(f"Opening communication channel to {config.device.name} devices...", flush=True)
        handle = cmd.open_device(
            device=config.device,
            protocol=config.protocol,
            interface=config.interface,
            port=config.port,
        )
        cmd.set_protocol_stack_settings(handle, config.baudrate, config.timeout_ms)
        self._handle = handle
        print(f"EPOS CAN driver connected successfully with handle: {handle}", flush=True)
        return handle

    def get_motor(self, motor_id: MotorId) -> EposMotorFacade:
        """Retrieves or registers an EposMotorFacade for the specified motor node."""
        if motor_id not in self._motors:
            if self._handle is None:
                raise RuntimeError("Cannot register motor: EPOS device channel not initialized.")
            motor_facade = EposMotorFacade(
                handle=self._handle,
                motor_id=motor_id,
                recovery_config=self._recovery_config,
                command_queue=self._command_queue,
            )
            self._motors[motor_id] = motor_facade
        return self._motors[motor_id]

    def connect(self, motor_id: MotorId):
        """Connects and clears active faults on the specified motor node."""
        motor = self.get_motor(motor_id)
        motor.connect()
        if cmd.is_fault(self._handle, motor_id):
            print(f"Active fault detected on Node {motor_id.name}. Clearing fault...", flush=True)
            cmd.clear_fault(self._handle, motor_id)
            time.sleep(0.05)

        start_pos = cmd.get_position(self._handle, motor_id)
        start_curr = cmd.get_current(self._handle, motor_id)
        print(
            f"[{motor_id.name}] Initial State: Position = {start_pos} QC, Current = {start_curr} mA",
            flush=True,
        )

    def enable(self, motor_id: MotorId):
        """Enables the power stage of the specified motor."""
        motor = self.get_motor(motor_id)
        motor.set_position_mode()

    def set_position_mode(self, motor_id: MotorId):
        """Switches the specified motor into Position Mode."""
        self.get_motor(motor_id).set_position_mode()

    def set_current_mode(self, motor_id: MotorId):
        """Switches the specified motor into Current / Torque Mode."""
        self.get_motor(motor_id).set_current_mode()

    def set_velocity_mode(self, motor_id: MotorId):
        """Switches the specified motor into Velocity Mode."""
        self.get_motor(motor_id).set_velocity_mode()

    def disable(self, motor_id: MotorId):
        """Disables the power stage of the specified motor."""
        motor = self.get_motor(motor_id)
        motor.set_disabled()

    def quick_stop(self, motor_id: MotorId):
        """Commands a quick stop deceleration on the specified motor."""
        motor = self.get_motor(motor_id)
        motor.quick_stop()

    def set_target_position(self, motor_id: MotorId, position: int) -> bool:
        """Commands target position to the specified motor."""
        return self.get_motor(motor_id).set_target_position(position)

    def set_target_current(self, motor_id: MotorId, current_ma: int) -> bool:
        """Commands target current to the specified motor."""
        return self.get_motor(motor_id).set_target_current(current_ma)

    def set_target_velocity(self, motor_id: MotorId, velocity_rpm: int) -> bool:
        """Commands target velocity to the specified motor."""
        return self.get_motor(motor_id).set_target_velocity(velocity_rpm)

    def get_motor_data(self, motor_id: MotorId) -> ServoMotorData:
        """Fetches telemetry for the specified motor."""
        return self.get_motor(motor_id).get_motor_data()

    def start_homing(self, motor_id: MotorId, config: HomingConfig):
        """Initiates homing calibration on the specified motor."""
        self.get_motor(motor_id).start_homing(config)

    def stop_homing(self, motor_id: MotorId):
        """Halts homing calibration on the specified motor."""
        self.get_motor(motor_id).stop_homing()

    def get_homing_state(self, motor_id: MotorId) -> Tuple[bool, bool]:
        """Returns (is_homing_attained, is_homing_error)."""
        return self.get_motor(motor_id).get_homing_state()

    def wait_for_homing(self, motor_id: MotorId, timeout_ms: int = 30_000) -> bool:
        """Waits for homing to complete on the specified motor."""
        return self.get_motor(motor_id).wait_for_homing(timeout_ms)

    def enable_all(self):
        """Enables all registered motors."""
        for motor in self._motors.values():
            motor.set_position_mode()

    def disable_all(self):
        """Disables all registered motors."""
        for motor in self._motors.values():
            motor.set_disabled()

    def quick_stop_all(self):
        """Triggers quick stop on all registered motors."""
        for motor in self._motors.values():
            motor.quick_stop()

    def shutdown(self, motor_ids: Optional[List[MotorId]] = None):
        """Disables motor power stages and closes the CAN device channel."""
        print("Disabling motor power stage and closing communication...", flush=True)
        targets = [self.get_motor(m) for m in motor_ids] if motor_ids else list(self._motors.values())
        for motor in targets:
            try:
                motor.set_disabled()
            except Exception as e:
                print(f"Warning: Failed to disable {motor.motor_id.name}: {e}", flush=True)

        if self._handle:
            try:
                cmd.close_device(self._handle)
                print("EPOS device closed safely.", flush=True)
            except Exception as e:
                print(f"Warning: Failed to close device: {e}", flush=True)
            self._handle = None
