"""
Unit tests for EposMotorFacade and EposFacade object-oriented state machine and fault recovery logic.
"""

import struct
import unittest
from multiprocessing import Queue
from unittest.mock import patch

from hermes.aidwear.prosthesis.motor_control.epos_facade import (
    EposMotorFacade,
    EposFacade,
)
from hermes.aidwear.prosthesis.motor_control.types import (
    EposDeviceConfig,
    EposRecoveryConfig,
    RecoveryStrategy,
)
from hermes.aidwear.prosthesis.utils.types import (
    MotorId,
    ServoMotorData,
    MotorCommand,
    ServoCanPacketEnum,
)


class TestEposMotorFacade(unittest.TestCase):
    def setUp(self):
        self.mock_handle = 0x12345678
        self.motor_id = MotorId.ANKLE
        self.recovery_config = EposRecoveryConfig(
            max_retries=-1,  # Indefinite retry policy
            retry_delay_s=0.001,
            strategy=RecoveryStrategy.BUMPLESS_HOLD,
            auto_recover=True,
        )

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_disable_state")
    def test_initial_state_and_connect(self, mock_disable):
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=self.recovery_config,
        )
        self.assertEqual(facade.current_state, facade.uninitialized)

        facade.connect()
        self.assertEqual(facade.current_state, facade.disabled)
        mock_disable.assert_called_with(self.mock_handle, self.motor_id)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_enable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_position_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.pm_set_position_must")
    def test_position_mode_and_setpoints(self, mock_set_pos, mock_act_pos, mock_enable):
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=self.recovery_config,
        )
        facade.connect()
        facade.set_position_mode()

        self.assertEqual(facade.current_state, facade.position_mode)
        mock_enable.assert_called_with(self.mock_handle, self.motor_id)
        mock_act_pos.assert_called_with(self.mock_handle, self.motor_id)

        # Send setpoint
        res = facade.set_target_position(1000)
        self.assertTrue(res)
        mock_set_pos.assert_called_with(self.mock_handle, self.motor_id, 1000)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_enable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_current_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.cm_set_current_must")
    def test_current_mode_and_setpoints(self, mock_set_curr, mock_act_curr, mock_enable):
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=self.recovery_config,
        )
        facade.connect()
        facade.set_current_mode()

        self.assertEqual(facade.current_state, facade.current_mode)
        mock_act_curr.assert_called_with(self.mock_handle, self.motor_id)

        res = facade.set_target_current(500)
        self.assertTrue(res)
        mock_set_curr.assert_called_with(self.mock_handle, self.motor_id, 500)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_enable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_position_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.pm_set_position_must")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_current_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.cm_set_current_must")
    def test_automatic_mode_switching_between_position_and_current(
        self,
        mock_set_curr,
        mock_act_curr,
        mock_set_pos,
        mock_act_pos,
        mock_enable,
    ):
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=self.recovery_config,
        )
        facade.connect()

        # 1. Drive is commanded in position mode
        res1 = facade.set_target_position(1000)
        self.assertTrue(res1)
        self.assertEqual(facade.current_state, facade.position_mode)
        mock_act_pos.assert_called_once_with(self.mock_handle, self.motor_id)
        mock_set_pos.assert_called_with(self.mock_handle, self.motor_id, 1000)

        # 2. Activity switches and upstream emits set_target_current()
        # Facade should automatically detect mode mismatch, activate current mode, and apply setpoint!
        res2 = facade.set_target_current(500)
        self.assertTrue(res2)
        self.assertEqual(facade.current_state, facade.current_mode)
        mock_act_curr.assert_called_once_with(self.mock_handle, self.motor_id)
        mock_set_curr.assert_called_with(self.mock_handle, self.motor_id, 500)

        # 3. Subsequent current command stays in current mode without re-activating
        res3 = facade.set_target_current(600)
        self.assertTrue(res3)
        self.assertEqual(facade.current_state, facade.current_mode)
        self.assertEqual(mock_act_curr.call_count, 1)
        mock_set_curr.assert_called_with(self.mock_handle, self.motor_id, 600)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_enable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_position_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.clear_fault")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.is_fault")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.get_position", return_value=1200)
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.pm_set_position_must")
    def test_automatic_fault_detection_and_recovery(
        self,
        mock_set_pos,
        mock_get_pos,
        mock_is_fault,
        mock_clear_fault,
        mock_act_pos,
        mock_enable,
    ):
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=self.recovery_config,
        )
        facade.connect()
        facade.set_position_mode()
        self.assertEqual(facade.current_state, facade.position_mode)

        # Simulate fault check during recovery: first clear_fault is called, then is_fault returns False
        mock_is_fault.return_value = False

        # Trigger fault
        facade.fault_detected()

        # Since auto_recover=True, on_enter_faulted triggers start_recovery,
        # which calls _execute_recovery_routine, clears fault, re-enables, and restores position_mode!
        self.assertEqual(facade.current_state, facade.position_mode)
        mock_clear_fault.assert_called_with(self.mock_handle, self.motor_id)
        mock_enable.assert_called_with(self.mock_handle, self.motor_id)
        mock_act_pos.assert_called_with(self.mock_handle, self.motor_id)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.is_fault", return_value=True)
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.clear_fault")
    def test_telemetry_polling_with_fault(self, mock_clear_fault, mock_is_fault):
        cfg = EposRecoveryConfig(auto_recover=False)
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=cfg,
        )
        facade.connect()

        # Polling telemetry when drive is in fault should return error=True without crashing
        data = facade.get_motor_data()
        self.assertIsInstance(data, ServoMotorData)
        self.assertTrue(data.error)
        self.assertEqual(facade.current_state, facade.faulted)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_enable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_position_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.clear_fault")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.is_fault", return_value=False)
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.get_position", return_value=500)
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.pm_set_position_must")
    def test_setpoint_buffering_during_recovery(
        self,
        mock_set_pos,
        mock_get_pos,
        mock_is_fault,
        mock_clear_fault,
        mock_act_pos,
        mock_enable,
    ):
        # Configure latest_target recovery
        cfg = EposRecoveryConfig(
            strategy=RecoveryStrategy.LATEST_TARGET,
            auto_recover=False,  # manually control recovery steps
        )
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=cfg,
        )
        facade.connect()
        facade.set_position_mode()

        # Trigger fault
        facade.fault_detected()
        self.assertEqual(facade.current_state, facade.faulted)

        # Upstream sends continuous target setpoints while motor is faulted!
        res1 = facade.set_target_position(2000)
        self.assertFalse(res1)  # Buffered, not immediately sent
        self.assertEqual(facade._pending_target_position, 2000)

        res2 = facade.set_target_position(2500)
        self.assertFalse(res2)
        self.assertEqual(facade._pending_target_position, 2500)

        # Now start recovery
        facade.start_recovery()
        # Recovery routine executes and applies latest buffered setpoint (2500)
        self.assertEqual(facade.current_state, facade.position_mode)
        mock_set_pos.assert_called_with(self.mock_handle, self.motor_id, 2500)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_enable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_position_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.pm_set_position_must")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_current_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.cm_set_current_must")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_velocity_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.vm_set_velocity_must")
    def test_command_queue_recording(
        self,
        mock_vm_set,
        mock_vm_act,
        mock_cm_set,
        mock_cm_act,
        mock_pm_set,
        mock_pm_act,
        mock_enable,
    ):
        cmd_queue = Queue()
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=self.recovery_config,
            command_queue=cmd_queue,
        )
        facade.connect()

        # Command position
        facade.set_target_position(12345)
        self.assertFalse(cmd_queue.empty())
        cmd1: MotorCommand = cmd_queue.get_nowait()
        self.assertEqual(cmd1.motor_id, self.motor_id)
        self.assertEqual(cmd1.control_mode, ServoCanPacketEnum.POSITION_MODE.value)
        self.assertEqual(struct.unpack("<q", cmd1.command_data)[0], 12345)
        self.assertEqual(len(cmd1.command_data), 8)
        self.assertEqual(len(cmd1.log_data), 20)
        self.assertTrue(cmd1.log_data.startswith(b"pos:12345"))

        # Command current
        facade.set_target_current(850)
        self.assertFalse(cmd_queue.empty())
        cmd2: MotorCommand = cmd_queue.get_nowait()
        self.assertEqual(cmd2.motor_id, self.motor_id)
        self.assertEqual(cmd2.control_mode, ServoCanPacketEnum.CURRENT_LOOP_MODE.value)
        self.assertEqual(struct.unpack("<q", cmd2.command_data)[0], 850)
        self.assertEqual(len(cmd2.command_data), 8)
        self.assertEqual(len(cmd2.log_data), 20)
        self.assertTrue(cmd2.log_data.startswith(b"cur:850"))

        # Command velocity
        facade.set_target_velocity(-1500)
        self.assertFalse(cmd_queue.empty())
        cmd3: MotorCommand = cmd_queue.get_nowait()
        self.assertEqual(cmd3.motor_id, self.motor_id)
        self.assertEqual(cmd3.control_mode, ServoCanPacketEnum.VELOCITY_MODE.value)
        self.assertEqual(struct.unpack("<q", cmd3.command_data)[0], -1500)
        self.assertEqual(len(cmd3.command_data), 8)
        self.assertEqual(len(cmd3.log_data), 20)
        self.assertTrue(cmd3.log_data.startswith(b"vel:-1500"))

    def test_command_recording_during_fault_and_queue_safety(self):
        cmd_queue = Queue()
        facade = EposMotorFacade(
            handle=self.mock_handle,
            motor_id=self.motor_id,
            recovery_config=self.recovery_config,
            command_queue=cmd_queue,
        )
        facade.connect()
        facade.fault_detected()
        self.assertEqual(facade.current_state, facade.faulted)

        # Commands should still be recorded to command_queue for upstream pipeline logging
        facade.set_target_position(9999)
        self.assertFalse(cmd_queue.empty())
        cmd = cmd_queue.get_nowait()
        self.assertEqual(struct.unpack("<q", cmd.command_data)[0], 9999)

        # Test safety if queue.put_nowait raises (e.g. closed or full)
        class FaultyQueue:
            def put_nowait(self, item):
                raise RuntimeError("Queue overflow")

        facade.command_queue = FaultyQueue()
        # Should not raise exception
        res = facade.set_target_position(8888)
        self.assertFalse(res)


class TestEposFacadeCoordinator(unittest.TestCase):
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.open_device", return_value=0x9999)
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_protocol_stack_settings")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_disable_state")
    def test_facade_coordinator_and_multi_motor(self, mock_disable, mock_stack, mock_open):
        facade = EposFacade()
        dev_cfg = EposDeviceConfig()
        handle = facade.init_device(dev_cfg)

        self.assertEqual(handle, 0x9999)
        self.assertEqual(facade.handle, 0x9999)

        # Register Ankle and Knee
        ankle = facade[MotorId.ANKLE]
        knee = facade[MotorId.KNEE]

        self.assertIsInstance(ankle, EposMotorFacade)
        self.assertIsInstance(knee, EposMotorFacade)
        self.assertEqual(ankle.motor_id, MotorId.ANKLE)
        self.assertEqual(knee.motor_id, MotorId.KNEE)

        # Broadcast disable
        facade.disable_all()
        self.assertEqual(ankle.current_state, ankle.disabled)
        self.assertEqual(knee.current_state, knee.disabled)

    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.open_device", return_value=0x9999)
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_protocol_stack_settings")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_disable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.set_enable_state")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.activate_position_mode")
    @patch("hermes.aidwear.prosthesis.motor_control.epos_commands.pm_set_position_must")
    def test_facade_coordinator_command_queue_propagation(
        self, mock_pm_set, mock_pm_act, mock_enable, mock_disable, mock_stack, mock_open
    ):
        cmd_queue = Queue()
        facade = EposFacade(command_queue=cmd_queue)
        facade.init_device(EposDeviceConfig())

        # Motor registered from facade receives command_queue
        ankle = facade.get_motor(MotorId.ANKLE)
        self.assertEqual(ankle.command_queue, cmd_queue)

        # Delegated command through facade records into queue
        facade.set_target_position(MotorId.ANKLE, 4321)
        self.assertFalse(cmd_queue.empty())
        item = cmd_queue.get_nowait()
        self.assertEqual(item.motor_id, MotorId.ANKLE)
        self.assertEqual(struct.unpack("<q", item.command_data)[0], 4321)

        # Updating command_queue dynamically propagates to motors
        new_queue = Queue()
        facade.command_queue = new_queue
        self.assertEqual(ankle.command_queue, new_queue)


if __name__ == "__main__":
    unittest.main()
