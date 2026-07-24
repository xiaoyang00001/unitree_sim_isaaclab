from __future__ import annotations

import time
import unittest

import numpy as np

try:
    from dds.g1_robot_dds import G1RobotDDS
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__IMUState_,
        unitree_hg_msg_dds__LowCmd_,
        unitree_hg_msg_dds__LowState_,
    )
except ModuleNotFoundError as import_error:
    G1RobotDDS = None
    UNITREE_IMPORT_ERROR = import_error
else:
    UNITREE_IMPORT_ERROR = None


class _FakeSharedMemory:
    def __init__(self, payload):
        self.payload = payload

    def read_data(self):
        return self.payload

    def write_data(self, payload):
        self.payload = payload


class _FakePublisher:
    def Write(self, message):
        del message


class _FakeCRC:
    def Crc(self, message):
        del message
        return 0


@unittest.skipIf(G1RobotDDS is None, f"Unitree SDK unavailable: {UNITREE_IMPORT_ERROR}")
class G1RobotDDSResetGraceTest(unittest.TestCase):
    def setUp(self) -> None:
        imu = np.array(
            [
                0.0,
                0.0,
                0.76,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                9.81,
                0.0,
                0.0,
                0.0,
            ],
            dtype=np.float32,
        )
        self.payload = {
            "joint_positions": np.linspace(-0.2, 0.2, 29, dtype=np.float32),
            "joint_velocities": np.full(29, 42.0, dtype=np.float32),
            "joint_torques": np.full(29, 7.0, dtype=np.float32),
            "base_imu_data": imu,
            "torso_imu_data": imu,
            "sample_time_monotonic": time.monotonic(),
            "sample_seq": 17,
        }

        self.dds = G1RobotDDS.__new__(G1RobotDDS)
        self.dds.node_name = "test_g1"
        self.dds.low_state = unitree_hg_msg_dds__LowState_()
        self.dds.torso_imu_state = unitree_hg_msg_dds__IMUState_()
        self.dds.input_shm = _FakeSharedMemory(self.payload)
        self.dds.output_shm = _FakeSharedMemory(None)
        self.dds.publisher = _FakePublisher()
        self.dds.torso_imu_publisher = _FakePublisher()
        self.dds.crc = _FakeCRC()
        self.dds._stats_window_start = time.monotonic()
        self.dds._lowstate_publish_count = 0
        self.dds._torso_imu_publish_count = 0
        self.dds._fresh_sample_count = 0
        self.dds._repeated_sample_publish_count = 0
        self.dds._last_sample_seq = None
        self.dds._lowcmd_packet_count = 0
        self.dds._lowcmd_content_update_count = 0
        self.dds._last_lowcmd_signature = None
        self.dds._last_sample_age_ms = 0.0
        self.dds._reset_state_grace_until = 0.0
        self.dds._reset_state_grace_active = False
        self.dds._reset_state_grace_samples = 0
        self.dds._reset_state_grace_max_abs_dq = 0.0
        self.dds._report_publish_stats = lambda: None

    def test_reset_grace_zeroes_only_dq_and_tau_then_restores_live_values(self) -> None:
        self.dds.begin_reset_state_grace(1.0, "test reset")
        self.dds.dds_publisher()

        for expected_q, motor in zip(
            self.payload["joint_positions"],
            self.dds.low_state.motor_state[:29],
        ):
            self.assertAlmostEqual(motor.q, float(expected_q))
            self.assertAlmostEqual(motor.dq, 0.0)
            self.assertAlmostEqual(motor.tau_est, 0.0)
        self.assertEqual(list(self.dds.low_state.imu_state.quaternion), [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(list(self.dds.torso_imu_state.quaternion), [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(self.dds._reset_state_grace_max_abs_dq, 42.0)
        self.assertEqual(self.dds._lowstate_publish_count, 1)
        self.assertEqual(self.dds._torso_imu_publish_count, 1)
        self.assertEqual(self.dds.low_state.tick, 17)
        self.assertEqual(self.dds._fresh_sample_count, 1)
        self.assertEqual(self.dds._repeated_sample_publish_count, 0)

        self.dds._reset_state_grace_until = time.monotonic() - 1.0
        self.payload["joint_velocities"] = np.full(29, 2.0, dtype=np.float32)
        self.payload["joint_torques"] = np.full(29, 3.0, dtype=np.float32)
        self.dds.dds_publisher()

        for motor in self.dds.low_state.motor_state[:29]:
            self.assertAlmostEqual(motor.dq, 2.0)
            self.assertAlmostEqual(motor.tau_est, 3.0)
        self.assertFalse(self.dds._reset_state_grace_active)
        self.assertEqual(self.dds._lowstate_publish_count, 2)
        self.assertEqual(self.dds._torso_imu_publish_count, 2)
        self.assertEqual(self.dds.low_state.tick, 17)
        self.assertEqual(self.dds._fresh_sample_count, 1)
        self.assertEqual(self.dds._repeated_sample_publish_count, 1)

    def test_reset_grace_can_be_rearmed_after_a_slow_reset(self) -> None:
        self.dds.begin_reset_state_grace(0.01, "pre-reset")
        pre_reset_deadline = self.dds._reset_state_grace_until

        self.dds.begin_reset_state_grace(1.0, "post-reset")

        self.assertGreater(self.dds._reset_state_grace_until, pre_reset_deadline + 0.9)
        self.dds.dds_publisher()
        for motor in self.dds.low_state.motor_state[:29]:
            self.assertAlmostEqual(motor.dq, 0.0)
            self.assertAlmostEqual(motor.tau_est, 0.0)

    def test_lowcmd_subscriber_preserves_complete_motor_command(self) -> None:
        message = unitree_hg_msg_dds__LowCmd_()
        message.mode_pr = 3
        message.mode_machine = 0xA2
        message.crc = 0
        for index, motor in enumerate(message.motor_cmd):
            motor.mode = index % 2
            motor.q = 0.1 * index
            motor.dq = 0.2 * index
            motor.tau = 0.3 * index
            motor.kp = 10.0 + index
            motor.kd = 1.0 + index

        self.dds.dds_subscriber(message)
        command = self.dds.output_shm.payload
        self.assertIsNotNone(command)
        self.assertEqual(command["mode_pr"], 3)
        self.assertEqual(command["mode_machine"], 0xA2)
        self.assertIn("receive_time_monotonic", command)
        motor_cmd = command["motor_cmd"]
        self.assertEqual(len(motor_cmd["modes"]), len(message.motor_cmd))
        for index in range(len(message.motor_cmd)):
            self.assertEqual(motor_cmd["modes"][index], index % 2)
            self.assertAlmostEqual(motor_cmd["positions"][index], 0.1 * index, places=6)
            self.assertAlmostEqual(motor_cmd["velocities"][index], 0.2 * index, places=6)
            self.assertAlmostEqual(motor_cmd["torques"][index], 0.3 * index, places=6)
            self.assertAlmostEqual(motor_cmd["kp"][index], 10.0 + index, places=6)
            self.assertAlmostEqual(motor_cmd["kd"][index], 1.0 + index, places=6)


if __name__ == "__main__":
    unittest.main()
