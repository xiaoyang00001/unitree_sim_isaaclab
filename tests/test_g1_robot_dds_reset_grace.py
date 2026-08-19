from __future__ import annotations

import contextlib
import io
import threading
import time
import unittest

import numpy as np

try:
    from dds.g1_robot_dds import (
        ISAAC_LOWSTATE_SYNC_MAGIC,
        SONIC_LOWCMD_SYNC_MAGIC,
        G1RobotDDS,
    )
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
        self.write_count = 0
        self.read_count = 0
        self.fail_writes = 0

    def read_data(self):
        self.read_count += 1
        return self.payload

    def write_data(self, payload):
        self.write_count += 1
        if self.fail_writes > 0:
            self.fail_writes -= 1
            return False
        self.payload = payload
        return True


class _FakePublisher:
    def __init__(self, label=None, write_log=None):
        self.label = label
        self.write_log = write_log

    def Write(self, message):
        del message
        if self.write_log is not None:
            self.write_log.append(self.label)


class _FakeCRC:
    def __init__(self):
        self.call_count = 0

    def Crc(self, message):
        self.call_count += 1
        return int(message.crc)


class _FailOnceCRC(_FakeCRC):
    def Crc(self, message):
        self.call_count += 1
        if self.call_count == 1:
            raise RuntimeError("injected CRC failure")
        return int(message.crc)


class _CapturingLowStatePublisher:
    def __init__(self):
        self.snapshots = []

    def Write(self, message):
        self.snapshots.append((int(message.tick), float(message.motor_state[0].q)))


class _ObservedTransactionLock:
    """Expose the watched thread's pre-acquire point to a deterministic test."""

    def __init__(self, watched_thread_name, acquire_barrier):
        self._lock = threading.Lock()
        self._watched_thread_name = watched_thread_name
        self._acquire_barrier = acquire_barrier

    def __enter__(self):
        if threading.current_thread().name == self._watched_thread_name:
            self._acquire_barrier.wait(timeout=2.0)
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        del exc_type, exc_value, traceback
        self._lock.release()


class _BlockingPublisher(_FakePublisher):
    def __init__(self, entered_barrier, release_barrier, label=None, write_log=None):
        super().__init__(label, write_log)
        self._entered_barrier = entered_barrier
        self._release_barrier = release_barrier
        self._blocked_once = False

    def Write(self, message):
        if not self._blocked_once:
            self._blocked_once = True
            self._entered_barrier.wait(timeout=2.0)
            self._release_barrier.wait(timeout=2.0)
        super().Write(message)


class _GraceBoundaryLowStatePublisher:
    def __init__(self, grace_returned):
        self._grace_returned = grace_returned
        self.snapshots = []

    def Write(self, message):
        self.snapshots.append(
            (
                self._grace_returned.is_set(),
                float(message.motor_state[0].dq),
                float(message.motor_state[0].tau_est),
            )
        )


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
        self.publish_order = []
        self.dds.publisher = _FakePublisher("lowstate", self.publish_order)
        self.dds.torso_imu_publisher = _FakePublisher("torso_imu", self.publish_order)
        self.dds.crc = _FakeCRC()
        self.dds._stats_window_start = time.monotonic()
        self.dds._lowstate_publish_count = 0
        self.dds._torso_imu_publish_count = 0
        self.dds._fresh_sample_count = 0
        self.dds._repeated_sample_publish_count = 0
        self.dds._lowstate_message_update_count = 0
        self.dds._lowstate_shm_read_count = 0
        self.dds._last_sample_seq = None
        self.dds._publish_transaction_lock = threading.Lock()
        self.dds._robot_state_lock = threading.RLock()
        self.dds._latest_robot_state = None
        self.dds._robot_state_generation = 0
        self.dds._applied_robot_state_key = None
        self.dds._lowstate_have_torso_imu = False
        self.dds._reset_epoch = 0
        self.dds._latest_written_reset_epoch = 0
        self.dds._last_published_reset_epoch = 0
        self.dds._lowcmd_packet_count = 0
        self.dds._lowcmd_content_update_count = 0
        self.dds._lowcmd_duplicate_fastpath_count = 0
        self.dds._lowcmd_shm_write_count = 0
        self.dds._last_lowcmd_signature = None
        self.dds._last_lowcmd_wire_fingerprint = None
        self.dds._latest_robot_command = None
        self.dds._last_lowcmd_receive_time = None
        self.dds._last_lowcmd_ack_tick = None
        self.dds._last_lowcmd_ack_reset_epoch = None
        self.dds._lowcmd_ack_match_count = 0
        self.dds._lowcmd_ack_stale_count = 0
        self.dds._last_sample_age_ms = 0.0
        self.dds._reset_state_grace_until = 0.0
        self.dds._reset_state_grace_active = False
        self.dds._reset_state_grace_samples = 0
        self.dds._reset_state_grace_max_abs_dq = 0.0
        self.dds._report_publish_stats = lambda: None

    def _write_cached_state(
        self,
        *,
        sample_seq=18,
        position=0.25,
        velocity=2.0,
        torque=3.0,
        sim_time_s=0.36,
    ) -> None:
        self.dds.write_robot_state(
            [position] * 29,
            [velocity] * 29,
            [torque] * 29,
            base_imu_data=self.payload["base_imu_data"],
            torso_imu_data=self.payload["torso_imu_data"],
            sample_seq=sample_seq,
            sim_time_s=sim_time_s,
        )

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
        self.assertEqual(self.publish_order[-2:], ["torso_imu", "lowstate"])
        self.assertEqual(self.dds.low_state.tick, 17)
        self.assertEqual(self.dds.low_state.reserve[0], 0)
        self.assertEqual(self.dds.low_state.reserve[1], 1)
        self.assertEqual(self.dds.low_state.reserve[3], ISAAC_LOWSTATE_SYNC_MAGIC)
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

    def test_cached_generation_reuses_idl_and_crc_but_keeps_publish_cadence(self) -> None:
        self._write_cached_state(sample_seq=18)

        self.dds.dds_publisher()
        self.dds.dds_publisher()

        self.assertEqual(self.dds.input_shm.read_count, 0)
        self.assertEqual(self.dds.crc.call_count, 1)
        self.assertEqual(self.dds._lowstate_message_update_count, 1)
        self.assertEqual(self.dds._lowstate_shm_read_count, 0)
        self.assertEqual(self.dds._fresh_sample_count, 1)
        self.assertEqual(self.dds._repeated_sample_publish_count, 1)
        self.assertEqual(
            self.publish_order,
            ["torso_imu", "lowstate", "torso_imu", "lowstate"],
        )

    def test_new_generation_with_same_sample_seq_rebuilds_message(self) -> None:
        self._write_cached_state(sample_seq=23, position=0.25)
        self.dds.dds_publisher()
        first_key = self.dds._applied_robot_state_key

        self._write_cached_state(sample_seq=23, position=0.75)
        self.dds.dds_publisher()

        self.assertNotEqual(self.dds._applied_robot_state_key, first_key)
        self.assertEqual(self.dds.crc.call_count, 2)
        self.assertEqual(self.dds._lowstate_message_update_count, 2)
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].q, 0.75)
        self.assertEqual(self.dds._fresh_sample_count, 1)
        self.assertEqual(self.dds._repeated_sample_publish_count, 1)

    def test_reset_grace_entry_and_exit_rebuild_same_generation(self) -> None:
        self._write_cached_state(sample_seq=31, velocity=8.0, torque=4.0)
        self.dds.dds_publisher()
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].dq, 8.0)

        self.dds.begin_reset_state_grace(1.0, "cached reset")
        self.dds.dds_publisher()
        self.dds.dds_publisher()
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].dq, 0.0)
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].tau_est, 0.0)
        self.assertEqual(self.dds.crc.call_count, 2)

        with self.dds._robot_state_lock:
            self.dds._reset_state_grace_until = time.monotonic() - 1.0
        self.dds.dds_publisher()

        self.assertAlmostEqual(self.dds.low_state.motor_state[0].dq, 8.0)
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].tau_est, 4.0)
        self.assertEqual(self.dds.crc.call_count, 3)
        self.assertEqual(self.dds._lowstate_message_update_count, 3)
        self.assertEqual(self.dds._applied_robot_state_key, (1, False))
        self.assertEqual(
            self.publish_order,
            ["torso_imu", "lowstate"] * 4,
        )

    def test_reset_grace_return_waits_for_inflight_publish_transaction(self) -> None:
        self._write_cached_state(sample_seq=37, velocity=8.0, torque=4.0)
        torso_entered = threading.Barrier(2)
        release_torso = threading.Barrier(2)
        grace_lock_attempted = threading.Barrier(2)
        grace_returned = threading.Event()
        watched_thread_name = "begin-reset-grace"
        self.dds._publish_transaction_lock = _ObservedTransactionLock(
            watched_thread_name, grace_lock_attempted
        )
        self.dds.torso_imu_publisher = _BlockingPublisher(
            torso_entered,
            release_torso,
            "torso_imu",
            self.publish_order,
        )
        lowstate_publisher = _GraceBoundaryLowStatePublisher(grace_returned)
        self.dds.publisher = lowstate_publisher

        publish_thread = threading.Thread(target=self.dds.dds_publisher)

        def begin_grace() -> None:
            self.dds.begin_reset_state_grace(1.0, "concurrent reset")
            grace_returned.set()

        grace_thread = threading.Thread(
            target=begin_grace,
            name=watched_thread_name,
        )
        publish_thread.start()
        torso_entered.wait(timeout=2.0)
        grace_thread.start()
        grace_lock_attempted.wait(timeout=2.0)

        self.assertFalse(grace_returned.is_set())
        release_torso.wait(timeout=2.0)
        publish_thread.join(timeout=2.0)
        grace_thread.join(timeout=2.0)
        self.assertFalse(publish_thread.is_alive())
        self.assertFalse(grace_thread.is_alive())
        self.assertTrue(grace_returned.is_set())

        self.dds.dds_publisher()

        self.assertEqual(
            lowstate_publisher.snapshots,
            [(False, 8.0, 4.0), (True, 0.0, 0.0)],
        )
        self.assertTrue(
            all(
                not returned or (dq == 0.0 and tau_est == 0.0)
                for returned, dq, tau_est in lowstate_publisher.snapshots
            )
        )

    def test_publisher_falls_back_to_shm_until_in_process_cache_exists(self) -> None:
        self.dds.dds_publisher()
        self.dds.dds_publisher()

        self.assertEqual(self.dds.input_shm.read_count, 2)
        self.assertEqual(self.dds._lowstate_shm_read_count, 2)
        self.assertEqual(self.dds.crc.call_count, 2)
        self.assertEqual(self.dds._lowstate_message_update_count, 2)

    def test_cache_deep_copies_inputs_and_survives_shm_write_failure(self) -> None:
        positions = [0.4] * 29
        velocities = [0.5] * 29
        torques = [0.6] * 29
        base_imu = self.payload["base_imu_data"].tolist()
        torso_imu = self.payload["torso_imu_data"].tolist()
        self.dds.input_shm.fail_writes = 1

        self.dds.write_robot_state(
            positions,
            velocities,
            torques,
            base_imu_data=base_imu,
            torso_imu_data=torso_imu,
            sample_seq=41,
            sim_time_s=0.82,
        )
        positions[0] = 99.0
        velocities[0] = 98.0
        torques[0] = 97.0
        base_imu[3] = 0.0
        torso_imu[3] = 0.0
        self.dds.dds_publisher()

        self.assertEqual(self.dds.input_shm.write_count, 1)
        self.assertEqual(self.dds.input_shm.read_count, 0)
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].q, 0.4)
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].dq, 0.5)
        self.assertAlmostEqual(self.dds.low_state.motor_state[0].tau_est, 0.6)
        self.assertEqual(list(self.dds.low_state.imu_state.quaternion), [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(self.dds.get_latest_written_state_info()["sample_seq"], 41)

    def test_legacy_none_sample_seq_rebuilds_and_advances_tick_each_cycle(self) -> None:
        self._write_cached_state(sample_seq=None)

        self.dds.dds_publisher()
        first_tick = int(self.dds.low_state.tick)
        self.dds.dds_publisher()

        self.assertEqual(first_tick, 1)
        self.assertEqual(self.dds.low_state.tick, 2)
        self.assertEqual(self.dds.crc.call_count, 2)
        self.assertEqual(self.dds._lowstate_message_update_count, 2)
        self.assertEqual(self.dds._fresh_sample_count, 2)
        self.assertEqual(self.dds._repeated_sample_publish_count, 0)
        self.assertIsNone(self.dds._applied_robot_state_key)

    def test_failed_message_update_does_not_advance_applied_generation(self) -> None:
        self._write_cached_state(sample_seq=51)
        self.dds.crc = _FailOnceCRC()

        self.dds.dds_publisher()
        self.assertIsNone(self.dds._applied_robot_state_key)
        self.assertEqual(self.dds._lowstate_message_update_count, 0)
        self.assertEqual(self.publish_order, [])

        self.dds.dds_publisher()
        self.assertEqual(self.dds._applied_robot_state_key, (1, False))
        self.assertEqual(self.dds.crc.call_count, 2)
        self.assertEqual(self.dds._lowstate_message_update_count, 1)
        self.assertEqual(self.publish_order, ["torso_imu", "lowstate"])

    def test_publish_stats_report_and_reset_message_updates_and_shm_reads(self) -> None:
        self.dds._stats_window_start = time.monotonic() - 10.0
        self.dds._lowstate_message_update_count = 7
        self.dds._lowstate_shm_read_count = 3
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            G1RobotDDS._report_publish_stats(self.dds)

        self.assertIn("message_updates=", output.getvalue())
        self.assertIn("shm_reads=", output.getvalue())
        self.assertEqual(self.dds._lowstate_message_update_count, 0)
        self.assertEqual(self.dds._lowstate_shm_read_count, 0)

    def test_concurrent_writer_and_publisher_keep_generation_snapshot_atomic(self) -> None:
        self._write_cached_state(sample_seq=1, position=1.0, sim_time_s=0.01)
        capturing_publisher = _CapturingLowStatePublisher()
        self.dds.publisher = capturing_publisher
        start = threading.Barrier(3)
        metadata_errors = []

        def write_states() -> None:
            start.wait()
            for sample_seq in range(2, 101):
                self._write_cached_state(
                    sample_seq=sample_seq,
                    position=float(sample_seq),
                    sim_time_s=sample_seq / 100.0,
                )
                time.sleep(0)

        def publish_states() -> None:
            start.wait()
            for _ in range(300):
                self.dds.dds_publisher()
                info = self.dds.get_latest_written_state_info()
                if info["sample_seq"] is not None and not np.isclose(
                    info["sim_time_s"], info["sample_seq"] / 100.0
                ):
                    metadata_errors.append(info)
                time.sleep(0)

        threads = [
            threading.Thread(target=write_states),
            threading.Thread(target=publish_states),
        ]
        for thread in threads:
            thread.start()
        start.wait()
        for thread in threads:
            thread.join(timeout=5.0)

        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertEqual(metadata_errors, [])
        self.assertGreater(len(capturing_publisher.snapshots), 0)
        for tick, first_motor_q in capturing_publisher.snapshots:
            self.assertAlmostEqual(first_motor_q, float(tick), places=5)

    def test_lowcmd_subscriber_preserves_complete_motor_command(self) -> None:
        message = unitree_hg_msg_dds__LowCmd_()
        message.mode_pr = 3
        message.mode_machine = 0xA2
        message.crc = 0
        message.reserve[0] = 17
        message.reserve[1] = 0
        message.reserve[2] = 123
        message.reserve[3] = SONIC_LOWCMD_SYNC_MAGIC
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
        self.assertEqual(command["ack_state_tick"], 17)
        self.assertEqual(command["ack_reset_epoch"], 0)
        self.assertEqual(command["control_step_count"], 123)
        self.assertEqual(command["sync_magic"], SONIC_LOWCMD_SYNC_MAGIC)
        motor_cmd = command["motor_cmd"]
        self.assertEqual(len(motor_cmd["modes"]), len(message.motor_cmd))
        for index in range(len(message.motor_cmd)):
            self.assertEqual(motor_cmd["modes"][index], index % 2)
            self.assertAlmostEqual(motor_cmd["positions"][index], 0.1 * index, places=6)
            self.assertAlmostEqual(motor_cmd["velocities"][index], 0.2 * index, places=6)
            self.assertAlmostEqual(motor_cmd["torques"][index], 0.3 * index, places=6)
            self.assertAlmostEqual(motor_cmd["kp"][index], 10.0 + index, places=6)
            self.assertAlmostEqual(motor_cmd["kd"][index], 1.0 + index, places=6)

    def test_lowcmd_duplicate_crc_uses_fastpath_and_refreshes_liveness(self) -> None:
        message = unitree_hg_msg_dds__LowCmd_()
        message.mode_machine = 0xA2
        message.crc = 0x12345678
        message.reserve[0] = 17
        message.reserve[1] = 0
        message.reserve[2] = 123
        message.reserve[3] = SONIC_LOWCMD_SYNC_MAGIC
        for index, motor in enumerate(message.motor_cmd):
            motor.mode = 1
            motor.q = 0.1 * index
            motor.kp = 10.0 + index
            motor.kd = 1.0 + index

        first_command = self.dds.dds_subscriber(message)
        first_receive_time = first_command["receive_time_monotonic"]
        self.assertEqual(self.dds.output_shm.write_count, 1)

        time.sleep(0.001)
        duplicate_command = self.dds.dds_subscriber(message)

        self.assertIs(duplicate_command, first_command)
        self.assertGreater(
            duplicate_command["receive_time_monotonic"], first_receive_time
        )
        self.assertEqual(self.dds._lowcmd_packet_count, 2)
        self.assertEqual(self.dds._lowcmd_content_update_count, 1)
        self.assertEqual(self.dds._lowcmd_duplicate_fastpath_count, 1)
        self.assertEqual(self.dds._lowcmd_shm_write_count, 1)
        self.assertEqual(self.dds.output_shm.write_count, 1)
        self.assertIs(self.dds.get_robot_command(), first_command)
        self.assertEqual(self.dds.output_shm.read_count, 0)

    def test_zero_crc_falls_back_to_full_content_comparison(self) -> None:
        message = unitree_hg_msg_dds__LowCmd_()
        message.crc = 0
        message.motor_cmd[0].mode = 1
        message.motor_cmd[0].q = 0.25
        self.dds.dds_subscriber(message)

        message.motor_cmd[0].q = 0.75
        command = self.dds.dds_subscriber(message)

        self.assertAlmostEqual(command["motor_cmd"]["positions"][0], 0.75)
        self.assertEqual(self.dds._lowcmd_duplicate_fastpath_count, 0)
        self.assertEqual(self.dds.output_shm.write_count, 2)

    def test_reset_epoch_is_published_only_with_the_newly_written_state(self) -> None:
        old_payload = dict(self.payload)
        old_payload["reset_epoch"] = 0
        self.dds.input_shm.payload = old_payload

        self.dds.begin_reset_epoch("test boundary")
        self.dds.dds_publisher()
        self.assertEqual(self.dds.low_state.reserve[0], 0)

        self.dds.write_robot_state(
            self.payload["joint_positions"],
            self.payload["joint_velocities"],
            self.payload["joint_torques"],
            base_imu_data=self.payload["base_imu_data"],
            torso_imu_data=self.payload["torso_imu_data"],
            sample_seq=18,
            sim_time_s=0.36,
        )
        self.dds.dds_publisher()

        self.assertEqual(self.dds.low_state.tick, 18)
        self.assertEqual(self.dds.low_state.reserve[0], 1)
        self.assertEqual(self.dds.get_latest_written_state_info()["reset_epoch"], 1)


if __name__ == "__main__":
    unittest.main()
