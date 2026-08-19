from __future__ import annotations

import importlib.util
import os
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import torch

os.environ.setdefault("UNITREE_DDS_DOMAIN", "91")
os.environ.setdefault("UNITREE_DDS_INTERFACE", "lo")

try:
    from action_provider.action_provider_sonic_dds import (
        DEX3_FALLBACK_LOWER_LIMITS,
        DEX3_FALLBACK_UPPER_LIMITS,
        SONIC_LOWCMD_SYNC_MAGIC,
        SonicDDSActionProvider,
    )
    from dds.dds_master import dds_manager
    from dds.dex3_dds import Dex3DDS
    from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER
    from robots.g1_sonic_urdf import DEX3_HAND_JOINT_NAMES
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
except ModuleNotFoundError as import_error:
    SonicDDSActionProvider = None
    IMPORT_ERROR = import_error
else:
    IMPORT_ERROR = None
    dex3_state_spec = importlib.util.spec_from_file_location(
        "dex3_state_test_module",
        Path(__file__).parents[1] / "tasks/common_observations/dex3_state.py",
    )
    dex3_state = importlib.util.module_from_spec(dex3_state_spec)
    dex3_state_spec.loader.exec_module(dex3_state)


class _FakeDDS:
    def __init__(self, command):
        self.command = command
        self.state_info = {
            "sample_seq": 1,
            "reset_epoch": 0,
        }

    def get_robot_command(self):
        return self.command

    def get_latest_written_state_info(self):
        return self.state_info


class _FakeDex3DDS:
    def __init__(self, commands):
        self.commands = commands

    def get_hand_commands(self):
        return self.commands


class _FakeDex3StateWriter:
    def __init__(self):
        self.last_write = None

    def write_hand_states(self, *values):
        self.last_write = values


class _FakeCommandSharedMemory:
    def __init__(self, payload=None):
        self.payload = payload
        self.read_count = 0
        self.write_count = 0
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


class _FakeActuator:
    def __init__(self, joint_indices: torch.Tensor, kp: torch.Tensor, kd: torch.Tensor):
        self.joint_indices = joint_indices
        self.stiffness = kp[:, joint_indices].clone()
        self.damping = kd[:, joint_indices].clone()


class _FakeRobot:
    def __init__(self, joint_names: list[str]):
        joint_count = len(joint_names)
        default_q = torch.linspace(-0.2, 0.2, joint_count).unsqueeze(0)
        for joint_index, joint_name in enumerate(joint_names):
            if joint_name.startswith(("left_hand_", "right_hand_")):
                default_q[0, joint_index] = 0.0
        default_dq = torch.zeros(1, joint_count)
        default_kp = torch.linspace(10.0, 20.0, joint_count).unsqueeze(0)
        default_kd = torch.linspace(1.0, 2.0, joint_count).unsqueeze(0)
        joint_pos_limits = torch.empty(1, joint_count, 2)
        joint_pos_limits[:, :, 0] = -10.0
        joint_pos_limits[:, :, 1] = 10.0
        joint_vel_limits = torch.full((1, joint_count), 100.0)
        joint_effort_limits = torch.full((1, joint_count), 200.0)
        for hand_index, hand_name in enumerate(DEX3_HAND_JOINT_NAMES):
            joint_index = joint_names.index(hand_name)
            joint_pos_limits[0, joint_index, 0] = DEX3_FALLBACK_LOWER_LIMITS[hand_index]
            joint_pos_limits[0, joint_index, 1] = DEX3_FALLBACK_UPPER_LIMITS[hand_index]
            joint_vel_limits[0, joint_index] = 3.14 if hand_name.endswith("thumb_0_joint") else 12.0
            joint_effort_limits[0, joint_index] = 2.45 if hand_name.endswith("thumb_0_joint") else 0.7

        self.data = SimpleNamespace(
            joint_names=joint_names,
            default_joint_pos=default_q,
            default_joint_vel=default_dq,
            default_joint_stiffness=default_kp,
            default_joint_damping=default_kd,
            default_root_state=torch.zeros(1, 13),
            joint_pos=default_q.clone(),
            joint_vel=default_dq.clone(),
            root_quat_w=torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
            root_pos_w=torch.tensor([[0.0, 0.0, 0.76]]),
            computed_torque=torch.zeros(1, joint_count),
            applied_torque=torch.zeros(1, joint_count),
            joint_pos_limits=joint_pos_limits,
            joint_vel_limits=joint_vel_limits,
            joint_effort_limits=joint_effort_limits,
        )
        split = joint_count // 2
        self.actuators = {
            "first": _FakeActuator(
                torch.arange(0, split, dtype=torch.long), default_kp, default_kd
            ),
            "second": _FakeActuator(
                torch.arange(split, joint_count, dtype=torch.long), default_kp, default_kd
            ),
        }
        self.written_kp = None
        self.written_kd = None
        self.written_root_state = None

    def write_joint_stiffness_to_sim(self, value):
        self.written_kp = value.clone()

    def write_joint_damping_to_sim(self, value):
        self.written_kd = value.clone()

    def write_root_state_to_sim(self, value):
        self.written_root_state = value.clone()


def _make_command(
    *,
    positions: list[float],
    velocities: list[float],
    torques: list[float],
    kp: list[float],
    kd: list[float],
    modes: list[int] | None = None,
    ack_state_tick: int | None = None,
    ack_reset_epoch: int | None = None,
):
    command = {
        "mode_machine": 0xA2,
        "receive_time_monotonic": time.monotonic(),
        "motor_cmd": {
            "modes": modes if modes is not None else [1] * 29,
            "positions": positions,
            "velocities": velocities,
            "torques": torques,
            "kp": kp,
            "kd": kd,
        },
    }
    if ack_state_tick is not None:
        command.update(
            {
                "ack_state_tick": ack_state_tick,
                "ack_reset_epoch": 0 if ack_reset_epoch is None else ack_reset_epoch,
                "control_step_count": 1,
                "sync_magic": SONIC_LOWCMD_SYNC_MAGIC,
            }
        )
    return command


def _make_hand_command(
    *,
    positions: list[float],
    velocities: list[float] | None = None,
    torques: list[float] | None = None,
    kp: list[float] | None = None,
    kd: list[float] | None = None,
    modes: list[int] | None = None,
    receive_time_monotonic: float | None = None,
):
    return {
        "modes": modes if modes is not None else [0x10 | index for index in range(7)],
        "positions": positions,
        "velocities": velocities if velocities is not None else [0.0] * 7,
        "torques": torques if torques is not None else [0.0] * 7,
        "kp": kp if kp is not None else [1.5] * 7,
        "kd": kd if kd is not None else [0.1] * 7,
        "receive_time_monotonic": (
            time.monotonic()
            if receive_time_monotonic is None
            else receive_time_monotonic
        ),
    }


@unittest.skipIf(
    SonicDDSActionProvider is None,
    f"SONIC DDS dependencies unavailable: {IMPORT_ERROR}",
)
class SonicDDSFullLowCmdTest(unittest.TestCase):
    def setUp(self) -> None:
        # Reverse the body order so the test proves explicit DDS-to-articulation
        # mapping rather than accidentally relying on identical array order.
        self.joint_names = list(reversed(G1_29DOF_DDS_JOINT_ORDER)) + list(
            DEX3_HAND_JOINT_NAMES
        )
        self.robot = _FakeRobot(self.joint_names)
        self.dds = _FakeDDS(
            _make_command(
                positions=[0.01 * index for index in range(29)],
                velocities=[1.0 + 0.01 * index for index in range(29)],
                torques=[2.0 + 0.01 * index for index in range(29)],
                kp=[30.0 + index for index in range(29)],
                kd=[3.0 + 0.1 * index for index in range(29)],
            )
        )
        self.left_hand_positions = [0.10, 0.10, 0.20, -0.10, -0.20, -0.10, -0.20]
        self.right_hand_positions = [-0.10, -0.10, -0.20, 0.10, 0.20, 0.10, 0.20]
        self.dex3_dds = _FakeDex3DDS(
            {
                "left_hand_cmd": _make_hand_command(
                    positions=self.left_hand_positions,
                    velocities=[0.20] * 7,
                    torques=[0.10] * 7,
                    kp=[1.50] * 7,
                    kd=[0.10] * 7,
                ),
                "right_hand_cmd": _make_hand_command(
                    positions=self.right_hand_positions,
                    velocities=[-0.20] * 7,
                    torques=[-0.10] * 7,
                    kp=[1.60] * 7,
                    kd=[0.12] * 7,
                ),
            }
        )
        action_manager = SimpleNamespace(
            active_terms=["joint_pos", "joint_vel", "joint_effort"],
            total_action_dim=3 * len(self.joint_names),
        )
        env = SimpleNamespace(
            scene={"robot": self.robot},
            device="cpu",
            num_envs=1,
            action_manager=action_manager,
            step_dt=0.02,
        )
        args = SimpleNamespace(
            sonic_lowcmd_timeout=0.1,
            sonic_ramp_seconds=0.0,
            sonic_max_target_step=0.0,
            sonic_handcmd_timeout=0.2,
            sonic_hand_max_target_error=0.25,
            stats_interval=1000.0,
        )
        with mock.patch.object(
            dds_manager,
            "get_object",
            side_effect=lambda name: {
                "g129": self.dds,
                "dex3": self.dex3_dds,
            }.get(name),
        ):
            self.provider = SonicDDSActionProvider(env, args)

    def _split_action(self, action: torch.Tensor):
        joint_count = len(self.joint_names)
        return (
            action[0, :joint_count],
            action[0, joint_count : 2 * joint_count],
            action[0, 2 * joint_count :],
        )

    def test_q_dq_tau_kp_kd_are_mapped_and_applied(self) -> None:
        action = self.provider.get_action(None)
        self.assertEqual(tuple(action.shape), (1, 3 * len(self.joint_names)))
        q_target, dq_target, tau_target = self._split_action(action)

        command = self.dds.command["motor_cmd"]
        for motor_index, joint_name in enumerate(G1_29DOF_DDS_JOINT_ORDER):
            joint_index = self.joint_names.index(joint_name)
            self.assertAlmostEqual(
                q_target[joint_index].item(), command["positions"][motor_index], places=6
            )
            self.assertAlmostEqual(
                dq_target[joint_index].item(), command["velocities"][motor_index], places=6
            )
            self.assertAlmostEqual(
                tau_target[joint_index].item(), command["torques"][motor_index], places=6
            )
            self.assertAlmostEqual(
                self.robot.written_kp[0, joint_index].item(), command["kp"][motor_index], places=6
            )
            self.assertAlmostEqual(
                self.robot.written_kd[0, joint_index].item(), command["kd"][motor_index], places=6
            )

        hand_positions = self.left_hand_positions + self.right_hand_positions
        hand_velocities = [0.20] * 7 + [-0.20] * 7
        hand_torques = [0.10] * 7 + [-0.10] * 7
        hand_kp = [1.50] * 7 + [1.60] * 7
        hand_kd = [0.10] * 7 + [0.12] * 7
        for hand_motor_index, hand_name in enumerate(DEX3_HAND_JOINT_NAMES):
            hand_index = self.joint_names.index(hand_name)
            self.assertAlmostEqual(
                q_target[hand_index].item(),
                hand_positions[hand_motor_index],
                places=6,
            )
            self.assertAlmostEqual(
                dq_target[hand_index].item(), hand_velocities[hand_motor_index], places=6
            )
            self.assertAlmostEqual(
                tau_target[hand_index].item(), hand_torques[hand_motor_index], places=6
            )
            self.assertAlmostEqual(
                self.robot.written_kp[0, hand_index].item(),
                hand_kp[hand_motor_index],
                places=6,
            )
            self.assertAlmostEqual(
                self.robot.written_kd[0, hand_index].item(),
                hand_kd[hand_motor_index],
                places=6,
            )

        for actuator in self.robot.actuators.values():
            self.assertTrue(
                torch.equal(
                    actuator.stiffness,
                    self.robot.written_kp[:, actuator.joint_indices],
                )
            )
            self.assertTrue(
                torch.equal(
                    actuator.damping,
                    self.robot.written_kd[:, actuator.joint_indices],
                )
            )

    def test_motor_disable_zeroes_that_motors_tau_kp_and_kd(self) -> None:
        modes = [1] * 29
        modes[0] = 0
        self.dds.command = _make_command(
            positions=[0.0] * 29,
            velocities=[0.0] * 29,
            torques=[5.0] * 29,
            kp=[0.0] * 29,
            kd=[8.0] * 29,
            modes=modes,
        )

        action = self.provider.get_action(None)
        _, _, tau_target = self._split_action(action)
        disabled_joint = self.joint_names.index(G1_29DOF_DDS_JOINT_ORDER[0])
        enabled_joint = self.joint_names.index(G1_29DOF_DDS_JOINT_ORDER[1])

        self.assertAlmostEqual(tau_target[disabled_joint].item(), 0.0)
        self.assertAlmostEqual(self.robot.written_kp[0, disabled_joint].item(), 0.0)
        self.assertAlmostEqual(self.robot.written_kd[0, disabled_joint].item(), 0.0)
        self.assertAlmostEqual(tau_target[enabled_joint].item(), 5.0)
        self.assertAlmostEqual(self.robot.written_kp[0, enabled_joint].item(), 0.0)
        self.assertAlmostEqual(self.robot.written_kd[0, enabled_joint].item(), 8.0)

    def test_motor_disable_remains_hard_off_during_startup_blend(self) -> None:
        modes = [1] * 29
        modes[0] = 0
        self.provider.ramp_duration_s = 10.0
        self.dds.command = _make_command(
            positions=[0.0] * 29,
            velocities=[1.0] * 29,
            torques=[5.0] * 29,
            kp=[30.0] * 29,
            kd=[3.0] * 29,
            modes=modes,
        )

        action = self.provider.get_action(None)
        _, _, tau_target = self._split_action(action)
        disabled_joint = self.joint_names.index(G1_29DOF_DDS_JOINT_ORDER[0])
        self.assertAlmostEqual(tau_target[disabled_joint].item(), 0.0)
        self.assertAlmostEqual(self.robot.written_kp[0, disabled_joint].item(), 0.0)
        self.assertAlmostEqual(self.robot.written_kd[0, disabled_joint].item(), 0.0)

    def test_timeout_holds_q_and_gains_but_zeroes_stale_dq_and_tau(self) -> None:
        first_action = self.provider.get_action(None)
        first_q, first_dq, first_tau = self._split_action(first_action)
        self.assertGreater(float(torch.abs(first_dq).max().item()), 0.0)
        self.assertGreater(float(torch.abs(first_tau).max().item()), 0.0)
        first_kp = self.robot.written_kp.clone()
        first_kd = self.robot.written_kd.clone()

        self.dds.command["receive_time_monotonic"] = time.monotonic() - 1.0
        held_action = self.provider.get_action(None)
        held_q, held_dq, held_tau = self._split_action(held_action)

        self.assertTrue(torch.equal(held_q, first_q))
        self.assertTrue(torch.equal(held_dq, self.robot.data.default_joint_vel[0]))
        self.assertTrue(torch.equal(held_tau, torch.zeros_like(held_tau)))
        self.assertTrue(torch.equal(self.robot.written_kp, first_kp))
        self.assertTrue(torch.equal(self.robot.written_kd, first_kd))

    def test_hand_timeout_does_not_pause_or_replace_body_lowcmd(self) -> None:
        first_action = self.provider.get_action(None)
        first_q, _, _ = self._split_action(first_action)
        first_kp = self.robot.written_kp.clone()
        first_kd = self.robot.written_kd.clone()

        stale_time = time.monotonic() - 1.0
        for command in self.dex3_dds.commands.values():
            command["receive_time_monotonic"] = stale_time
        self.dds.command["receive_time_monotonic"] = time.monotonic()

        action = self.provider.get_action(None)
        q_target, dq_target, tau_target = self._split_action(action)

        body_joint = self.joint_names.index(G1_29DOF_DDS_JOINT_ORDER[0])
        self.assertGreater(abs(dq_target[body_joint].item()), 0.0)
        self.assertGreater(abs(tau_target[body_joint].item()), 0.0)
        for hand_name in DEX3_HAND_JOINT_NAMES:
            hand_joint = self.joint_names.index(hand_name)
            self.assertAlmostEqual(q_target[hand_joint].item(), first_q[hand_joint].item())
            self.assertAlmostEqual(dq_target[hand_joint].item(), 0.0)
            self.assertAlmostEqual(tau_target[hand_joint].item(), 0.0)
            self.assertAlmostEqual(
                self.robot.written_kp[0, hand_joint].item(), first_kp[0, hand_joint].item()
            )
            self.assertAlmostEqual(
                self.robot.written_kd[0, hand_joint].item(), first_kd[0, hand_joint].item()
            )

    def test_hand_timeout_mode_relaxes_only_that_motor(self) -> None:
        modes = [0x10 | index for index in range(7)]
        modes[0] |= 0x80
        self.dex3_dds.commands["left_hand_cmd"] = _make_hand_command(
            positions=self.left_hand_positions,
            velocities=[1.0] * 7,
            torques=[0.5] * 7,
            kp=[1.5] * 7,
            kd=[0.1] * 7,
            modes=modes,
        )

        action = self.provider.get_action(None)
        q_target, dq_target, tau_target = self._split_action(action)
        disabled_joint = self.joint_names.index(DEX3_HAND_JOINT_NAMES[0])
        enabled_joint = self.joint_names.index(DEX3_HAND_JOINT_NAMES[1])

        self.assertAlmostEqual(q_target[disabled_joint].item(), 0.0)
        self.assertAlmostEqual(dq_target[disabled_joint].item(), 0.0)
        self.assertAlmostEqual(tau_target[disabled_joint].item(), 0.0)
        self.assertAlmostEqual(self.robot.written_kp[0, disabled_joint].item(), 0.0)
        self.assertAlmostEqual(self.robot.written_kd[0, disabled_joint].item(), 0.0)
        self.assertAlmostEqual(q_target[enabled_joint].item(), self.left_hand_positions[1])
        self.assertAlmostEqual(dq_target[enabled_joint].item(), 1.0)
        self.assertAlmostEqual(tau_target[enabled_joint].item(), 0.5)

    def test_hand_target_error_and_effort_are_safety_limited(self) -> None:
        self.dex3_dds.commands["left_hand_cmd"] = _make_hand_command(
            positions=[1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0],
            torques=[10.0] * 7,
        )

        action = self.provider.get_action(None)
        q_target, _, tau_target = self._split_action(action)
        thumb_0_joint = self.joint_names.index("left_hand_thumb_0_joint")
        thumb_1_joint = self.joint_names.index("left_hand_thumb_1_joint")

        self.assertAlmostEqual(q_target[thumb_0_joint].item(), 0.25, places=6)
        self.assertAlmostEqual(q_target[thumb_1_joint].item(), 0.25, places=6)
        self.assertAlmostEqual(tau_target[thumb_0_joint].item(), 2.45, places=6)
        self.assertAlmostEqual(tau_target[thumb_1_joint].item(), 0.7, places=6)

    def test_damping_only_packet_is_executed_instead_of_held(self) -> None:
        self.dds.command = _make_command(
            positions=[0.0] * 29,
            velocities=[0.0] * 29,
            torques=[0.0] * 29,
            kp=[0.0] * 29,
            kd=[8.0] * 29,
        )

        action = self.provider.get_action(None)
        q_target, dq_target, tau_target = self._split_action(action)
        for joint_name in G1_29DOF_DDS_JOINT_ORDER:
            joint_index = self.joint_names.index(joint_name)
            self.assertAlmostEqual(q_target[joint_index].item(), 0.0)
            self.assertAlmostEqual(dq_target[joint_index].item(), 0.0)
            self.assertAlmostEqual(tau_target[joint_index].item(), 0.0)
            self.assertAlmostEqual(self.robot.written_kp[0, joint_index].item(), 0.0)
            self.assertAlmostEqual(self.robot.written_kd[0, joint_index].item(), 8.0)

    def test_matching_lowstate_ack_allows_exactly_the_current_environment_step(self) -> None:
        self.provider.sync_with_lowstate = True
        self.provider._environment_step_ready = False
        self.dds.state_info = {"sample_seq": 42, "reset_epoch": 3}
        self.dds.command.update(
            {
                "ack_state_tick": 42,
                "ack_reset_epoch": 3,
                "control_step_count": 99,
                "sync_magic": SONIC_LOWCMD_SYNC_MAGIC,
            }
        )

        action = self.provider.get_action(None)

        self.assertIsNotNone(action)
        self.assertTrue(self.provider.can_step_environment())
        self.assertEqual(self.provider._sync_last_ack_tick, 42)
        self.assertEqual(self.provider._sync_last_ack_reset_epoch, 3)

    def test_stale_lowstate_ack_pauses_environment_instead_of_reusing_command(self) -> None:
        self.provider.sync_with_lowstate = True
        self.provider.sync_wait_timeout_s = 0.01
        self.provider.sync_poll_interval_s = 0.0002
        self.provider._environment_step_ready = False
        self.dds.state_info = {"sample_seq": 55, "reset_epoch": 4}
        self.dds.command.update(
            {
                "ack_state_tick": 54,
                "ack_reset_epoch": 4,
                "control_step_count": 100,
                "sync_magic": SONIC_LOWCMD_SYNC_MAGIC,
            }
        )

        action = self.provider.get_action(None)
        q_target, dq_target, tau_target = self._split_action(action)

        self.assertFalse(self.provider.can_step_environment())
        self.assertTrue(torch.equal(q_target, self.robot.data.default_joint_pos[0]))
        self.assertTrue(torch.equal(dq_target, self.robot.data.default_joint_vel[0]))
        self.assertTrue(torch.equal(tau_target, torch.zeros_like(tau_target)))
        self.assertEqual(self.provider._sync_timeout_count, 1)

    def test_arm_target_step_limit_does_not_slow_leg_balance_targets(self) -> None:
        self.provider.group_max_target_step["arms"] = 0.05
        positions = []
        for joint_name in G1_29DOF_DDS_JOINT_ORDER:
            joint_index = self.joint_names.index(joint_name)
            positions.append(
                float(self.robot.data.default_joint_pos[0, joint_index].item()) + 1.0
            )
        self.dds.command = _make_command(
            positions=positions,
            velocities=[0.0] * 29,
            torques=[0.0] * 29,
            kp=[30.0] * 29,
            kd=[3.0] * 29,
        )

        action = self.provider.get_action(None)
        q_target, _, _ = self._split_action(action)
        leg_joint_index = self.joint_names.index(G1_29DOF_DDS_JOINT_ORDER[0])
        arm_joint_index = self.joint_names.index(G1_29DOF_DDS_JOINT_ORDER[15])

        self.assertAlmostEqual(q_target[leg_joint_index].item(), positions[0], places=6)
        self.assertAlmostEqual(
            q_target[arm_joint_index].item(),
            self.robot.data.default_joint_pos[0, arm_joint_index].item() + 0.05,
            places=6,
        )
        self.assertEqual(self.provider._group_limit_counts["legs"], 0)
        self.assertGreater(self.provider._group_limit_counts["arms"], 0)


@unittest.skipIf(
    SonicDDSActionProvider is None,
    f"SONIC DDS dependencies unavailable: {IMPORT_ERROR}",
)
class Dex3DDSAndStateTest(unittest.TestCase):
    @staticmethod
    def _make_dex3_dds(output_shm=None) -> Dex3DDS:
        with mock.patch.object(Dex3DDS, "setup_shared_memory"):
            dds = Dex3DDS(node_name="test_dex3")
        dds.input_shm = _FakeCommandSharedMemory()
        dds.output_shm = (
            _FakeCommandSharedMemory() if output_shm is None else output_shm
        )
        return dds

    @staticmethod
    def _make_hand_command() -> object:
        message = unitree_hg_msg_dds__HandCmd_()
        message.reserve[:] = [11, 12, 13, 14]
        for motor_index, motor in enumerate(message.motor_cmd):
            motor.mode = 0x10 | motor_index
            motor.q = 0.1 * motor_index
            motor.dq = 0.2 * motor_index
            motor.tau = 0.3 * motor_index
            motor.kp = 1.5 + motor_index
            motor.kd = 0.1 + 0.01 * motor_index
            motor.reserve = 100 + motor_index
        return message

    def test_dex3_dds_preserves_complete_handcmd_and_receive_time(self) -> None:
        message = self._make_hand_command()

        before = time.monotonic()
        command = Dex3DDS.process_hand_command(
            SimpleNamespace(node_name="test"), message, "left"
        )
        after = time.monotonic()

        self.assertEqual(command["modes"], [0x10 | index for index in range(7)])
        self.assertEqual(len(command["positions"]), 7)
        self.assertEqual(len(command["velocities"]), 7)
        self.assertEqual(len(command["torques"]), 7)
        self.assertEqual(len(command["kp"]), 7)
        self.assertEqual(len(command["kd"]), 7)
        self.assertGreaterEqual(command["receive_time_monotonic"], before)
        self.assertLessEqual(command["receive_time_monotonic"], after)

    def test_dex3_duplicate_fastpath_refreshes_liveness_without_shm_write(self) -> None:
        output_shm = _FakeCommandSharedMemory()
        dds = self._make_dex3_dds(output_shm)
        message = self._make_hand_command()

        with mock.patch(
            "dds.dex3_dds.time.monotonic", side_effect=[100.0, 100.001]
        ):
            dds.dds_subscriber(message, "left")
            first_commands = dds.get_hand_commands()
            first_receive_time = first_commands["left_hand_cmd"][
                "receive_time_monotonic"
            ]
            self.assertEqual(output_shm.write_count, 1)
            self.assertEqual(output_shm.read_count, 0)

            dds.dds_subscriber(message, "left")
        second_commands = dds.get_hand_commands()

        self.assertGreater(
            second_commands["left_hand_cmd"]["receive_time_monotonic"],
            first_receive_time,
        )
        self.assertEqual(output_shm.write_count, 1)
        self.assertEqual(output_shm.read_count, 0)
        self.assertEqual(dds._handcmd_packet_counts["left"], 2)
        self.assertEqual(dds._handcmd_change_counts["left"], 1)
        self.assertEqual(dds._handcmd_duplicate_counts["left"], 1)

    def test_dex3_fastpath_keeps_left_and_right_liveness_independent(self) -> None:
        dds = self._make_dex3_dds()
        message = self._make_hand_command()

        dds.dds_subscriber(message, "left")
        dds.dds_subscriber(message, "right")
        before = dds.get_hand_commands()
        time.sleep(0.001)
        dds.dds_subscriber(message, "left")
        after = dds.get_hand_commands()

        self.assertGreater(
            after["left_hand_cmd"]["receive_time_monotonic"],
            before["left_hand_cmd"]["receive_time_monotonic"],
        )
        self.assertEqual(
            after["right_hand_cmd"]["receive_time_monotonic"],
            before["right_hand_cmd"]["receive_time_monotonic"],
        )
        self.assertEqual(dds._handcmd_change_counts, {"left": 1, "right": 1})
        self.assertEqual(dds._handcmd_duplicate_counts, {"left": 1, "right": 0})

    def test_dex3_fingerprint_covers_all_control_and_reserve_fields(self) -> None:
        dds = self._make_dex3_dds()
        message = self._make_hand_command()
        dds.dds_subscriber(message, "left")

        mutations = (
            lambda: setattr(message.motor_cmd[0], "mode", 0x21),
            lambda: setattr(message.motor_cmd[0], "q", 0.25),
            lambda: setattr(message.motor_cmd[0], "dq", 0.35),
            lambda: setattr(message.motor_cmd[0], "tau", 0.45),
            lambda: setattr(message.motor_cmd[0], "kp", 2.5),
            lambda: setattr(message.motor_cmd[0], "kd", 0.5),
            lambda: setattr(message.motor_cmd[0], "reserve", 999),
            lambda: message.reserve.__setitem__(0, 777),
        )
        for expected_change_count, mutate in enumerate(mutations, start=2):
            with self.subTest(change=expected_change_count):
                mutate()
                dds.dds_subscriber(message, "left")
                self.assertEqual(
                    dds._handcmd_change_counts["left"], expected_change_count
                )
        self.assertEqual(dds.output_shm.write_count, 1 + len(mutations))

    def test_dex3_shm_heartbeat_and_failed_write_retry(self) -> None:
        output_shm = _FakeCommandSharedMemory()
        output_shm.fail_writes = 1
        dds = self._make_dex3_dds(output_shm)
        message = self._make_hand_command()

        dds.dds_subscriber(message, "left")
        self.assertTrue(dds._handcmd_shm_dirty)
        self.assertEqual(output_shm.write_count, 1)

        dds.dds_subscriber(message, "left")

        self.assertFalse(dds._handcmd_shm_dirty)
        self.assertEqual(output_shm.write_count, 2)
        self.assertEqual(dds._handcmd_shm_write_count, 1)
        self.assertIn("left_hand_cmd", output_shm.payload)

    def test_dex3_cache_works_without_shm_and_falls_back_before_first_packet(self) -> None:
        fallback = _FakeCommandSharedMemory({"from_shm": True})
        dds = self._make_dex3_dds(fallback)

        self.assertEqual(dds.get_hand_commands(), {"from_shm": True})
        self.assertEqual(fallback.read_count, 1)

        dds.output_shm = None
        dds.dds_subscriber(self._make_hand_command(), "right")
        commands = dds.get_hand_commands()
        self.assertEqual(len(commands["right_hand_cmd"]["positions"]), 7)
        self.assertEqual(commands["left_hand_cmd"], {})

    def test_dex3_concurrent_sides_publish_complete_snapshot(self) -> None:
        dds = self._make_dex3_dds()
        barrier = threading.Barrier(3)

        def publish(side):
            barrier.wait()
            dds.dds_subscriber(self._make_hand_command(), side)

        threads = [
            threading.Thread(target=publish, args=(side,))
            for side in ("left", "right")
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(timeout=1.0)

        self.assertTrue(all(not thread.is_alive() for thread in threads))
        commands = dds.get_hand_commands()
        self.assertEqual(len(commands["left_hand_cmd"]["positions"]), 7)
        self.assertEqual(len(commands["right_hand_cmd"]["positions"]), 7)

    def test_dex3_out_of_order_callback_does_not_roll_back_command(self) -> None:
        dds = self._make_dex3_dds()
        newer = self._make_hand_command()
        older = self._make_hand_command()
        older.motor_cmd[0].q = -0.75

        with mock.patch("dds.dex3_dds.time.monotonic", side_effect=[2.0, 1.0]):
            dds.dds_subscriber(newer, "left")
            dds.dds_subscriber(older, "left")

        command = dds.get_hand_commands()["left_hand_cmd"]
        self.assertAlmostEqual(command["positions"][0], newer.motor_cmd[0].q)
        self.assertEqual(command["receive_time_monotonic"], 2.0)
        self.assertEqual(dds._handcmd_out_of_order_counts["left"], 1)

    def test_dex3_state_uses_name_mapping_and_dds_order(self) -> None:
        joint_names = list(reversed(G1_29DOF_DDS_JOINT_ORDER)) + list(
            reversed(DEX3_HAND_JOINT_NAMES)
        )
        values = torch.arange(len(joint_names), dtype=torch.float32).unsqueeze(0)
        robot = SimpleNamespace(
            data=SimpleNamespace(
                joint_names=joint_names,
                joint_pos=values,
                joint_vel=values + 100.0,
                applied_torque=values + 200.0,
            )
        )
        env = SimpleNamespace(scene={"robot": robot})
        writer = _FakeDex3StateWriter()

        with mock.patch.object(dex3_state, "_get_dex3_dds_instance", return_value=writer):
            hand_positions = dex3_state.get_robot_dex3_joint_states(
                env,
                enable_dds=True,
                dds_min_interval_ms=0.0,
            )

        expected = torch.tensor(
            [joint_names.index(name) for name in DEX3_HAND_JOINT_NAMES],
            dtype=torch.float32,
        )
        self.assertTrue(torch.equal(hand_positions[0], expected))
        self.assertIsNotNone(writer.last_write)
        self.assertEqual(writer.last_write[0], expected[:7].tolist())
        self.assertEqual(writer.last_write[3], expected[7:].tolist())
        self.assertEqual(writer.last_write[1], (expected[:7] + 100.0).tolist())
        self.assertEqual(writer.last_write[4], (expected[7:] + 100.0).tolist())
        self.assertEqual(writer.last_write[2], (expected[:7] + 200.0).tolist())
        self.assertEqual(writer.last_write[5], (expected[7:] + 200.0).tolist())


if __name__ == "__main__":
    unittest.main()
