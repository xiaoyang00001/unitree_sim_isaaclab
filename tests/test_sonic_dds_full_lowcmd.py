from __future__ import annotations

import os
import time
import unittest
from types import SimpleNamespace
from unittest import mock

import torch

os.environ.setdefault("UNITREE_DDS_DOMAIN", "91")
os.environ.setdefault("UNITREE_DDS_INTERFACE", "lo")

try:
    from action_provider.action_provider_sonic_dds import SonicDDSActionProvider
    from dds.dds_master import dds_manager
    from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER
    from robots.g1_sonic_urdf import DEX3_HAND_JOINT_NAMES
except ModuleNotFoundError as import_error:
    SonicDDSActionProvider = None
    IMPORT_ERROR = import_error
else:
    IMPORT_ERROR = None


class _FakeDDS:
    def __init__(self, command):
        self.command = command

    def get_robot_command(self):
        return self.command


class _FakeActuator:
    def __init__(self, joint_indices: torch.Tensor, kp: torch.Tensor, kd: torch.Tensor):
        self.joint_indices = joint_indices
        self.stiffness = kp[:, joint_indices].clone()
        self.damping = kd[:, joint_indices].clone()


class _FakeRobot:
    def __init__(self, joint_names: list[str]):
        joint_count = len(joint_names)
        default_q = torch.linspace(-0.2, 0.2, joint_count).unsqueeze(0)
        default_dq = torch.zeros(1, joint_count)
        default_kp = torch.linspace(10.0, 20.0, joint_count).unsqueeze(0)
        default_kd = torch.linspace(1.0, 2.0, joint_count).unsqueeze(0)
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
            joint_effort_limits=torch.full((1, joint_count), 200.0),
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
):
    return {
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
        action_manager = SimpleNamespace(
            active_terms=["joint_pos", "joint_vel", "joint_effort"],
            total_action_dim=3 * len(self.joint_names),
        )
        env = SimpleNamespace(
            scene={"robot": self.robot},
            device="cpu",
            num_envs=1,
            action_manager=action_manager,
        )
        args = SimpleNamespace(
            sonic_lowcmd_timeout=0.1,
            sonic_ramp_seconds=0.0,
            sonic_max_target_step=0.0,
            stats_interval=1000.0,
        )
        with mock.patch.object(dds_manager, "get_object", return_value=self.dds):
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

        for hand_name in DEX3_HAND_JOINT_NAMES:
            hand_index = self.joint_names.index(hand_name)
            self.assertAlmostEqual(
                q_target[hand_index].item(),
                self.robot.data.default_joint_pos[0, hand_index].item(),
            )
            self.assertAlmostEqual(dq_target[hand_index].item(), 0.0)
            self.assertAlmostEqual(tau_target[hand_index].item(), 0.0)

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


if __name__ == "__main__":
    unittest.main()
