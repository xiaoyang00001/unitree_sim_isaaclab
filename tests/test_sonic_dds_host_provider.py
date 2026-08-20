"""host 五机器人组合器与观测缓存隔离的单元测试（不启动 Isaac Sim）。

覆盖工作包 B 的三类静默故障模式：
- 645 维动作拼接的五个 129 维切片落位；
- 五路 ack AND 锁步（任一路未 ack 整个 env 不 step）与倒地恢复广播；
- 观测缓存按 asset 键控（buffer aliasing 与 sample_seq 串台都是排查代价极高的静默错误）。
"""

from __future__ import annotations

import importlib.util
import os
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace

import torch

os.environ.setdefault("UNITREE_DDS_DOMAIN", "91")
os.environ.setdefault("UNITREE_DDS_INTERFACE", "lo")

try:
    from action_provider.action_provider_sonic_dds_host import (
        HOST_ACTION_TERMS,
        SonicDDSHostActionProvider,
    )
    from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER
    from robots.g1_sonic_urdf import DEX3_HAND_JOINT_NAMES
    from robots.sonic_multi_robot import sonic_host_action_terms
except ModuleNotFoundError as import_error:
    SonicDDSHostActionProvider = None
    IMPORT_ERROR = import_error
else:
    IMPORT_ERROR = None
    g1_state_spec = importlib.util.spec_from_file_location(
        "g1_state_test_module",
        Path(__file__).parents[1] / "tasks/common_observations/g1_29dof_state.py",
    )
    g1_state = importlib.util.module_from_spec(g1_state_spec)
    g1_state_spec.loader.exec_module(g1_state)
    dex3_state_spec = importlib.util.spec_from_file_location(
        "dex3_state_test_module",
        Path(__file__).parents[1] / "tasks/common_observations/dex3_state.py",
    )
    dex3_state = importlib.util.module_from_spec(dex3_state_spec)
    dex3_state_spec.loader.exec_module(dex3_state)


class _FakeChannel:
    def __init__(self, action: torch.Tensor, ready: bool = True):
        self._action = action
        self._ready = ready
        self.control_started = False
        self.fall_recovery_active = False
        self.recovery_calls = []
        self.start_calls = 0
        self.stop_calls = 0
        self.cleanup_calls = 0
        self._num_joints = 43

    def get_action(self, env):
        return self._action

    def can_step_environment(self) -> bool:
        return self._ready

    def begin_fall_recovery(self, **kwargs):
        self.recovery_calls.append(kwargs)

    def start(self):
        self.start_calls += 1

    def stop(self):
        self.stop_calls += 1

    def cleanup(self):
        self.cleanup_calls += 1


def _make_host(*channels: _FakeChannel) -> "SonicDDSHostActionProvider":
    host = SonicDDSHostActionProvider.__new__(SonicDDSHostActionProvider)
    host.channels = {
        "robot" if index == 1 else f"robot_{index}": channel
        for index, channel in enumerate(channels, start=1)
    }
    return host


@unittest.skipIf(SonicDDSHostActionProvider is None, f"import failed: {IMPORT_ERROR}")
class SonicDDSHostProviderTest(unittest.TestCase):
    @mock.patch(
        "action_provider.action_provider_sonic_dds_host.SonicDDSActionProvider"
    )
    def test_constructor_builds_five_isolated_channels(self, provider_cls):
        provider_cls.side_effect = [
            _FakeChannel(torch.zeros(1, 129)) for _ in range(5)
        ]
        terms = {name: SimpleNamespace(action_dim=43) for name in HOST_ACTION_TERMS}
        terms["scene_state_sync"] = SimpleNamespace(action_dim=0)
        action_manager = SimpleNamespace(
            active_terms=HOST_ACTION_TERMS + ("scene_state_sync",),
            get_term=lambda name: terms[name],
            total_action_dim=645,
        )

        host = SonicDDSHostActionProvider(
            SimpleNamespace(action_manager=action_manager),
            SimpleNamespace(sonic_robot_count=5),
        )

        self.assertEqual(
            tuple(host.channels),
            ("robot", "robot_2", "robot_3", "robot_4", "robot_5"),
        )
        self.assertEqual(provider_cls.call_count, 5)
        self.assertEqual(
            [call.kwargs["robot_dds_name"] for call in provider_cls.call_args_list],
            ["g129", "g129_r2", "g129_r3", "g129_r4", "g129_r5"],
        )
        self.assertEqual(
            [call.kwargs["dex3_dds_name"] for call in provider_cls.call_args_list],
            ["dex3", "dex3_r2", "dex3_r3", "dex3_r4", "dex3_r5"],
        )

    def test_action_concat_slices(self):
        segments = [
            torch.arange(129, dtype=torch.float32).unsqueeze(0) + 1000.0 * index
            for index in range(5)
        ]
        host = _make_host(*(_FakeChannel(segment) for segment in segments))
        action = host.get_action(env=None)
        self.assertEqual(action.shape, (1, 645))
        for index, segment in enumerate(segments):
            self.assertTrue(
                torch.equal(action[:, index * 129 : (index + 1) * 129], segment)
            )

    def test_action_none_propagates(self):
        a1 = torch.zeros(1, 129)
        channels = [_FakeChannel(a1) for _ in range(5)]
        channels[3]._action = None
        host = _make_host(*channels)
        self.assertIsNone(host.get_action(env=None))

    def test_can_step_requires_all_five_acks(self):
        self.assertTrue(
            _make_host(*[_FakeChannel(torch.zeros(1, 129)) for _ in range(5)])
            .can_step_environment()
        )
        for blocked_index in range(5):
            channels = [
                _FakeChannel(torch.zeros(1, 129), ready=index != blocked_index)
                for index in range(5)
            ]
            self.assertFalse(
                _make_host(*channels).can_step_environment(),
                f"channel {blocked_index + 1} must gate env.step",
            )

    def test_control_state_aggregation_and_recovery_broadcast(self):
        channels = [_FakeChannel(torch.zeros(1, 129)) for _ in range(5)]
        host = _make_host(*channels)
        self.assertFalse(host.control_started)
        channels[4].control_started = True
        self.assertTrue(host.control_started)
        channels[2].fall_recovery_active = True
        self.assertTrue(host.fall_recovery_active)

        host.begin_fall_recovery(hold_duration_s=1.0, blend_duration_s=0.5, reason="test")
        for channel in channels:
            self.assertEqual(len(channel.recovery_calls), 1)
            self.assertEqual(channel.recovery_calls[0]["reason"], "test")

        host.start()
        host.stop()
        host.cleanup()
        for channel in channels:
            self.assertEqual(channel.start_calls, 1)
            self.assertEqual(channel.stop_calls, 1)
            self.assertEqual(channel.cleanup_calls, 1)

    def test_layout_validation_rejects_wrong_order(self):
        terms = {name: SimpleNamespace(action_dim=43) for name in HOST_ACTION_TERMS}
        terms["scene_state_sync"] = SimpleNamespace(action_dim=0)
        action_manager = SimpleNamespace(
            active_terms=("joint_pos", "joint_vel", "joint_effort", "scene_state_sync"),
            get_term=lambda name: terms[name],
            total_action_dim=129,
        )
        host = _make_host(*[_FakeChannel(torch.zeros(1, 129)) for _ in range(5)])
        env = SimpleNamespace(action_manager=action_manager)
        with self.assertRaises(ValueError):
            host._validate_action_layout(env)

    def test_layout_validation_accepts_five_robot_terms(self):
        terms = {name: SimpleNamespace(action_dim=43) for name in HOST_ACTION_TERMS}
        terms["scene_state_sync"] = SimpleNamespace(action_dim=0)
        terms["env_reset_sync"] = SimpleNamespace(action_dim=0)
        action_manager = SimpleNamespace(
            active_terms=HOST_ACTION_TERMS + ("scene_state_sync", "env_reset_sync"),
            get_term=lambda name: terms[name],
            total_action_dim=645,
        )
        host = _make_host(*[_FakeChannel(torch.zeros(1, 129)) for _ in range(5)])
        env = SimpleNamespace(action_manager=action_manager)
        host._validate_action_layout(env)  # 不应抛异常

    def test_layout_terms_remain_backward_compatible_for_two_robots(self):
        self.assertEqual(
            sonic_host_action_terms(2),
            (
                "joint_pos",
                "joint_vel",
                "joint_effort",
                "joint_pos_2",
                "joint_vel_2",
                "joint_effort_2",
            ),
        )


class _FakeArticulationData:
    def __init__(self, joint_names, base: float):
        count = len(joint_names)
        self.joint_names = list(joint_names)
        self.joint_pos = torch.full((1, count), base, dtype=torch.float32)
        self.joint_vel = torch.full((1, count), base + 0.1, dtype=torch.float32)
        self.applied_torque = torch.full((1, count), base + 0.2, dtype=torch.float32)


class _FakeScene(dict):
    pass


@unittest.skipIf(SonicDDSHostActionProvider is None, f"import failed: {IMPORT_ERROR}")
class ObservationCacheIsolationTest(unittest.TestCase):
    def _make_env(self, bases: tuple[float, ...], joint_names=None):
        joint_names = joint_names or G1_29DOF_DDS_JOINT_ORDER
        scene = _FakeScene()
        for index, base in enumerate(bases, start=1):
            asset_name = "robot" if index == 1 else f"robot_{index}"
            scene[asset_name] = SimpleNamespace(
                data=_FakeArticulationData(joint_names, base)
            )
        return SimpleNamespace(scene=scene, step_dt=0.02)

    def test_buffers_are_isolated_per_asset(self):
        g1_state._obs_caches.clear()
        env = self._make_env((1.0, 2.0, 3.0, 4.0, 5.0))
        outputs = []
        for index in range(1, 6):
            asset_name = "robot" if index == 1 else f"robot_{index}"
            dds_name = "g129" if index == 1 else f"g129_r{index}"
            outputs.append(
                g1_state.get_robot_boy_joint_states(
                    env,
                    enable_dds=False,
                    asset_name=asset_name,
                    dds_object_name=dds_name,
                )
            )
        self.assertEqual(len({id(output) for output in outputs}), 5)
        for index, output in enumerate(outputs, start=1):
            self.assertTrue(bool((output[:, :29] == float(index)).all()))

    def test_sample_seq_is_isolated_per_asset(self):
        g1_state._obs_caches.clear()
        caches = [
            g1_state._get_obs_cache("robot" if index == 1 else f"robot_{index}")
            for index in range(1, 6)
        ]
        caches[0]["sample_seq"] += 5
        self.assertTrue(
            all(cache["sample_seq"] == 0 for cache in caches[1:]),
            "sample_seq 串台会污染锁步 ack 判据",
        )

    def test_dex3_buffers_are_isolated_per_asset(self):
        dex3_state._obs_caches.clear()
        joint_names = tuple(G1_29DOF_DDS_JOINT_ORDER) + tuple(DEX3_HAND_JOINT_NAMES)
        env = self._make_env((1.0, 2.0, 3.0, 4.0, 5.0), joint_names=joint_names)
        outputs = []
        for index in range(1, 6):
            asset_name = "robot" if index == 1 else f"robot_{index}"
            dds_name = "dex3" if index == 1 else f"dex3_r{index}"
            outputs.append(
                dex3_state.get_robot_dex3_joint_states(
                    env,
                    enable_dds=False,
                    asset_name=asset_name,
                    dds_object_name=dds_name,
                )
            )
        self.assertEqual(len({id(output) for output in outputs}), 5)
        for index, output in enumerate(outputs, start=1):
            self.assertEqual(output.shape, (1, 14))
            self.assertTrue(bool((output == float(index)).all()))


if __name__ == "__main__":
    unittest.main()
