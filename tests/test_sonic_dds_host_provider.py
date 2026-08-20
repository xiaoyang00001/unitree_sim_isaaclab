"""host 三机器人组合器与观测缓存隔离的单元测试（不启动 Isaac Sim）。

覆盖工作包 B 的三类静默故障模式：
- 387 维动作拼接的切片落位（robot、robot_2、robot_3 依次排列）；
- 三路 ack AND 锁步（任一路未 ack 整个 env 不 step）与倒地恢复广播；
- 观测缓存按 asset 键控（buffer aliasing 与 sample_seq 串台都是排查代价极高的静默错误）。
"""

from __future__ import annotations

import importlib.util
import os
import unittest
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


class _FakeChannel:
    def __init__(self, action: torch.Tensor, ready: bool = True):
        self._action = action
        self._ready = ready
        self.control_started = False
        self.fall_recovery_active = False
        self.recovery_calls = []
        self._num_joints = 43

    def get_action(self, env):
        return self._action

    def can_step_environment(self) -> bool:
        return self._ready

    def begin_fall_recovery(self, **kwargs):
        self.recovery_calls.append(kwargs)


def _make_host(
    ch1: _FakeChannel,
    ch2: _FakeChannel,
    ch3: _FakeChannel,
) -> "SonicDDSHostActionProvider":
    host = SonicDDSHostActionProvider.__new__(SonicDDSHostActionProvider)
    host.channels = {"robot": ch1, "robot_2": ch2, "robot_3": ch3}
    return host


@unittest.skipIf(SonicDDSHostActionProvider is None, f"import failed: {IMPORT_ERROR}")
class SonicDDSHostProviderTest(unittest.TestCase):
    def test_action_concat_slices(self):
        a1 = torch.arange(129, dtype=torch.float32).unsqueeze(0)
        a2 = torch.arange(129, dtype=torch.float32).unsqueeze(0) + 1000.0
        a3 = torch.arange(129, dtype=torch.float32).unsqueeze(0) + 2000.0
        host = _make_host(_FakeChannel(a1), _FakeChannel(a2), _FakeChannel(a3))
        action = host.get_action(env=None)
        self.assertEqual(action.shape, (1, 387))
        self.assertTrue(torch.equal(action[:, :129], a1))
        self.assertTrue(torch.equal(action[:, 129:258], a2))
        self.assertTrue(torch.equal(action[:, 258:], a3))

    def test_action_none_propagates(self):
        a1 = torch.zeros(1, 129)
        ch2 = _FakeChannel(a1)
        ch2._action = None
        host = _make_host(_FakeChannel(a1), ch2, _FakeChannel(a1))
        self.assertIsNone(host.get_action(env=None))

    def test_can_step_requires_all_three_acks(self):
        for ready1 in (False, True):
            for ready2 in (False, True):
                for ready3 in (False, True):
                    expected = ready1 and ready2 and ready3
                    host = _make_host(
                        _FakeChannel(torch.zeros(1, 129), ready=ready1),
                        _FakeChannel(torch.zeros(1, 129), ready=ready2),
                        _FakeChannel(torch.zeros(1, 129), ready=ready3),
                    )
                    self.assertIs(
                        host.can_step_environment(),
                        expected,
                        (ready1, ready2, ready3),
                    )

    def test_control_state_aggregation_and_recovery_broadcast(self):
        ch1 = _FakeChannel(torch.zeros(1, 129))
        ch2 = _FakeChannel(torch.zeros(1, 129))
        ch3 = _FakeChannel(torch.zeros(1, 129))
        host = _make_host(ch1, ch2, ch3)
        self.assertFalse(host.control_started)
        ch3.control_started = True
        self.assertTrue(host.control_started)
        ch1.fall_recovery_active = True
        self.assertTrue(host.fall_recovery_active)

        host.begin_fall_recovery(hold_duration_s=1.0, blend_duration_s=0.5, reason="test")
        for channel in (ch1, ch2, ch3):
            self.assertEqual(len(channel.recovery_calls), 1)
            self.assertEqual(channel.recovery_calls[0]["reason"], "test")

    def test_layout_validation_rejects_wrong_order(self):
        terms = {name: SimpleNamespace(action_dim=43) for name in HOST_ACTION_TERMS}
        terms["scene_state_sync"] = SimpleNamespace(action_dim=0)
        action_manager = SimpleNamespace(
            active_terms=("joint_pos", "joint_vel", "joint_effort", "scene_state_sync"),
            get_term=lambda name: terms[name],
            total_action_dim=129,
        )
        host = _make_host(
            _FakeChannel(torch.zeros(1, 129)),
            _FakeChannel(torch.zeros(1, 129)),
            _FakeChannel(torch.zeros(1, 129)),
        )
        env = SimpleNamespace(action_manager=action_manager)
        with self.assertRaises(ValueError):
            host._validate_action_layout(env)

    def test_layout_validation_accepts_host_terms(self):
        terms = {name: SimpleNamespace(action_dim=43) for name in HOST_ACTION_TERMS}
        terms["scene_state_sync"] = SimpleNamespace(action_dim=0)
        terms["env_reset_sync"] = SimpleNamespace(action_dim=0)
        action_manager = SimpleNamespace(
            active_terms=HOST_ACTION_TERMS + ("scene_state_sync", "env_reset_sync"),
            get_term=lambda name: terms[name],
            total_action_dim=387,
        )
        host = _make_host(
            _FakeChannel(torch.zeros(1, 129)),
            _FakeChannel(torch.zeros(1, 129)),
            _FakeChannel(torch.zeros(1, 129)),
        )
        env = SimpleNamespace(action_manager=action_manager)
        host._validate_action_layout(env)  # 不应抛异常


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
    def _make_env(self, base_1: float, base_2: float, base_3: float):
        scene = _FakeScene()
        scene["robot"] = SimpleNamespace(
            data=_FakeArticulationData(G1_29DOF_DDS_JOINT_ORDER, base_1)
        )
        scene["robot_2"] = SimpleNamespace(
            data=_FakeArticulationData(G1_29DOF_DDS_JOINT_ORDER, base_2)
        )
        scene["robot_3"] = SimpleNamespace(
            data=_FakeArticulationData(G1_29DOF_DDS_JOINT_ORDER, base_3)
        )
        return SimpleNamespace(scene=scene, step_dt=0.02)

    def test_buffers_are_isolated_per_asset(self):
        g1_state._obs_caches.clear()
        env = self._make_env(1.0, 2.0, 3.0)
        out_1 = g1_state.get_robot_boy_joint_states(env, enable_dds=False)
        out_2 = g1_state.get_robot_boy_joint_states(
            env, enable_dds=False, asset_name="robot_2", dds_object_name="g129_r2"
        )
        out_3 = g1_state.get_robot_boy_joint_states(
            env, enable_dds=False, asset_name="robot_3", dds_object_name="g129_r3"
        )
        # aliasing 检查：后续 term 的 gather 不得覆写先前 term 已返回的张量
        self.assertIsNot(out_1, out_2)
        self.assertIsNot(out_2, out_3)
        self.assertTrue(bool((out_1[:, :29] == 1.0).all()), "robot 段被 robot_2 覆写")
        self.assertTrue(bool((out_2[:, :29] == 2.0).all()))
        self.assertTrue(bool((out_3[:, :29] == 3.0).all()))

    def test_sample_seq_is_isolated_per_asset(self):
        g1_state._obs_caches.clear()
        cache_1 = g1_state._get_obs_cache("robot")
        cache_2 = g1_state._get_obs_cache("robot_2")
        cache_3 = g1_state._get_obs_cache("robot_3")
        cache_1["sample_seq"] += 5
        self.assertEqual(cache_2["sample_seq"], 0, "sample_seq 串台会污染锁步 ack 判据")
        self.assertEqual(cache_3["sample_seq"], 0, "sample_seq 串台会污染第三路锁步")


if __name__ == "__main__":
    unittest.main()
