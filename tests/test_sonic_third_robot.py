"""第三台 SONIC 机器人固定接线的源码契约测试（不启动 Isaac Sim）。"""

from __future__ import annotations

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]


def _source(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


class SonicThirdRobotWiringTests(unittest.TestCase):
    def test_host_provider_is_exactly_three_channels_and_387_dims(self) -> None:
        source = _source("action_provider/action_provider_sonic_dds_host.py")
        for value in (
            '"robot_3": SonicDDSActionProvider(',
            'asset_name="robot_3"',
            'robot_dds_name="g129_r3"',
            'dex3_dds_name="dex3_r3"',
            'foot_contact_name="foot_contact_3"',
            'log_suffix=":r3"',
            '"joint_pos_3"',
            '"joint_vel_3"',
            '"joint_effort_3"',
            "387-dim",
        ):
            self.assertIn(value, source)
        self.assertNotIn("robot_4", source)
        self.assertNotIn("robot_5", source)

    def test_dds_factory_registers_only_fixed_r3_pair(self) -> None:
        source = _source("dds/dds_create.py")
        self.assertIn('getattr(args_cli, "enable_third_robot_dds", False)', source)
        self.assertIn('node_name="g1_robot_r3", topic_prefix="rt/r3", shm_suffix="_r3"', source)
        self.assertIn('register_object("g129_r3", g1_robot_r3)', source)
        self.assertIn('node_name="dex3_r3", topic_prefix="rt/r3", shm_suffix="_r3"', source)
        self.assertIn('register_object("dex3_r3", dex3_r3)', source)
        self.assertNotIn("g129_r4", source)
        self.assertNotIn("dex3_r4", source)

    def test_scene_adds_robot3_without_changing_robot4_or_robot5(self) -> None:
        source = _source(
            "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
        )
        for value in (
            '"robot_3": "robot_3"',
            '"robot_3": "peer_robot_3"',
            '"robot_3": "/World/envs/env_0/PeerRobot3"',
            'cfg.prim_path = "{ENV_REGEX_NS}/Robot3"',
            'cfg.prim_path = "{ENV_REGEX_NS}/PeerRobot3"',
            "robot_3: ArticulationCfg | None = _make_third_local_robot_cfg()",
            'asset_name="robot_3"',
            '"dds_object_name": "g129_r3"',
            '"dds_object_name": "dex3_r3"',
        ):
            self.assertIn(value, source)
        self.assertNotIn("robot_4:", source)
        self.assertNotIn("robot_5:", source)

    def test_sim_seeds_publishes_and_resets_third_channel(self) -> None:
        source = _source("sim_main.py")
        self.assertIn('args_cli.enable_third_robot_dds = is_scene_sync_host', source)
        self.assertIn('["g129", "g129_r2", "g129_r3"]', source)
        self.assertIn('asset_name="robot_3"', source)
        self.assertIn('dds_object_name="g129_r3"', source)
        self.assertIn('dds_object_name="dex3_r3"', source)
        self.assertIn("three lock-step channels", source)

    def test_bringup_pins_r3_prefix_and_private_ports(self) -> None:
        source = _source("tools/pipeline_pico_bringup.sh")
        for value in (
            'G1_LOCAL_ROBOT_ID=3 SONIC_DDS_TOPIC_PREFIX="rt/r3"',
            "--zmq-port 5576",
            "--zmq-out-port 5577",
            "--udp-out-port 5577",
            '"$LOG_DIR/deploy_r3.log"',
            '"$LOG_DIR/dk_r3"',
            'sonic_dds:r3.*CONTROL marker received',
        ):
            self.assertIn(value, source)
        self.assertNotIn("rt/r4", source)
        self.assertNotIn("rt/r5", source)


if __name__ == "__main__":
    unittest.main()
