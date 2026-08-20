"""Dependency-free contracts for the five-robot SONIC conveyor topology."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

from robots.sonic_multi_robot import (
    sonic_host_action_terms,
    sonic_robot_channel_specs,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SYNC_IDENTITY_PATH = (
    REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/sync_identity.py"
)
CONVEYOR_CFG_PATH = (
    REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py"
)
SIM_MAIN_PATH = REPO_ROOT / "sim_main.py"
DDS_CREATE_PATH = REPO_ROOT / "dds/dds_create.py"
DDS_MASTER_PATH = REPO_ROOT / "dds/dds_master.py"
SCENE_SYNC_ENV_PATH = REPO_ROOT / "configs/scene_sync.env"


def _load_sync_identity():
    spec = importlib.util.spec_from_file_location(
        "_test_sonic_multi_robot_sync_identity", SYNC_IDENTITY_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _FakeSharedMemory:
    def __init__(self) -> None:
        self.cleanup_calls = 0

    def cleanup(self) -> None:
        self.cleanup_calls += 1


class _FakeDDSObject:
    def __init__(
        self,
        *args,
        publisher_setup=True,
        subscriber_setup=True,
        **kwargs,
    ) -> None:
        self.args = args
        self.kwargs = kwargs
        self.publisher_setup = publisher_setup
        self.subscriber_setup = subscriber_setup
        self.publisher_setup_calls = 0
        self.subscriber_setup_calls = 0
        self.publishing = False
        self.subscribing = False
        self.input_shm = _FakeSharedMemory()
        self.output_shm = _FakeSharedMemory()
        self.input_shm_handle = self.input_shm
        self.output_shm_handle = self.output_shm

    def setup_publisher(self):
        self.publisher_setup_calls += 1
        return self.publisher_setup

    def setup_subscriber(self):
        self.subscriber_setup_calls += 1
        return self.subscriber_setup

    def dds_publisher(self) -> None:
        pass

    def stop_communication(self) -> None:
        self.publishing = False
        self.subscribing = False


class _FakeDDSFactory:
    def __init__(
        self,
        fail_node_name: str | None = None,
        fail_publisher_node_name: str | None = None,
        fail_subscriber_node_name: str | None = None,
    ) -> None:
        self.fail_node_name = fail_node_name
        self.fail_publisher_node_name = fail_publisher_node_name
        self.fail_subscriber_node_name = fail_subscriber_node_name
        self.created = []

    def __call__(self, *args, **kwargs):
        node_name = kwargs.get("node_name")
        if self.fail_node_name is not None and node_name == self.fail_node_name:
            raise RuntimeError(f"constructor failed for {node_name}")
        obj = _FakeDDSObject(
            *args,
            publisher_setup=not (
                self.fail_publisher_node_name is not None
                and node_name == self.fail_publisher_node_name
            ),
            subscriber_setup=not (
                self.fail_subscriber_node_name is not None
                and node_name == self.fail_subscriber_node_name
            ),
            **kwargs,
        )
        self.created.append(obj)
        return obj


class _FakeDDSManager:
    def __init__(
        self,
        *,
        reject_name: str | None = None,
        start_failure: str | None = None,
    ) -> None:
        self.reject_name = reject_name
        self.start_failure = start_failure
        self.objects = {}
        self.rejected_objects = []
        self.publish_names = None
        self.subscribe_names = None
        self.cleanup_calls = 0

    def register_object(self, name, obj) -> bool:
        if name == self.reject_name:
            self.rejected_objects.append(obj)
            return False
        self.objects[name] = obj
        return True

    def start_publishing(self, names) -> None:
        self.publish_names = list(names)
        if self.start_failure == "publishing":
            raise RuntimeError("start_publishing failed")

    def start_subscribing(self, names) -> None:
        self.subscribe_names = list(names)
        if self.start_failure == "subscribing":
            raise RuntimeError("start_subscribing failed")

    def cleanup(self) -> None:
        self.cleanup_calls += 1
        for obj in self.objects.values():
            for attribute_name in ("input_shm", "output_shm"):
                shared_memory = getattr(obj, attribute_name, None)
                if shared_memory is not None:
                    shared_memory.cleanup()
                    setattr(obj, attribute_name, None)
        self.objects.clear()


def _fake_module(name: str, **attributes) -> ModuleType:
    module = ModuleType(name)
    for attribute_name, value in attributes.items():
        setattr(module, attribute_name, value)
    return module


def _load_fake_runtime_dds_master():
    unitree_package = _fake_module("unitree_sdk2py")
    unitree_package.__path__ = []
    unitree_core_package = _fake_module("unitree_sdk2py.core")
    unitree_core_package.__path__ = []
    fake_modules = {
        "unitree_sdk2py": unitree_package,
        "unitree_sdk2py.core": unitree_core_package,
        "unitree_sdk2py.core.channel": _fake_module(
            "unitree_sdk2py.core.channel",
            ChannelFactoryInitialize=lambda *args: None,
        ),
        "dds.dds_base": _fake_module("dds.dds_base", DDSObject=object),
    }
    module_name = f"_test_dds_master_{id(fake_modules)}"
    spec = importlib.util.spec_from_file_location(module_name, DDS_MASTER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    with mock.patch.dict(sys.modules, fake_modules):
        spec.loader.exec_module(module)
    return module


def _call_create_dds_objects(
    manager: _FakeDDSManager,
    *,
    g1_factory=None,
    dex3_factory=None,
):
    g1_factory = g1_factory or _FakeDDSFactory()
    dex3_factory = dex3_factory or _FakeDDSFactory()
    fake_modules = {
        "dds.dds_master": _fake_module("dds.dds_master", dds_manager=manager),
        "dds.g1_robot_dds": _fake_module(
            "dds.g1_robot_dds", G1RobotDDS=g1_factory
        ),
        "dds.dex3_dds": _fake_module("dds.dex3_dds", Dex3DDS=dex3_factory),
        "dds.reset_pose_dds": _fake_module(
            "dds.reset_pose_dds", ResetPoseCmdDDS=_FakeDDSFactory()
        ),
        "dds.sim_state_dds": _fake_module(
            "dds.sim_state_dds", SimStateDDS=_FakeDDSFactory()
        ),
        "dds.rewards_dds": _fake_module(
            "dds.rewards_dds", RewardsDDS=_FakeDDSFactory()
        ),
    }
    args = SimpleNamespace(
        robot_type="g129",
        enable_dex3_dds=True,
        enable_dex1_dds=False,
        enable_inspire_dds=False,
        enable_wholebody_dds=False,
        sonic_robot_count=5,
        task="Isaac-G1-29DoF-Sonic-Conveyor",
    )
    module_name = f"_test_dds_create_{id(manager)}"
    spec = importlib.util.spec_from_file_location(module_name, DDS_CREATE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    with mock.patch.dict(sys.modules, fake_modules):
        spec.loader.exec_module(module)
        return module.create_dds_objects(args, object())


class SonicChannelNamingTest(unittest.TestCase):
    def test_five_channels_have_unique_scene_dds_topic_and_shm_names(self) -> None:
        specs = sonic_robot_channel_specs(5)

        self.assertEqual([spec.asset_name for spec in specs], [
            "robot", "robot_2", "robot_3", "robot_4", "robot_5"
        ])
        self.assertEqual([spec.prim_name for spec in specs], [
            "Robot", "Robot2", "Robot3", "Robot4", "Robot5"
        ])
        self.assertEqual([spec.robot_dds_name for spec in specs], [
            "g129", "g129_r2", "g129_r3", "g129_r4", "g129_r5"
        ])
        self.assertEqual([spec.dex3_dds_name for spec in specs], [
            "dex3", "dex3_r2", "dex3_r3", "dex3_r4", "dex3_r5"
        ])
        self.assertEqual([spec.topic_prefix for spec in specs], [
            "rt", "rt/r2", "rt/r3", "rt/r4", "rt/r5"
        ])
        self.assertEqual([spec.shm_suffix for spec in specs], [
            "", "_r2", "_r3", "_r4", "_r5"
        ])
        for field in (
            "asset_name", "prim_name", "global_name", "robot_dds_name",
            "dex3_dds_name", "foot_contact_name", "topic_prefix", "shm_suffix",
        ):
            values = [getattr(spec, field) for spec in specs]
            self.assertEqual(len(values), len(set(values)), field)

    def test_five_robot_action_layout_is_15_terms_and_645_dimensions(self) -> None:
        terms = sonic_host_action_terms(5)

        self.assertEqual(len(terms), 15)
        self.assertEqual(terms[:3], ("joint_pos", "joint_vel", "joint_effort"))
        self.assertEqual(terms[-3:], ("joint_pos_5", "joint_vel_5", "joint_effort_5"))
        self.assertEqual(len(terms) * 43, 645)


class SonicRobotCountResolverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.sync_identity = _load_sync_identity()

    def test_accepts_staged_counts_two_through_five(self) -> None:
        for count in range(2, 6):
            with self.subTest(count=count), mock.patch.dict(
                os.environ, {"ISAACLAB_SONIC_ROBOT_COUNT": str(count)}, clear=False
            ):
                self.assertEqual(
                    self.sync_identity.resolve_sonic_robot_count(load_env=False), count
                )

    def test_invalid_count_falls_back_to_historical_two(self) -> None:
        for value in ("one", "1", "6"):
            with self.subTest(value=value), mock.patch.dict(
                os.environ, {"ISAACLAB_SONIC_ROBOT_COUNT": value}, clear=False
            ):
                self.assertEqual(
                    self.sync_identity.resolve_sonic_robot_count(load_env=False), 2
                )

    def test_active_count_follows_layout_and_pushcart_falls_back_to_two(self) -> None:
        for count in (3, 5):
            for layout, expected in (
                ("1", count),
                ("true", count),
                ("0", 2),
                ("off", 2),
            ):
                with self.subTest(count=count, layout=layout), mock.patch.dict(
                    os.environ,
                    {
                        "ISAACLAB_SONIC_ROBOT_COUNT": str(count),
                        "ISAACLAB_TOTES_ON_CONVEYOR": layout,
                    },
                    clear=False,
                ):
                    self.assertEqual(
                        self.sync_identity.resolve_active_sonic_robot_count(
                            load_env=False
                        ),
                        expected,
                    )


class DDSManagerStartupTest(unittest.TestCase):
    def _manager_with_objects(self, *objects):
        manager = _load_fake_runtime_dds_master().dds_manager
        for index, obj in enumerate(objects, start=1):
            self.assertTrue(manager.register_object(f"channel_{index}", obj))
        return manager

    def test_publisher_false_rolls_back_flags_list_and_running_state(self) -> None:
        first = _FakeDDSObject()
        rejected = _FakeDDSObject(publisher_setup=False)
        untouched = _FakeDDSObject()
        manager = self._manager_with_objects(first, rejected, untouched)
        try:
            with self.assertRaisesRegex(RuntimeError, "channel_2"):
                manager.start_publishing()

            self.assertFalse(manager.publishing_running)
            self.assertEqual(manager._pub_list, [])
            self.assertIsNone(manager.publish_thread)
            self.assertFalse(first.publishing)
            self.assertFalse(rejected.publishing)
            self.assertFalse(untouched.publishing)
            self.assertEqual(first.publisher_setup_calls, 1)
            self.assertEqual(rejected.publisher_setup_calls, 1)
            self.assertEqual(untouched.publisher_setup_calls, 0)
        finally:
            manager.cleanup()

    def test_subscriber_false_rolls_back_flags_and_running_state(self) -> None:
        first = _FakeDDSObject()
        rejected = _FakeDDSObject(subscriber_setup=False)
        untouched = _FakeDDSObject()
        manager = self._manager_with_objects(first, rejected, untouched)
        try:
            with self.assertRaisesRegex(RuntimeError, "channel_2"):
                manager.start_subscribing()

            self.assertFalse(manager.subscribing_running)
            self.assertFalse(first.subscribing)
            self.assertFalse(rejected.subscribing)
            self.assertFalse(untouched.subscribing)
            self.assertEqual(first.subscriber_setup_calls, 1)
            self.assertEqual(rejected.subscriber_setup_calls, 1)
            self.assertEqual(untouched.subscriber_setup_calls, 0)
        finally:
            manager.cleanup()

    def test_successful_start_preserves_all_enabled_channels(self) -> None:
        first = _FakeDDSObject()
        second = _FakeDDSObject()
        manager = self._manager_with_objects(first, second)
        try:
            manager.start_publishing(["channel_1", "channel_2"])
            manager.start_subscribing(["channel_1", "channel_2"])

            self.assertTrue(manager.publishing_running)
            self.assertTrue(manager.subscribing_running)
            self.assertEqual(manager._pub_list, ["channel_1", "channel_2"])
            self.assertTrue(first.publishing and second.publishing)
            self.assertTrue(first.subscribing and second.subscribing)
        finally:
            manager.cleanup()


class DDSManagerImmediatePublishTest(unittest.TestCase):
    @staticmethod
    def _prepare_publish_loop(runtime_module, publisher):
        manager = runtime_module.dds_manager
        manager.register_object("channel", publisher)
        publisher.publishing = True
        manager._pub_list = ["channel"]
        manager._pub_interval["channel"] = 1.0
        manager._pub_next_ts["channel"] = 0.0
        return manager

    def test_regular_next_due_is_arranged_before_publisher_runs(self) -> None:
        runtime_module = _load_fake_runtime_dds_master()

        class InspectingPublisher(_FakeDDSObject):
            def __init__(self):
                super().__init__()
                self.observed_due = None

            def dds_publisher(self) -> None:
                self.observed_due = manager._pub_next_ts["channel"]
                manager.publishing_running = False

        publisher = InspectingPublisher()
        manager = self._prepare_publish_loop(runtime_module, publisher)
        manager.publishing_running = True
        manager._wake_event = mock.Mock()
        manager._wake_event.wait.return_value = False

        try:
            with mock.patch.object(
                runtime_module.time, "perf_counter", return_value=100.0
            ):
                manager._publish_loop()

            self.assertEqual(publisher.observed_due, 101.0)
            self.assertEqual(manager._pub_next_ts["channel"], 101.0)
        finally:
            manager.cleanup()

    def test_notification_during_publish_remains_due_after_return(self) -> None:
        runtime_module = _load_fake_runtime_dds_master()
        publisher_entered = threading.Event()
        release_publisher = threading.Event()

        class BlockingPublisher(_FakeDDSObject):
            def __init__(self):
                super().__init__()
                self.publish_calls = 0

            def dds_publisher(self) -> None:
                self.publish_calls += 1
                if self.publish_calls == 1:
                    publisher_entered.set()
                    release_publisher.wait()

        class StopWhenIdleEvent:
            """Consume one explicit wake, then stop instead of wall-clock sleeping."""

            def __init__(self, manager):
                self._event = threading.Event()
                self._manager = manager

            def set(self) -> None:
                self._event.set()

            def clear(self) -> None:
                self._event.clear()

            def wait(self, timeout=None) -> bool:
                if self._event.is_set():
                    return True
                self._manager.publishing_running = False
                return False

        publisher = BlockingPublisher()
        manager = self._prepare_publish_loop(runtime_module, publisher)
        manager.enable_immediate_publish("channel")
        manager._wake_event = StopWhenIdleEvent(manager)
        manager.publishing_running = True
        publish_thread = threading.Thread(target=manager._publish_loop)

        try:
            with mock.patch.object(
                runtime_module.time, "perf_counter", return_value=100.0
            ):
                publish_thread.start()
                self.assertTrue(publisher_entered.wait(timeout=1.0))
                manager.notify_fresh_sample("channel")
                release_publisher.set()
                publish_thread.join(timeout=1.0)

            self.assertFalse(publish_thread.is_alive())
            self.assertEqual(publisher.publish_calls, 2)
            self.assertEqual(manager._pub_next_ts["channel"], 101.0)
        finally:
            release_publisher.set()
            manager.publishing_running = False
            manager._wake_event.set()
            publish_thread.join(timeout=1.0)
            manager.cleanup()


class DDSCreateRollbackTest(unittest.TestCase):
    def test_all_extra_channels_reject_false_and_cleanup_before_start(self) -> None:
        required_names = [
            channel_name
            for spec in sonic_robot_channel_specs(5)[1:]
            for channel_name in (spec.robot_dds_name, spec.dex3_dds_name)
        ]

        for rejected_name in required_names:
            with self.subTest(rejected_name=rejected_name):
                manager = _FakeDDSManager(reject_name=rejected_name)
                with self.assertRaisesRegex(RuntimeError, rejected_name):
                    _call_create_dds_objects(manager)

                self.assertEqual(manager.cleanup_calls, 1)
                self.assertIsNone(manager.publish_names)
                self.assertIsNone(manager.subscribe_names)
                self.assertEqual(len(manager.rejected_objects), 1)
                rejected_object = manager.rejected_objects[0]
                self.assertIsNone(rejected_object.input_shm)
                self.assertIsNone(rejected_object.output_shm)
                self.assertEqual(rejected_object.input_shm_handle.cleanup_calls, 1)
                self.assertEqual(rejected_object.output_shm_handle.cleanup_calls, 1)

    def test_extra_constructor_exception_rolls_back_before_start(self) -> None:
        manager = _FakeDDSManager()
        g1_factory = _FakeDDSFactory(fail_node_name="g1_robot_r4")

        with self.assertRaisesRegex(RuntimeError, "constructor failed for g1_robot_r4"):
            _call_create_dds_objects(manager, g1_factory=g1_factory)

        self.assertEqual(manager.cleanup_calls, 1)
        self.assertIsNone(manager.publish_names)
        self.assertIsNone(manager.subscribe_names)
        self.assertEqual(manager.objects, {})

    def test_start_exceptions_roll_back_and_preserve_original_error(self) -> None:
        for phase in ("publishing", "subscribing"):
            with self.subTest(phase=phase):
                manager = _FakeDDSManager(start_failure=phase)
                with self.assertRaisesRegex(RuntimeError, f"start_{phase} failed"):
                    _call_create_dds_objects(manager)

                self.assertEqual(manager.cleanup_calls, 1)
                self.assertIsNotNone(manager.publish_names)
                if phase == "publishing":
                    self.assertIsNone(manager.subscribe_names)
                else:
                    self.assertIsNotNone(manager.subscribe_names)
                self.assertEqual(manager.objects, {})

    def test_setup_false_from_real_manager_triggers_factory_rollback(self) -> None:
        cases = (
            (
                "publisher",
                _FakeDDSFactory(fail_publisher_node_name="g1_robot_r4"),
                _FakeDDSFactory(),
                "g129_r4",
            ),
            (
                "subscriber",
                _FakeDDSFactory(),
                _FakeDDSFactory(fail_subscriber_node_name="dex3_r5"),
                "dex3_r5",
            ),
        )
        for phase, g1_factory, dex3_factory, rejected_name in cases:
            with self.subTest(phase=phase, rejected_name=rejected_name):
                manager = _load_fake_runtime_dds_master().dds_manager
                with self.assertRaisesRegex(RuntimeError, rejected_name):
                    _call_create_dds_objects(
                        manager,
                        g1_factory=g1_factory,
                        dex3_factory=dex3_factory,
                    )

                self.assertFalse(manager.publishing_running)
                self.assertFalse(manager.subscribing_running)
                self.assertEqual(manager._pub_list, [])
                self.assertIsNone(manager.publish_thread)
                self.assertEqual(manager.objects, {})
                created_objects = g1_factory.created + dex3_factory.created
                self.assertTrue(created_objects)
                for obj in created_objects:
                    self.assertIsNone(obj.input_shm)
                    self.assertIsNone(obj.output_shm)
                    self.assertEqual(obj.input_shm_handle.cleanup_calls, 1)
                    self.assertEqual(obj.output_shm_handle.cleanup_calls, 1)


class FiveRobotWiringSourceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg_source = CONVEYOR_CFG_PATH.read_text(encoding="utf-8")
        cls.sim_source = SIM_MAIN_PATH.read_text(encoding="utf-8")
        cls.dds_source = DDS_CREATE_PATH.read_text(encoding="utf-8")

    def test_scene_promotes_three_standby_slots_to_articulations(self) -> None:
        self.assertIn("def _make_additional_local_robot_cfg(robot_id: int)", self.cfg_source)
        for robot_id in range(3, 6):
            self.assertIn(f"robot_{robot_id}: ArticulationCfg | None", self.cfg_source)
            self.assertIn(f"foot_contact_{robot_id}: ContactSensorCfg | None", self.cfg_source)
            self.assertIn(f"peer_robot_{robot_id}: ArticulationCfg | AssetBaseCfg | None", self.cfg_source)
        self.assertIn(
            "_ACTIVE_SONIC_EXTRA_POSE_INDICES = sonic_active_extra_pose_indices(",
            self.cfg_source,
        )
        self.assertIn(
            "_STATIC_SONIC_EXTRA_POSE_INDICES = sonic_standby_pose_indices(",
            self.cfg_source,
        )
        self.assertIn("pose_index = sonic_extra_robot_pose_index(robot_id)", self.cfg_source)
        for pose_index in range(3):
            self.assertIn(
                f"if {pose_index} in _STATIC_SONIC_EXTRA_POSE_INDICES",
                self.cfg_source,
            )

    def test_scene_publishes_and_viewer_applies_all_configured_robots(self) -> None:
        self.assertIn(
            "spec.global_name: spec.asset_name for spec in ACTIVE_SONIC_CHANNEL_SPECS",
            self.cfg_source,
        )
        self.assertIn("for spec in ACTIVE_SONIC_CHANNEL_SPECS", self.cfg_source)
        self.assertIn("ACTIVE_SONIC_ROBOT_COUNT >= 5", self.cfg_source)

    def test_dds_seed_publish_and_reset_paths_use_shared_specs(self) -> None:
        self.assertIn(
            "extra_specs = sonic_robot_channel_specs(int(sonic_robot_count))[1:]",
            self.dds_source,
        )
        self.assertIn("for spec in extra_specs", self.dds_source)
        self.assertIn("topic_prefix=spec.topic_prefix", self.dds_source)
        self.assertIn("shm_suffix=spec.shm_suffix", self.dds_source)
        self.assertIn("for _spec in sonic_host_channel_specs", self.sim_source)
        self.assertIn(
            "reset_dds_names = [spec.robot_dds_name for spec in sonic_host_channel_specs]",
            self.sim_source,
        )
        self.assertIn(
            "dex3_dds_names = [spec.dex3_dds_name for spec in sonic_host_channel_specs]",
            self.sim_source,
        )
        self.assertIn("args_cli.handstate_pub_hz", self.sim_source)
        self.assertIn(
            'parser.error("--handstate_pub_hz must be a finite positive value")',
            self.sim_source,
        )
        self.assertIn(
            'parser.error("--handstate_pub_hz is supported only for SONIC Dex3 tasks")',
            self.sim_source,
        )
        self.assertIn(
            'parser.error("--handstate_pub_hz is unavailable with replay_data")',
            self.sim_source,
        )
        self.assertIn("for _dds_name in dex3_dds_names", self.sim_source)
        self.assertIn(
            "if runtime_dds_enabled and args_cli.task in sonic_dex3_task_names:\n"
            "            # A fresh hand sample is tied to the same completed PhysX step as\n"
            "            # LowState.  Wake the publisher immediately",
            self.sim_source,
        )

    def test_shared_scene_config_defaults_to_three_robot_topology(self) -> None:
        source = SCENE_SYNC_ENV_PATH.read_text(encoding="utf-8")
        self.assertIn("ISAACLAB_SONIC_ROBOT_COUNT=3", source)


if __name__ == "__main__":
    unittest.main()
