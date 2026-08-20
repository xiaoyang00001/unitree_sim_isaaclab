"""LowState DDS authority guard contracts without touching production domains."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

import dds.dds_authority_guard as authority_guard
from dds.dds_authority_guard import (
    DEFAULT_DISCOVERY_TIMEOUT_S,
    DDSAuthorityConflictError,
    DDSAuthorityError,
    claim_lowstate_authority,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DDS_CREATE_PATH = REPO_ROOT / "dds/dds_create.py"
TEST_DOMAIN = 181


class _StaticReader:
    def __init__(self, samples=(), error: Exception | None = None) -> None:
        self.samples = list(samples)
        self.error = error
        self.read_calls = 0

    def read(self, limit):
        self.read_calls += 1
        if self.error is not None:
            raise self.error
        return list(self.samples)


def _sample_info(*, alive: bool = True):
    return SimpleNamespace(valid_data=True, instance_state=16 if alive else 32)


def _publication(
    topic: str,
    *,
    writer_key: str = "writer-guid",
    participant_key: str = "participant-guid",
    alive: bool = True,
):
    return SimpleNamespace(
        key=writer_key,
        participant_key=participant_key,
        topic_name=topic,
        type_name="unitree_hg::msg::dds_::LowState_",
        sample_info=_sample_info(alive=alive),
    )


def _participant(
    *,
    key: str = "participant-guid",
    pid: str = "43210",
    process: str = "deploy",
    hostname: str = "sonic-host",
):
    qos = [
        SimpleNamespace(key="__Pid", value=pid),
        SimpleNamespace(key="__ProcessName", value=process),
        SimpleNamespace(key="__Hostname", value=hostname),
    ]
    return SimpleNamespace(key=key, qos=qos, sample_info=_sample_info())


def _fake_discovery(publications=(), participants=()):
    publication_reader = _StaticReader(publications)
    participant_reader = _StaticReader(participants)
    return {
        "participant_factory": lambda domain_id: SimpleNamespace(
            domain_id=domain_id, guid="probe-participant"
        ),
        "reader_factory": lambda participant: (
            publication_reader,
            participant_reader,
        ),
        "owned_participant_key_factory": lambda: "owned-business-participant",
        "discovery_timeout_s": 0.0,
    }


class DDSAuthorityDiscoveryTest(unittest.TestCase):
    def test_default_discovery_window_keeps_field_margin(self) -> None:
        self.assertEqual(DEFAULT_DISCOVERY_TIMEOUT_S, 1.5)

    def test_default_lock_namespace_does_not_follow_xdg_runtime_dir(self) -> None:
        with mock.patch.dict(
            os.environ, {"XDG_RUNTIME_DIR": "/tmp/authority-a"}, clear=False
        ):
            first = authority_guard._default_lock_directory()
        with mock.patch.dict(
            os.environ, {"XDG_RUNTIME_DIR": "/tmp/authority-b"}, clear=False
        ):
            second = authority_guard._default_lock_directory()

        self.assertEqual(first, second)
        runtime_parent = Path("/run/user") / str(os.getuid())
        expected = (
            runtime_parent / "unitree_sim_isaaclab"
            if runtime_parent.is_dir()
            and os.access(runtime_parent, os.W_OK | os.X_OK)
            else Path(tempfile.gettempdir())
            / f"unitree_sim_isaaclab-{os.getuid()}"
        )
        self.assertEqual(first, expected)

    def test_same_process_reclaim_repeats_discovery_and_detects_new_writer(self) -> None:
        topics = ("rt/lowstate", "rt/r2/lowstate")
        with tempfile.TemporaryDirectory() as lock_directory:
            first = claim_lowstate_authority(
                topics,
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                **_fake_discovery(),
            )

            with self.assertRaisesRegex(
                DDSAuthorityConflictError, "alive LowState DataWriter"
            ):
                claim_lowstate_authority(
                    topics,
                    domain_id=TEST_DOMAIN,
                    lock_directory=lock_directory,
                    **_fake_discovery(
                        [_publication("rt/r2/lowstate")], [_participant()]
                    ),
                )

        self.assertEqual(first.topics, topics)
        self.assertEqual(first.domain_id, TEST_DOMAIN)
        self.assertEqual(first.pid, os.getpid())

    def test_reclaim_ignores_only_owned_and_probe_participant_guids(self) -> None:
        topic = "rt/owned_reentry/lowstate"
        with tempfile.TemporaryDirectory() as lock_directory:
            claim_lowstate_authority(
                [topic],
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                **_fake_discovery(),
            )

            reclaim = claim_lowstate_authority(
                [topic],
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                **_fake_discovery(
                    (
                        _publication(
                            topic,
                            writer_key="owned-writer",
                            participant_key="owned-business-participant",
                        ),
                        _publication(
                            topic,
                            writer_key="probe-writer",
                            participant_key="probe-participant",
                        ),
                    )
                ),
            )
            self.assertEqual(reclaim.topics, (topic,))

            with self.assertRaisesRegex(
                DDSAuthorityConflictError, "participant=external-participant"
            ):
                claim_lowstate_authority(
                    [topic],
                    domain_id=TEST_DOMAIN,
                    lock_directory=lock_directory,
                    **_fake_discovery(
                        [
                            _publication(
                                topic,
                                writer_key="external-writer",
                                participant_key="external-participant",
                            )
                        ],
                        [_participant(key="external-participant")],
                    ),
                )

    def test_real_cyclone_reclaim_ignores_owned_writer_but_rejects_external(self) -> None:
        try:
            from cyclonedds.domain import DomainParticipant
            from cyclonedds.pub import DataWriter
            from cyclonedds.topic import Topic
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        except Exception as exc:  # pragma: no cover - production env has bindings.
            self.skipTest(f"CycloneDDS integration unavailable: {exc}")

        domain_id = 150 + os.getpid() % 20
        topic_name = f"rt/authority_guard_test_{os.getpid()}/lowstate"
        business_participant = DomainParticipant(domain_id)
        owned_key_factory = lambda: business_participant.guid

        with tempfile.TemporaryDirectory() as lock_directory:
            claim_lowstate_authority(
                [topic_name],
                domain_id=domain_id,
                lock_directory=lock_directory,
                discovery_timeout_s=0.1,
                owned_participant_key_factory=owned_key_factory,
            )
            topic = Topic(business_participant, topic_name, LowState_)
            owned_writer = DataWriter(business_participant, topic)

            claim_lowstate_authority(
                [topic_name],
                domain_id=domain_id,
                lock_directory=lock_directory,
                discovery_timeout_s=0.1,
                owned_participant_key_factory=owned_key_factory,
            )

            external_participant = DomainParticipant(domain_id)
            external_topic = Topic(external_participant, topic_name, LowState_)
            external_writer = DataWriter(external_participant, external_topic)
            with self.assertRaisesRegex(
                DDSAuthorityConflictError, "alive LowState DataWriter"
            ):
                claim_lowstate_authority(
                    [topic_name],
                    domain_id=domain_id,
                    lock_directory=lock_directory,
                    discovery_timeout_s=1.0,
                    owned_participant_key_factory=owned_key_factory,
                )

        self.assertIsNotNone(owned_writer)
        self.assertIsNotNone(external_writer)

    def test_alive_exact_writer_fails_closed_with_builtin_owner_diagnostics(self) -> None:
        topic = "rt/r4/lowstate"
        dependencies = _fake_discovery(
            [_publication(topic)],
            [_participant(pid="24680", process="groot_deploy", hostname="pico")],
        )
        with tempfile.TemporaryDirectory() as lock_directory:
            with self.assertRaises(DDSAuthorityConflictError) as raised:
                claim_lowstate_authority(
                    [topic],
                    domain_id=TEST_DOMAIN,
                    lock_directory=lock_directory,
                    **dependencies,
                )

            # A failed discovery claim must release the provisional flock.
            claim_lowstate_authority(
                [topic],
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                **_fake_discovery(),
            )

        diagnostic = str(raised.exception)
        self.assertIn("alive LowState DataWriter", diagnostic)
        self.assertIn("pid=24680", diagnostic)
        self.assertIn("process='groot_deploy'", diagnostic)
        self.assertIn("host='pico'", diagnostic)
        self.assertIn("writer=writer-guid", diagnostic)
        self.assertIn(f"domain={TEST_DOMAIN}", diagnostic)

    def test_disposed_exact_writer_and_alive_other_topic_do_not_conflict(self) -> None:
        target = "rt/r5/lowstate"
        publications = (
            _publication(target, writer_key="disposed", alive=False),
            _publication("rt/r4/lowstate", writer_key="other", alive=True),
        )
        with tempfile.TemporaryDirectory() as lock_directory:
            claim = claim_lowstate_authority(
                [target],
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                **_fake_discovery(publications, [_participant()]),
            )
        self.assertEqual(claim.topics, (target,))

    def test_builtin_reader_error_fails_closed_and_releases_flock(self) -> None:
        topic = "rt/r3/lowstate"
        failing_dependencies = _fake_discovery()
        failing_dependencies["reader_factory"] = lambda participant: (
            _StaticReader(error=OSError("builtin read failed")),
            _StaticReader(),
        )
        with tempfile.TemporaryDirectory() as lock_directory:
            with self.assertRaisesRegex(
                DDSAuthorityError, "Builtin discovery failed.*refusing startup"
            ):
                claim_lowstate_authority(
                    [topic],
                    domain_id=TEST_DOMAIN,
                    lock_directory=lock_directory,
                    **failing_dependencies,
                )

            claim_lowstate_authority(
                [topic],
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                **_fake_discovery(),
            )


class DDSAuthorityFlockTest(unittest.TestCase):
    @unittest.skipUnless(hasattr(os, "fork"), "requires POSIX os.fork")
    def test_fork_during_discovery_drops_provisional_fd_and_rebuilds_mutex(self) -> None:
        topic = "rt/fork_contract/lowstate"
        child_exit_codes: list[int] = []

        def fork_while_provisional(domain_id):
            pending_fds = tuple(authority_guard._PROVISIONAL_TOPIC_LOCKS)
            self.assertTrue(pending_fds)
            child_pid = os.fork()
            if child_pid == 0:
                exit_code = 0
                try:
                    if authority_guard._HELD_TOPIC_LOCKS:
                        exit_code = 31
                    elif authority_guard._PROVISIONAL_TOPIC_LOCKS:
                        exit_code = 32
                    elif not authority_guard._CLAIM_MUTEX.acquire(blocking=False):
                        exit_code = 33
                    else:
                        authority_guard._CLAIM_MUTEX.release()
                        for fd in pending_fds:
                            try:
                                os.fstat(fd)
                            except OSError:
                                continue
                            exit_code = 34
                            break
                except BaseException:
                    exit_code = 35
                os._exit(exit_code)

            _, wait_status = os.waitpid(child_pid, 0)
            child_exit_codes.append(os.waitstatus_to_exitcode(wait_status))
            return SimpleNamespace(domain_id=domain_id, guid="fork-probe")

        with tempfile.TemporaryDirectory() as lock_directory:
            claim_lowstate_authority(
                [topic],
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                discovery_timeout_s=0.0,
                participant_factory=fork_while_provisional,
                reader_factory=lambda participant: (_StaticReader(), _StaticReader()),
                owned_participant_key_factory=lambda: "fork-owned",
            )

        self.assertEqual(child_exit_codes, [0])
        self.assertEqual(authority_guard._PROVISIONAL_TOPIC_LOCKS, {})

    def test_subprocess_cannot_pass_local_topic_lock_and_reports_owner(self) -> None:
        topic = "rt/r2/lowstate"
        with tempfile.TemporaryDirectory() as lock_directory:
            claim_lowstate_authority(
                [topic],
                domain_id=TEST_DOMAIN,
                lock_directory=lock_directory,
                **_fake_discovery(),
            )
            child_source = """
from dds.dds_authority_guard import claim_lowstate_authority

class EmptyReader:
    def read(self, limit):
        return []

try:
    claim_lowstate_authority(
        [TOPIC],
        domain_id=DOMAIN,
        lock_directory=LOCK_DIRECTORY,
        discovery_timeout_s=0.0,
        participant_factory=lambda domain_id: object(),
        reader_factory=lambda participant: (EmptyReader(), EmptyReader()),
    )
except Exception as exc:
    print(type(exc).__name__ + ": " + str(exc))
    raise SystemExit(23)
raise SystemExit(0)
""".replace("TOPIC", repr(topic)).replace(
                "DOMAIN", str(TEST_DOMAIN)
            ).replace(
                "LOCK_DIRECTORY", repr(lock_directory)
            )
            completed = subprocess.run(
                [sys.executable, "-c", child_source],
                cwd=REPO_ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=10.0,
                check=False,
            )

        self.assertEqual(completed.returncode, 23, completed.stdout)
        self.assertIn("DDSAuthorityConflictError", completed.stdout)
        self.assertIn(f"pid={os.getpid()}", completed.stdout)
        self.assertIn(f"topic={topic!r}", completed.stdout)


class _FakeSharedMemory:
    def __init__(self) -> None:
        self.cleanup_calls = 0

    def cleanup(self) -> None:
        self.cleanup_calls += 1


class _FakeDDSObject:
    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs
        self.input_shm = _FakeSharedMemory()
        self.output_shm = _FakeSharedMemory()
        self.input_shm_handle = self.input_shm
        self.output_shm_handle = self.output_shm


class _FakeDDSManager:
    dds_initialized = True
    dds_domain = TEST_DOMAIN

    def __init__(
        self,
        events: list[tuple],
        *,
        reject_name: str | None = None,
        start_error: str | None = None,
    ) -> None:
        self.events = events
        self.objects = {}
        self.cleanup_calls = 0
        self.reject_name = reject_name
        self.start_error = start_error
        self.g1_objects: list[_FakeDDSObject] = []

    def register_object(self, name, obj) -> bool:
        if name == self.reject_name:
            return False
        self.objects[name] = obj
        return True

    def start_publishing(self, names) -> None:
        self.events.append(("start_publishing", tuple(names)))
        if self.start_error == "publishing":
            raise RuntimeError("replay publishing start failed")

    def start_subscribing(self, names) -> None:
        self.events.append(("start_subscribing", tuple(names)))
        if self.start_error == "subscribing":
            raise RuntimeError("replay subscribing start failed")

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


def _args(**overrides):
    values = {
        "robot_type": "g129",
        "enable_dex3_dds": False,
        "enable_dex1_dds": False,
        "enable_inspire_dds": False,
        "enable_wholebody_dds": False,
        "sonic_robot_count": 4,
        "task": "Isaac-G1-29DoF-Sonic-Conveyor",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class DDSAuthorityCreateWiringTest(unittest.TestCase):
    def test_normal_path_claims_all_exact_sonic_topics_before_g1_construction(self) -> None:
        module, manager, events = _invoke_normal_create()

        expected_topics = (
            "rt/lowstate",
            "rt/r2/lowstate",
            "rt/r3/lowstate",
            "rt/r4/lowstate",
        )
        self.assertEqual(events[0], ("claim", expected_topics, TEST_DOMAIN))
        constructor_indexes = [
            index for index, event in enumerate(events) if event[0] == "g1_constructor"
        ]
        self.assertEqual(len(constructor_indexes), 4)
        self.assertTrue(all(index > 0 for index in constructor_indexes))
        self.assertGreater(
            next(i for i, event in enumerate(events) if event[0] == "start_publishing"),
            max(constructor_indexes),
        )
        self.assertEqual(manager.cleanup_calls, 0)

    def test_guard_failure_stops_before_any_g1_constructor_or_start(self) -> None:
        failure = DDSAuthorityConflictError("writer already alive")
        module, manager, events, fake_modules = _prepare_dds_create_fakes(
            claim_error=failure
        )
        with mock.patch.dict(sys.modules, fake_modules):
            with self.assertRaisesRegex(
                DDSAuthorityConflictError, "writer already alive"
            ):
                module.create_dds_objects(_args(), object())

        self.assertEqual(
            events,
            [
                (
                    "claim",
                    (
                        "rt/lowstate",
                        "rt/r2/lowstate",
                        "rt/r3/lowstate",
                        "rt/r4/lowstate",
                    ),
                    TEST_DOMAIN,
                )
            ],
        )
        self.assertEqual(manager.cleanup_calls, 1)

    def test_normal_primary_registration_false_cleans_candidate_and_rolls_back(self) -> None:
        module, manager, events, fake_modules = _prepare_dds_create_fakes(
            reject_name="g129"
        )
        with mock.patch.dict(sys.modules, fake_modules):
            with self.assertRaisesRegex(RuntimeError, "required DDS object 'g129'"):
                module.create_dds_objects(_args(), object())

        self.assertEqual(events[0][0], "claim")
        self.assertEqual(events[1], ("g1_constructor", "rt"))
        self.assertEqual(len(events), 2)
        self.assertEqual(manager.cleanup_calls, 1)
        candidate = manager.g1_objects[0]
        self.assertIsNone(candidate.input_shm)
        self.assertIsNone(candidate.output_shm)
        self.assertEqual(candidate.input_shm_handle.cleanup_calls, 1)
        self.assertEqual(candidate.output_shm_handle.cleanup_calls, 1)

    def test_replay_claims_only_rt_lowstate_before_constructor(self) -> None:
        module, manager, events, fake_modules = _prepare_dds_create_fakes()
        with mock.patch.dict(sys.modules, fake_modules):
            module.create_dds_objects_replay(_args(), object())

        self.assertEqual(events[0], ("claim", ("rt/lowstate",), TEST_DOMAIN))
        self.assertEqual(events[1], ("g1_constructor", "rt"))
        self.assertEqual(events[2][0], "start_publishing")

    def test_replay_constructor_failure_rolls_back_before_registration_or_start(self) -> None:
        module, manager, events, fake_modules = _prepare_dds_create_fakes(
            constructor_error=RuntimeError("replay G1 constructor failed")
        )
        with mock.patch.dict(sys.modules, fake_modules):
            with self.assertRaisesRegex(RuntimeError, "G1 constructor failed"):
                module.create_dds_objects_replay(_args(), object())

        self.assertEqual(events[0][0], "claim")
        self.assertEqual(events[1], ("g1_constructor", "rt"))
        self.assertEqual(len(events), 2)
        self.assertEqual(manager.cleanup_calls, 1)
        self.assertEqual(manager.objects, {})

    def test_replay_registration_false_cleans_candidate_and_rolls_back_before_start(self) -> None:
        module, manager, events, fake_modules = _prepare_dds_create_fakes(
            reject_name="g129"
        )
        with mock.patch.dict(sys.modules, fake_modules):
            with self.assertRaisesRegex(RuntimeError, "required DDS object 'g129'"):
                module.create_dds_objects_replay(_args(), object())

        self.assertEqual(events[0][0], "claim")
        self.assertEqual(events[1], ("g1_constructor", "rt"))
        self.assertEqual(len(events), 2)
        self.assertEqual(manager.cleanup_calls, 1)
        self.assertEqual(manager.objects, {})
        candidate = manager.g1_objects[0]
        self.assertIsNone(candidate.input_shm)
        self.assertIsNone(candidate.output_shm)
        self.assertEqual(candidate.input_shm_handle.cleanup_calls, 1)
        self.assertEqual(candidate.output_shm_handle.cleanup_calls, 1)

    def test_replay_each_start_failure_rolls_back_and_preserves_error(self) -> None:
        for phase in ("publishing", "subscribing"):
            with self.subTest(phase=phase):
                module, manager, events, fake_modules = _prepare_dds_create_fakes(
                    start_error=phase
                )
                with mock.patch.dict(sys.modules, fake_modules):
                    with self.assertRaisesRegex(
                        RuntimeError, f"replay {phase} start failed"
                    ):
                        module.create_dds_objects_replay(_args(), object())

                self.assertEqual(events[0][0], "claim")
                self.assertEqual(events[1], ("g1_constructor", "rt"))
                self.assertEqual(events[2][0], "start_publishing")
                if phase == "publishing":
                    self.assertEqual(len(events), 3)
                else:
                    self.assertEqual(events[3][0], "start_subscribing")
                self.assertEqual(manager.cleanup_calls, 1)
                self.assertEqual(manager.objects, {})


def _prepare_dds_create_fakes(
    *,
    claim_error: Exception | None = None,
    constructor_error: Exception | None = None,
    reject_name: str | None = None,
    start_error: str | None = None,
):
    events: list[tuple] = []
    manager = _FakeDDSManager(
        events,
        reject_name=reject_name,
        start_error=start_error,
    )

    class G1RobotDDS(_FakeDDSObject):
        def __init__(self, *args, **kwargs) -> None:
            events.append(("g1_constructor", kwargs.get("topic_prefix", "rt")))
            if constructor_error is not None:
                raise constructor_error
            super().__init__(*args, **kwargs)
            manager.g1_objects.append(self)

    G1RobotDDS.__module__ = "dds.g1_robot_dds"

    def fake_claim(topics, *, domain_id):
        events.append(("claim", tuple(topics), domain_id))
        if claim_error is not None:
            raise claim_error

    object_factory = lambda *args, **kwargs: _FakeDDSObject(*args, **kwargs)
    fake_modules = {
        "dds.dds_master": _fake_module("dds.dds_master", dds_manager=manager),
        "dds.dds_authority_guard": _fake_module(
            "dds.dds_authority_guard", claim_lowstate_authority=fake_claim
        ),
        "dds.g1_robot_dds": _fake_module(
            "dds.g1_robot_dds", G1RobotDDS=G1RobotDDS
        ),
        "dds.dex3_dds": _fake_module("dds.dex3_dds", Dex3DDS=object_factory),
        "dds.reset_pose_dds": _fake_module(
            "dds.reset_pose_dds", ResetPoseCmdDDS=object_factory
        ),
        "dds.sim_state_dds": _fake_module(
            "dds.sim_state_dds", SimStateDDS=object_factory
        ),
        "dds.rewards_dds": _fake_module(
            "dds.rewards_dds", RewardsDDS=object_factory
        ),
    }
    module_name = f"_test_dds_authority_create_{id(events)}"
    spec = importlib.util.spec_from_file_location(module_name, DDS_CREATE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    with mock.patch.dict(sys.modules, fake_modules):
        spec.loader.exec_module(module)
    return module, manager, events, fake_modules


def _invoke_normal_create():
    module, manager, events, fake_modules = _prepare_dds_create_fakes()
    with mock.patch.dict(sys.modules, fake_modules):
        module.create_dds_objects(_args(), object())
    return module, manager, events


if __name__ == "__main__":
    unittest.main()
