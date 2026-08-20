# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Fail-closed ownership guard for Isaac's G1 LowState publishers.

Two independent Isaac processes publishing the same ``LowState`` topic make a
SONIC deploy alternate between unrelated robot states.  DDS discovery alone is
not sufficient to prevent that: two local processes can both finish discovery
before either creates its writer.  This module therefore claims a non-blocking
``flock`` for every domain/topic first, then checks CycloneDDS publication
discovery while those locks are held.

The file lock serializes discovery plus writer construction for processes on
this host.  Builtin discovery also rejects an already-visible writer on another
host, but it is not a distributed consensus protocol: two remote hosts that
start simultaneously can both finish discovery before either writer becomes
visible.  That cross-host cold-start race requires an external coordinator.

Successful lock file descriptors intentionally live until process exit.  DDS
manager cleanup must not create a window in which a second simulator can claim
the topics while this process is still alive and able to restart publishing.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import errno
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from typing import Any, Callable, Iterable, Sequence

try:
    import fcntl
except ImportError:  # pragma: no cover - the production simulator is Linux.
    fcntl = None


DEFAULT_DISCOVERY_TIMEOUT_S = 1.5
_DISCOVERY_POLL_INTERVAL_S = 0.05
_DISCOVERY_READ_LIMIT = 65536
_OWNER_PROPERTY_KEYS = ("__Pid", "__ProcessName", "__Hostname")


class DDSAuthorityError(RuntimeError):
    """Base error for a LowState authority claim that must stop startup."""


class DDSAuthorityConflictError(DDSAuthorityError):
    """Another local claimant or an alive DDS writer already owns a topic."""


@dataclass(frozen=True)
class LowStateAuthorityClaim:
    """Description of topics owned for the lifetime of this process."""

    domain_id: int
    topics: tuple[str, ...]
    pid: int


@dataclass
class _HeldTopicLock:
    domain_id: int
    topic: str
    path: Path
    fd: int


_CLAIM_MUTEX = threading.RLock()
_HELD_TOPIC_LOCKS: dict[str, _HeldTopicLock] = {}
_PROVISIONAL_TOPIC_LOCKS: dict[int, _HeldTopicLock] = {}


def _default_lock_directory() -> Path:
    uid = os.getuid()
    runtime_dir = Path("/run/user") / str(uid)
    if runtime_dir.is_dir() and os.access(runtime_dir, os.W_OK | os.X_OK):
        return runtime_dir / "unitree_sim_isaaclab"
    return Path(tempfile.gettempdir()) / f"unitree_sim_isaaclab-{uid}"


def _normalize_topics(topics: Iterable[str]) -> tuple[str, ...]:
    normalized: list[str] = []
    seen: set[str] = set()
    for value in topics:
        topic = str(value)
        if not topic or topic != topic.strip():
            raise DDSAuthorityError(
                f"LowState authority topic must be a non-empty exact name, got {value!r}"
            )
        if topic not in seen:
            normalized.append(topic)
            seen.add(topic)
    if not normalized:
        raise DDSAuthorityError("LowState authority claim requires at least one topic")
    return tuple(normalized)


def _topic_lock_path(lock_directory: Path, domain_id: int, topic: str) -> Path:
    digest = hashlib.sha256(topic.encode("utf-8")).hexdigest()[:20]
    readable_topic = "".join(
        character if character.isalnum() else "_" for character in topic
    ).strip("_")
    readable_topic = readable_topic[-48:] or "topic"
    return lock_directory / f"domain-{domain_id}-{readable_topic}-{digest}.lock"


def _local_owner_record(domain_id: int, topic: str) -> dict[str, Any]:
    process_name = Path(sys.argv[0]).name if sys.argv else "python"
    return {
        "pid": os.getpid(),
        "process": process_name or "python",
        "hostname": socket.gethostname(),
        "domain": domain_id,
        "topic": topic,
        "claimed_at": datetime.now(timezone.utc).isoformat(),
    }


def _read_lock_owner(fd: int) -> str:
    try:
        os.lseek(fd, 0, os.SEEK_SET)
        raw = os.read(fd, 16384).decode("utf-8", errors="replace").strip()
    except OSError as exc:
        return f"unreadable ({exc})"
    if not raw:
        return "unknown (lock file contains no owner record)"
    try:
        owner = json.loads(raw)
    except (TypeError, ValueError):
        return f"unparseable ({raw!r})"
    return (
        f"pid={owner.get('pid', 'unknown')}, "
        f"process={owner.get('process', 'unknown')!r}, "
        f"host={owner.get('hostname', 'unknown')!r}, "
        f"claimed_at={owner.get('claimed_at', 'unknown')}, "
        f"domain={owner.get('domain', 'unknown')}, "
        f"topic={owner.get('topic', 'unknown')!r}"
    )


def _write_lock_owner(fd: int, owner: dict[str, Any]) -> None:
    payload = (json.dumps(owner, ensure_ascii=False, sort_keys=True) + "\n").encode(
        "utf-8"
    )
    os.lseek(fd, 0, os.SEEK_SET)
    os.ftruncate(fd, 0)
    view = memoryview(payload)
    while view:
        written = os.write(fd, view)
        if written <= 0:
            raise OSError("short write while recording DDS authority owner")
        view = view[written:]
    os.fsync(fd)


def _acquire_topic_lock(
    *, domain_id: int, topic: str, lock_directory: Path
) -> _HeldTopicLock:
    if fcntl is None:
        raise DDSAuthorityError(
            "LowState authority guard requires POSIX fcntl.flock; refusing DDS startup"
        )

    try:
        lock_directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    except OSError as exc:
        raise DDSAuthorityError(
            f"cannot create DDS authority lock directory {lock_directory}: {exc}"
        ) from exc

    path = _topic_lock_path(lock_directory, domain_id, topic)
    open_flags = os.O_RDWR | os.O_CREAT
    open_flags |= getattr(os, "O_CLOEXEC", 0)
    open_flags |= getattr(os, "O_NOFOLLOW", 0)
    fd: int | None = None
    try:
        fd = os.open(path, open_flags, 0o600)
        os.set_inheritable(fd, False)
    except OSError as exc:
        if fd is not None:
            os.close(fd)
        raise DDSAuthorityError(
            f"cannot open DDS authority lock {path}: {exc}; refusing startup"
        ) from exc

    held = _HeldTopicLock(domain_id=domain_id, topic=topic, path=path, fd=fd)
    _PROVISIONAL_TOPIC_LOCKS[fd] = held
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno not in (errno.EACCES, errno.EAGAIN):
                raise DDSAuthorityError(
                    f"cannot claim DDS authority lock {path}: {exc}; refusing startup"
                ) from exc
            owner = _read_lock_owner(fd)
            raise DDSAuthorityConflictError(
                "LowState authority is already held by a local process: "
                f"domain={domain_id}, topic={topic!r}, owner=({owner}), lock={path}"
            ) from exc

        _write_lock_owner(fd, _local_owner_record(domain_id, topic))
        return held
    except Exception:
        _PROVISIONAL_TOPIC_LOCKS.pop(fd, None)
        os.close(fd)
        raise


def _close_uncommitted_locks(locks: Sequence[_HeldTopicLock]) -> None:
    for held in reversed(locks):
        _PROVISIONAL_TOPIC_LOCKS.pop(held.fd, None)
        try:
            os.close(held.fd)
        except OSError:
            pass


def _default_participant_factory(domain_id: int):
    from cyclonedds.domain import DomainParticipant

    # dds_master has already initialized a Cyclone Domain with the same id and
    # interface configuration.  A fresh participant is sufficient for Builtin
    # discovery and does not create a business-topic DataWriter.
    return DomainParticipant(domain_id)


def _default_reader_factory(participant):
    from cyclonedds.builtin import (
        BuiltinDataReader,
        BuiltinTopicDcpsParticipant,
        BuiltinTopicDcpsPublication,
    )

    return (
        BuiltinDataReader(participant, BuiltinTopicDcpsPublication),
        BuiltinDataReader(participant, BuiltinTopicDcpsParticipant),
    )


def _default_owned_participant_key():
    """Return the GUID of the Unitree ChannelFactory business participant."""

    from unitree_sdk2py.core.channel import ChannelFactory

    factory = ChannelFactory()
    participant = getattr(factory, "_ChannelFactory__participant", None)
    if participant is None:
        raise RuntimeError("Unitree ChannelFactory participant is not initialized")
    participant_key = getattr(participant, "guid", None)
    if participant_key is None:
        raise RuntimeError("Unitree ChannelFactory participant GUID is unavailable")
    return participant_key


def _is_alive_sample(sample: Any) -> bool:
    try:
        from cyclonedds.core import InstanceState

        alive_state = InstanceState.Alive
    except Exception:
        # The constant is stable in CycloneDDS and the fallback keeps pure-fake
        # tests independent of an installed native binding.
        alive_state = 16
    info = getattr(sample, "sample_info", None)
    return bool(
        info is not None
        and getattr(info, "valid_data", False)
        and int(getattr(info, "instance_state", 0)) == int(alive_state)
    )


def _owner_properties(participant_sample: Any | None) -> dict[str, str]:
    properties: dict[str, str] = {}
    if participant_sample is None:
        return properties
    qos = getattr(participant_sample, "qos", None)
    if qos is None:
        return properties
    try:
        policies = iter(qos)
    except TypeError:
        return properties
    for policy in policies:
        key = getattr(policy, "key", None)
        if key in _OWNER_PROPERTY_KEYS:
            properties[key] = str(getattr(policy, "value", "unknown"))
    return properties


def _describe_discovered_writer(
    publication: Any, participant_sample: Any | None
) -> str:
    owner = _owner_properties(participant_sample)
    return (
        f"topic={getattr(publication, 'topic_name', 'unknown')!r}, "
        f"writer={getattr(publication, 'key', 'unknown')}, "
        f"participant={getattr(publication, 'participant_key', 'unknown')}, "
        f"pid={owner.get('__Pid', 'unknown')}, "
        f"process={owner.get('__ProcessName', 'unknown')!r}, "
        f"host={owner.get('__Hostname', 'unknown')!r}, "
        f"type={getattr(publication, 'type_name', 'unknown')!r}"
    )


def _discover_alive_writers(
    *,
    domain_id: int,
    topics: set[str],
    timeout_s: float,
    participant_factory: Callable[[int], Any],
    reader_factory: Callable[[Any], tuple[Any, Any]],
    owned_participant_key: Any,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> list[str]:
    try:
        participant = participant_factory(domain_id)
        publication_reader, participant_reader = reader_factory(participant)
        probe_participant_key = getattr(participant, "guid", None)
        if probe_participant_key is None:
            raise RuntimeError("Builtin probe participant GUID is unavailable")
    except Exception as exc:
        raise DDSAuthorityError(
            "cannot initialize CycloneDDS Builtin discovery for LowState "
            f"authority (domain={domain_id}): {exc}; refusing startup"
        ) from exc

    deadline = monotonic() + timeout_s
    discovered: dict[Any, Any] = {}
    participant_samples: dict[Any, Any] = {}
    try:
        while True:
            publications = publication_reader.read(_DISCOVERY_READ_LIMIT)
            participants = participant_reader.read(_DISCOVERY_READ_LIMIT)

            for sample in participants:
                key = getattr(sample, "key", None)
                if key is not None and _is_alive_sample(sample):
                    participant_samples[key] = sample

            for sample in publications:
                if (
                    getattr(sample, "topic_name", None) in topics
                    and _is_alive_sample(sample)
                    and getattr(sample, "participant_key", None)
                    not in (owned_participant_key, probe_participant_key)
                ):
                    key = getattr(sample, "key", id(sample))
                    discovered[key] = sample

            if discovered and all(
                getattr(sample, "participant_key", None) in participant_samples
                for sample in discovered.values()
            ):
                break

            remaining = deadline - monotonic()
            if remaining <= 0.0:
                break
            sleep(min(_DISCOVERY_POLL_INTERVAL_S, remaining))
    except DDSAuthorityError:
        raise
    except Exception as exc:
        raise DDSAuthorityError(
            "CycloneDDS Builtin discovery failed while checking LowState "
            f"authority (domain={domain_id}, topics={sorted(topics)!r}): {exc}; "
            "refusing startup"
        ) from exc

    return [
        _describe_discovered_writer(
            publication,
            participant_samples.get(getattr(publication, "participant_key", None)),
        )
        for publication in discovered.values()
    ]


def claim_lowstate_authority(
    topics: Iterable[str],
    *,
    domain_id: int,
    discovery_timeout_s: float = DEFAULT_DISCOVERY_TIMEOUT_S,
    lock_directory: str | os.PathLike[str] | None = None,
    participant_factory: Callable[[int], Any] | None = None,
    reader_factory: Callable[[Any], tuple[Any, Any]] | None = None,
    owned_participant_key_factory: Callable[[], Any] | None = None,
    monotonic: Callable[[], float] | None = None,
    sleep: Callable[[float], None] | None = None,
) -> LowStateAuthorityClaim:
    """Claim exclusive authority over exact LowState topics in one DDS domain.

    The local per-topic locks close the discovery-to-writer TOCTOU window.  The
    Builtin discovery pass then catches already-running writers, including
    writers on other hosts.  It cannot atomically arbitrate two different hosts
    that start at the same instant; callers that need that guarantee require a
    distributed coordinator.  Any local lock, discovery, or read error is
    fail-closed.

    The Unitree business participant and this call's probe participant are
    excluded by exact GUID, so a same-process retry does not diagnose its own
    writer.  No PID-wide exclusion is used.  ``participant_factory``,
    ``reader_factory`` and ``owned_participant_key_factory`` are dependency-
    injection seams for deterministic tests; production callers use defaults.
    """

    try:
        domain_id = int(domain_id)
    except (TypeError, ValueError) as exc:
        raise DDSAuthorityError(
            f"DDS authority domain must be an integer, got {domain_id!r}"
        ) from exc
    if domain_id < 0:
        raise DDSAuthorityError(
            f"DDS authority domain must be non-negative, got {domain_id}"
        )
    try:
        discovery_timeout_s = float(discovery_timeout_s)
    except (TypeError, ValueError) as exc:
        raise DDSAuthorityError(
            f"DDS authority discovery timeout must be numeric, got {discovery_timeout_s!r}"
        ) from exc
    if discovery_timeout_s < 0.0:
        raise DDSAuthorityError(
            "DDS authority discovery timeout must be non-negative, "
            f"got {discovery_timeout_s}"
        )

    exact_topics = _normalize_topics(topics)
    lock_dir = Path(lock_directory) if lock_directory else _default_lock_directory()
    participant_factory = participant_factory or _default_participant_factory
    reader_factory = reader_factory or _default_reader_factory
    owned_participant_key_factory = (
        owned_participant_key_factory or _default_owned_participant_key
    )
    monotonic = monotonic or time.monotonic
    sleep = sleep or time.sleep

    with _CLAIM_MUTEX:
        new_locks: list[_HeldTopicLock] = []
        try:
            for topic in sorted(exact_topics):
                path = _topic_lock_path(lock_dir, domain_id, topic)
                registry_key = str(path.absolute())
                if registry_key in _HELD_TOPIC_LOCKS:
                    continue
                held = _acquire_topic_lock(
                    domain_id=domain_id,
                    topic=topic,
                    lock_directory=lock_dir,
                )
                new_locks.append(held)

            try:
                owned_participant_key = owned_participant_key_factory()
            except Exception as exc:
                raise DDSAuthorityError(
                    "cannot identify the Unitree DDS business participant for "
                    f"LowState authority (domain={domain_id}): {exc}; refusing startup"
                ) from exc
            if owned_participant_key is None:
                raise DDSAuthorityError(
                    "Unitree DDS business participant GUID is empty; refusing startup"
                )

            # A same-process re-claim keeps the existing flock but must repeat
            # discovery.  State may have changed since the original claim, and
            # silently skipping this pass would turn re-entry into a guard
            # bypass (including when a writer already exists in this process).
            conflicts = _discover_alive_writers(
                domain_id=domain_id,
                topics=set(exact_topics),
                timeout_s=discovery_timeout_s,
                participant_factory=participant_factory,
                reader_factory=reader_factory,
                owned_participant_key=owned_participant_key,
                monotonic=monotonic,
                sleep=sleep,
            )
            if conflicts:
                diagnostics = "; ".join(conflicts)
                raise DDSAuthorityConflictError(
                    "alive LowState DataWriter already discovered; refusing "
                    f"startup in domain={domain_id}: {diagnostics}"
                )

            for held in new_locks:
                registry_key = str(held.path.absolute())
                _HELD_TOPIC_LOCKS[registry_key] = held
                _PROVISIONAL_TOPIC_LOCKS.pop(held.fd, None)
        except Exception:
            _close_uncommitted_locks(new_locks)
            raise

    print(
        "[dds_authority_guard] LowState authority claimed "
        f"(domain={domain_id}, pid={os.getpid()}, topics={list(exact_topics)!r}, "
        f"discovery={discovery_timeout_s:.2f}s)"
    )
    return LowStateAuthorityClaim(
        domain_id=domain_id,
        topics=exact_topics,
        pid=os.getpid(),
    )


def _prepare_for_fork() -> None:
    """Serialize fork against claims running in another thread."""

    _CLAIM_MUTEX.acquire()


def _resume_after_fork_in_parent() -> None:
    _CLAIM_MUTEX.release()


def _reset_after_fork_in_child() -> None:
    """Drop inherited locks and replace a possibly orphaned mutex in child."""

    global _CLAIM_MUTEX
    inherited = {
        held.fd: held
        for held in (
            tuple(_HELD_TOPIC_LOCKS.values())
            + tuple(_PROVISIONAL_TOPIC_LOCKS.values())
        )
    }
    for held in inherited.values():
        try:
            os.close(held.fd)
        except OSError:
            pass
    _HELD_TOPIC_LOCKS.clear()
    _PROVISIONAL_TOPIC_LOCKS.clear()
    _CLAIM_MUTEX = threading.RLock()


if hasattr(os, "register_at_fork"):
    os.register_at_fork(
        before=_prepare_for_fork,
        after_in_parent=_resume_after_fork_in_parent,
        after_in_child=_reset_after_fork_in_child,
    )
