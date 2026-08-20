"""Dependency-free helpers for merging implicit actuator configurations.

Isaac Lab evaluates every actuator group once per physics substep.  Combining
groups is therefore useful for articulations whose actuator model class is the
same for every joint, but it is only safe if the resulting per-joint values are
identical.  This module resolves the original regular expressions against an
explicit joint list and emits exact joint-name dictionaries, so overlaps and
missing coverage fail before the simulator starts.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import hashlib
import json
import math
import re


ACTUATOR_PROPERTY_FIELDS = (
    "effort_limit",
    "velocity_limit",
    "effort_limit_sim",
    "velocity_limit_sim",
    "stiffness",
    "damping",
    "armature",
    "friction",
    "dynamic_friction",
    "viscous_friction",
)

_USE_USD = object()


def _normalize_implicit_limit_aliases(
    group_name: str, group: object
) -> dict[str, object | None]:
    """Apply ``ImplicitActuator.__init__`` limit-alias rules without mutation.

    Isaac Lab mutates each implicit actuator config before resolving its joint
    parameters.  The merger has to flatten the post-mutation values; otherwise
    a ``*_sim``-only source group would incorrectly look as if its legacy alias
    still fell back to USD.

    Legacy-only ``velocity_limit`` is deliberately rejected.  Isaac Lab clears
    that value for backwards compatibility and then uses the USD velocity
    limit.  Silently carrying the legacy number into a merged group would
    change physics, while silently dropping it would hide an unsupported input
    shape from callers.
    """

    effort_limit = getattr(group, "effort_limit", None)
    effort_limit_sim = getattr(group, "effort_limit_sim", None)
    if effort_limit_sim is None and effort_limit is not None:
        effort_limit_sim = effort_limit
    elif effort_limit_sim is not None and effort_limit is None:
        effort_limit = effort_limit_sim
    elif (
        effort_limit_sim is not None
        and effort_limit is not None
        and effort_limit_sim != effort_limit
    ):
        raise ValueError(
            f"actuator group {group_name!r} has different effort_limit and "
            "effort_limit_sim values"
        )

    velocity_limit = getattr(group, "velocity_limit", None)
    velocity_limit_sim = getattr(group, "velocity_limit_sim", None)
    if velocity_limit_sim is None and velocity_limit is not None:
        raise ValueError(
            f"actuator group {group_name!r} has a legacy-only velocity_limit; "
            "ImplicitActuator clears that value before using the USD fallback, "
            "so this input cannot be merged safely"
        )
    if velocity_limit_sim is not None and velocity_limit is None:
        velocity_limit = velocity_limit_sim
    elif (
        velocity_limit_sim is not None
        and velocity_limit is not None
        and velocity_limit_sim != velocity_limit
    ):
        raise ValueError(
            f"actuator group {group_name!r} has different velocity_limit and "
            "velocity_limit_sim values"
        )

    return {
        "effort_limit": effort_limit,
        "effort_limit_sim": effort_limit_sim,
        "velocity_limit": velocity_limit,
        "velocity_limit_sim": velocity_limit_sim,
    }


def _resolve_patterns(
    patterns: Sequence[str], joint_names: Sequence[str], *, description: str
) -> list[str]:
    """Mirror Isaac Lab's strict regex resolution in articulation order."""

    pattern_hits = {pattern: [] for pattern in patterns}
    resolved: list[str] = []
    for joint_name in joint_names:
        matches = [pattern for pattern in patterns if re.fullmatch(pattern, joint_name)]
        if len(matches) > 1:
            raise ValueError(
                f"{description} gives multiple matches for {joint_name!r}: {matches}"
            )
        if matches:
            pattern_hits[matches[0]].append(joint_name)
            resolved.append(joint_name)
    missing_patterns = [pattern for pattern, hits in pattern_hits.items() if not hits]
    if missing_patterns:
        raise ValueError(
            f"{description} has expressions that match no joint: {missing_patterns}"
        )
    return resolved


def _resolve_config_value(
    value: object,
    joint_names: Sequence[str],
    *,
    description: str,
) -> dict[str, object]:
    """Resolve one actuator field with Isaac Lab's scalar/dict semantics."""

    if isinstance(value, Mapping):
        # ActuatorBase._parse_joint_parameter initializes the tensor to zero and
        # then fills dict matches.  Consequently a non-None partial dict means
        # zero for an unmatched joint, not the USD value.
        resolved_names = _resolve_patterns(
            tuple(value), joint_names, description=description
        )
        result = {joint_name: 0.0 for joint_name in joint_names}
        for joint_name in resolved_names:
            matching_pattern = next(
                pattern for pattern in value if re.fullmatch(pattern, joint_name)
            )
            result[joint_name] = value[matching_pattern]
        return result
    return {joint_name: value for joint_name in joint_names}


def merge_implicit_actuator_groups(
    actuators: Mapping[str, object],
    joint_names: Sequence[str],
    *,
    merged_name: str,
    actuator_cfg_type: type,
    partial_usd_defaults: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Merge implicit groups while preserving every resolved joint property.

    ``partial_usd_defaults`` is required only when some source groups leave a
    field as ``None`` (use the USD value) while another group configures that
    field.  A single actuator config cannot represent per-joint ``None``;
    callers must provide the known USD value for those joints or the merge
    fails closed.
    """

    if not actuators:
        raise ValueError("cannot merge an empty actuator mapping")
    ordered_joint_names = tuple(joint_names)
    if not ordered_joint_names:
        raise ValueError("cannot merge actuators without joint names")
    if len(set(ordered_joint_names)) != len(ordered_joint_names):
        raise ValueError("joint_names contains duplicates")

    groups: list[tuple[str, object, list[str], dict[str, object | None]]] = []
    coverage = {joint_name: [] for joint_name in ordered_joint_names}
    for group_name, group in actuators.items():
        if not isinstance(group, actuator_cfg_type):
            raise TypeError(
                f"actuator group {group_name!r} is {type(group).__name__}, "
                f"expected {actuator_cfg_type.__name__}"
            )
        group_joint_names = _resolve_patterns(
            tuple(group.joint_names_expr),
            ordered_joint_names,
            description=f"actuator group {group_name!r}",
        )
        for joint_name in group_joint_names:
            coverage[joint_name].append(group_name)
        normalized_limits = _normalize_implicit_limit_aliases(group_name, group)
        groups.append((group_name, group, group_joint_names, normalized_limits))

    invalid_coverage = {
        joint_name: names for joint_name, names in coverage.items() if len(names) != 1
    }
    if invalid_coverage:
        raise ValueError(
            "actuator groups must cover every joint exactly once; "
            f"invalid coverage={invalid_coverage}"
        )

    usd_defaults = dict(partial_usd_defaults or {})
    merged_fields: dict[str, dict[str, object]] = {}
    for field_name in ACTUATOR_PROPERTY_FIELDS:
        per_joint: dict[str, object] = {}
        for group_name, group, group_joint_names, normalized_limits in groups:
            value = normalized_limits.get(
                field_name, getattr(group, field_name, None)
            )
            if value is None:
                per_joint.update(
                    {joint_name: _USE_USD for joint_name in group_joint_names}
                )
                continue
            per_joint.update(
                _resolve_config_value(
                    value,
                    group_joint_names,
                    description=f"{group_name!r}.{field_name}",
                )
            )

        uses_usd = [
            joint_name
            for joint_name, value in per_joint.items()
            if value is _USE_USD
        ]
        if len(uses_usd) == len(ordered_joint_names):
            # Leaving the merged field as None preserves the original USD
            # fallback for every joint (notably effort_limit/velocity_limit).
            continue
        if uses_usd:
            if field_name not in usd_defaults:
                raise ValueError(
                    f"cannot merge partially configured field {field_name!r}; "
                    f"USD defaults are required for joints {uses_usd}"
                )
            resolved_defaults = _resolve_config_value(
                usd_defaults[field_name],
                ordered_joint_names,
                description=f"USD default for {field_name!r}",
            )
            for joint_name in uses_usd:
                per_joint[joint_name] = resolved_defaults[joint_name]

        merged_fields[field_name] = {
            joint_name: per_joint[joint_name] for joint_name in ordered_joint_names
        }

    return {
        merged_name: actuator_cfg_type(
            joint_names_expr=list(ordered_joint_names), **merged_fields
        )
    }


def maybe_merge_implicit_actuator_groups(
    actuators: Mapping[str, object],
    joint_names: Sequence[str],
    *,
    enabled: bool,
    merged_name: str,
    actuator_cfg_type: type,
    partial_usd_defaults: Mapping[str, object] | None = None,
) -> Mapping[str, object]:
    """Return the original mapping unchanged unless merging is opted in."""

    if not enabled:
        return actuators
    return merge_implicit_actuator_groups(
        actuators,
        joint_names,
        merged_name=merged_name,
        actuator_cfg_type=actuator_cfg_type,
        partial_usd_defaults=partial_usd_defaults,
    )


def _single_env_tensor_values(
    value: object, expected_count: int, *, description: str
) -> list[str]:
    """Convert a runtime actuator tensor row into canonical finite floats.

    This intentionally uses tensor duck typing instead of importing torch so
    the startup contract remains unit-testable without launching Isaac Sim.
    The conveyor scene has exactly one environment, hence every runtime field
    must be shaped as one row with one value per actuator joint.
    """

    detached = value.detach() if callable(getattr(value, "detach", None)) else value
    host_value = (
        detached.cpu()
        if callable(getattr(detached, "cpu", None))
        else detached
    )
    if not callable(getattr(host_value, "tolist", None)):
        raise TypeError(f"{description} is not a tensor-like value")
    nested = host_value.tolist()
    if (
        not isinstance(nested, (list, tuple))
        or len(nested) != 1
        or not isinstance(nested[0], (list, tuple))
    ):
        raise ValueError(
            f"{description} must have one environment row, got {nested!r}"
        )
    row = nested[0]
    if len(row) != expected_count:
        raise ValueError(
            f"{description} has {len(row)} values for {expected_count} joints"
        )

    canonical: list[str] = []
    for index, item in enumerate(row):
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise TypeError(
                f"{description}[{index}] is not numeric: {item!r}"
            ) from exc
        if not math.isfinite(number):
            raise ValueError(f"{description}[{index}] is not finite: {number!r}")
        # +0 and -0 are physically identical; normalize them before hashing.
        canonical.append((0.0 if number == 0.0 else number).hex())
    return canonical


def _runtime_actuator_snapshot(robot: object, *, asset_name: str) -> tuple[str, int, int]:
    """Build one group-layout-independent snapshot in robot joint order."""

    joint_names = tuple(getattr(robot, "joint_names", ()))
    if not joint_names:
        raise ValueError(f"{asset_name}: robot has no joint_names")
    if len(set(joint_names)) != len(joint_names):
        raise ValueError(f"{asset_name}: robot.joint_names contains duplicates")
    actuators = getattr(robot, "actuators", None)
    if not isinstance(actuators, Mapping) or not actuators:
        raise TypeError(f"{asset_name}: robot.actuators must be a non-empty mapping")

    owners = {joint_name: [] for joint_name in joint_names}
    values = {
        joint_name: {field_name: [] for field_name in ACTUATOR_PROPERTY_FIELDS}
        for joint_name in joint_names
    }
    for group_name, actuator in actuators.items():
        actuator_joint_names = tuple(getattr(actuator, "joint_names", ()))
        if not actuator_joint_names:
            raise ValueError(f"{asset_name}.{group_name}: actuator has no joint_names")
        for joint_name in actuator_joint_names:
            if joint_name not in owners:
                raise ValueError(
                    f"{asset_name}.{group_name}: unknown joint {joint_name!r}"
                )

        field_rows = {}
        for field_name in ACTUATOR_PROPERTY_FIELDS:
            if not hasattr(actuator, field_name):
                raise AttributeError(
                    f"{asset_name}.{group_name}: missing runtime field {field_name!r}"
                )
            field_rows[field_name] = _single_env_tensor_values(
                getattr(actuator, field_name),
                len(actuator_joint_names),
                description=f"{asset_name}.{group_name}.{field_name}",
            )

        for index, joint_name in enumerate(actuator_joint_names):
            owners[joint_name].append(str(group_name))
            for field_name in ACTUATOR_PROPERTY_FIELDS:
                values[joint_name][field_name].append(
                    (str(group_name), field_rows[field_name][index])
                )

    invalid_owners = {
        joint_name: group_names
        for joint_name, group_names in owners.items()
        if len(group_names) != 1
    }
    if invalid_owners:
        raise ValueError(
            f"{asset_name}: every joint must be covered exactly once; "
            f"invalid coverage={invalid_owners}"
        )

    snapshot = []
    for joint_name in joint_names:
        field_snapshot = []
        for field_name in ACTUATOR_PROPERTY_FIELDS:
            field_values = values[joint_name][field_name]
            if len(field_values) != 1:
                raise ValueError(
                    f"{asset_name}.{joint_name}.{field_name}: expected exactly "
                    f"one value, got {field_values}"
                )
            field_snapshot.append((field_name, field_values[0][1]))
        snapshot.append((joint_name, field_snapshot))

    payload = json.dumps(
        snapshot, ensure_ascii=False, separators=(",", ":")
    ).encode("utf-8")
    digest = hashlib.sha256(b"sonic-actuator-runtime-v1\n" + payload).hexdigest()
    return digest, len(actuators), len(joint_names)


def validate_sonic_runtime_actuators(
    env: object,
    env_ids: object | None,
    *,
    asset_names: Sequence[str],
    require_merged: bool,
    expected_merged_joint_names: Sequence[str] | None = None,
) -> dict[str, str]:
    """Validate and hash initialized host SONIC actuator tensors at startup.

    The canonical hash excludes group boundaries, so a six-group baseline and
    a semantics-equivalent one-group A/B run produce the same digest.  The
    event is wired only behind ``ISAACLAB_SONIC_VALIDATE_ACTUATORS``; therefore
    none of these CPU copies or synchronizations occur in normal runs.
    """

    del env_ids  # Startup validation is articulation-wide, not reset-specific.
    ordered_asset_names = tuple(asset_names)
    if not ordered_asset_names:
        raise ValueError("runtime actuator validation requires host asset names")
    if len(set(ordered_asset_names)) != len(ordered_asset_names):
        raise ValueError("runtime actuator validation asset_names contains duplicates")

    scene = getattr(env, "scene", None)
    if scene is None:
        raise AttributeError("runtime actuator validation requires env.scene")
    expected_names = (
        tuple(expected_merged_joint_names)
        if expected_merged_joint_names is not None
        else None
    )
    if expected_names is not None and len(set(expected_names)) != len(expected_names):
        raise ValueError("expected_merged_joint_names contains duplicates")

    hashes: dict[str, str] = {}
    for asset_name in ordered_asset_names:
        try:
            robot = scene[asset_name]
        except (KeyError, TypeError) as exc:
            raise KeyError(f"host SONIC asset {asset_name!r} is unavailable") from exc

        joint_names = tuple(getattr(robot, "joint_names", ()))
        actuators = getattr(robot, "actuators", None)
        if require_merged:
            if not isinstance(actuators, Mapping) or tuple(actuators) != ("sonic_all",):
                groups = tuple(actuators) if isinstance(actuators, Mapping) else None
                raise ValueError(
                    f"{asset_name}: merged A/B requires only sonic_all, got {groups}"
                )
            if len(joint_names) != 43:
                raise ValueError(
                    f"{asset_name}: merged A/B requires 43 robot joints, "
                    f"got {len(joint_names)}"
                )
            if expected_names is None or set(joint_names) != set(expected_names):
                raise ValueError(
                    f"{asset_name}: robot joint name set differs from expected SONIC set"
                )
            actuator = actuators["sonic_all"]
            actuator_joint_names = tuple(getattr(actuator, "joint_names", ()))
            if actuator_joint_names != joint_names:
                raise ValueError(
                    f"{asset_name}.sonic_all: actuator.joint_names must equal "
                    "robot.joint_names"
                )
            joint_indices = getattr(actuator, "joint_indices", None)
            if not (
                isinstance(joint_indices, slice)
                and joint_indices.start is None
                and joint_indices.stop is None
                and joint_indices.step is None
            ):
                raise ValueError(
                    f"{asset_name}.sonic_all: joint_indices must be slice(None), "
                    f"got {joint_indices!r}"
                )

        digest, group_count, joint_count = _runtime_actuator_snapshot(
            robot, asset_name=asset_name
        )
        hashes[asset_name] = digest
        print(
            "[conveyor_event] SONIC actuator contract: "
            f"asset={asset_name} groups={group_count} joints={joint_count} "
            f"sha256={digest}"
        )

    if len(set(hashes.values())) != 1:
        raise ValueError(f"host SONIC actuator hashes differ: {hashes}")
    return hashes
