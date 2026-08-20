"""Contracts for the opt-in SONIC true-robot actuator merge."""

from __future__ import annotations

import ast
import contextlib
import importlib.util
import io
from pathlib import Path
import re
import sys
from types import SimpleNamespace
import unittest

from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER
from robots.g1_sonic_urdf import DEX3_HAND_JOINT_NAMES


_REPO_ROOT = Path(__file__).resolve().parents[1]
_CONVEYOR_DIR = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor"
_CONVEYOR_CFG = _CONVEYOR_DIR / "conveyor_env_cfg.py"
_SONIC_CFG = (
    _REPO_ROOT
    / "tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py"
)

# Independent contract: this list must not be derived from production, or a
# production omission would silently shrink the test oracle with it.
_EXPECTED_ACTUATOR_PROPERTY_FIELDS = (
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


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_MERGE = _load_module(
    "sonic_actuator_merge_for_test", _CONVEYOR_DIR / "actuator_merge.py"
)
_JOINT_NAMES = (*G1_29DOF_DDS_JOINT_ORDER, *DEX3_HAND_JOINT_NAMES)
# Synthetic non-DDS permutation used only to prove that runtime validation
# compares the expected 43-name set while preserving robot-provided order.  It
# is deliberately not an assertion about the live PhysX/URDF joint order.
_PERMUTED_RUNTIME_JOINT_NAMES = (
    *G1_29DOF_DDS_JOINT_ORDER[:22],
    *DEX3_HAND_JOINT_NAMES[:7],
    *G1_29DOF_DDS_JOINT_ORDER[22:],
    *DEX3_HAND_JOINT_NAMES[7:],
)


class _FakeImplicitActuatorCfg:
    """Small stand-in that accepts the same fields used by the real config."""

    def __init__(self, *, joint_names_expr, **kwargs) -> None:
        self.joint_names_expr = list(joint_names_expr)
        for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS:
            setattr(self, field_name, kwargs.get(field_name))


class _FakeTensor:
    """One-environment tensor duck type for runtime tests without torch."""

    def __init__(self, values) -> None:
        self.values = list(values)

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return [list(self.values)]


def _function(tree: ast.Module, name: str) -> ast.FunctionDef:
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _class_assignment(tree: ast.Module, class_name: str, name: str) -> ast.Assign:
    class_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node
        for node in class_node.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == name
            for target in node.targets
        )
    )


def _compile_factory(name: str, namespace: dict[str, object]):
    """Compile one real conveyor factory with injected lightweight globals."""

    tree = ast.parse(_CONVEYOR_CFG.read_text(encoding="utf-8"))
    function = _function(tree, name)
    module = ast.Module(body=[function], type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(_CONVEYOR_CFG), "exec"), namespace)
    return namespace[name]


def _actual_sonic_actuator_cfgs() -> dict[str, _FakeImplicitActuatorCfg]:
    """Evaluate only make_sonic_robot_cfg's real actuator literal."""

    tree = ast.parse(_SONIC_CFG.read_text(encoding="utf-8"))
    function = _function(tree, "make_sonic_robot_cfg")
    actuator_value = next(
        node.value
        for node in ast.walk(function)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == "cfg"
            and target.attr == "actuators"
            for target in node.targets
        )
    )

    natural_frequency = 10.0 * 2.0 * 3.1415926535
    damping_ratio = 2.0
    armatures = {
        "5020": 0.003609725,
        "7520_14": 0.010177520,
        "7520_22": 0.025101925,
        "4010": 0.00425,
    }
    namespace: dict[str, object] = {
        "ImplicitActuatorCfg": _FakeImplicitActuatorCfg,
        "DEX3_HAND_JOINT_NAMES": DEX3_HAND_JOINT_NAMES,
        "DEX3_PHASE1_EFFORT_LIMITS": {
            name: 2.45 if name.endswith("thumb_0_joint") else 0.7
            for name in DEX3_HAND_JOINT_NAMES
        },
        "DEX3_VELOCITY_LIMITS": {
            name: 3.14 if name.endswith("thumb_0_joint") else 12.0
            for name in DEX3_HAND_JOINT_NAMES
        },
    }
    for suffix, armature in armatures.items():
        namespace[f"SONIC_ARMATURE_{suffix}"] = armature
        namespace[f"SONIC_STIFFNESS_{suffix}"] = armature * natural_frequency**2
        namespace[f"SONIC_DAMPING_{suffix}"] = (
            2.0 * damping_ratio * armature * natural_frequency
        )
    expression = ast.Expression(body=actuator_value)
    ast.fix_missing_locations(expression)
    return eval(compile(expression, str(_SONIC_CFG), "eval"), namespace)


def _runtime_group_joint_names(
    joint_order: tuple[str, ...] = _PERMUTED_RUNTIME_JOINT_NAMES,
) -> dict[str, tuple[str, ...]]:
    """Resolve the real six config groups for fake initialized actuators."""

    groups = {}
    for group_name, cfg in _actual_sonic_actuator_cfgs().items():
        groups[group_name] = tuple(
            joint_name
            for joint_name in joint_order
            if _group_matches(cfg, joint_name)
        )
    return groups


def _fake_runtime_robot(
    groups: dict[str, tuple[str, ...] | list[str]],
    *,
    robot_joint_names: tuple[str, ...] = _PERMUTED_RUNTIME_JOINT_NAMES,
):
    """Create deterministic runtime tensors whose values depend only on joints."""

    joint_value_index = {
        joint_name: index for index, joint_name in enumerate(_JOINT_NAMES)
    }
    actuators = {}
    for group_name, group_joint_names_value in groups.items():
        group_joint_names = tuple(group_joint_names_value)
        fields = {}
        for field_index, field_name in enumerate(
            _EXPECTED_ACTUATOR_PROPERTY_FIELDS
        ):
            fields[field_name] = _FakeTensor(
                (field_index + 1) * 1000.0
                + joint_value_index[joint_name] * 0.125
                for joint_name in group_joint_names
            )
        actuators[group_name] = SimpleNamespace(
            joint_names=list(group_joint_names),
            joint_indices=(
                slice(None)
                if group_joint_names == robot_joint_names
                else list(range(len(group_joint_names)))
            ),
            **fields,
        )
    return SimpleNamespace(
        joint_names=list(robot_joint_names),
        actuators=actuators,
    )


def _group_matches(group: _FakeImplicitActuatorCfg, joint_name: str) -> bool:
    matches = [
        pattern
        for pattern in group.joint_names_expr
        if re.fullmatch(pattern, joint_name)
    ]
    if len(matches) > 1:
        raise AssertionError(f"{joint_name} has overlapping expressions: {matches}")
    return bool(matches)


def _resolve_parameter(
    value: object,
    joint_names: tuple[str, ...] | list[str],
    default_values: dict[str, object],
    *,
    field_name: str,
) -> dict[str, object]:
    """Mirror ``ActuatorBase._parse_joint_parameter`` without torch."""

    if value is None:
        return {joint_name: default_values[joint_name] for joint_name in joint_names}
    if isinstance(value, dict):
        result = {joint_name: 0.0 for joint_name in joint_names}
        pattern_hits = {pattern: [] for pattern in value}
        for joint_name in joint_names:
            matches = [pattern for pattern in value if re.fullmatch(pattern, joint_name)]
            if len(matches) > 1:
                raise AssertionError(
                    f"{joint_name} has overlapping {field_name} values: {matches}"
                )
            if matches:
                pattern_hits[matches[0]].append(joint_name)
                result[joint_name] = value[matches[0]]
        missing = [pattern for pattern, hits in pattern_hits.items() if not hits]
        if missing:
            raise AssertionError(f"{field_name} has unmatched expressions: {missing}")
        return result
    return {joint_name: value for joint_name in joint_names}


def _simulate_implicit_actuator_init(
    group: _FakeImplicitActuatorCfg,
    joint_names: tuple[str, ...] | list[str],
    usd_defaults: dict[str, dict[str, object]],
) -> dict[str, dict[str, object]]:
    """Resolve the ten runtime fields after real ImplicitActuator alias rules."""

    # ImplicitActuator.__init__ mutates these four config values before
    # ActuatorBase resolves tensors.  Keep this simulation side-effect-free.
    effort_limit = group.effort_limit
    effort_limit_sim = group.effort_limit_sim
    if effort_limit_sim is None and effort_limit is not None:
        effort_limit_sim = effort_limit
    elif effort_limit_sim is not None and effort_limit is None:
        effort_limit = effort_limit_sim
    elif (
        effort_limit_sim is not None
        and effort_limit is not None
        and effort_limit_sim != effort_limit
    ):
        raise ValueError("different effort_limit aliases")

    velocity_limit = group.velocity_limit
    velocity_limit_sim = group.velocity_limit_sim
    if velocity_limit_sim is None and velocity_limit is not None:
        # Backwards-compatibility behavior in ImplicitActuator: the legacy-only
        # velocity value was historically ignored, so it is actively cleared.
        velocity_limit = None
    elif velocity_limit_sim is not None and velocity_limit is None:
        velocity_limit = velocity_limit_sim
    elif (
        velocity_limit_sim is not None
        and velocity_limit is not None
        and velocity_limit_sim != velocity_limit
    ):
        raise ValueError("different velocity_limit aliases")

    config_values = {
        field_name: getattr(group, field_name)
        for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS
    }
    config_values.update(
        {
            "effort_limit": effort_limit,
            "effort_limit_sim": effort_limit_sim,
            "velocity_limit": velocity_limit,
            "velocity_limit_sim": velocity_limit_sim,
        }
    )

    runtime: dict[str, dict[str, object]] = {}
    for field_name in (
        "effort_limit_sim",
        "velocity_limit_sim",
        "stiffness",
        "damping",
        "armature",
        "friction",
        "dynamic_friction",
        "viscous_friction",
    ):
        runtime[field_name] = _resolve_parameter(
            config_values[field_name],
            joint_names,
            usd_defaults[field_name],
            field_name=field_name,
        )

    runtime["velocity_limit"] = _resolve_parameter(
        velocity_limit,
        joint_names,
        runtime["velocity_limit_sim"],
        field_name="velocity_limit",
    )
    effort_default = (
        usd_defaults["effort_limit_sim"]
        if effort_limit is None
        else runtime["effort_limit_sim"]
    )
    runtime["effort_limit"] = _resolve_parameter(
        effort_limit,
        joint_names,
        effort_default,
        field_name="effort_limit",
    )
    return runtime


def _simulate_actuator_mapping(
    actuators: dict[str, _FakeImplicitActuatorCfg],
    joint_names: tuple[str, ...],
    usd_defaults: dict[str, dict[str, object]],
) -> dict[str, dict[str, object]]:
    """Resolve all groups into one independent per-joint runtime oracle."""

    result = {
        field_name: {} for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS
    }
    coverage = {joint_name: [] for joint_name in joint_names}
    for group_name, group in actuators.items():
        selected = [
            joint_name for joint_name in joint_names if _group_matches(group, joint_name)
        ]
        group_runtime = _simulate_implicit_actuator_init(
            group, selected, usd_defaults
        )
        for joint_name in selected:
            coverage[joint_name].append(group_name)
            for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS:
                result[field_name][joint_name] = group_runtime[field_name][joint_name]
    invalid = {name: groups for name, groups in coverage.items() if len(groups) != 1}
    if invalid:
        raise AssertionError(f"invalid actuator coverage: {invalid}")
    return result


class SonicActuatorMergeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.original = _actual_sonic_actuator_cfgs()
        self.defaults = {
            field_name: {
                joint_name: (index + 1) * 0.125
                for index, joint_name in enumerate(_JOINT_NAMES)
            }
            for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS
        }
        # Both limit aliases receive the same underlying USD joint property in
        # ActuatorBase; keeping these equal is part of the independent oracle.
        self.defaults["effort_limit"] = dict(self.defaults["effort_limit_sim"])
        self.defaults["velocity_limit"] = dict(
            self.defaults["velocity_limit_sim"]
        )
        # These are the only mixed None/configured fields in the real six
        # groups.  The SONIC URDF/PhysxJointAPI leaves body values at zero.
        for field_name in (
            "friction",
            "dynamic_friction",
            "viscous_friction",
        ):
            self.defaults[field_name] = {
                joint_name: 0.0 for joint_name in _JOINT_NAMES
            }

    def _merge(self):
        return _MERGE.merge_implicit_actuator_groups(
            self.original,
            _JOINT_NAMES,
            merged_name="sonic_all",
            actuator_cfg_type=_FakeImplicitActuatorCfg,
            partial_usd_defaults={
                "friction": 0.0,
                "dynamic_friction": 0.0,
                "viscous_friction": 0.0,
            },
        )

    def test_real_six_groups_cover_all_43_joints_exactly_once(self) -> None:
        self.assertEqual(
            _MERGE.ACTUATOR_PROPERTY_FIELDS,
            _EXPECTED_ACTUATOR_PROPERTY_FIELDS,
        )
        self.assertEqual(len(_JOINT_NAMES), 43)
        self.assertEqual(
            tuple(self.original),
            ("legs", "feet", "waist", "waist_yaw", "arms", "hands"),
        )
        for joint_name in _JOINT_NAMES:
            matching_groups = [
                name
                for name, group in self.original.items()
                if _group_matches(group, joint_name)
            ]
            self.assertEqual(len(matching_groups), 1, joint_name)

        merged = self._merge()
        self.assertEqual(tuple(merged), ("sonic_all",))
        merged_group = merged["sonic_all"]
        self.assertEqual(merged_group.joint_names_expr, list(_JOINT_NAMES))
        for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS:
            field_values = getattr(merged_group, field_name)
            self.assertIsInstance(field_values, dict, field_name)
            self.assertEqual(tuple(field_values), _JOINT_NAMES, field_name)
        # Isaac Lab normalizes an all-joint selection to slice(None).  The next
        # live host A/B will assert the real tensor index; this pure test locks
        # the prerequisite: one group selecting all 43 joints in exact order.
        resolved_ids = [
            index
            for index, joint_name in enumerate(_JOINT_NAMES)
            if _group_matches(merged_group, joint_name)
        ]
        self.assertEqual(resolved_ids, list(range(43)))

    def test_all_ten_properties_are_equal_per_joint_after_merge(self) -> None:
        original_runtime = _simulate_actuator_mapping(
            self.original, _JOINT_NAMES, self.defaults
        )
        merged_runtime = _simulate_actuator_mapping(
            self._merge(), _JOINT_NAMES, self.defaults
        )
        for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS:
            self.assertEqual(
                merged_runtime[field_name],
                original_runtime[field_name],
                f"runtime field {field_name} differs after merge",
            )

    def test_oracle_models_implicit_limit_alias_and_legacy_velocity_rules(self) -> None:
        joint_name = _JOINT_NAMES[0]
        defaults = {
            field_name: {joint_name: self.defaults[field_name][joint_name]}
            for field_name in _EXPECTED_ACTUATOR_PROPERTY_FIELDS
        }

        sim_only = _FakeImplicitActuatorCfg(
            joint_names_expr=[joint_name],
            effort_limit_sim=3.0,
            velocity_limit_sim=4.0,
        )
        runtime = _simulate_implicit_actuator_init(
            sim_only, [joint_name], defaults
        )
        self.assertEqual(runtime["effort_limit"][joint_name], 3.0)
        self.assertEqual(runtime["effort_limit_sim"][joint_name], 3.0)
        self.assertEqual(runtime["velocity_limit"][joint_name], 4.0)
        self.assertEqual(runtime["velocity_limit_sim"][joint_name], 4.0)

        legacy_effort = _FakeImplicitActuatorCfg(
            joint_names_expr=[joint_name], effort_limit=5.0
        )
        runtime = _simulate_implicit_actuator_init(
            legacy_effort, [joint_name], defaults
        )
        self.assertEqual(runtime["effort_limit"][joint_name], 5.0)
        self.assertEqual(runtime["effort_limit_sim"][joint_name], 5.0)

        legacy_velocity = _FakeImplicitActuatorCfg(
            joint_names_expr=[joint_name], velocity_limit=99.0
        )
        runtime = _simulate_implicit_actuator_init(
            legacy_velocity, [joint_name], defaults
        )
        expected_usd = defaults["velocity_limit_sim"][joint_name]
        self.assertEqual(runtime["velocity_limit"][joint_name], expected_usd)
        self.assertEqual(runtime["velocity_limit_sim"][joint_name], expected_usd)
        self.assertNotEqual(runtime["velocity_limit"][joint_name], 99.0)

    def test_disabled_path_returns_original_six_group_mapping_by_identity(self) -> None:
        result = _MERGE.maybe_merge_implicit_actuator_groups(
            self.original,
            _JOINT_NAMES,
            enabled=False,
            merged_name="sonic_all",
            actuator_cfg_type=_FakeImplicitActuatorCfg,
            partial_usd_defaults={
                "friction": 0.0,
                "dynamic_friction": 0.0,
                "viscous_friction": 0.0,
            },
        )
        self.assertIs(result, self.original)
        self.assertEqual(len(result), 6)

    def test_partial_usd_fallback_cannot_be_silently_guessed(self) -> None:
        with self.assertRaisesRegex(ValueError, "USD defaults are required"):
            _MERGE.merge_implicit_actuator_groups(
                self.original,
                _JOINT_NAMES,
                merged_name="sonic_all",
                actuator_cfg_type=_FakeImplicitActuatorCfg,
            )

    def test_legacy_only_velocity_limit_fails_closed(self) -> None:
        legacy_only = {
            "all": _FakeImplicitActuatorCfg(
                joint_names_expr=list(_JOINT_NAMES),
                velocity_limit=9.0,
            )
        }
        with self.assertRaisesRegex(ValueError, "legacy-only velocity_limit"):
            _MERGE.merge_implicit_actuator_groups(
                legacy_only,
                _JOINT_NAMES,
                merged_name="sonic_all",
                actuator_cfg_type=_FakeImplicitActuatorCfg,
            )

    def test_conflicting_limit_aliases_fail_like_implicit_actuator(self) -> None:
        for legacy_field, sim_field in (
            ("effort_limit", "effort_limit_sim"),
            ("velocity_limit", "velocity_limit_sim"),
        ):
            with self.subTest(legacy_field=legacy_field):
                conflicting = {
                    "all": _FakeImplicitActuatorCfg(
                        joint_names_expr=list(_JOINT_NAMES),
                        **{legacy_field: 1.0, sim_field: 2.0},
                    )
                }
                with self.assertRaisesRegex(ValueError, f"different {legacy_field}"):
                    _MERGE.merge_implicit_actuator_groups(
                        conflicting,
                        _JOINT_NAMES,
                        merged_name="sonic_all",
                        actuator_cfg_type=_FakeImplicitActuatorCfg,
                    )

    def test_production_local_wrapper_reads_the_global_flag(self) -> None:
        namespace = {
            "maybe_merge_implicit_actuator_groups": (
                _MERGE.maybe_merge_implicit_actuator_groups
            ),
            "_SONIC_JOINT_NAMES": _JOINT_NAMES,
            "SONIC_MERGE_ACTUATORS": False,
            "ImplicitActuatorCfg": _FakeImplicitActuatorCfg,
            "_SONIC_PARTIAL_USD_DEFAULTS": {
                "friction": 0.0,
                "dynamic_friction": 0.0,
                "viscous_friction": 0.0,
            },
        }
        maybe_merge = _compile_factory(
            "_maybe_merge_local_sonic_actuator_groups", namespace
        )

        disabled = maybe_merge(self.original)
        self.assertIs(disabled, self.original)
        self.assertEqual(tuple(disabled), tuple(self.original))

        # The same compiled production function must observe the module global
        # at call time; this rules out a test-only injected fake gate.
        namespace["SONIC_MERGE_ACTUATORS"] = True
        enabled = maybe_merge(self.original)
        self.assertEqual(tuple(enabled), ("sonic_all",))
        self.assertEqual(
            enabled["sonic_all"].joint_names_expr, list(_JOINT_NAMES)
        )

    def test_true_robot_factories_construct_six_or_one_group_from_the_flag(self) -> None:
        class FakeRobotCfg:
            def __init__(self, actuators) -> None:
                self.actuators = actuators
                self.prim_path = "{ENV_REGEX_NS}/Robot"
                self.init_state = SimpleNamespace(pos=None, rot=None)
                self.spawn = SimpleNamespace(visible=True)

        for enabled, expected_names in (
            (False, tuple(self.original)),
            (True, ("sonic_all",)),
        ):
            with self.subTest(enabled=enabled):
                source_mappings: list[dict] = []

                def make_robot():
                    actuators = _actual_sonic_actuator_cfgs()
                    source_mappings.append(actuators)
                    return FakeRobotCfg(actuators)

                namespace = {
                    "ArticulationCfg": FakeRobotCfg,
                    "VIEWER_MODE": False,
                    "make_sonic_robot_cfg": make_robot,
                    "maybe_merge_implicit_actuator_groups": (
                        _MERGE.maybe_merge_implicit_actuator_groups
                    ),
                    "_SONIC_JOINT_NAMES": _JOINT_NAMES,
                    "SONIC_MERGE_ACTUATORS": enabled,
                    "ImplicitActuatorCfg": _FakeImplicitActuatorCfg,
                    "_SONIC_PARTIAL_USD_DEFAULTS": {
                        "friction": 0.0,
                        "dynamic_friction": 0.0,
                        "viscous_friction": 0.0,
                    },
                    "LOCAL_ROBOT_POS": (1.0, 2.0, 3.0),
                    "LOCAL_ROBOT_ROT": (1.0, 0.0, 0.0, 0.0),
                    "CONTACT_REPORT_MODE": "ankles",
                    "configure_robot_contact_reports": lambda *args: None,
                    "_make_peer_robot_cfg": lambda: None,
                }
                _compile_factory(
                    "_maybe_merge_local_sonic_actuator_groups", namespace
                )
                local_factory = _compile_factory(
                    "_make_local_robot_cfg", namespace
                )
                local = local_factory()
                self.assertEqual(tuple(local.actuators), expected_names)
                self.assertEqual(local.init_state.pos, (1.0, 2.0, 3.0))
                self.assertEqual(len(source_mappings), 1)
                if not enabled:
                    self.assertIs(local.actuators, source_mappings[-1])

                namespace.update(
                    {
                        "sonic_robot_channel_spec": lambda robot_id: SimpleNamespace(
                            prim_name=f"Robot{robot_id}"
                        ),
                        "_scene_robot_pose": lambda robot_id: (
                            (float(robot_id), 4.0, 0.76),
                            (1.0, 0.0, 0.0, 0.0),
                        ),
                    }
                )
                additional_factory = _compile_factory(
                    "_make_additional_local_robot_cfg", namespace
                )
                for robot_id in (2, 5):
                    additional = additional_factory(robot_id)
                    self.assertEqual(tuple(additional.actuators), expected_names)
                    self.assertEqual(
                        additional.prim_path, f"{{ENV_REGEX_NS}}/Robot{robot_id}"
                    )
                    self.assertEqual(
                        additional.init_state.pos, (float(robot_id), 4.0, 0.76)
                    )
                    if not enabled:
                        self.assertIs(additional.actuators, source_mappings[-1])
                with self.assertRaisesRegex(ValueError, ">= 2"):
                    additional_factory(1)
                self.assertEqual(len(source_mappings), 3)

    def test_peer_default_factory_uses_same_strict_runtime_equivalent_merge(self) -> None:
        class FakeRobotCfg:
            def __init__(self, actuators) -> None:
                self.actuators = actuators
                self.prim_path = None
                self.init_state = SimpleNamespace(pos=None, rot=None)
                self.spawn = SimpleNamespace(activate_contact_sensors=True)

        class ExistingPeerUsd:
            def exists(self) -> bool:
                return True

            def __str__(self) -> str:
                return "/tmp/g1_43dof_peer.usd"

        def cfg_object(**kwargs):
            return SimpleNamespace(**kwargs)

        for local_flag in (False, True):
            with self.subTest(SONIC_MERGE_ACTUATORS=local_flag):
                source_mappings: list[dict] = []

                def make_robot():
                    actuators = _actual_sonic_actuator_cfgs()
                    source_mappings.append(actuators)
                    return FakeRobotCfg(actuators)

                namespace = {
                    "ArticulationCfg": FakeRobotCfg,
                    "merge_implicit_actuator_groups": (
                        _MERGE.merge_implicit_actuator_groups
                    ),
                    "_SONIC_JOINT_NAMES": _JOINT_NAMES,
                    "ImplicitActuatorCfg": _FakeImplicitActuatorCfg,
                    "_SONIC_PARTIAL_USD_DEFAULTS": {
                        "friction": 0.0,
                        "dynamic_friction": 0.0,
                        "viscous_friction": 0.0,
                    },
                    "SONIC_MERGE_ACTUATORS": local_flag,
                    "make_sonic_robot_cfg": make_robot,
                    "PEER_ROBOT_POS": (2.0, 3.0, 0.76),
                    "PEER_ROBOT_ROT": (1.0, 0.0, 0.0, 0.0),
                    "_PEER_ROBOT_USD": ExistingPeerUsd(),
                    "_PEER_RIGID_PROPS": {},
                    "UsdFileCfg": cfg_object,
                    "sim_utils": SimpleNamespace(
                        RigidBodyPropertiesCfg=cfg_object,
                        ArticulationRootPropertiesCfg=cfg_object,
                    ),
                    "_env_int": lambda _name, default: default,
                }
                _compile_factory("_merge_sonic_actuator_groups", namespace)
                _compile_factory("_merge_peer_actuator_groups", namespace)
                peer_factory = _compile_factory("_make_peer_robot_cfg", namespace)
                peer = peer_factory()

                self.assertEqual(len(source_mappings), 1)
                self.assertEqual(tuple(peer.actuators), ("peer_all",))
                self.assertEqual(
                    peer.actuators["peer_all"].joint_names_expr,
                    list(_JOINT_NAMES),
                )
                self.assertEqual(peer.prim_path, "{ENV_REGEX_NS}/PeerRobot")
                self.assertEqual(peer.init_state.pos, (2.0, 3.0, 0.76))
                original_runtime = _simulate_actuator_mapping(
                    source_mappings[0], _JOINT_NAMES, self.defaults
                )
                peer_runtime = _simulate_actuator_mapping(
                    peer.actuators, _JOINT_NAMES, self.defaults
                )
                self.assertEqual(peer_runtime, original_runtime)

    def test_runtime_validator_hashes_four_six_group_robots_without_torch(self) -> None:
        asset_names = ("robot", "robot_2", "robot_3", "robot_4")
        groups = _runtime_group_joint_names()
        scene = {
            asset_name: _fake_runtime_robot(groups) for asset_name in asset_names
        }
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            hashes = _MERGE.validate_sonic_runtime_actuators(
                SimpleNamespace(scene=scene),
                None,
                asset_names=asset_names,
                require_merged=False,
                expected_merged_joint_names=_JOINT_NAMES,
            )

        self.assertEqual(tuple(hashes), asset_names)
        self.assertEqual(len(set(hashes.values())), 1)
        for asset_name in asset_names:
            self.assertIn(
                f"asset={asset_name} groups=6 joints=43 sha256={hashes[asset_name]}",
                output.getvalue(),
            )

    def test_runtime_validator_enforces_merged_fast_path_and_stable_hash(self) -> None:
        asset_names = ("robot", "robot_2", "robot_3", "robot_4")
        self.assertNotEqual(_PERMUTED_RUNTIME_JOINT_NAMES, _JOINT_NAMES)
        self.assertEqual(set(_PERMUTED_RUNTIME_JOINT_NAMES), set(_JOINT_NAMES))
        merged_groups = {"sonic_all": _PERMUTED_RUNTIME_JOINT_NAMES}
        scene = {
            asset_name: _fake_runtime_robot(merged_groups)
            for asset_name in asset_names
        }
        with contextlib.redirect_stdout(io.StringIO()):
            merged_hashes = _MERGE.validate_sonic_runtime_actuators(
                SimpleNamespace(scene=scene),
                None,
                asset_names=asset_names,
                require_merged=True,
                expected_merged_joint_names=_JOINT_NAMES,
            )
            baseline_hash = _MERGE.validate_sonic_runtime_actuators(
                SimpleNamespace(scene={"robot": _fake_runtime_robot(
                    _runtime_group_joint_names()
                )}),
                None,
                asset_names=("robot",),
                require_merged=False,
                expected_merged_joint_names=_JOINT_NAMES,
            )["robot"]

        self.assertEqual(len(set(merged_hashes.values())), 1)
        self.assertEqual(next(iter(merged_hashes.values())), baseline_hash)

        bad_cases = {}
        wrong_group = _fake_runtime_robot({"all": _JOINT_NAMES})
        bad_cases["only sonic_all"] = wrong_group

        forty_two_names = _JOINT_NAMES[:-1]
        bad_cases["43 robot joints"] = _fake_runtime_robot(
            {"sonic_all": forty_two_names},
            robot_joint_names=forty_two_names,
        )

        wrong_set = _fake_runtime_robot(merged_groups)
        wrong_set.joint_names[-1] = "unexpected_joint"
        bad_cases["joint name set"] = wrong_set

        wrong_names = _fake_runtime_robot(merged_groups)
        wrong_names.actuators["sonic_all"].joint_names = list(
            reversed(_PERMUTED_RUNTIME_JOINT_NAMES)
        )
        bad_cases["actuator.joint_names"] = wrong_names

        wrong_slice = _fake_runtime_robot(merged_groups)
        wrong_slice.actuators["sonic_all"].joint_indices = list(range(43))
        bad_cases["slice\\(None\\)"] = wrong_slice

        for expected_error, robot in bad_cases.items():
            with self.subTest(expected_error=expected_error):
                with self.assertRaisesRegex(ValueError, expected_error):
                    _MERGE.validate_sonic_runtime_actuators(
                        SimpleNamespace(scene={"robot": robot}),
                        None,
                        asset_names=("robot",),
                        require_merged=True,
                        expected_merged_joint_names=_JOINT_NAMES,
                    )

    def test_runtime_validator_rejects_overlap_and_cross_robot_drift(self) -> None:
        overlapping_groups = _runtime_group_joint_names()
        overlapping_groups["duplicate"] = (_JOINT_NAMES[0],)
        with self.assertRaisesRegex(ValueError, "covered exactly once"):
            _MERGE.validate_sonic_runtime_actuators(
                SimpleNamespace(
                    scene={"robot": _fake_runtime_robot(overlapping_groups)}
                ),
                None,
                asset_names=("robot",),
                require_merged=False,
            )

        missing_field = _fake_runtime_robot(_runtime_group_joint_names())
        delattr(missing_field.actuators["legs"], "armature")
        with self.assertRaisesRegex(AttributeError, "missing runtime field 'armature'"):
            _MERGE.validate_sonic_runtime_actuators(
                SimpleNamespace(scene={"robot": missing_field}),
                None,
                asset_names=("robot",),
                require_merged=False,
            )

        asset_names = ("robot", "robot_2", "robot_3", "robot_4")
        groups = _runtime_group_joint_names()
        scene = {
            asset_name: _fake_runtime_robot(groups) for asset_name in asset_names
        }
        scene["robot_4"].actuators["legs"].stiffness.values[0] += 1.0
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, "hashes differ"):
                _MERGE.validate_sonic_runtime_actuators(
                    SimpleNamespace(scene=scene),
                    None,
                    asset_names=asset_names,
                    require_merged=False,
                )

    def test_runtime_validation_startup_term_is_strictly_opt_in(self) -> None:
        tree = ast.parse(_CONVEYOR_CFG.read_text(encoding="utf-8"))
        flag_assignment = next(
            node
            for node in tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "SONIC_VALIDATE_ACTUATORS"
                for target in node.targets
            )
        )
        call = flag_assignment.value
        self.assertIsInstance(call, ast.Call)
        assert isinstance(call, ast.Call)
        self.assertEqual(call.func.id, "_env_bool")
        self.assertEqual(call.args[0].value, "ISAACLAB_SONIC_VALIDATE_ACTUATORS")
        self.assertIs(call.args[1].value, False)

        host_names_assignment = next(
            node
            for node in tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "_HOST_SONIC_ASSET_NAMES"
                for target in node.targets
            )
        )
        host_names_expression = ast.Expression(body=host_names_assignment.value)
        ast.fix_missing_locations(host_names_expression)
        asset_names = eval(
            compile(host_names_expression, str(_CONVEYOR_CFG), "eval"),
            {"ACTIVE_SONIC_ROBOT_COUNT": 4},
        )
        self.assertEqual(asset_names, ("robot", "robot_2", "robot_3", "robot_4"))

        assignment = _class_assignment(
            tree, "ConveyorEventsCfg", "validate_sonic_actuators"
        )
        expression = ast.Expression(body=assignment.value)
        ast.fix_missing_locations(expression)

        def evaluate(*, enabled: bool, host_mode: bool):
            return eval(
                compile(expression, str(_CONVEYOR_CFG), "eval"),
                {
                    "EventTerm": lambda **kwargs: SimpleNamespace(**kwargs),
                    "validate_sonic_runtime_actuators": (
                        _MERGE.validate_sonic_runtime_actuators
                    ),
                    "SONIC_VALIDATE_ACTUATORS": enabled,
                    "HOST_MODE": host_mode,
                    "_HOST_SONIC_ASSET_NAMES": asset_names,
                    "SONIC_MERGE_ACTUATORS": True,
                    "_SONIC_JOINT_NAMES": _JOINT_NAMES,
                },
            )

        self.assertIsNone(evaluate(enabled=False, host_mode=True))
        self.assertIsNone(evaluate(enabled=True, host_mode=False))
        term = evaluate(enabled=True, host_mode=True)
        self.assertIs(term.func, _MERGE.validate_sonic_runtime_actuators)
        self.assertEqual(term.mode, "startup")
        self.assertEqual(term.params["asset_names"], asset_names)
        self.assertIs(term.params["require_merged"], True)
        self.assertEqual(
            term.params["expected_merged_joint_names"], _JOINT_NAMES
        )
        # EventManager invokes startup terms as
        # func(self._env, env_ids, **term_cfg.params).
        scene = {
            asset_name: _fake_runtime_robot(
                {"sonic_all": _PERMUTED_RUNTIME_JOINT_NAMES}
            )
            for asset_name in asset_names
        }
        with contextlib.redirect_stdout(io.StringIO()):
            hashes = term.func(SimpleNamespace(scene=scene), None, **term.params)
        self.assertEqual(tuple(hashes), asset_names)
        self.assertEqual(len(set(hashes.values())), 1)

    def test_viewer_factory_never_applies_true_robot_opt_in(self) -> None:
        peer = SimpleNamespace(
            actuators={"peer_all": object()},
            prim_path=None,
            init_state=SimpleNamespace(pos=None, rot=None),
            spawn=SimpleNamespace(visible=True),
        )

        def unexpected_merge(_actuators):
            raise AssertionError("viewer ghost must not enter the true-robot merge path")

        factory = _compile_factory(
            "_make_local_robot_cfg",
            {
                "ArticulationCfg": object,
                "VIEWER_MODE": True,
                "_make_peer_robot_cfg": lambda: peer,
                "_maybe_merge_local_sonic_actuator_groups": unexpected_merge,
                "make_sonic_robot_cfg": lambda: unexpected_merge(None),
                "LOCAL_ROBOT_POS": (0.0, -30.0, 0.76),
                "LOCAL_ROBOT_ROT": (1.0, 0.0, 0.0, 0.0),
                "CONTACT_REPORT_MODE": "off",
                "configure_robot_contact_reports": lambda *args: None,
            },
        )
        result = factory()
        self.assertIs(result, peer)
        self.assertEqual(tuple(result.actuators), ("peer_all",))
        self.assertFalse(result.spawn.visible)

    def test_conveyor_flag_defaults_off_and_only_true_robot_builders_use_it(self) -> None:
        tree = ast.parse(_CONVEYOR_CFG.read_text(encoding="utf-8"))
        flag_assignment = next(
            node
            for node in tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "SONIC_MERGE_ACTUATORS"
                for target in node.targets
            )
        )
        self.assertIsInstance(flag_assignment.value, ast.Call)
        call = flag_assignment.value
        assert isinstance(call, ast.Call)
        self.assertEqual(call.func.id, "_env_bool")
        self.assertEqual(call.args[0].value, "ISAACLAB_SONIC_MERGE_ACTUATORS")
        self.assertIs(call.args[1].value, False)

        for function_name in (
            "_make_local_robot_cfg",
            "_make_additional_local_robot_cfg",
        ):
            function = _function(tree, function_name)
            merge_calls = [
                node
                for node in ast.walk(function)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "_maybe_merge_local_sonic_actuator_groups"
            ]
            self.assertEqual(len(merge_calls), 1, function_name)

        peer_wrapper = _function(tree, "_merge_peer_actuator_groups")
        peer_calls = [
            node
            for node in ast.walk(peer_wrapper)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_merge_sonic_actuator_groups"
        ]
        self.assertEqual(len(peer_calls), 1)
        merged_name_keyword = next(
            keyword
            for keyword in peer_calls[0].keywords
            if keyword.arg == "merged_name"
        )
        self.assertEqual(merged_name_keyword.value.value, "peer_all")

        peer_factory = _function(tree, "_make_peer_robot_cfg")
        peer_factory_calls = [
            node
            for node in ast.walk(peer_factory)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_merge_peer_actuator_groups"
        ]
        self.assertEqual(len(peer_factory_calls), 1)
        for function in (peer_wrapper, peer_factory):
            self.assertFalse(
                any(
                    isinstance(node, ast.Name)
                    and node.id == "SONIC_MERGE_ACTUATORS"
                    for node in ast.walk(function)
                ),
                function.name,
            )

        base_source = _SONIC_CFG.read_text(encoding="utf-8")
        self.assertNotIn("ISAACLAB_SONIC_MERGE_ACTUATORS", base_source)


if __name__ == "__main__":
    unittest.main()
