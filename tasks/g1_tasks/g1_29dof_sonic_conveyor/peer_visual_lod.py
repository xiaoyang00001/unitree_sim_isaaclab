"""Pure-visual G1 mirror kinematics and USD Xform writer.

The normal peer mirror is an Isaac Lab ``Articulation`` even though it never
owns physics.  ``visual_lod`` mode instead spawns a small USD made only from
Xforms and analytic render primitives, then applies the scene-state frame with
this module.  The math helpers deliberately have no Isaac Sim, USD, torch, or
NumPy dependency so the 43-DoF mapping can be tested in a normal Python
process.

Joint origins and axes are copied from the generated SONIC 29-DoF + Dex3 URDF
(``g1_29dof_with_hand_rev_1_0_sonic_isaaclab.urdf``).  Runtime USD imports are
lazy: importing this module never starts Kit and never registers PhysX state.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from typing import Sequence


PEER_ROBOT_MODES = ("articulation", "visual_lod")


def resolve_peer_robot_mode(value: str | None) -> str:
    """Normalize the opt-in peer mode and reject silent fallback typos."""

    mode = "articulation" if value is None or not value.strip() else value.strip().lower()
    if mode not in PEER_ROBOT_MODES:
        allowed = ", ".join(PEER_ROBOT_MODES)
        raise ValueError(f"ISAACLAB_PEER_ROBOT_MODE={value!r} 无效，可选值：{allowed}")
    return mode


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class FixedLinkSpec:
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _joint(
    name: str,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    axis: tuple[float, float, float],
) -> JointSpec:
    return JointSpec(name, parent, child, xyz, rpy, axis)


# Topological order is intentional.  The wire order is supplied in every
# scene-state frame and can differ; apply_joint_positions maps by joint name.
JOINT_SPECS: tuple[JointSpec, ...] = (
    _joint("left_hip_pitch_joint", "pelvis", "left_hip_pitch_link", (0.0, 0.064452, -0.1027), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("left_hip_roll_joint", "left_hip_pitch_link", "left_hip_roll_link", (0.0, 0.052, -0.030465), (0.0, -0.1749, 0.0), (1.0, 0.0, 0.0)),
    _joint("left_hip_yaw_joint", "left_hip_roll_link", "left_hip_yaw_link", (0.025001, 0.0, -0.12412), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_knee_joint", "left_hip_yaw_link", "left_knee_link", (-0.078273, 0.0021489, -0.17734), (0.0, 0.1749, 0.0), (0.0, 1.0, 0.0)),
    _joint("left_ankle_pitch_joint", "left_knee_link", "left_ankle_pitch_link", (0.0, -0.000094445, -0.30001), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("left_ankle_roll_joint", "left_ankle_pitch_link", "left_ankle_roll_link", (0.0, 0.0, -0.017558), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    _joint("right_hip_pitch_joint", "pelvis", "right_hip_pitch_link", (0.0, -0.064452, -0.1027), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("right_hip_roll_joint", "right_hip_pitch_link", "right_hip_roll_link", (0.0, -0.052, -0.030465), (0.0, -0.1749, 0.0), (1.0, 0.0, 0.0)),
    _joint("right_hip_yaw_joint", "right_hip_roll_link", "right_hip_yaw_link", (0.025001, 0.0, -0.12412), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_knee_joint", "right_hip_yaw_link", "right_knee_link", (-0.078273, -0.0021489, -0.17734), (0.0, 0.1749, 0.0), (0.0, 1.0, 0.0)),
    _joint("right_ankle_pitch_joint", "right_knee_link", "right_ankle_pitch_link", (0.0, 0.000094445, -0.30001), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("right_ankle_roll_joint", "right_ankle_pitch_link", "right_ankle_roll_link", (0.0, 0.0, -0.017558), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    _joint("waist_yaw_joint", "pelvis", "waist_yaw_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("waist_roll_joint", "waist_yaw_link", "waist_roll_link", (-0.0039635, 0.0, 0.044), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    _joint("waist_pitch_joint", "waist_roll_link", "torso_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("left_shoulder_pitch_joint", "torso_link", "left_shoulder_pitch_link", (0.0039563, 0.10022, 0.24778), (0.27931, 0.000054949, -0.00019159), (0.0, 1.0, 0.0)),
    _joint("left_shoulder_roll_joint", "left_shoulder_pitch_link", "left_shoulder_roll_link", (0.0, 0.038, -0.013831), (-0.27925, 0.0, 0.0), (1.0, 0.0, 0.0)),
    _joint("left_shoulder_yaw_joint", "left_shoulder_roll_link", "left_shoulder_yaw_link", (0.0, 0.00624, -0.1032), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_elbow_joint", "left_shoulder_yaw_link", "left_elbow_link", (0.015783, 0.0, -0.080518), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("left_wrist_roll_joint", "left_elbow_link", "left_wrist_roll_link", (0.1, 0.00188791, -0.01), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    _joint("left_wrist_pitch_joint", "left_wrist_roll_link", "left_wrist_pitch_link", (0.038, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("left_wrist_yaw_joint", "left_wrist_pitch_link", "left_wrist_yaw_link", (0.046, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_shoulder_pitch_joint", "torso_link", "right_shoulder_pitch_link", (0.0039563, -0.10021, 0.24778), (-0.27931, 0.000054949, 0.00019159), (0.0, 1.0, 0.0)),
    _joint("right_shoulder_roll_joint", "right_shoulder_pitch_link", "right_shoulder_roll_link", (0.0, -0.038, -0.013831), (0.27925, 0.0, 0.0), (1.0, 0.0, 0.0)),
    _joint("right_shoulder_yaw_joint", "right_shoulder_roll_link", "right_shoulder_yaw_link", (0.0, -0.00624, -0.1032), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_elbow_joint", "right_shoulder_yaw_link", "right_elbow_link", (0.015783, 0.0, -0.080518), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("right_wrist_roll_joint", "right_elbow_link", "right_wrist_roll_link", (0.1, -0.00188791, -0.01), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    _joint("right_wrist_pitch_joint", "right_wrist_roll_link", "right_wrist_pitch_link", (0.038, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("right_wrist_yaw_joint", "right_wrist_pitch_link", "right_wrist_yaw_link", (0.046, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_hand_thumb_0_joint", "left_hand_palm_link", "left_hand_thumb_0_link", (0.0255, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("left_hand_thumb_1_joint", "left_hand_thumb_0_link", "left_hand_thumb_1_link", (-0.0025, -0.0193, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_hand_thumb_2_joint", "left_hand_thumb_1_link", "left_hand_thumb_2_link", (0.0, -0.0458, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_hand_middle_0_joint", "left_hand_palm_link", "left_hand_middle_0_link", (0.0777, 0.0016, -0.0285), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_hand_middle_1_joint", "left_hand_middle_0_link", "left_hand_middle_1_link", (0.0458, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_hand_index_0_joint", "left_hand_palm_link", "left_hand_index_0_link", (0.0777, 0.0016, 0.0285), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("left_hand_index_1_joint", "left_hand_index_0_link", "left_hand_index_1_link", (0.0458, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_hand_thumb_0_joint", "right_hand_palm_link", "right_hand_thumb_0_link", (0.0255, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    _joint("right_hand_thumb_1_joint", "right_hand_thumb_0_link", "right_hand_thumb_1_link", (-0.0025, 0.0193, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_hand_thumb_2_joint", "right_hand_thumb_1_link", "right_hand_thumb_2_link", (0.0, 0.0458, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_hand_middle_0_joint", "right_hand_palm_link", "right_hand_middle_0_link", (0.0777, -0.0016, -0.0285), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_hand_middle_1_joint", "right_hand_middle_0_link", "right_hand_middle_1_link", (0.0458, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_hand_index_0_joint", "right_hand_palm_link", "right_hand_index_0_link", (0.0777, -0.0016, 0.0285), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    _joint("right_hand_index_1_joint", "right_hand_index_0_link", "right_hand_index_1_link", (0.0458, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
)


FIXED_LINK_SPECS: tuple[FixedLinkSpec, ...] = (
    FixedLinkSpec("left_wrist_yaw_link", "left_hand_palm_link", (0.0415, 0.003, 0.0)),
    FixedLinkSpec("right_wrist_yaw_link", "right_hand_palm_link", (0.0415, -0.003, 0.0)),
    FixedLinkSpec("torso_link", "head_link", (0.0039635, 0.0, -0.044)),
)

JOINT_NAMES: tuple[str, ...] = tuple(spec.name for spec in JOINT_SPECS)
JOINT_NAME_SET = frozenset(JOINT_NAMES)
JOINT_BY_NAME = {spec.name: spec for spec in JOINT_SPECS}


def validate_joint_order(joint_names: Sequence[str]) -> tuple[str, ...]:
    """Validate a wire-order mapping without requiring one particular order."""

    names = tuple(str(name) for name in joint_names)
    if len(names) != len(JOINT_SPECS):
        raise ValueError(f"visual_lod 需要 {len(JOINT_SPECS)} 个关节，收到 {len(names)} 个")
    if len(set(names)) != len(names):
        raise ValueError("visual_lod 关节顺序包含重复名称")
    actual = frozenset(names)
    if actual != JOINT_NAME_SET:
        missing = sorted(JOINT_NAME_SET - actual)
        unexpected = sorted(actual - JOINT_NAME_SET)
        raise ValueError(f"visual_lod 关节集合不匹配：missing={missing}, unexpected={unexpected}")
    return names


def joint_order_hash(joint_names: Sequence[str]) -> str:
    """Return the scene-state protocol's stable 16-hex joint-order hash."""

    names = tuple(str(name) for name in joint_names)
    return hashlib.sha256("\0".join(names).encode("utf-8")).hexdigest()[:16]


def validate_wire_joint_order(
    joint_names: Sequence[str], expected_hash: str
) -> tuple[str, ...]:
    """Validate visual joint membership and that names reproduce the wire hash."""

    names = validate_joint_order(joint_names)
    calculated = joint_order_hash(names)
    if calculated != str(expected_hash):
        raise ValueError(
            f"joint_names hash mismatch: payload={expected_hash} calculated={calculated}"
        )
    return names


def _quat_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _normalized_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    if len(quaternion) != 4:
        raise ValueError(f"quaternion expected 4 values, received {len(quaternion)}")
    values = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("quaternion is zero or non-finite")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _rpy_quaternion(rpy: Sequence[float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return _normalized_quaternion(
        (
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
        )
    )


def joint_local_quaternion(spec: JointSpec, angle: float) -> tuple[float, float, float, float]:
    """Return URDF ``R(rpy) * R(axis, q)`` as a normalized wxyz quaternion."""

    ax, ay, az = spec.axis
    axis_norm = math.sqrt(ax * ax + ay * ay + az * az)
    if axis_norm < 1.0e-12:
        raise ValueError(f"joint {spec.name!r} has a zero axis")
    half = float(angle) * 0.5
    scale = math.sin(half) / axis_norm
    axis_quaternion = (math.cos(half), ax * scale, ay * scale, az * scale)
    return _normalized_quaternion(_quat_multiply(_rpy_quaternion(spec.rpy), axis_quaternion))


Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


def _pose_matrix(
    xyz: Sequence[float], quaternion_wxyz: Sequence[float]
) -> Matrix4:
    if len(xyz) != 3:
        raise ValueError(f"translation expected 3 values, received {len(xyz)}")
    w, x, y, z = _normalized_quaternion(quaternion_wxyz)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w), float(xyz[0])),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w), float(xyz[1])),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y), float(xyz[2])),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matrix_multiply(first: Matrix4, second: Matrix4) -> Matrix4:
    return tuple(
        tuple(sum(first[row][k] * second[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )  # type: ignore[return-value]


def _link_parent_transforms(
    joint_positions: dict[str, float],
) -> tuple[list[tuple[str, str, Matrix4]], list[tuple[str, str, Matrix4]]]:
    joints = [
        (spec.parent, spec.child, _pose_matrix(spec.xyz, joint_local_quaternion(spec, joint_positions[spec.name])))
        for spec in JOINT_SPECS
    ]
    fixed = [
        (spec.parent, spec.child, _pose_matrix(spec.xyz, _rpy_quaternion(spec.rpy)))
        for spec in FIXED_LINK_SPECS
    ]
    return joints, fixed


def forward_kinematics(
    root_state: Sequence[float],
    joint_positions: Sequence[float],
    joint_names: Sequence[str],
) -> dict[str, Matrix4]:
    """Compute link world transforms for one scene-state robot frame.

    ``root_state`` follows Isaac Lab's 13-value convention.  Velocities are
    accepted but ignored because a visual mirror owns no physical state.
    """

    if len(root_state) != 13:
        raise ValueError(f"root_state expected 13 values, received {len(root_state)}")
    names = validate_joint_order(joint_names)
    if len(joint_positions) != len(names):
        raise ValueError(
            f"joint_positions expected {len(names)} values, received {len(joint_positions)}"
        )
    values = tuple(float(value) for value in (*root_state, *joint_positions))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("visual_lod frame contains non-finite values")
    by_name = {name: float(value) for name, value in zip(names, joint_positions)}
    result: dict[str, Matrix4] = {
        "pelvis": _pose_matrix(root_state[:3], root_state[3:7]),
    }
    joint_transforms, fixed_transforms = _link_parent_transforms(by_name)
    pending = [*joint_transforms, *fixed_transforms]
    while pending:
        deferred: list[tuple[str, str, Matrix4]] = []
        for parent, child, local in pending:
            parent_world = result.get(parent)
            if parent_world is None:
                deferred.append((parent, child, local))
                continue
            result[child] = _matrix_multiply(parent_world, local)
        if len(deferred) == len(pending):
            unresolved = sorted(f"{parent}->{child}" for parent, child, _ in deferred)
            raise RuntimeError(f"visual_lod FK topology is disconnected: {unresolved}")
        pending = deferred
    return result


def link_paths(root_prim_path: str) -> dict[str, str]:
    """Return every animated/fixed link's expected USD path."""

    root = root_prim_path.rstrip("/")
    paths = {"pelvis": f"{root}/pelvis"}
    pending: list[tuple[str, str]] = [
        *((spec.parent, spec.child) for spec in JOINT_SPECS),
        *((spec.parent, spec.child) for spec in FIXED_LINK_SPECS),
    ]
    while pending:
        deferred = []
        for parent, child in pending:
            parent_path = paths.get(parent)
            if parent_path is None:
                deferred.append((parent, child))
            else:
                paths[child] = f"{parent_path}/{child}"
        if len(deferred) == len(pending):
            raise RuntimeError(f"visual_lod path topology is disconnected: {deferred}")
        pending = deferred
    return paths


class UsdVisualLodMirror:
    """Write one pure-visual mirror on the active USD stage.

    The class only touches Xform translate/orient attributes.  It never calls
    an Isaac Lab asset API and never applies Physics, PhysX, Articulation,
    Collision, Actuator, or ContactReport schemas.
    """

    def __init__(self, root_prim_path: str, *, stage=None):
        if stage is None:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("visual_lod 无法取得当前 USD stage")

        from pxr import Gf

        self._gf = Gf
        self.root_prim_path = root_prim_path.rstrip("/")
        root_prim = stage.GetPrimAtPath(self.root_prim_path)
        if not root_prim.IsValid():
            raise RuntimeError(f"visual_lod 根 Prim 不存在：{self.root_prim_path}")
        self._root_translate = root_prim.GetAttribute("xformOp:translate")
        self._root_orient = root_prim.GetAttribute("xformOp:orient")
        if not self._root_translate.IsValid() or not self._root_orient.IsValid():
            raise RuntimeError(f"visual_lod 根 Prim 缺少 translate/orient：{self.root_prim_path}")

        paths = link_paths(self.root_prim_path)
        self._joint_orients = {}
        for spec in JOINT_SPECS:
            prim_path = paths[spec.child]
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                raise RuntimeError(f"visual_lod 关节 Link Prim 不存在：{prim_path}")
            orient = prim.GetAttribute("xformOp:orient")
            if not orient.IsValid():
                raise RuntimeError(f"visual_lod Link 缺少 orient：{prim_path}")
            self._joint_orients[spec.name] = orient

        # Correct non-zero URDF origin RPY immediately, before the first frame.
        self.apply_joint_positions((0.0,) * len(JOINT_SPECS), JOINT_NAMES)

    def _gf_quaternion(self, quaternion: Sequence[float]):
        w, x, y, z = _normalized_quaternion(quaternion)
        return self._gf.Quatf(w, self._gf.Vec3f(x, y, z))

    def apply_joint_positions(
        self, joint_positions: Sequence[float], joint_names: Sequence[str]
    ) -> None:
        names = validate_joint_order(joint_names)
        if len(joint_positions) != len(names):
            raise ValueError(
                f"joint_positions expected {len(names)} values, received {len(joint_positions)}"
            )
        by_name = {name: float(value) for name, value in zip(names, joint_positions)}
        if not all(math.isfinite(value) for value in by_name.values()):
            raise ValueError("visual_lod joint_positions contains non-finite values")
        for spec in JOINT_SPECS:
            self._joint_orients[spec.name].Set(
                self._gf_quaternion(joint_local_quaternion(spec, by_name[spec.name]))
            )

    def apply_state(
        self,
        root_state: Sequence[float],
        joint_positions: Sequence[float],
        joint_names: Sequence[str],
    ) -> None:
        if len(root_state) != 13:
            raise ValueError(f"root_state expected 13 values, received {len(root_state)}")
        values = tuple(float(value) for value in root_state)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("visual_lod root_state contains non-finite values")
        self._root_translate.Set(self._gf.Vec3d(*values[:3]))
        self._root_orient.Set(self._gf_quaternion(values[3:7]))
        self.apply_joint_positions(joint_positions, joint_names)
