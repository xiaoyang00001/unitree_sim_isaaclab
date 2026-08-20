# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
g1_29dof state
"""     
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Sequence

import torch

from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def get_robot_boy_joint_names() -> list[str]:
    return [
        # leg joints (12)
        # left leg (6)
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        # right leg (6)
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        # waist joints (3)
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",

        # arm joints (14)
        # left arm (7)
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        # right arm (7)
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

def get_robot_arm_joint_names() -> list[str]:
    return [
        # arm joints (14)
        # left arm (7)
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        # right arm (7)
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

# DDS 实例缓存：按注册名键控（host 双机器人模式下有 "g129" 与 "g129_r2" 两套）。
_g1_robot_dds_by_name: dict = {}

# 观测缓存：索引张量 + 预分配缓冲。DDS 发布周期由调用任务决定；
# SONIC 任务必须在每个 50 Hz 控制步发布一个新物理样本，不再用墙钟限速。
# ⚠️ 按 asset 名键控：host 模式下 robot 与 robot_2 各有一个 ObsTerm，共用缓冲会让
# 第二个 term 的 gather 就地覆盖第一个 term 已返回的观测张量（aliasing），
# 共用 sample_seq 会让两条锁步链路的 tick 交错、ack 永不匹配。
_obs_caches: dict = {}


def _get_obs_cache(asset_name: str) -> dict:
    cache = _obs_caches.get(asset_name)
    if cache is None:
        cache = {
            "device": None,
            "joint_names": None,
            "batch": None,
            "boy_idx_t": None,
            "boy_idx_batch": None,
            "pos_buf": None,
            "vel_buf": None,
            "torque_buf": None,
            "combined_buf": None,
            "dds_last_ns": 0,
            "sample_seq": 0,
        }
        _obs_caches[asset_name] = cache
    return cache

# Cache body-name resolution; the selected wholebody asset has dedicated pelvis
# and torso IMU links, but fallbacks keep diagnostics useful for related assets.
_imu_body_cache = {
    "body_names": None,
    "indices": {},
}


def _get_g1_robot_dds_instance(dds_object_name: str = "g129"):
    """Borrow the manager-owned G1 DDS object once it has been registered."""
    instance = _g1_robot_dds_by_name.get(dds_object_name)
    if instance is None:
        # Importing dds_master constructs the global DDSManager and initializes
        # the Unitree DDS participant.  Keep that side effect behind the
        # enable_dds call path so pure scene-sync viewers remain ZMQ-only.
        from dds.dds_master import dds_manager

        instance = dds_manager.get_object(dds_object_name)
        if instance is not None:
            _g1_robot_dds_by_name[dds_object_name] = instance
            print(f"[g1_state] G1 robot DDS communication instance obtained ({dds_object_name})")

    return instance

def get_robot_boy_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
    dds_min_interval_ms: float = 20.0,
    asset_name: str = "robot",
    dds_object_name: str = "g129",
) -> torch.Tensor:
    """get the robot body joint states, positions and velocities

    Args:
        env: ManagerBasedRLEnv - reinforcement learning environment instance
        enable_dds: bool - whether to enable the DDS publish function
        dds_min_interval_ms: optional wall-clock throttle for legacy tasks.
            Set to 0 for SONIC so that every environment control step is a
            distinct LowState sample.  A 20 ms wall-clock gate is not safe at
            a nominal 50 Hz because small scheduler jitter can alias it down
            to 25--33 Hz.
        asset_name: 场景里的机器人资产名（host 第二机器人传 "robot_2"）。
        dds_object_name: DDS 管理器里的注册名（host 第二机器人传 "g129_r2"）。

    Returns:
        torch.Tensor
        - the first 29 elements are joint positions
        - the middle 29 elements are joint velocities
        - the last 29 elements are joint torques
    """
    # get all joint states
    robot = env.scene[asset_name]
    joint_pos = robot.data.joint_pos
    joint_vel = robot.data.joint_vel
    joint_torque = robot.data.applied_torque  # use applied_torque to get joint torques
    device = joint_pos.device
    batch = joint_pos.shape[0]

    # 预计算并缓存索引张量（列索引）；缓存按 asset 键控
    _obs_cache = _get_obs_cache(asset_name)
    articulation_joint_names = tuple(robot.data.joint_names)
    if (
        _obs_cache["device"] != device
        or _obs_cache["joint_names"] != articulation_joint_names
        or _obs_cache["boy_idx_t"] is None
    ):
        joint_to_index = {name: index for index, name in enumerate(articulation_joint_names)}
        missing = [name for name in G1_29DOF_DDS_JOINT_ORDER if name not in joint_to_index]
        if missing:
            raise ValueError(f"G1 LowState mapping is missing joints: {missing}")
        boy_joint_indices = [joint_to_index[name] for name in G1_29DOF_DDS_JOINT_ORDER]
        _obs_cache["boy_idx_t"] = torch.tensor(boy_joint_indices, dtype=torch.long, device=device)
        _obs_cache["device"] = device
        _obs_cache["joint_names"] = articulation_joint_names
        _obs_cache["batch"] = None  # force re-init batch-shaped buffers

    idx_t = _obs_cache["boy_idx_t"]
    n = idx_t.numel()

    # 预分配/复用 batch 形状索引与输出缓冲
    if _obs_cache["batch"] != batch or _obs_cache["boy_idx_batch"] is None:
        _obs_cache["boy_idx_batch"] = idx_t.unsqueeze(0).expand(batch, n)
        _obs_cache["pos_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["vel_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["torque_buf"] = torch.empty(batch, n, device=device, dtype=joint_pos.dtype)
        _obs_cache["combined_buf"] = torch.empty(batch, n * 3, device=device, dtype=joint_pos.dtype)
        _obs_cache["batch"] = batch

    idx_batch = _obs_cache["boy_idx_batch"]
    pos_buf = _obs_cache["pos_buf"]
    vel_buf = _obs_cache["vel_buf"]
    torque_buf = _obs_cache["torque_buf"]
    combined_buf = _obs_cache["combined_buf"]

    # 使用 gather(out=...) 填充，避免新张量分配
    try:
        torch.gather(joint_pos, 1, idx_batch, out=pos_buf)
        torch.gather(joint_vel, 1, idx_batch, out=vel_buf)
        torch.gather(joint_torque, 1, idx_batch, out=torque_buf)
    except TypeError:
        pos_buf.copy_(torch.gather(joint_pos, 1, idx_batch))
        vel_buf.copy_(torch.gather(joint_vel, 1, idx_batch))
        torque_buf.copy_(torch.gather(joint_torque, 1, idx_batch))

    # 组合为一个缓冲，避免 cat 分配
    combined_buf[:, 0:n].copy_(pos_buf)
    combined_buf[:, n:2*n].copy_(vel_buf)
    combined_buf[:, 2*n:3*n].copy_(torque_buf)

    # Write to DDS.  SONIC passes dds_min_interval_ms=0 and therefore publishes
    # exactly once per observation/control step.  Other tasks keep the legacy
    # wall-clock throttle unless they explicitly opt out.
    if enable_dds and combined_buf.shape[0] > 0:
        try:
            now_ns = time.monotonic_ns()
            min_interval_ns = max(0, int(float(dds_min_interval_ms) * 1_000_000))
            if now_ns - _obs_cache["dds_last_ns"] >= min_interval_ns:
                g1_robot_dds = _get_g1_robot_dds_instance(dds_object_name)
                if g1_robot_dds:
                    base_imu_data = get_robot_imu_data(
                        env,
                        # The released policy was trained from the articulation
                        # root (pelvis), not from a separate visual IMU body.
                        body_candidates=("pelvis", "imu_in_pelvis"),
                        asset_name=asset_name,
                    )
                    torso_imu_data = get_robot_imu_data(
                        env,
                        body_candidates=("torso_link", "imu_in_torso"),
                        asset_name=asset_name,
                    )
                    if base_imu_data.shape[0] > 0 and torso_imu_data.shape[0] > 0:
                        _obs_cache["sample_seq"] += 1
                        sample_seq = int(_obs_cache["sample_seq"])
                        step_dt = float(getattr(env, "step_dt", 0.02))
                        sim_time_s = sample_seq * step_dt
                        g1_robot_dds.write_robot_state(
                            pos_buf[0].contiguous().cpu().numpy(),
                            vel_buf[0].contiguous().cpu().numpy(),
                            torque_buf[0].contiguous().cpu().numpy(),
                            base_imu_data=base_imu_data[0].contiguous().cpu().numpy(),
                            torso_imu_data=torso_imu_data[0].contiguous().cpu().numpy(),
                            sample_seq=sample_seq,
                            sim_time_s=sim_time_s,
                        )
                        _obs_cache["dds_last_ns"] = now_ns
        except Exception as e:
            print(f"[g1_state] Error writing robot state to DDS: {e}")
    
    return combined_buf


def quat_to_rot_matrix(q):
    """
    q: [B,4] assumed (w,x,y,z)
    returns R: [B,3,3] such that v_world = R @ v_body
    """
    w = q[:, 0:1]
    x = q[:, 1:2]
    y = q[:, 2:3]
    z = q[:, 3:4]

    # precompute
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z
    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z

    # rotation matrix elements
    r00 = ww + xx - yy - zz
    r01 = 2 * (xy - wz)
    r02 = 2 * (xz + wy)

    r10 = 2 * (xy + wz)
    r11 = ww - xx + yy - zz
    r12 = 2 * (yz - wx)

    r20 = 2 * (xz - wy)
    r21 = 2 * (yz + wx)
    r22 = ww - xx - yy + zz

    R = torch.cat([
        torch.cat([r00, r01, r02], dim=1).unsqueeze(1),
        torch.cat([r10, r11, r12], dim=1).unsqueeze(1),
        torch.cat([r20, r21, r22], dim=1).unsqueeze(1),
    ], dim=1)  # [B,3,3] but built transposed blocks; fix shape next

    # Currently R is [B,3,3] where rows are correct; reshape properly:
    R = R.view(-1, 3, 3)
    return R

def _resolve_imu_body_index(data, body_candidates: Sequence[str]) -> int:
    body_names = tuple(data.body_names)
    if _imu_body_cache["body_names"] != body_names:
        _imu_body_cache["body_names"] = body_names
        _imu_body_cache["indices"] = {}

    cache_key = tuple(body_candidates)
    cached_index = _imu_body_cache["indices"].get(cache_key)
    if cached_index is not None:
        return cached_index

    for body_name in body_candidates:
        if body_name in body_names:
            body_index = body_names.index(body_name)
            _imu_body_cache["indices"][cache_key] = body_index
            return body_index

    raise ValueError(
        f"None of the required IMU bodies {tuple(body_candidates)} exist; "
        f"available bodies: {body_names}"
    )


def _get_gravity_w(env, reference: torch.Tensor) -> torch.Tensor:
    gravity = (0.0, 0.0, -9.81)
    try:
        gravity = tuple(env.cfg.sim.gravity)
    except Exception:
        try:
            gravity = tuple(env.sim.physx.gravity)
        except Exception:
            pass

    return torch.tensor(gravity, dtype=reference.dtype, device=reference.device).unsqueeze(0).expand_as(reference)


def get_robot_imu_data(
    env,
    body_candidates: Sequence[str] = ("pelvis", "imu_in_pelvis"),
    asset_name: str = "robot",
) -> torch.Tensor:
    """
    Returns [batch, 13] = pos(world,3) | quat(w,x,y,z) | acc_body(3) | gyro_body(3)

    Isaac Lab body quaternions are already wxyz. PhysX provides world-frame
    link acceleration and angular velocity; both are transformed into the
    selected IMU link frame. The accelerometer value is proper acceleration,
    so gravity is subtracted before the frame transform.
    """
    data = env.scene[asset_name].data
    body_index = _resolve_imu_body_index(data, body_candidates)

    body_pose_w = data.body_link_pose_w[:, body_index]
    body_vel_w = data.body_link_vel_w[:, body_index]
    body_acc_w = data.body_com_acc_w[:, body_index]

    pos_w = body_pose_w[:, :3]
    quat_wxyz = body_pose_w[:, 3:7]
    ang_vel_w = body_vel_w[:, 3:6]
    lin_acc_w = body_acc_w[:, :3]

    quat_norm = torch.linalg.vector_norm(quat_wxyz, dim=1, keepdim=True)
    if bool((quat_norm < 1.0e-6).any()):
        raise ValueError(f"Invalid zero-norm quaternion for IMU bodies {tuple(body_candidates)}")
    quat_wxyz = quat_wxyz / quat_norm

    if not bool(torch.isfinite(torch.cat((pos_w, quat_wxyz, lin_acc_w, ang_vel_w), dim=1)).all()):
        raise ValueError(f"Non-finite IMU state for bodies {tuple(body_candidates)}")

    gravity_w = _get_gravity_w(env, lin_acc_w)
    proper_acc_w = lin_acc_w - gravity_w

    # build rotation matrices R_body->world ; to convert world->body use R^T
    R_body_to_world = quat_to_rot_matrix(quat_wxyz)  # [B,3,3]
    R_world_to_body = R_body_to_world.transpose(1, 2)  # [B,3,3]

    acc_body = torch.bmm(R_world_to_body, proper_acc_w.unsqueeze(-1)).squeeze(-1)
    gyro_body = torch.bmm(R_world_to_body, ang_vel_w.unsqueeze(-1)).squeeze(-1)

    imu_data = torch.cat([pos_w, quat_wxyz, acc_body, gyro_body], dim=1)
    return imu_data
