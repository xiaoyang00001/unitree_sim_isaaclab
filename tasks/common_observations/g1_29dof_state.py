# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
g1_29dof state
"""     
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Sequence

import torch

from dds.dds_master import dds_manager
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

# global variable to cache the DDS instance
_g1_robot_dds = None

# 观测缓存：索引张量与DDS限速（50FPS）+ 预分配缓冲
_obs_cache = {
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
    "dds_min_interval_ms": 20,
}

# Cache body-name resolution; the selected wholebody asset has dedicated pelvis
# and torso IMU links, but fallbacks keep diagnostics useful for related assets.
_imu_body_cache = {
    "body_names": None,
    "indices": {},
}


def _get_g1_robot_dds_instance():
    """Borrow the manager-owned G1 DDS object once it has been registered."""
    global _g1_robot_dds

    if _g1_robot_dds is None:
        _g1_robot_dds = dds_manager.get_object("g129")
        if _g1_robot_dds is not None:
            print("[g1_state] G1 robot DDS communication instance obtained")

    return _g1_robot_dds

def get_robot_boy_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
) -> torch.Tensor:
    """get the robot body joint states, positions and velocities
    
    Args:
        env: ManagerBasedRLEnv - reinforcement learning environment instance
        enable_dds: bool - whether to enable the DDS publish function
    
    Returns:
        torch.Tensor
        - the first 29 elements are joint positions
        - the middle 29 elements are joint velocities
        - the last 29 elements are joint torques
    """
    # get all joint states
    joint_pos = env.scene["robot"].data.joint_pos
    joint_vel = env.scene["robot"].data.joint_vel
    joint_torque = env.scene["robot"].data.applied_torque  # use applied_torque to get joint torques
    device = joint_pos.device
    batch = joint_pos.shape[0]

    # 预计算并缓存索引张量（列索引）
    global _obs_cache
    articulation_joint_names = tuple(env.scene["robot"].data.joint_names)
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

    # write to DDS（限速发布，避免高频CPU拷贝）
    if enable_dds and combined_buf.shape[0] > 0:
        try:
            now_ns = time.monotonic_ns()
            min_interval_ns = int(_obs_cache["dds_min_interval_ms"] * 1_000_000)
            if now_ns - _obs_cache["dds_last_ns"] >= min_interval_ns:
                g1_robot_dds = _get_g1_robot_dds_instance()
                if g1_robot_dds:
                    base_imu_data = get_robot_imu_data(
                        env,
                        # The released policy was trained from the articulation
                        # root (pelvis), not from a separate visual IMU body.
                        body_candidates=("pelvis", "imu_in_pelvis"),
                    )
                    torso_imu_data = get_robot_imu_data(
                        env,
                        body_candidates=("torso_link", "imu_in_torso"),
                    )
                    if base_imu_data.shape[0] > 0 and torso_imu_data.shape[0] > 0:
                        g1_robot_dds.write_robot_state(
                            pos_buf[0].contiguous().cpu().numpy(),
                            vel_buf[0].contiguous().cpu().numpy(),
                            torque_buf[0].contiguous().cpu().numpy(),
                            base_imu_data=base_imu_data[0].contiguous().cpu().numpy(),
                            torso_imu_data=torso_imu_data[0].contiguous().cpu().numpy(),
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
) -> torch.Tensor:
    """
    Returns [batch, 13] = pos(world,3) | quat(w,x,y,z) | acc_body(3) | gyro_body(3)

    Isaac Lab body quaternions are already wxyz. PhysX provides world-frame
    link acceleration and angular velocity; both are transformed into the
    selected IMU link frame. The accelerometer value is proper acceleration,
    so gravity is subtracted before the frame transform.
    """
    data = env.scene["robot"].data
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
