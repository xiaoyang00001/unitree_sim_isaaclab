"""Dex3 articulation state observation and DDS state publisher."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import torch

from robots.g1_sonic_urdf import DEX3_HAND_JOINT_NAMES

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# 观测缓存按 asset 名键控（host 双机器人下 robot 与 robot_2 各一份）：
# 共用 pos_buf 会让两个 ObsTerm 互相覆写返回值（aliasing），不是性能问题是正确性问题。
_obs_caches: dict = {}


def _get_obs_cache(asset_name: str) -> dict:
    cache = _obs_caches.get(asset_name)
    if cache is None:
        cache = {
            "mapping_key": None,
            "hand_idx_t": None,
            "batch": None,
            "dtype": None,
            "hand_idx_batch": None,
            "pos_buf": None,
            "vel_buf": None,
            "torque_buf": None,
            "dds_last_monotonic": 0.0,
            "missing_mapping_logged": False,
        }
        _obs_caches[asset_name] = cache
    return cache


_dex3_dds_by_name: dict = {}
_dds_obtained_logged_names: set = set()


def get_robot_girl_joint_names() -> list[str]:
    """Return the Unitree Dex3 DDS order (left seven, then right seven)."""

    return list(DEX3_HAND_JOINT_NAMES)


def _get_dex3_dds_instance(dds_object_name: str = "dex3"):
    """Get the DDS object after it has been registered by ``sim_main``.

    Environment reset observations run before DDS registration, so a failed
    early lookup must not be cached permanently.
    """

    instance = _dex3_dds_by_name.get(dds_object_name)
    if instance is not None:
        return instance

    try:
        from dds.dds_master import dds_manager

        # Avoid DDSManager.get_object() here because its expected pre-startup
        # miss prints a warning every time an environment is reset.
        instance = dds_manager.objects.get(dds_object_name)
        if instance is not None:
            _dex3_dds_by_name[dds_object_name] = instance
            if dds_object_name not in _dds_obtained_logged_names:
                print(f"[dex3_state] Dex3 DDS state publisher connected ({dds_object_name})")
                _dds_obtained_logged_names.add(dds_object_name)
    except Exception as exc:
        print(f"[dex3_state] Failed to obtain Dex3 DDS object: {exc}")
        instance = None
    return instance


def _resolve_hand_indices(robot, device: torch.device, _obs_cache: dict) -> torch.Tensor | None:
    joint_names = tuple(robot.data.joint_names)
    mapping_key = (str(device), joint_names)
    if _obs_cache["mapping_key"] == mapping_key:
        return _obs_cache["hand_idx_t"]

    joint_to_index = {name: index for index, name in enumerate(joint_names)}
    missing = [name for name in DEX3_HAND_JOINT_NAMES if name not in joint_to_index]
    if missing:
        _obs_cache["mapping_key"] = mapping_key
        _obs_cache["hand_idx_t"] = None
        if not _obs_cache["missing_mapping_logged"]:
            print(
                "[dex3_state] Articulation has no complete 14-DoF Dex3 mapping; "
                "hand state publishing is disabled for this task"
            )
            _obs_cache["missing_mapping_logged"] = True
        return None

    indices = torch.tensor(
        [joint_to_index[name] for name in DEX3_HAND_JOINT_NAMES],
        dtype=torch.long,
        device=device,
    )
    _obs_cache["mapping_key"] = mapping_key
    _obs_cache["hand_idx_t"] = indices
    _obs_cache["batch"] = None
    _obs_cache["dtype"] = None
    _obs_cache["hand_idx_batch"] = None
    _obs_cache["missing_mapping_logged"] = False
    return indices


def get_robot_dex3_joint_states(
    env: ManagerBasedRLEnv,
    enable_dds: bool = True,
    dds_min_interval_ms: float = 20.0,
    asset_name: str = "robot",
    dds_object_name: str = "dex3",
) -> torch.Tensor:
    """Return Dex3 joint positions and optionally publish actual hand states.

    Joint lookup is name-based and always follows the Unitree command/state
    order.  This is important for the SONIC 43-DoF carrier because Isaac's
    articulation order is not guaranteed to match URDF or DDS order.
    host 第二机器人传 asset_name="robot_2", dds_object_name="dex3_r2"。
    """

    _obs_cache = _get_obs_cache(asset_name)
    robot = env.scene[asset_name]
    joint_pos = robot.data.joint_pos
    joint_vel = robot.data.joint_vel
    joint_torque = getattr(robot.data, "applied_torque", None)
    if joint_torque is None:
        joint_torque = getattr(robot.data, "computed_torque", None)
    if joint_torque is None:
        joint_torque = torch.zeros_like(joint_pos)

    device = joint_pos.device
    batch = joint_pos.shape[0]
    idx_t = _resolve_hand_indices(robot, device, _obs_cache)
    if idx_t is None:
        return joint_pos.new_empty((batch, 0))

    joint_count = idx_t.numel()
    if (
        _obs_cache["batch"] != batch
        or _obs_cache["dtype"] != joint_pos.dtype
        or _obs_cache["hand_idx_batch"] is None
    ):
        _obs_cache["hand_idx_batch"] = idx_t.unsqueeze(0).expand(batch, joint_count)
        _obs_cache["pos_buf"] = torch.empty(
            batch, joint_count, device=device, dtype=joint_pos.dtype
        )
        _obs_cache["vel_buf"] = torch.empty_like(_obs_cache["pos_buf"])
        _obs_cache["torque_buf"] = torch.empty_like(_obs_cache["pos_buf"])
        _obs_cache["batch"] = batch
        _obs_cache["dtype"] = joint_pos.dtype

    idx_batch = _obs_cache["hand_idx_batch"]
    pos_buf = _obs_cache["pos_buf"]
    vel_buf = _obs_cache["vel_buf"]
    torque_buf = _obs_cache["torque_buf"]

    try:
        torch.gather(joint_pos, 1, idx_batch, out=pos_buf)
        torch.gather(joint_vel, 1, idx_batch, out=vel_buf)
        torch.gather(joint_torque, 1, idx_batch, out=torque_buf)
    except TypeError:
        # Older torch builds do not accept ``out`` for every backend.
        pos_buf.copy_(torch.gather(joint_pos, 1, idx_batch))
        vel_buf.copy_(torch.gather(joint_vel, 1, idx_batch))
        torque_buf.copy_(torch.gather(joint_torque, 1, idx_batch))

    if enable_dds and batch > 0:
        interval_s = max(0.0, float(dds_min_interval_ms)) / 1000.0
        now = time.monotonic()
        if now - _obs_cache["dds_last_monotonic"] >= interval_s:
            dex3_dds = _get_dex3_dds_instance(dds_object_name)
            if dex3_dds is not None:
                pos = pos_buf[0].detach().cpu().tolist()
                vel = vel_buf[0].detach().cpu().tolist()
                torque = torque_buf[0].detach().cpu().tolist()
                dex3_dds.write_hand_states(
                    pos[:7],
                    vel[:7],
                    torque[:7],
                    pos[7:],
                    vel[7:],
                    torque[7:],
                )
                _obs_cache["dds_last_monotonic"] = now

    return pos_buf
