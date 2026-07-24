#!/usr/bin/env python3
"""Print the dynamics and frame data needed to audit a SONIC robot task."""

from __future__ import annotations

import argparse
import json
import os
import traceback

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="Isaac-G1-29DoF-Sonic")
parser.add_argument(
    "--summary-only",
    action="store_true",
    help="omit the per-body inertia table while retaining totals and frame checks",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PROJECT_ROOT", project_root)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import tasks  # noqa: F401, E402
from isaaclab.utils.math import quat_apply_inverse, quat_inv, quat_mul  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402


def _tensor_list(value: torch.Tensor, digits: int = 8) -> list[float]:
    return [round(float(item), digits) for item in value.detach().cpu().flatten()]


def _relative_pose(
    parent_pose_w: torch.Tensor,
    child_pose_w: torch.Tensor,
) -> tuple[list[float], list[float]]:
    parent_pos_w = parent_pose_w[:3]
    parent_quat_w = parent_pose_w[3:7]
    child_pos_w = child_pose_w[:3]
    child_quat_w = child_pose_w[3:7]
    pos_parent = quat_apply_inverse(parent_quat_w.unsqueeze(0), (child_pos_w - parent_pos_w).unsqueeze(0))[0]
    quat_parent = quat_mul(quat_inv(parent_quat_w.unsqueeze(0)), child_quat_w.unsqueeze(0))[0]
    if quat_parent[0] < 0:
        quat_parent = -quat_parent
    return _tensor_list(pos_parent), _tensor_list(quat_parent)


def main() -> None:
    env = None
    try:
        cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
        env = gym.make(args_cli.task, cfg=cfg).unwrapped
        env.reset()

        robot = env.scene["robot"]
        data = robot.data
        body_names = list(data.body_names)
        joint_names = list(data.joint_names)
        masses = robot.root_physx_view.get_masses()[0]
        inertias = robot.root_physx_view.get_inertias()[0]
        poses = data.body_link_pose_w[0]

        report: dict[str, object] = {
            "task": args_cli.task,
            "physics_dt": float(cfg.sim.dt),
            "decimation": int(cfg.decimation),
            "control_hz": 1.0 / (float(cfg.sim.dt) * int(cfg.decimation)),
            "num_joints": len(joint_names),
            "num_bodies": len(body_names),
            "total_mass_kg": round(float(masses.sum()), 8),
            "joint_names": joint_names,
            "body_names": body_names,
            "relative_frames": {},
        }

        body_rows = []
        for index, name in enumerate(body_names):
            body_rows.append(
                {
                    "index": index,
                    "name": name,
                    "mass_kg": round(float(masses[index]), 8),
                    "inertia": _tensor_list(inertias[index]),
                    "position_w": _tensor_list(poses[index, :3]),
                    "quaternion_wxyz": _tensor_list(poses[index, 3:7]),
                }
            )
        if not args_cli.summary_only:
            report["bodies"] = body_rows

        for parent_name, child_name in (
            ("pelvis", "imu_in_pelvis"),
            ("pelvis", "torso_link"),
            ("torso_link", "imu_in_torso"),
        ):
            if parent_name in body_names and child_name in body_names:
                parent_index = body_names.index(parent_name)
                child_index = body_names.index(child_name)
                rel_pos, rel_quat = _relative_pose(poses[parent_index], poses[child_index])
                report["relative_frames"][f"{parent_name}->{child_name}"] = {
                    "position": rel_pos,
                    "quaternion_wxyz": rel_quat,
                }

        print("SONIC_MODEL_DIAGNOSTIC_BEGIN")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("SONIC_MODEL_DIAGNOSTIC_END")
    except BaseException:
        print("SONIC_MODEL_DIAGNOSTIC_ERROR")
        traceback.print_exc()
        raise
    finally:
        if env is not None:
            env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
