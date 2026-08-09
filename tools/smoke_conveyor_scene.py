# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Isaac-G1-29DoF-Sonic-Conveyor 无头冒烟：流水线送筐 +（可选）双进程 ZMQ 同步。

不走 DDS/deploy，直接以默认关节位姿 step 环境，验证场景与同步链路本身。

用法（conda env_isaaclab，仓库根目录）：

  # Phase 0 单机场景冒烟（不建 socket）：筐应从入料端 y=20.95/21.55 流到工位 y≈17.698 停住
  # （坐标已含流水线整体北移 Δ=3.55；expected_stop_y 是从 env cfg 动态取的，改常量即自动跟随）
  python tools/smoke_conveyor_scene.py --steps 800

  # PhysX Surface Velocity 实验 A/B（固定由 ID=1 驱动，默认 50 mm 容差）
  python tools/smoke_conveyor_scene.py --drive-mode surface_velocity --robot-id 1 --steps 800
  # 路线验收的 30 mm 严格口径可另加：--stop-tolerance 0.03

  # Phase 2 双进程同步自测（127.0.0.1）：先起 ID=1，再起 ID=2
  python tools/smoke_conveyor_scene.py --robot-id 1 --sync 1 --steps 1200
  python tools/smoke_conveyor_scene.py --robot-id 2 --sync 1 --steps 1200
  # 判定：ID=2 侧筐（本地 kinematic 镜像、驱动关闭）y 仍应下降＝对端状态在写入
"""

import argparse
import os
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="Isaac-G1-29DoF-Sonic-Conveyor")
parser.add_argument("--steps", type=int, default=800)
parser.add_argument("--sync", default="0", choices=["0", "1"], help="ISAACLAB_SCENE_SYNC")
parser.add_argument("--robot-id", type=int, default=None, choices=[0, 1, 2], help="1/2=对等端, 0=纯镜像 viewer")
parser.add_argument("--device", default="cpu")
parser.add_argument("--report-every", type=int, default=100)
parser.add_argument(
    "--drive-mode",
    default=os.environ.get("ISAACLAB_CONVEYOR_DRIVE_MODE", "legacy"),
    choices=["legacy", "surface_velocity"],
    help="流水线 A/B 后端；surface_velocity 固定只允许 ID=1 驱动",
)
parser.add_argument(
    "--expect-stop-y",
    type=float,
    default=None,
    help="覆盖当前任务配置的停止 Y 目标",
)
parser.add_argument(
    "--stop-tolerance",
    type=float,
    default=0.05,
    help="停止目标的绝对误差上限（m，默认 0.05）",
)
parser.add_argument(
    "--skip-stop-check",
    action="store_true",
    help="跳过权威端停止精度断言（默认按任务配置自动检查）",
)
parser.add_argument(
    "--pick-lead-at",
    type=int,
    default=None,
    metavar="STEP",
    help=(
        "在第 STEP 步把队首箱子搬离带面，模拟机器人取件；"
        "随后断言后一个箱子补位到工位（流水线节拍验收）"
    ),
)
args = parser.parse_args()

# 环境变量必须在 import tasks 之前定型（env cfg 在 import 时读取）。
os.environ["ISAACLAB_SCENE_SYNC"] = args.sync
os.environ["ISAACLAB_CONVEYOR_DRIVE_MODE"] = args.drive_mode
if args.robot_id is not None:
    os.environ["ISAACLAB_LOCAL_ROBOT_ID"] = str(args.robot_id)
_REPO_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("PROJECT_ROOT", str(_REPO_ROOT))
os.environ.setdefault("GR00T_WBC_ROOT", str(Path.home() / "GR00T-WholeBodyControl"))
# 防 SteamVR active_runtime 被静默挂载拖垮 headless 帧率（历史坑）。
os.environ.setdefault("XR_RUNTIME_JSON", "/nonexistent/openxr-disabled.json")

# 本脚本在 tools/ 下，仓库根不会自动进 sys.path（sim_main.py 在根目录没这个问题）。
import sys  # noqa: E402

sys.path.insert(0, str(_REPO_ROOT))

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher(headless=True, device=args.device)
simulation_app = app_launcher.app

from time import monotonic  # noqa: E402

import gymnasium as gym  # noqa: E402
import tasks  # noqa: F401,E402  (触发 gym.register)
import torch  # noqa: E402

from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402
from tasks.g1_tasks.g1_29dof_sonic_conveyor import conveyor_env_cfg  # noqa: E402


def main() -> int:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    env.reset()

    expected_stop_y = None
    if not args.skip_stop_check:
        expected_stop_y = (
            args.expect_stop_y
            if args.expect_stop_y is not None
            else conveyor_env_cfg.CONVEYOR_Y_STOP
        )

    robot = env.scene["robot"]
    default_q = robot.data.default_joint_pos.clone()
    zeros = torch.zeros_like(default_q)
    # SONIC 动作张量 field-major：q 目标 + dq 目标 + 前馈 tau。
    action = torch.cat([default_q, zeros, zeros], dim=-1)

    # 监控清单跟着实际布局走：流水线布局是纸箱队列，推车布局仍是两塑料筐。
    watched_names = (
        conveyor_env_cfg.CONVEYOR_BELT_BOX_NAMES or conveyor_env_cfg.CONVEYOR_TOTE_NAMES
    )
    if not watched_names:
        print("[smoke] 当前布局没有可监控的流水线物体", flush=True)
        env.close()
        return 1
    watched = {name: env.scene[name] for name in watched_names}
    # 纸箱走整带节拍：队首压到工位，整条带一起停，后面的**保持出生时的相对间距**
    # （不会挤到贴紧前车）。所以停位 = y_stop + 该箱相对队首的出生偏移。
    # 塑料筐没有队列语义，两个筐各自直接停在工位（偏移取 0 即退化成这种）。
    belt_box_mode = bool(conveyor_env_cfg.CONVEYOR_BELT_BOX_NAMES)
    peer = env.scene["peer_robot"]

    # 方案 b（主循环挂载）：apply_actions 是 no-op，宿主要自己 pump。
    # 本脚本没有 sim_main 主循环，就在每个 env.step 后代跑一轮（≈200Hz pump，
    # 发布经 publish_decimation 节流，行为与 50Hz 主循环等价）。
    pump_terms = []
    for term_name in ("scene_state_sync", "env_reset_sync"):
        try:
            term = env.action_manager.get_term(term_name)
        except (AttributeError, KeyError, ValueError):
            continue
        if getattr(term.cfg, "external_pump", False):
            pump_terms.append(term)

    def snapshot(tag: str) -> None:
        parts = []
        for name, obj in watched.items():
            p = obj.data.root_pos_w[0]
            parts.append(f"{name} x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")
        pp = peer.data.root_pos_w[0]
        parts.append(f"peer_robot x={pp[0]:.3f} y={pp[1]:.3f} z={pp[2]:.3f}")
        print(f"[smoke {tag}] " + " | ".join(parts), flush=True)

    def pick_lead_box() -> str | None:
        """把队首搬离带面，模拟机器人取件。

        直接写位姿而不是真去抓：本脚本没有操作臂，而队列判据只看"还在不在带面
        窗口内"，抬走与抓走对它是同一件事。搬到 +X 侧、抬高到 1.2 m，正好落在
        z 窗口与 x 窗口之外。
        """

        lead_name, lead_obj = None, None
        for name, obj in watched.items():
            y = float(obj.data.root_pos_w[0, 1])
            if lead_obj is None or y < float(lead_obj.data.root_pos_w[0, 1]):
                lead_name, lead_obj = name, obj
        if lead_obj is None:
            return None

        pose = lead_obj.data.root_state_w[:, :7].clone()
        pose[:, 0] = -4.0
        pose[:, 2] = 1.2
        lead_obj.write_root_pose_to_sim(pose)
        lead_obj.write_root_velocity_to_sim(torch.zeros_like(lead_obj.data.root_vel_w))
        print(f"[smoke] 第 {args.pick_lead_at} 步取走队首 {lead_name}（搬到带面外）", flush=True)
        return lead_name

    snapshot("start")
    start_y = {name: float(obj.data.root_pos_w[0, 1]) for name, obj in watched.items()}
    picked_name: str | None = None

    t0 = monotonic()
    for step in range(1, args.steps + 1):
        env.step(action)
        for term in pump_terms:
            term.pump()
        if args.pick_lead_at is not None and step == args.pick_lead_at:
            picked_name = pick_lead_box()
        if step % max(1, args.report_every) == 0:
            snapshot(f"step={step}")
    elapsed = monotonic() - t0

    end_y = {name: float(obj.data.root_pos_w[0, 1]) for name, obj in watched.items()}
    end_z = {name: float(obj.data.root_pos_w[0, 2]) for name, obj in watched.items()}
    hz = args.steps / max(elapsed, 1e-6)
    print(f"[smoke] {args.steps} steps in {elapsed:.1f}s -> env_hz={hz:.1f}", flush=True)

    # 被取走的那个已经不在带面上，自然退出全部带面判定；剩下的按队列重新编号，
    # 于是"下一个补位到工位"就落在 graded_names[0] 的槽位断言里。
    graded_names = [name for name in watched_names if name != picked_name]
    moved = {name: start_y[name] - end_y[name] for name in graded_names}
    on_belt = {name: abs(end_z[name] - 0.775) < 0.05 for name in graded_names}
    print(f"[smoke] 位移(-Y) {moved} | 仍在带面 {on_belt}", flush=True)

    sync_on = args.sync == "1"
    authority = os.environ.get("ISAACLAB_LOCAL_ROBOT_ID", "1") == "1"
    surface_non_authority_offline = (
        not sync_on and args.drive_mode == "surface_velocity" and not authority
    )
    if surface_non_authority_offline:
        # Surface Velocity 固定由 ID=1 物体权威端驱动；ID=2 即使关闭同步做
        # 单机诊断也必须保持静止，防止以后恢复同步时出现双权威。
        ok = all(abs(m) < 0.02 for m in moved.values())
        print("[smoke] ID=2 Surface Velocity 权威门禁：预期本地料筐不动", flush=True)
    elif (not sync_on) or authority:
        # 权威端判定：筐被驱动明显移动且没掉下带面。
        ok = all(m > 0.3 for m in moved.values()) and all(on_belt.values())
    else:
        # 镜像端判定：本地驱动关闭，位移只能来自对端帧写入（对端没起时会是 0，PENDING）。
        ok = all(m > 0.1 for m in moved.values())
        if not ok:
            print("[smoke] 镜像端筐未动：对端 ID=1 未在跑？（单独跑镜像端时此结果为预期）", flush=True)

    if (
        expected_stop_y is not None
        and ((not sync_on) or authority)
        and not surface_non_authority_offline
    ):
        # 整带节拍：队首停在工位，其余保持出生时的相对间距一起停下。取件后整列
        # 前进一格，于是断言 graded_names[0] 落在工位上就等于验证"整带重启并再次停位"。
        lead_spawn = start_y[graded_names[0]] if belt_box_mode else 0.0
        expected_slots = {
            name: expected_stop_y + ((start_y[name] - lead_spawn) if belt_box_mode else 0.0)
            for name in graded_names
        }
        stop_errors = {
            name: abs(end_y[name] - expected_slots[name]) for name in graded_names
        }
        stop_ok = all(error <= args.stop_tolerance for error in stop_errors.values())
        slots_text = ", ".join(f"{name}={y:.3f}" for name, y in expected_slots.items())
        print(
            f"[smoke] 停止目标 [{slots_text}] | 误差 {stop_errors} "
            f"| 容差={args.stop_tolerance:.3f} | {'PASS' if stop_ok else 'FAIL'}",
            flush=True,
        )
        if picked_name is not None:
            print(
                f"[smoke] 节拍验收：取走 {picked_name} 后 {graded_names[0]} 应补位到工位 "
                f"y={expected_stop_y:.3f}，实测 y={end_y[graded_names[0]]:.3f}",
                flush=True,
            )
        ok = ok and stop_ok
    print(f"[smoke] RESULT: {'PASS' if ok else 'FAIL'}", flush=True)

    env.close()
    return 0 if ok else 1


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    # SteamVR/OpenXR 环境下 close() 有挂死史；显式退出兜底。
    os._exit(code)
