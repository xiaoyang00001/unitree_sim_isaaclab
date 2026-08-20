# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Isaac-G1-29DoF-Sonic-Conveyor 无头冒烟：流水线送箱 +（可选）双进程 ZMQ 同步。

不走 DDS/deploy，直接以默认关节位姿 step 环境，验证场景与同步链路本身。

默认布局是"看不到头的入料端"（endless intake，**西拐**，Δ=0.25 整体北移）：
17 个箱/包以 pitch 0.75 沿入口弯道路径出生（队首在主线 y≈15.53，主线 5 +
弧上 3 + X 支线 9，队尾 s=-0.94 (-13.70, 20.05)），经支线 +X → 圆角弧 →
主线 -Y 流到工位 y≈14.398 停住；队首行程≈1.13 m（实测带速 ~0.244 m/s ⇒
约 230 步到位），默认 1600 步为整列稳定和可选离线后持续停线验收留出时间。
位移/停位判定都按沿路径距离 s。
ISAACLAB_CONVEYOR_ENDLESS=off 回退直线带头（旧行为，y 判定）。
（expected_stop_y 从 env cfg 动态取，改常量自动跟随）

用法（conda env_isaaclab，仓库根目录）：

  # Phase 0 单机场景冒烟（不建 socket）
  python tools/smoke_conveyor_scene.py --steps 1600

  # 单批次停线验收：第 1000 步仅抬高队首，保持 50 步，再横向移出并断言下一个不补位
  python tools/smoke_conveyor_scene.py --steps 1600 --pick-lead-at 1000 --depart-lead-after 50

  # PhysX Surface Velocity 实验 A/B（固定由 ID=1 驱动，默认 50 mm 容差）
  # ⚠️ surface_velocity 只驱动主线碰撞面，弯道形态未验证（README 已知限制）
  python tools/smoke_conveyor_scene.py --drive-mode surface_velocity --robot-id 1 --steps 800
  # 路线验收的 30 mm 严格口径可另加：--stop-tolerance 0.03

  # Phase 2 双进程同步自测（127.0.0.1）：先起 ID=1，再起 ID=2
  python tools/smoke_conveyor_scene.py --robot-id 1 --sync 1 --steps 1600
  python tools/smoke_conveyor_scene.py --robot-id 2 --sync 1 --steps 1600
  # 判定：ID=2 侧箱（本地 kinematic 镜像、驱动关闭）沿路径仍应前进＝对端状态在写入
"""

import argparse
import os
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="Isaac-G1-29DoF-Sonic-Conveyor")
parser.add_argument("--steps", type=int, default=1600)
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
        "在第 STEP 步把队首箱子原地抬高，模拟机器人取件；"
        "保持停线后再横向移出流水线通道，并断言后一个箱子仍停在原停位、不补位"
    ),
)
parser.add_argument(
    "--depart-lead-after",
    "--place-lead-after",
    dest="depart_lead_after",
    type=int,
    default=50,
    metavar="STEPS",
    help=(
        "配合 --pick-lead-at：原 XY 抬高后保持 STEPS 步，再把箱根横向移出流水线"
        "（默认 50；旧参数名 --place-lead-after 仍兼容）"
    ),
)
args = parser.parse_args()
if args.depart_lead_after < 0:
    parser.error("--depart-lead-after 不能为负数")
if args.pick_lead_at is not None and args.drive_mode != "legacy":
    parser.error("--pick-lead-at 的平面偏离门验收仅支持 legacy 后端")
if (
    args.pick_lead_at is not None
    and args.pick_lead_at + args.depart_lead_after > args.steps
):
    parser.error("--steps 必须覆盖 pick-lead-at + depart-lead-after，才能完成离线后持续停线验收")

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
from tasks.g1_tasks.g1_29dof_sonic_conveyor import endless_intake  # noqa: E402

# 弯道形态（endless intake）下位移/停位都换算成沿路径距离 s 来判——
# 出生在支线/弧段上的箱子 y 几乎不变，用 y 位移判会误报 FAIL。
ENDLESS = conveyor_env_cfg.ENDLESS_INTAKE.enabled


def _progress(x: float, y: float) -> float:
    """弯道形态给沿路径距离，直线形态给 -y（两者都是"越大越靠下游"）。"""

    return endless_intake.path_s_of_point(x, y) if ENDLESS else -y


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
    half_by_name = dict(
        zip(
            conveyor_env_cfg.CONVEYOR_BELT_BOX_NAMES,
            conveyor_env_cfg.CONVEYOR_BELT_BOX_HALF_LENGTHS,
        )
    )
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

    def pick_lead_box() -> tuple[str, object] | None:
        """把队首原地抬高，模拟机器人刚取起但尚未偏离流水线。

        直接写位姿而不是真去抓：本脚本没有操作臂。这里只改 Z 到 1.2 m，保持原 XY，
        因而箱子已经脱离带面驱动，却仍占用流水线平面通道并按住后续队列。队首=沿路径
        最靠下游（弯道形态按 s 判）。
        """

        lead_name, lead_obj, lead_p = None, None, None
        for name, obj in watched.items():
            p = _progress(float(obj.data.root_pos_w[0, 0]), float(obj.data.root_pos_w[0, 1]))
            if lead_p is None or p > lead_p:
                lead_name, lead_obj, lead_p = name, obj, p
        if lead_obj is None:
            return None

        pose = lead_obj.data.root_state_w[:, :7].clone()
        pose[:, 2] = 1.2
        lead_obj.write_root_pose_to_sim(pose)
        lead_obj.write_root_velocity_to_sim(torch.zeros_like(lead_obj.data.root_vel_w))
        print(
            f"[smoke] 第 {args.pick_lead_at} 步抬高队首 {lead_name}"
            "（XY 不变，仍在流水线上方）",
            flush=True,
        )
        return lead_name, lead_obj

    def move_lead_off_conveyor(name: str, obj) -> None:
        """把已抬高的队首横向移出主线/L 形通道，触发立即放行。"""

        pose = obj.data.root_state_w[:, :7].clone()
        pose[:, 0] = -4.0
        obj.write_root_pose_to_sim(pose)
        obj.write_root_velocity_to_sim(torch.zeros_like(obj.data.root_vel_w))
        print(
            f"[smoke] 第 {args.pick_lead_at + args.depart_lead_after} 步把 {name} "
            "横向移出流水线通道，触发偏离完成锁存",
            flush=True,
        )

    snapshot("start")
    start_xy = {
        name: (float(obj.data.root_pos_w[0, 0]), float(obj.data.root_pos_w[0, 1]))
        for name, obj in watched.items()
    }
    start_p = {name: _progress(*xy) for name, xy in start_xy.items()}
    picked_name: str | None = None
    picked_obj = None
    hold_lead_name: str | None = None
    hold_start_p: float | None = None
    hold_displacement: float | None = None
    post_depart_hold_p: float | None = None

    t0 = monotonic()
    for step in range(1, args.steps + 1):
        env.step(action)
        for term in pump_terms:
            term.pump()
        if args.pick_lead_at is not None and step == args.pick_lead_at:
            picked = pick_lead_box()
            if picked is not None:
                picked_name, picked_obj = picked
                remaining = [name for name in watched_names if name != picked_name]
                if remaining:
                    hold_lead_name = max(
                        remaining,
                        key=lambda name: _progress(
                            float(watched[name].data.root_pos_w[0, 0]),
                            float(watched[name].data.root_pos_w[0, 1]),
                        ),
                    )
                    hold_start_p = _progress(
                        float(watched[hold_lead_name].data.root_pos_w[0, 0]),
                        float(watched[hold_lead_name].data.root_pos_w[0, 1]),
                    )
        if (
            picked_name is not None
            and picked_obj is not None
            and args.pick_lead_at is not None
            and step == args.pick_lead_at + args.depart_lead_after
        ):
            if hold_lead_name is not None and hold_start_p is not None:
                hold_end_p = _progress(
                    float(watched[hold_lead_name].data.root_pos_w[0, 0]),
                    float(watched[hold_lead_name].data.root_pos_w[0, 1]),
                )
                hold_displacement = hold_end_p - hold_start_p
                post_depart_hold_p = hold_end_p
            move_lead_off_conveyor(picked_name, picked_obj)
        if step % max(1, args.report_every) == 0:
            snapshot(f"step={step}")
    elapsed = monotonic() - t0

    end_xy = {
        name: (float(obj.data.root_pos_w[0, 0]), float(obj.data.root_pos_w[0, 1]))
        for name, obj in watched.items()
    }
    end_p = {name: _progress(*xy) for name, xy in end_xy.items()}
    end_z = {name: float(obj.data.root_pos_w[0, 2]) for name, obj in watched.items()}
    hz = args.steps / max(elapsed, 1e-6)
    print(f"[smoke] {args.steps} steps in {elapsed:.1f}s -> env_hz={hz:.1f}", flush=True)

    # 被取走的箱子已经横向偏出流水线并完成锁存，退出带面评分；剩下的按队列重新
    # 编号，于是“偏离后下一个补位到工位”落在 graded_names[0] 的槽位断言里。
    graded_names = [name for name in watched_names if name != picked_name]
    moved = {name: round(end_p[name] - start_p[name], 4) for name in graded_names}
    on_belt = {name: abs(end_z[name] - 0.775) < 0.05 for name in graded_names}
    _metric = "沿路径 s" if ENDLESS else "-Y"
    print(f"[smoke] 位移({_metric}) {moved} | 仍在带面 {on_belt}", flush=True)

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

    if hold_displacement is not None:
        hold_ok = abs(hold_displacement) < 0.05
        print(
            f"[smoke] 搬运等待期：{hold_lead_name} 位移 {hold_displacement:.4f} m "
            f"（上限 0.05）| {'PASS' if hold_ok else 'FAIL'}",
            flush=True,
        )
        ok = ok and hold_ok

    if post_depart_hold_p is not None and hold_lead_name is not None:
        post_depart_displacement = end_p[hold_lead_name] - post_depart_hold_p
        post_depart_ok = abs(post_depart_displacement) < 0.05
        print(
            f"[smoke] 首批离线后持续停线：{hold_lead_name} 位移 "
            f"{post_depart_displacement:.4f} m（上限 0.05）| "
            f"{'PASS' if post_depart_ok else 'FAIL'}",
            flush=True,
        )
        ok = ok and post_depart_ok

    if (
        expected_stop_y is not None
        and ((not sync_on) or authority)
        and not surface_non_authority_offline
    ):
        # 整带节拍：首批到位后终止停线。取件后使用原队首的出生进度作为槽位基准，
        # 断言余下箱子仍停在原槽位，而不是把 graded_names[0] 当成新队首补到主工位。
        stop_p = (
            endless_intake.path_s_of_main_y(expected_stop_y) if ENDLESS else -expected_stop_y
        )
        picked_stop_text: str | None = None
        if ENDLESS and belt_box_mode:
            # 弯道形态：弧段拖滑速度（~0.218 m/s）低于直线段（~0.244），前车先
            # 出弧提速时对后车的间距最多被拉伸 ≈ 弧长 2.2 × (0.244/0.218−1)
            # ≈ 0.26 m（Δ=0.25 布局实测 +0.26）；反过来，深藏队尾（出生间距
            # 9.71）在前车过弧期间自己还在直线段，间距会收缩 ~0.3 m。"槽位 =
            # 出生间距整体平移"对拖滑物理不成立，判据改为：
            #   ① 队首停在工位（这是节拍的硬语义）；
            #   ② 其余仍按下游序排列，且每对相邻间距落在
            #      [max(该对真实半长和+queue_gap, 出生间距−0.45),
            #       出生间距+0.35] 内（C 型大箱把最小安全间距抬到 0.57 m；±界给
            #      弧段拉伸/收缩留余量）。
            lead = graded_names[0]
            lead_spawn_p = start_p[picked_name] if picked_name is not None else start_p[lead]
            lead_target_p = stop_p - (lead_spawn_p - start_p[lead])
            lead_error = abs(end_p[lead] - lead_target_p)
            spacing = [
                round(end_p[graded_names[i]] - end_p[graded_names[i + 1]], 4)
                for i in range(len(graded_names) - 1)
            ]
            spawn_gaps = [
                start_p[graded_names[i]] - start_p[graded_names[i + 1]]
                for i in range(len(graded_names) - 1)
            ]
            bounds = [
                (
                    max(
                        half_by_name.get(graded_names[index], 0.19)
                        + half_by_name.get(graded_names[index + 1], 0.19)
                        + conveyor_env_cfg.BELT_BOX_QUEUE_GAP,
                        gap - 0.45,
                    ),
                    gap + 0.35,
                )
                for index, gap in enumerate(spawn_gaps)
            ]
            spacing_ok = all(
                lo <= gap <= hi for gap, (lo, hi) in zip(spacing, bounds)
            )
            stop_ok = lead_error <= args.stop_tolerance and spacing_ok
            bounds_text = ", ".join(f"[{lo:.2f},{hi:.2f}]" for lo, hi in bounds)
            print(
                f"[smoke] 停止目标 {lead} s={lead_target_p:.3f}"
                f"（主工位 y={expected_stop_y:.3f}）| "
                f"队首误差 {lead_error:.4f}（容差 {args.stop_tolerance:.3f}）| "
                f"停稳间距 {spacing}（按各对出生间距判界 {bounds_text}；"
                f"弧段拖滑拉伸/深藏队尾收缩属预期）| {'PASS' if stop_ok else 'FAIL'}",
                flush=True,
            )
            if picked_name is not None:
                picked_stop_text = (
                    f"[smoke] 节拍验收：{picked_name} XY 偏出流水线后 {lead} 不补位，"
                    f"保持 s={lead_target_p:.3f}，实测 s={end_p[lead]:.3f}"
                )
        else:
            lead_spawn_p = (
                start_p[picked_name]
                if belt_box_mode and picked_name is not None
                else start_p[graded_names[0]]
                if belt_box_mode
                else stop_p
            )
            expected_slots = {
                name: stop_p - (lead_spawn_p - start_p[name]) if belt_box_mode else stop_p
                for name in graded_names
            }
            stop_errors = {
                name: round(abs(end_xy[name][1] - (-expected_slots[name])), 4)
                for name in graded_names
            }
            slots_text = ", ".join(
                f"{name}={-p:.3f}" for name, p in expected_slots.items()
            )
            stop_ok = all(error <= args.stop_tolerance for error in stop_errors.values())
            print(
                f"[smoke] 停止目标 [{slots_text}] | 误差 {stop_errors} "
                f"| 容差={args.stop_tolerance:.3f} | {'PASS' if stop_ok else 'FAIL'}",
                flush=True,
            )
            if picked_name is not None:
                lead = graded_names[0]
                picked_stop_text = (
                    f"[smoke] 节拍验收：{picked_name} XY 偏出流水线后 {lead} 不补位，"
                    f"保持 y={-expected_slots[lead]:.3f}，实测 y={end_xy[lead][1]:.3f}"
                )
        if picked_stop_text is not None:
            print(picked_stop_text, flush=True)
        ok = ok and stop_ok
    print(f"[smoke] RESULT: {'PASS' if ok else 'FAIL'}", flush=True)

    env.close()
    return 0 if ok else 1


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    # SteamVR/OpenXR 环境下 close() 有挂死史；显式退出兜底。
    os._exit(code)
