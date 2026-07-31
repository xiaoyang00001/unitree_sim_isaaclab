# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Isaac-G1-29DoF-Sonic-Conveyor 无头冒烟：流水线送筐 +（可选）双进程 ZMQ 同步。

不走 DDS/deploy，直接以默认关节位姿 step 环境，验证场景与同步链路本身。

用法（conda env_isaaclab，仓库根目录）：

  # Phase 0 单机场景冒烟（不建 socket）：筐应从入料端 y=17.4/18.0 流到工位 y≈14.148 停住
  python tools/smoke_conveyor_scene.py --steps 800

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
parser.add_argument("--robot-id", type=int, default=None, choices=[1, 2])
parser.add_argument("--device", default="cpu")
parser.add_argument("--report-every", type=int, default=100)
args = parser.parse_args()

# 环境变量必须在 import tasks 之前定型（env cfg 在 import 时读取）。
os.environ["ISAACLAB_SCENE_SYNC"] = args.sync
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


def main() -> int:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    env.reset()

    robot = env.scene["robot"]
    default_q = robot.data.default_joint_pos.clone()
    zeros = torch.zeros_like(default_q)
    # SONIC 动作张量 field-major：q 目标 + dq 目标 + 前馈 tau。
    action = torch.cat([default_q, zeros, zeros], dim=-1)

    tote_names = ("cart2_tote1", "cart2_tote2")
    watched = {name: env.scene[name] for name in tote_names}
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

    snapshot("start")
    start_y = {name: float(obj.data.root_pos_w[0, 1]) for name, obj in watched.items()}

    t0 = monotonic()
    for step in range(1, args.steps + 1):
        env.step(action)
        for term in pump_terms:
            term.pump()
        if step % max(1, args.report_every) == 0:
            snapshot(f"step={step}")
    elapsed = monotonic() - t0

    end_y = {name: float(obj.data.root_pos_w[0, 1]) for name, obj in watched.items()}
    end_z = {name: float(obj.data.root_pos_w[0, 2]) for name, obj in watched.items()}
    hz = args.steps / max(elapsed, 1e-6)
    print(f"[smoke] {args.steps} steps in {elapsed:.1f}s -> env_hz={hz:.1f}", flush=True)

    moved = {name: start_y[name] - end_y[name] for name in watched}
    on_belt = {name: abs(end_z[name] - 0.775) < 0.05 for name in watched}
    print(f"[smoke] 位移(-Y) {moved} | 仍在带面 {on_belt}", flush=True)

    sync_on = args.sync == "1"
    authority = os.environ.get("ISAACLAB_LOCAL_ROBOT_ID", "1") == "1"
    if (not sync_on) or authority:
        # 权威端判定：筐被驱动明显移动且没掉下带面。
        ok = all(m > 0.3 for m in moved.values()) and all(on_belt.values())
    else:
        # 镜像端判定：本地驱动关闭，位移只能来自对端帧写入（对端没起时会是 0，PENDING）。
        ok = all(m > 0.1 for m in moved.values())
        if not ok:
            print("[smoke] 镜像端筐未动：对端 ID=1 未在跑？（单独跑镜像端时此结果为预期）", flush=True)
    print(f"[smoke] RESULT: {'PASS' if ok else 'FAIL'}", flush=True)

    env.close()
    return 0 if ok else 1


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    # SteamVR/OpenXR 环境下 close() 有挂死史；显式退出兜底。
    os._exit(code)
