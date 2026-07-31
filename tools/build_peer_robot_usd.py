# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""手动预热镜像机器人（PeerRobot）USD——可选步骤，平时不需要跑。

自动化路径：`Isaac-G1-29DoF-Sonic-Conveyor` 被 gym.make 选中时,
`G129SonicConveyorEnvCfg.__post_init__` 会调用
`tasks.g1_tasks.g1_29dof_sonic_conveyor.peer_usd_builder.ensure_peer_robot_usd()`
按需生成/复用产物（机器本地缓存,与合成 URDF 同级的临时目录,不入库）。

本脚本只是同一函数的独立入口,用于想把首跑的 1-2 分钟转换挪到正式启动之外的场合
（如部署冒烟、CI）。产物已存在且 URDF 没变时秒过;传 `--force` 强制重转。

    conda activate env_isaaclab
    python tools/build_peer_robot_usd.py [--force]

Windows 上请勿用 GBK 控制台裸跑（中文输出会 UnicodeEncodeError）:
    set PYTHONUTF8=1 后再跑,或使用 build_peer_win.bat。
"""

import os
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("PROJECT_ROOT", str(_REPO_ROOT))
os.environ.setdefault("GR00T_WBC_ROOT", str(Path.home() / "GR00T-WholeBodyControl"))
os.environ.setdefault("XR_RUNTIME_JSON", "/nonexistent/openxr-disabled.json")
sys.path.insert(0, str(_REPO_ROOT))

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher(headless=True, device="cpu")
simulation_app = app_launcher.app


def main() -> int:
    from tasks.g1_tasks.g1_29dof_sonic_conveyor.peer_usd_builder import ensure_peer_robot_usd

    usd_path = ensure_peer_robot_usd(force="--force" in sys.argv[1:])
    print(f"[build_peer] DONE: {usd_path}", flush=True)
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    os._exit(code)
