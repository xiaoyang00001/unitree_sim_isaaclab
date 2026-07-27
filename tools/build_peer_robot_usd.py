# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""一次性生成镜像机器人（PeerRobot）专用 USD：URDF 转换 + 去掉全部碰撞体。

Isaac-G1-29DoF-Sonic-Conveyor 的对端镜像机器人必须无碰撞（纯跟随体，不与
权威物理打架）。但 IsaacLab 的 UrdfConverter 固定输出 instanceable 格式
（urdf_converter.py 注释明言，cfg 的 make_instanceable 是死字段），碰撞体
藏在 instance 原型里，spawn 时的 collision_props(collision_enabled=False)
根本改不到——实测表现为镜像体开局被地面弹出、无重力下以 ~0.58 m/s 恒速
上飘。因此仿照源分支"物理改动烘进资产文件"的做法，离线转换一份专用 USD
并把每个 link 的 collisions scope 整体 SetActive(False)。

用法（每台机器跑一次，GR00T_WBC_ROOT 指向 GR00T-WholeBodyControl 检出）：

    conda activate env_isaaclab
    python tools/build_peer_robot_usd.py

产物: tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot/g1_43dof_peer.usd
（derived 资产不入 git；conveyor_env_cfg 检测到产物缺失会回退 URDF 直转并告警。）
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
    from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import make_sonic_robot_cfg

    from pxr import Usd, UsdPhysics

    from isaaclab.sim.converters import UrdfConverter

    out_dir = _REPO_ROOT / "tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot"
    out_dir.mkdir(parents=True, exist_ok=True)

    # UrdfFileCfg 本身就是 UrdfConverterCfg（多继承），直接喂给转换器，
    # 保证与本机 robot 同一份 URDF、同一套关节/惯量参数。
    conv_cfg = make_sonic_robot_cfg().spawn
    conv_cfg.usd_dir = str(out_dir)
    conv_cfg.usd_file_name = "g1_43dof_peer.usd"
    conv_cfg.force_usd_conversion = True

    print(f"[build_peer] URDF: {conv_cfg.asset_path}", flush=True)
    converter = UrdfConverter(conv_cfg)
    usd_path = converter.usd_path
    print(f"[build_peer] 转换产物: {usd_path}", flush=True)

    stage = Usd.Stage.Open(usd_path)
    deactivated = 0
    attr_disabled = 0
    # 转换产物里 link/collisions 是真实 prim（其下 mesh 才是 instance 引用），
    # 整个 scope 去激活即可让 PhysX 完全不解析这些碰撞体。
    for prim in stage.Traverse():
        if prim.GetName() == "collisions":
            prim.SetActive(False)
            deactivated += 1
    # 兜底：非 instance 的散装 collider（若有）逐个关属性。
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            api = UsdPhysics.CollisionAPI(prim)
            attr = api.GetCollisionEnabledAttr()
            if not attr:
                attr = api.CreateCollisionEnabledAttr(True)
            attr.Set(False)
            attr_disabled += 1
    stage.GetRootLayer().Save()
    print(
        f"[build_peer] 去激活 collisions scope ×{deactivated}，散装 collider 关闭 ×{attr_disabled}",
        flush=True,
    )
    if deactivated == 0 and attr_disabled == 0:
        print("[build_peer] ⚠️ 没找到任何碰撞体——转换产物结构可能变了，请人工检查", flush=True)
        return 1
    print(f"[build_peer] DONE: {usd_path}", flush=True)
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    os._exit(code)
