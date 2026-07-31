# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""运行时按需生成对端镜像机器人（PeerRobot）的无碰撞 USD——机器本地缓存，不入库。

与 Isaac-G1-29DoF-Sonic 主机器人完全同源：同一份运行时合成的 SONIC URDF、同一个
UrdfConverter，只是把每个 link 的 ``collisions`` scope 整体 SetActive(False)。
镜像体的关节/root 由 scene_state 帧直写，带碰撞体会与权威端物理打架；而
UrdfConverter 固定输出 instanceable 格式，碰撞体藏在 instance 原型里，spawn 时的
``collision_props(collision_enabled=False)`` 根本改不到——实测表现为镜像体开局被
地面弹出、无重力下以 ~0.58 m/s 恒速上飘。所以碰撞去激活必须烘在文件层。

缓存目录与合成 URDF 同级（``/tmp`` 或 ``%TEMP%`` 下的
``unitree_sim_isaaclab_sonic_<user>/peer_usd/``）：

* URDF 没变时 UrdfConverter 的 asset-hash 检查直接复用旧产物，秒过；
* URDF 变了自动重转，随后幂等地补跑一遍碰撞去激活；
* 临时目录被清（如重启）就重新生成，无需任何手动步骤。

必须在 kit app 已启动后调用（``gym.make`` 阶段满足）；独立预热入口是
``tools/build_peer_robot_usd.py``。
"""

from __future__ import annotations

from pathlib import Path

from robots.g1_sonic_urdf import default_sonic_g1_43dof_output_path

PEER_USD_FILE_NAME = "g1_43dof_peer.usd"


def peer_usd_output_dir() -> Path:
    return default_sonic_g1_43dof_output_path().parent / "peer_usd"


def peer_usd_path() -> Path:
    """镜像机器人 USD 的机器本地缓存路径。import 期可安全调用（纯路径计算）。"""

    return peer_usd_output_dir() / PEER_USD_FILE_NAME


def _deactivate_collisions(usd_path: str) -> tuple[int, int]:
    """幂等地去激活全部碰撞体。返回 (collisions scope 数, 散装 collider 数)。

    注意必须用 AllPrims 谓词遍历：默认的 stage.Traverse() 会跳过 inactive 子树，
    第二次进来时已烘好的 collisions scope 会被漏数，误判成"产物里没有碰撞体"。
    """

    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(usd_path)
    deactivated = 0
    attr_disabled = 0
    changed = False
    for prim in Usd.PrimRange.Stage(stage, Usd.PrimAllPrimsPredicate):
        if prim.GetName() == "collisions":
            deactivated += 1
            if prim.IsActive():
                prim.SetActive(False)
                changed = True
    for prim in Usd.PrimRange.Stage(stage, Usd.PrimAllPrimsPredicate):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            api = UsdPhysics.CollisionAPI(prim)
            attr = api.GetCollisionEnabledAttr()
            if not attr:
                attr = api.CreateCollisionEnabledAttr(True)
            if attr.Get() is not False:
                changed = True
                attr.Set(False)
            attr_disabled += 1
    if changed:
        stage.GetRootLayer().Save()
    return deactivated, attr_disabled


def ensure_peer_robot_usd(*, force: bool = False) -> Path:
    """确保镜像机器人 USD 存在且碰撞已去激活，返回产物路径。

    UrdfFileCfg 本身就是 UrdfConverterCfg（多继承），直接喂给转换器，保证与本机
    robot 同一份 URDF、同一套关节/惯量参数。
    """

    from isaaclab.sim.converters import UrdfConverter

    from tasks.g1_tasks.g1_29dof_dex3_sonic.g1_29dof_dex3_sonic_env_cfg import make_sonic_robot_cfg

    out_dir = peer_usd_output_dir()
    out_dir.mkdir(parents=True, exist_ok=True)

    conv_cfg = make_sonic_robot_cfg().spawn
    conv_cfg.usd_dir = str(out_dir)
    conv_cfg.usd_file_name = PEER_USD_FILE_NAME
    conv_cfg.force_usd_conversion = force

    converter = UrdfConverter(conv_cfg)
    usd_path = Path(converter.usd_path)

    deactivated, attr_disabled = _deactivate_collisions(str(usd_path))
    if deactivated == 0 and attr_disabled == 0:
        raise RuntimeError(
            f"镜像机器人 USD 里没找到任何碰撞体——转换产物结构可能变了，请人工检查: {usd_path}"
        )
    print(
        f"[peer_usd] 镜像机器人 USD 就绪(collisions scope×{deactivated} 已去激活): {usd_path}",
        flush=True,
    )
    return usd_path
