# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""场景同步身份的单一真源（stdlib-only，可在 AppLauncher 之前加载）。

为什么要有这个模块：sim_main 与 conveyor_env_cfg 曾各自解析
``ISAACLAB_LOCAL_ROBOT_ID``——前者在 argparse 后用进程 env 的**字面比较**判 viewer，
后者在 import 期先把 ``configs/scene_sync.env`` setdefault 进 os.environ 再 ``int()``
解析。两套判定在「ID 写在 env 文件而非 shell export」或「"00"/"+0" 非规范写法」下
会得出**相反结论**：一侧按 viewer 建场景（ghost + 双镜像 + 不发布），另一侧仍按对等端
选 sonic_dds / DDS domain 1——静默复现 4Hz 锁步病，且 ghost 的 rt/lowstate 混进同机
host↔deploy 的锁步链路（2026-08-02 评审 C1/C4/C5）。

两条加载路径、同一份实现：
- ``conveyor_env_cfg`` 用包内相对 import；
- ``sim_main`` 用 ``importlib`` 按文件路径加载——不能 import tasks 包，那会在
  AppLauncher 起来之前把 isaaclab 拖进来。
两个模块实例的输入相同（env 文件 setdefault 幂等、进程 env 恒优先），结论必然一致。
"""

import os
import re
from pathlib import Path

# 支持 ${VAR} / $VAR 两种引用写法的展开（最多迭代 10 轮防环）。
_ENV_REF_RE = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}|\$([A-Za-z_][A-Za-z0-9_]*)")


def _expand_config_refs(values: dict) -> dict:
    expanded = dict(values)
    for _ in range(10):
        changed = False
        next_values = {}
        for key, value in expanded.items():
            next_value = _ENV_REF_RE.sub(
                lambda match: expanded.get(match.group(1) or match.group(2), ""),
                value,
            )
            next_values[key] = next_value
            changed |= next_value != value
        expanded = next_values
        if not changed:
            break
    return expanded


def _load_env_file(path: Path) -> dict:
    values = {}
    if not path.exists():
        return values
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            values[key] = value
    return _expand_config_refs(values)


def _project_root() -> Path:
    # sim_main.py 启动即设 PROJECT_ROOT；独立诊断脚本走 __file__ 回退
    # （本文件在 tasks/g1_tasks/<pkg>/ 下，parents[3] 即仓库根）。
    root = os.environ.get("PROJECT_ROOT", "").strip()
    if root:
        return Path(root)
    return Path(__file__).resolve().parents[3]


def load_scene_sync_env(verbose_tag: str = "[scene_sync]") -> str | None:
    """把 scene_sync.env（或 ISAACLAB_SCENE_SYNC_ENV_FILE 指定文件）setdefault 进
    os.environ。幂等：进程环境变量永远优先，重复调用无副作用。

    Returns: 实际加载的文件路径（str），没有可加载文件时返回 None。
    """
    candidates = []
    explicit = os.environ.get("ISAACLAB_SCENE_SYNC_ENV_FILE", "").strip()
    if explicit:
        candidates.append(Path(explicit).expanduser())
    candidates.append(_project_root() / "configs" / "scene_sync.env")
    for path in candidates:
        values = _load_env_file(path)
        if values:
            print(f"{verbose_tag} scene-sync config loaded: {path}")
            for key, value in values.items():
                os.environ.setdefault(key, value)
            return str(path)
    print(f"{verbose_tag} no scene-sync config file; using built-in defaults")
    return None


def resolve_local_robot_id(verbose_tag: str = "[scene_sync]", load_env: bool = True) -> int:
    """解析本机身份：1/2 = 对等端；0 = 纯镜像 viewer。非法值回退 1。

    与 conveyor_env_cfg 的历史行为逐字等价（int() 解析 + 集合校验 + 同款提示文案）。
    """
    if load_env:
        load_scene_sync_env(verbose_tag)
    raw_value = os.environ.get("ISAACLAB_LOCAL_ROBOT_ID", "").strip() or "1"
    try:
        robot_id = int(raw_value)
    except ValueError:
        print(f"{verbose_tag} Invalid ISAACLAB_LOCAL_ROBOT_ID={raw_value!r}; using robot 1.")
        return 1
    if robot_id not in {0, 1, 2}:
        print(f"{verbose_tag} Unsupported ISAACLAB_LOCAL_ROBOT_ID={raw_value!r}; using robot 1.")
        robot_id = 1
    return robot_id


def resolve_host_both_robots(verbose_tag: str = "[scene_sync]", load_env: bool = True) -> bool:
    """host 双机器人模式（工作包 B）：``ISAACLAB_HOST_BOTH_ROBOTS=1`` 且身份是 ID=1。

    host 天然继承 ID=1 的全部权威语义（物体权威、复位广播、bind 15555——正是 viewer
    固定连接的上游）。ID=0（viewer）或 ID=2（对等端）下该标志无意义，打印警告并忽略，
    保证既有模式零回归。与 :func:`resolve_local_robot_id` 一样必须作为单一真源被
    sim_main 与 conveyor_env_cfg 共用——别在任何一侧重新实现这个判定。
    """
    if load_env:
        load_scene_sync_env(verbose_tag)
    raw_value = os.environ.get("ISAACLAB_HOST_BOTH_ROBOTS", "").strip().lower()
    enabled = raw_value in {"1", "true", "yes", "on"}
    if not enabled:
        return False
    if resolve_local_robot_id(verbose_tag, load_env=False) != 1:
        print(
            f"{verbose_tag} ISAACLAB_HOST_BOTH_ROBOTS=1 只在 ISAACLAB_LOCAL_ROBOT_ID=1 "
            "下生效（host 必须是权威端）；已忽略该标志。"
        )
        return False
    return True
