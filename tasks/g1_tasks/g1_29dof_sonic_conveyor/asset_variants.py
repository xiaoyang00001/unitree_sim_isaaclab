"""流水线任务 USD 资产变体选择。

该模块刻意不依赖 Isaac Sim，便于在普通 Python 单元测试中验证环境变量解析。
visual-only 变体只替换 ``/Root/ConveyorBelt`` 的 payload。legacy 驱动默认
沿用背景选择器的基线，Surface Velocity 则强制使用叠加 clean background
的 visual-only adapter，避免原生输送机碰撞与任务 proxy 重叠。
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path


CONVEYOR_VISUAL_ONLY_ENV = "ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET"
LEGACY_BACKGROUND_USD = "warehouse-simple6_v61.usd"
CLEAN_BACKGROUND_USD = "warehouse-simple6_v61_visual_only.usda"
VISUAL_ONLY_BACKGROUND_USD = "warehouse-simple6_v61_conveyor_visual_only.usda"


def _env_bool(environ: Mapping[str, str], name: str, default: bool) -> bool:
    value = environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def resolve_conveyor_background_usd(
    assets_dir: Path,
    environ: Mapping[str, str] | None = None,
    *,
    baseline_path: Path | None = None,
    drive_mode: str | None = None,
) -> Path:
    """返回当前环境选择的背景层。

    ``baseline_path`` 由背景清理层选择器传入，确保
    ``ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61`` 的显式回退不会被本模块悄悄
    改回 clean wrapper。``drive_mode=surface_velocity`` 会强制使用纯视觉
    输送机；legacy 仍由环境变量显式 opt-in。
    """

    source = os.environ if environ is None else environ
    # 保留 standalone 兼容：没有 clean wrapper 时以 legacy v61 为基线。
    if baseline_path is None:
        baseline_path = assets_dir / (
            CLEAN_BACKGROUND_USD
            if (assets_dir / CLEAN_BACKGROUND_USD).is_file()
            else LEGACY_BACKGROUND_USD
        )
    if drive_mode not in {None, "legacy", "surface_velocity"}:
        raise ValueError(f"未知流水线驱动模式: {drive_mode!r}")
    use_visual_only = drive_mode == "surface_velocity" or _env_bool(
        source, CONVEYOR_VISUAL_ONLY_ENV, False
    )
    if not use_visual_only:
        return baseline_path

    adapter = assets_dir / VISUAL_ONLY_BACKGROUND_USD
    if not adapter.is_file():
        raise FileNotFoundError(
            f"已启用 {CONVEYOR_VISUAL_ONLY_ENV}，但缺少资产层：{adapter}"
        )
    if (assets_dir / CLEAN_BACKGROUND_USD).is_file():
        expected_sublayer = f"subLayers = [@./{CLEAN_BACKGROUND_USD}@]"
        if expected_sublayer not in adapter.read_text(encoding="utf-8"):
            raise RuntimeError(
                "conveyor visual-only adapter 仍指向 legacy v61，会绕过背景物理清理层；"
                "请重新运行 python tools/build_conveyor_visual_only_usd.py"
            )
    return adapter
