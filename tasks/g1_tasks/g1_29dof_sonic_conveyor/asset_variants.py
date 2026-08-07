"""流水线任务 USD 资产变体选择。

该模块刻意不依赖 Isaac Sim，便于在普通 Python 单元测试中验证环境变量解析。
visual-only 变体只替换 ``/Root/ConveyorBelt`` 的 payload；独立分支默认使用原始
v61，合并背景清理层后默认沿用该清理层，因此未显式开启时不会改变所在分支的基线。
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
) -> Path:
    """返回当前环境选择的背景层；visual-only 必须显式 opt-in。"""

    source = os.environ if environ is None else environ
    # 独立分支没有第一优先级产出的 clean wrapper，仍以 legacy v61 为基线；
    # 合并该 wrapper 后自动沿用它，避免切换输送机资产时重新引入根层装饰物物理。
    baseline = (
        CLEAN_BACKGROUND_USD
        if (assets_dir / CLEAN_BACKGROUND_USD).is_file()
        else LEGACY_BACKGROUND_USD
    )
    if not _env_bool(source, CONVEYOR_VISUAL_ONLY_ENV, False):
        return assets_dir / baseline

    adapter = assets_dir / VISUAL_ONLY_BACKGROUND_USD
    if not adapter.is_file():
        raise FileNotFoundError(
            f"已启用 {CONVEYOR_VISUAL_ONLY_ENV}，但缺少资产层：{adapter}"
        )
    if baseline == CLEAN_BACKGROUND_USD:
        expected_sublayer = f"subLayers = [@./{CLEAN_BACKGROUND_USD}@]"
        if expected_sublayer not in adapter.read_text(encoding="utf-8"):
            raise RuntimeError(
                "conveyor visual-only adapter 仍指向 legacy v61，会绕过背景物理清理层；"
                "请重新运行 python tools/build_conveyor_visual_only_usd.py"
            )
    return adapter
