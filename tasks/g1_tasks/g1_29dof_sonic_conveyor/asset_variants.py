"""流水线任务 USD 资产变体选择。

该模块刻意不依赖 Isaac Sim，便于在普通 Python 单元测试中验证环境变量解析。
visual-only 变体只替换 ``/Root/ConveyorBelt`` 的 payload。legacy 驱动默认
沿用背景选择器的基线，Surface Velocity 则强制使用纯视觉输送机。

adapter 按 baseline 路由：完整仓库背景（clean/legacy v61）使用固定叠加 clean
background 的仓库 adapter；workcell-lite 背景使用专用组合层，其 baseline 是
轻量工位而不是完整 clean-v61，避免启用纯视觉输送机后丢失轻量工位效果。
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path


CONVEYOR_VISUAL_ONLY_ENV = "ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET"
LEGACY_BACKGROUND_USD = "warehouse-simple6_v61.usd"
CLEAN_BACKGROUND_USD = "warehouse-simple6_v61_visual_only.usda"
WORKCELL_LITE_BACKGROUND_USD = "conveyor_workcell_lite.usd"
VISUAL_ONLY_BACKGROUND_USD = "warehouse-simple6_v61_conveyor_visual_only.usda"
WORKCELL_LITE_VISUAL_ONLY_BACKGROUND_USD = (
    "conveyor_workcell_lite_conveyor_visual_only.usda"
)
_GENERATOR_HINT = "请重新运行 python tools/build_conveyor_visual_only_usd.py"

# workcell-lite 的 ConveyorBelt 经 reference 取自 clean wrapper，payload 定义在
# 引用目标的 layer stack 内部，上层 delete payload 跨 reference arc 无效。专用
# adapter 因此改为把 reference 重定向到完整仓库 adapter 的同名子树：payload
# 替换在那个 layer stack 内完成，v61 对该 Prim 的 xform 与子 Prim 覆盖全部保留。
WORKCELL_LITE_ADAPTER_DELETE_REFERENCE = (
    f"delete references = @./{CLEAN_BACKGROUND_USD}@</Root/ConveyorBelt>"
)
WORKCELL_LITE_ADAPTER_PREPEND_REFERENCE = (
    f"prepend references = @./{VISUAL_ONLY_BACKGROUND_USD}@</Root/ConveyorBelt>"
)


def _env_bool(environ: Mapping[str, str], name: str, default: bool) -> bool:
    value = environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _require_warehouse_adapter(assets_dir: Path) -> Path:
    """校验并返回完整仓库背景的 conveyor visual-only adapter。"""

    adapter = assets_dir / VISUAL_ONLY_BACKGROUND_USD
    if not adapter.is_file():
        # 纯视觉输送机有两条触发路径：环境变量显式 opt-in，或 surface_velocity
        # 驱动强制——后者下用户可能从未设置过该环境变量，文案不能只指前者。
        raise FileNotFoundError(
            f"已启用纯视觉输送机（{CONVEYOR_VISUAL_ONLY_ENV}=1 或 surface_velocity "
            f"驱动强制），但缺少资产层：{adapter}；" + _GENERATOR_HINT
        )
    if (assets_dir / CLEAN_BACKGROUND_USD).is_file():
        expected_sublayer = f"subLayers = [@./{CLEAN_BACKGROUND_USD}@]"
        if expected_sublayer not in adapter.read_text(encoding="utf-8"):
            raise RuntimeError(
                "conveyor visual-only adapter 仍指向 legacy v61，会绕过背景物理清理层；"
                + _GENERATOR_HINT
            )
    return adapter


def _require_workcell_lite_adapter(assets_dir: Path) -> Path:
    """校验并返回 workcell-lite 背景的专用组合层。

    组合层经 reference 复用完整仓库 adapter 的 ``/Root/ConveyorBelt`` 子树，
    所以先按相同标准校验被依赖的完整 adapter，再校验组合层自身：subLayer 必须
    是轻量工位 baseline，且 ConveyorBelt 引用替换必须指向完整 adapter。
    """

    _require_warehouse_adapter(assets_dir)
    adapter = assets_dir / WORKCELL_LITE_VISUAL_ONLY_BACKGROUND_USD
    if not adapter.is_file():
        raise FileNotFoundError(
            "已选择 workcell-lite 背景并启用纯视觉输送机，但缺少专用组合层："
            f"{adapter}；" + _GENERATOR_HINT
        )
    text = adapter.read_text(encoding="utf-8")
    expected_sublayer = f"subLayers = [@./{WORKCELL_LITE_BACKGROUND_USD}@]"
    if expected_sublayer not in text:
        raise RuntimeError(
            "workcell-lite conveyor visual-only adapter 未以轻量工位为 baseline，"
            "会退回完整仓库背景；" + _GENERATOR_HINT
        )
    if (
        WORKCELL_LITE_ADAPTER_DELETE_REFERENCE not in text
        or WORKCELL_LITE_ADAPTER_PREPEND_REFERENCE not in text
    ):
        raise RuntimeError(
            "workcell-lite conveyor visual-only adapter 未把 ConveyorBelt 引用"
            "替换为纯视觉输送机子树，原生输送机碰撞仍会进入组合；" + _GENERATOR_HINT
        )
    return adapter


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
    输送机；legacy 仍由环境变量显式 opt-in。启用纯视觉输送机后按 baseline
    文件名路由 adapter：workcell-lite 用专用组合层，其余用完整仓库 adapter。
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

    if baseline_path.name == WORKCELL_LITE_BACKGROUND_USD:
        return _require_workcell_lite_adapter(assets_dir)
    return _require_warehouse_adapter(assets_dir)
