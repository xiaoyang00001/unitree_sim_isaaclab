"""Background asset selection for the conveyor task.

This module intentionally has no Isaac Sim imports so the selection and fallback contract can be
tested without starting Kit.  The visual-only USD is a strong override layer on top of the original
v61 scene; the original asset remains available for controlled A/B comparisons.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path


BACKGROUND_MODE_ENV = "ISAACLAB_CONVEYOR_BACKGROUND"
DEFAULT_BACKGROUND_MODE = "visual_only"
BACKGROUND_ASSET_FILENAMES = {
    "visual_only": "warehouse-simple6_v61_visual_only.usda",
    "legacy_v61": "warehouse-simple6_v61.usd",
}


def resolve_background_asset(
    assets_dir: Path,
    environ: Mapping[str, str] | None = None,
) -> tuple[str, Path]:
    """Resolve the selected background mode and verify that its local layer exists."""

    values = os.environ if environ is None else environ
    raw_mode = values.get(BACKGROUND_MODE_ENV, DEFAULT_BACKGROUND_MODE)
    mode = raw_mode.strip().lower() or DEFAULT_BACKGROUND_MODE
    if mode not in BACKGROUND_ASSET_FILENAMES:
        choices = ", ".join(sorted(BACKGROUND_ASSET_FILENAMES))
        raise ValueError(f"{BACKGROUND_MODE_ENV}={raw_mode!r} 无效，可选值: {choices}")

    asset_path = assets_dir / BACKGROUND_ASSET_FILENAMES[mode]
    if not asset_path.is_file():
        raise FileNotFoundError(f"流水线背景资产不存在: {asset_path}")
    return mode, asset_path
