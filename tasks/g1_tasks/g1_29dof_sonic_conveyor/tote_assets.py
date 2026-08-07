"""Dependency-free tote collision asset selection.

The conveyor task keeps the SimReady tote mesh for rendering, but defaults to a
five-box open-container collider.  The historical convex-decomposition wrapper
remains available for controlled fallback and grasp comparisons.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path


TOTE_COLLIDER_ENV = "ISAACLAB_TOTE_COLLIDER"
DEFAULT_TOTE_COLLIDER = "compound"
TOTE_ASSET_FILENAMES = {
    "compound": "tote_b04_compound_physics.usda",
    "convex_decomposition": "tote_b04_physics.usda",
}


def resolve_tote_asset(
    props_dir: Path,
    environ: Mapping[str, str] | None = None,
) -> tuple[str, Path]:
    """Resolve the tote collider variant and fail fast on stale configuration."""

    values = os.environ if environ is None else environ
    raw_mode = values.get(TOTE_COLLIDER_ENV, DEFAULT_TOTE_COLLIDER)
    mode = raw_mode.strip().lower() or DEFAULT_TOTE_COLLIDER
    if mode not in TOTE_ASSET_FILENAMES:
        choices = ", ".join(sorted(TOTE_ASSET_FILENAMES))
        raise ValueError(f"{TOTE_COLLIDER_ENV}={raw_mode!r} 无效，可选值: {choices}")

    asset_path = props_dir / TOTE_ASSET_FILENAMES[mode]
    if not asset_path.is_file():
        raise FileNotFoundError(f"料筐碰撞资产不存在: {asset_path}")
    return mode, asset_path
