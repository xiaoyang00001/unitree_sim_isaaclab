"""Compatibility helpers for Isaac Lab APIs that changed across major versions."""

from __future__ import annotations

from functools import lru_cache
from importlib import metadata as importlib_metadata

import torch


@lru_cache(maxsize=1)
def isaac_runtime_quaternions_are_xyzw() -> bool:
    """Return whether Isaac runtime pose tensors use ``xyzw`` quaternion order.

    Isaac Lab 5 exposes articulation pose tensors in ``wxyz`` order. Isaac Lab
    6 changed those tensors to ``xyzw``. Reading package metadata avoids
    importing ``isaacsim``, which would try to bootstrap Kit in unit tests.
    """
    for distribution_name in ("isaacsim", "isaacsim-core"):
        try:
            version_text = importlib_metadata.version(distribution_name)
        except importlib_metadata.PackageNotFoundError:
            continue

        try:
            major_text = version_text.split(".", maxsplit=1)[0]
            return int(major_text) >= 6
        except (TypeError, ValueError):
            continue

    # Isaac Lab 5 installations and source-only unit tests may not expose an
    # isaacsim distribution. Preserve the legacy wxyz behavior in that case.
    return False


def isaac_quat_to_wxyz(
    quaternion: torch.Tensor,
    *,
    input_is_xyzw: bool | None = None,
) -> torch.Tensor:
    """Convert an Isaac runtime quaternion tensor to Unitree's ``wxyz`` order.

    Args:
        quaternion: Tensor whose last dimension contains four quaternion values.
        input_is_xyzw: Explicit source order for tests or external callers. When
            omitted, the installed Isaac Sim major version selects the order.
    """
    if quaternion.shape[-1] != 4:
        raise ValueError(
            f"Expected quaternion last dimension to be 4, got {tuple(quaternion.shape)}"
        )

    if input_is_xyzw is None:
        input_is_xyzw = isaac_runtime_quaternions_are_xyzw()
    if not input_is_xyzw:
        return quaternion

    return torch.cat((quaternion[..., 3:4], quaternion[..., :3]), dim=-1)
