"""Dependency-free prop selection for the conveyor scene.

The conveyor task inherits a packing table and three cubes from the generic SONIC scene and used
to spawn two complete cart groups for both layouts.  Most of those assets are outside the active
workflow.  Keeping the selection here makes the spawn and scene-sync inventories share one source
of truth without importing Isaac Lab in ordinary unit tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


PROP_MODE_ENV = "ISAACLAB_CONVEYOR_PROPS"
DEFAULT_PROP_MODE = "layout"
VALID_PROP_MODES = ("layout", "legacy_props")

_LEGACY_PROP_NAMES = (
    "pushcart",
    "cart_box1",
    "cart_box2",
    "test_box",
    "pushcart_2",
    "cart2_tote1",
    "cart2_tote2",
    "cube_1",
    "cube_2",
    "cube_3",
)


@dataclass(frozen=True)
class ConveyorSceneProps:
    """Resolved prop inventory used by spawning and scene synchronization."""

    mode: str
    spawned_names: tuple[str, ...]
    inherited_packing_table: bool

    @property
    def legacy_props(self) -> bool:
        return self.mode == "legacy_props"

    def spawns(self, name: str) -> bool:
        return name in self.spawned_names


def resolve_scene_props(
    environ: Mapping[str, str],
    *,
    totes_on_conveyor: bool,
) -> ConveyorSceneProps:
    """Choose only the props required by the active layout.

    ``layout`` is intentionally sparse:

    * conveyor layout: the two task totes only;
    * pushcart layout: ``pushcart_2`` and the two totes stacked on it.

    ``legacy_props`` restores the previous two-cart scene, including the inherited packing table
    and three cubes, for visual comparisons and rollback.
    """

    raw_mode = environ.get(PROP_MODE_ENV, DEFAULT_PROP_MODE)
    mode = str(raw_mode).strip().lower() or DEFAULT_PROP_MODE
    if mode not in VALID_PROP_MODES:
        choices = ", ".join(VALID_PROP_MODES)
        raise ValueError(f"{PROP_MODE_ENV}={raw_mode!r} 无效，可选值: {choices}")

    if mode == "legacy_props":
        return ConveyorSceneProps(
            mode=mode,
            spawned_names=_LEGACY_PROP_NAMES,
            inherited_packing_table=True,
        )

    required = ("cart2_tote1", "cart2_tote2")
    if not totes_on_conveyor:
        required = ("pushcart_2", *required)
    return ConveyorSceneProps(
        mode=mode,
        spawned_names=required,
        inherited_packing_table=False,
    )
