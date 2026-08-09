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
    belt_box_names: tuple[str, ...] = (),
) -> ConveyorSceneProps:
    """Choose only the props required by the active layout.

    ``layout`` is intentionally sparse:

    * conveyor layout: the belt-box queue only.  The two plastic totes are gone —
      the boxes排在工位上游，是新的作业对象；
    * pushcart layout: ``pushcart_2`` and the two totes stacked on it（推车布局的
      作业闭环仍然是塑料筐，不生成纸箱）。

    ``legacy_props`` restores the previous two-cart scene, including the inherited packing table
    and three cubes, for visual comparisons and rollback.  它是**纯回退**：不叠加纸箱队列。

    ⚠️ 别把纸箱叠上来。两塑料筐的流水线出生位就在带上 (-5.35, 17.4) / (-5.89, 18.0)，
    而纸箱队列的第 4、5 个正好也在 y=17.4 / 18.0 的带中线上；筐沿 X 半宽 0.15、纸箱
    0.125，而车道距中线只有 0.27 ⇒ 两对物体各三轴互穿 5 mm，开局就是非法初始状态。
    也不存在"摘掉纸箱会让同步清单和实际 spawn 脱节"的顾虑：``SYNC_OBJECT_NAMES`` 就是
    从本函数的返回值派生的，SceneCfg 的每个纸箱字段也都以 ``spawns()`` 为门，两边同源。
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

    if totes_on_conveyor:
        required = tuple(belt_box_names)
    else:
        required = ("pushcart_2", "cart2_tote1", "cart2_tote2")
    return ConveyorSceneProps(
        mode=mode,
        spawned_names=required,
        inherited_packing_table=False,
    )
