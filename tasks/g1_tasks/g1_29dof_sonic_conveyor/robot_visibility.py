"""Render-only visibility policy for the conveyor scene's robot assets.

The policy deliberately changes only USD/Imageable visibility.  It must not
remove an articulation or a scene-sync entry: a hidden robot can still be a
SONIC authority, a DDS endpoint, or a scene-state peer.

``ISAACLAB_CONVEYOR_VISIBLE_ROBOTS`` accepts the following compact forms:

* ``first`` (the default): show global ``robot_1`` only;
* ``all``: show both work robots and all three standby robots;
* ``none``: hide every robot visual;
* a comma-separated list such as ``robot_1,standby_robot_2``.

The names are global scene identities, so the same setting has the expected
meaning on an ID=1 host, an ID=2 peer, and an ID=0 viewer.
"""

from __future__ import annotations

import os
from collections.abc import Mapping


VISIBLE_ROBOTS_ENV = "ISAACLAB_CONVEYOR_VISIBLE_ROBOTS"
DEFAULT_VISIBILITY_MODE = "first"
WORK_ROBOT_NAMES = ("robot_1", "robot_2")
STANDBY_ROBOT_NAMES = tuple(f"standby_robot_{index}" for index in range(1, 4))
ALL_ROBOT_NAMES = WORK_ROBOT_NAMES + STANDBY_ROBOT_NAMES


def _normalise_name(value: str) -> str:
    """Accept the short names users commonly type in a launch command."""

    name = value.strip().lower().replace("-", "_")
    aliases = {
        "r1": "robot_1",
        "r2": "robot_2",
        "robot1": "robot_1",
        "robot2": "robot_2",
        "standby1": "standby_robot_1",
        "standby2": "standby_robot_2",
        "standby3": "standby_robot_3",
    }
    return aliases.get(name, name)


def resolve_visible_robot_names(
    environ: Mapping[str, str] | None = None,
) -> frozenset[str]:
    """Resolve the global robot names whose visuals should be rendered.

    Invalid/unknown tokens are ignored with a safe fallback to ``robot_1``.
    This module is imported while the task registry is eagerly loaded, so a
    malformed optional display variable must not prevent unrelated tasks from
    registering.
    """

    values = os.environ if environ is None else environ
    raw = values.get(VISIBLE_ROBOTS_ENV, DEFAULT_VISIBILITY_MODE)
    mode = str(raw).strip().lower()
    if not mode or mode == "first":
        return frozenset(("robot_1",))
    if mode in {"all", "*"}:
        return frozenset(ALL_ROBOT_NAMES)
    if mode in {"none", "off", "hidden"}:
        return frozenset()

    selected: set[str] = set()
    for token in mode.split(","):
        name = _normalise_name(token)
        if name in ALL_ROBOT_NAMES:
            selected.add(name)
        elif name in {"standby", "standby_robots"}:
            selected.update(STANDBY_ROBOT_NAMES)
    return frozenset(selected or ("robot_1",))


VISIBLE_ROBOT_NAMES = resolve_visible_robot_names()
"""Import-time policy used by the Isaac Lab config factories."""


def is_robot_visual_visible(global_name: str) -> bool:
    """Return whether a global robot visual should be spawned visible."""

    return global_name in VISIBLE_ROBOT_NAMES
