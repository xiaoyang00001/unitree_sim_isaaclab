# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import conveyor_env_cfg
from . import multi_scene_env_cfg

gym.register(
    id="Isaac-G1-29DoF-Sonic-Conveyor",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": conveyor_env_cfg.G129SonicConveyorEnvCfg,
    },
    disable_env_checker=True,
)

# Complete real-scene previews reuse the Conveyor multi-robot contract.  The
# warehouse task above remains the functional conveyor baseline; these IDs
# swap in a complete room/building background and explicitly disable conveyor
# props/events where that USD has no belt.
_MULTI_SCENE_TASKS = {
    "Isaac-G1-29DoF-Sonic-Conveyor-Warehouse": conveyor_env_cfg.G129SonicConveyorEnvCfg,
    "Isaac-G1-29DoF-Sonic-Conveyor-Apartment": multi_scene_env_cfg.G129SonicMultiApartmentEnvCfg,
    "Isaac-G1-29DoF-Sonic-Conveyor-Staircase": multi_scene_env_cfg.G129SonicMultiStaircaseEnvCfg,
    "Isaac-G1-29DoF-Sonic-Conveyor-Office": multi_scene_env_cfg.G129SonicMultiOfficeEnvCfg,
}

for _task_id, _env_cfg in _MULTI_SCENE_TASKS.items():
    gym.register(
        id=_task_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        kwargs={"env_cfg_entry_point": _env_cfg},
        disable_env_checker=True,
    )
