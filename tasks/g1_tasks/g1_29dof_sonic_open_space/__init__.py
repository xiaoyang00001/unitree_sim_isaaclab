# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Single-robot SONIC locomotion scene with an unobstructed ground plane."""

import gymnasium as gym

from . import open_space_env_cfg


_TERRAIN_TASKS = {
    "Isaac-G1-29DoF-Sonic-Grass": open_space_env_cfg.G129SonicGrassEnvCfg,
    "Isaac-G1-29DoF-Sonic-Gravel": open_space_env_cfg.G129SonicOpenSpaceEnvCfg,
    # Backward-compatible name; Gravel remains the default real terrain.
    "Isaac-G1-29DoF-Sonic-OpenSpace": open_space_env_cfg.G129SonicOpenSpaceEnvCfg,
    "Isaac-G1-29DoF-Sonic-Mud": open_space_env_cfg.G129SonicMudEnvCfg,
    "Isaac-G1-29DoF-Sonic-Slate": open_space_env_cfg.G129SonicSlateEnvCfg,
    "Isaac-G1-29DoF-Sonic-Snow": open_space_env_cfg.G129SonicSnowEnvCfg,
    "Isaac-G1-29DoF-Sonic-Warehouse": open_space_env_cfg.G129SonicWarehouseEnvCfg,
    "Isaac-G1-29DoF-Sonic-Apartment": open_space_env_cfg.G129SonicApartmentEnvCfg,
    "Isaac-G1-29DoF-Sonic-Staircase": open_space_env_cfg.G129SonicStaircaseEnvCfg,
    "Isaac-G1-29DoF-Sonic-Office": open_space_env_cfg.G129SonicOfficeEnvCfg,
}

for _task_id, _env_cfg in _TERRAIN_TASKS.items():
    gym.register(
        id=_task_id,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        kwargs={"env_cfg_entry_point": _env_cfg},
        disable_env_checker=True,
    )
