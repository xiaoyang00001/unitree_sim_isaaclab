# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import g1_29dof_dex3_sonic_env_cfg


gym.register(
    id="Isaac-G1-29DoF-Dex3-Sonic",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": g1_29dof_dex3_sonic_env_cfg.G129Dex3SonicEnvCfg,
    },
    disable_env_checker=True,
)


gym.register(
    id="Isaac-G1-29DoF-Sonic",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": g1_29dof_dex3_sonic_env_cfg.G129SonicEnvCfg,
    },
    disable_env_checker=True,
)
