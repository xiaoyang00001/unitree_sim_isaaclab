# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import cafe_env_cfg


gym.register(
    id="Isaac-G1-29DoF-Sonic-Cafe",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": cafe_env_cfg.G129DualSonicCafeEnvCfg,
    },
    disable_env_checker=True,
)
