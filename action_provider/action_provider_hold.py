# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Hold-pose action source for scene-sync viewer instances.

viewer（纯镜像观看端）没有 deploy，也就没有锁步 ack 可等——挂 sonic_dds 会让主循环
每圈都吃满 250ms 的 sync 超时（实测被压到 4Hz，镜像 apply 同频）。本 provider 把
本机 ghost 机器人钉在默认站姿并立即放行每一步，主循环回到 step_hz 满频。

动作布局沿用 SONIC 桥的 field-major 约定：[q_target(43), dq_target(43), tau_ff(43)]。
若任务的动作维度对不上（挂错任务），退回全零并打印一次警告——ghost 无碰撞、无重力、
停在场外，塌成零位姿也不影响镜像语义。
"""

from typing import Optional

import torch

from action_provider.action_base import ActionProvider


class HoldActionProvider(ActionProvider):
    """Always-ready provider that pins the local robot at its default pose."""

    def __init__(self, env, args_cli=None):
        super().__init__("HoldActionProvider")
        self._action: Optional[torch.Tensor] = None

    def get_action(self, env) -> Optional[torch.Tensor]:
        if self._action is None:
            robot = env.scene["robot"]
            default_q = robot.data.default_joint_pos.clone()
            zeros = torch.zeros_like(default_q)
            action = torch.cat([default_q, zeros, zeros], dim=-1)
            total_dim = env.action_manager.total_action_dim
            if action.shape[1] != total_dim:
                print(
                    f"[hold] action layout mismatch: built {action.shape[1]} dims, "
                    f"task expects {total_dim}; falling back to zeros"
                )
                action = torch.zeros((env.num_envs, total_dim), device=env.device)
            self._action = action
        return self._action

    def start(self):
        # 无后台线程可跑：动作是常量，起线程只会白吃调度。
        self.is_running = True
        print(f"[{self.name}] ActionProvider started (constant hold pose, no thread)")

    def stop(self):
        self.is_running = False
        print(f"[{self.name}] ActionProvider stopped")
