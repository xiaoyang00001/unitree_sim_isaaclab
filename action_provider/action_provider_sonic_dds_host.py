# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Host dual-robot action source（工作包 B）。

同一 Isaac 进程里 robot_1 + robot_2 都是全动力学 SONIC 机器人，各由一套 GR00T deploy
经独立 DDS 通道驱动（robot_1: rt/*；robot_2: rt/r2/*）。本组合器持两个
SonicDDSActionProvider 通道，负责：

- 258 维动作拼接：``[r1_q, r1_dq, r1_tau, r2_q, r2_dq, r2_tau]``——机器人内 field-major、
  机器人间 robot-major，与 HostConveyorActionsCfg 的 term 顺序逐段对应；
- 双 ack AND 锁步：两条通道都拿到对当前 PhysX tick 的 LowCmd ack 才放行 env.step。
  通道内的等待互相重叠（ch1 等待期间 ch2 的 ack 已在路上），稳态代价≈max 而非 sum；
  双双超时的最坏情况是 2×sonic_sync_wait_timeout——host 必须两套 deploy 同时在跑；
- 倒地恢复/控制状态的广播语义：begin_fall_recovery 下发到两通道（复位是整场景的），
  control_started/fall_recovery_active 用 any 聚合（sim_main 的既有判断保持兼容）。
"""

from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from action_provider.action_provider_sonic_dds import SonicDDSActionProvider

HOST_ACTION_TERMS = (
    "joint_pos",
    "joint_vel",
    "joint_effort",
    "joint_pos_2",
    "joint_vel_2",
    "joint_effort_2",
)


class SonicDDSHostActionProvider(ActionProvider):
    """Two lock-stepped SONIC channels feeding one 258-dim action tensor."""

    def __init__(self, env, args_cli):
        super().__init__("sonic_dds_host")
        # 两通道各自校验自身关节映射；六 term 的全局布局校验在下面统一做。
        self.channels = {
            "robot": SonicDDSActionProvider(
                env,
                args_cli,
                validate_action_layout=False,
            ),
            "robot_2": SonicDDSActionProvider(
                env,
                args_cli,
                asset_name="robot_2",
                robot_dds_name="g129_r2",
                dex3_dds_name="dex3_r2",
                foot_contact_name="foot_contact_2",
                validate_action_layout=False,
                log_suffix=":r2",
            ),
        }
        self._validate_action_layout(env)
        print(
            "[sonic_dds_host] dual-robot channels ready: "
            "robot<-g129(rt/*), robot_2<-g129_r2(rt/r2/*); "
            "env.step gates on BOTH lock-step acks"
        )

    def _validate_action_layout(self, env) -> None:
        action_manager = getattr(env, "action_manager", None)
        if action_manager is None:
            return
        active_terms = tuple(action_manager.active_terms)
        if active_terms[: len(HOST_ACTION_TERMS)] != HOST_ACTION_TERMS:
            raise ValueError(
                "host dual-robot action terms must be ordered as "
                f"{HOST_ACTION_TERMS}, got {active_terms}"
            )
        nonzero_extras = [
            name
            for name in active_terms[len(HOST_ACTION_TERMS):]
            if action_manager.get_term(name).action_dim != 0
        ]
        if nonzero_extras:
            raise ValueError(
                "host action layout allows only zero-dim auxiliary terms after "
                f"{HOST_ACTION_TERMS}, got non-zero terms {nonzero_extras}"
            )
        expected = sum(ch._num_joints * 3 for ch in self.channels.values())
        if action_manager.total_action_dim != expected:
            raise ValueError(
                "host action dimension must contain q/dq/tau for every joint of both "
                f"robots: expected {expected}, got {action_manager.total_action_dim}"
            )

    def get_action(self, env) -> Optional[torch.Tensor]:
        # 通道的 get_action 按当前实现永不返回 None（拿不到命令也回 hold 张量）；
        # 万一未来行为变化，整体返回 None 交给 RobotController 的 fallback，别拼半截。
        segments = []
        for channel in self.channels.values():
            segment = channel.get_action(env)
            if segment is None:
                return None
            segments.append(segment)
        return torch.cat(segments, dim=1)

    def can_step_environment(self) -> bool:
        # 双 ack AND：单侧未 ack 则整个 env 不 step，sample_seq 不前进；
        # 已 ack 的一侧下一轮对同一 tick 仍然匹配，锁步自愈。
        return all(channel.can_step_environment() for channel in self.channels.values())

    @property
    def control_started(self) -> bool:
        return any(channel.control_started for channel in self.channels.values())

    @property
    def fall_recovery_active(self) -> bool:
        return any(channel.fall_recovery_active for channel in self.channels.values())

    def begin_fall_recovery(self, **kwargs) -> None:
        # 复位是整场景语义：两台机器人一起回默认位，两条通道都要走恢复事务。
        for channel in self.channels.values():
            channel.begin_fall_recovery(**kwargs)

    def start(self):
        self.is_running = True
        for channel in self.channels.values():
            channel.start()

    def stop(self):
        self.is_running = False
        for channel in self.channels.values():
            channel.stop()

    def cleanup(self):
        for channel in self.channels.values():
            channel.cleanup()
