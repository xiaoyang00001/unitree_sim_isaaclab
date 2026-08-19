# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

"""Host multi-robot SONIC action source（工作包 B）。

同一 Isaac 进程里最多五台全动力学 SONIC 机器人各由一套 GR00T deploy 经独立 DDS
通道驱动（robot_1: rt/*；其余 robot_N: rt/rN/*）。本组合器持有每台机器人的
SonicDDSActionProvider 通道，负责：

- N×129 维动作拼接：每台依次为 q/dq/tau，机器人间 robot-major，与
  HostConveyorActionsCfg 的 term 顺序逐段对应；五机时总计 645 维；
- N 路 ack AND 锁步：所有通道都拿到当前 PhysX tick 的 LowCmd ack 才放行 env.step。
  通道内等待互相重叠，稳态代价接近最慢通道；host 必须让所有配置的 deploy 同时运行；
- 倒地恢复/控制状态的广播语义：begin_fall_recovery 下发到全部通道（复位是整场景的），
  control_started/fall_recovery_active 用 any 聚合（sim_main 的既有判断保持兼容）。
"""

from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from action_provider.action_provider_sonic_dds import SonicDDSActionProvider
from robots.sonic_multi_robot import (
    SONIC_ROBOT_COUNT_MAX,
    SONIC_ROBOT_COUNT_MIN,
    sonic_host_action_terms,
    sonic_robot_channel_specs,
)

# Public maximum-layout contract used by source-level and unit tests.
HOST_ACTION_TERMS = sonic_host_action_terms(SONIC_ROBOT_COUNT_MAX)


class SonicDDSHostActionProvider(ActionProvider):
    """Two to five lock-stepped SONIC channels feeding one action tensor."""

    def __init__(self, env, args_cli):
        super().__init__("sonic_dds_host")
        self.robot_count = int(getattr(args_cli, "sonic_robot_count", 2))
        if not SONIC_ROBOT_COUNT_MIN <= self.robot_count <= SONIC_ROBOT_COUNT_MAX:
            raise ValueError(
                "host SONIC robot count must be in "
                f"[{SONIC_ROBOT_COUNT_MIN}, {SONIC_ROBOT_COUNT_MAX}], got {self.robot_count}"
            )
        self.channel_specs = sonic_robot_channel_specs(self.robot_count)
        # 每个通道校验自身关节映射；N×3 term 的全局布局在下面统一校验。
        self.channels = {
            spec.asset_name: SonicDDSActionProvider(
                env,
                args_cli,
                asset_name=spec.asset_name,
                robot_dds_name=spec.robot_dds_name,
                dex3_dds_name=spec.dex3_dds_name,
                foot_contact_name=spec.foot_contact_name,
                validate_action_layout=False,
                log_suffix=spec.log_suffix,
            )
            for spec in self.channel_specs
        }
        self._validate_action_layout(env)
        channel_summary = ", ".join(
            f"{spec.asset_name}<-{spec.robot_dds_name}({spec.topic_prefix}/*)"
            for spec in self.channel_specs
        )
        print(
            f"[sonic_dds_host] {self.robot_count}-robot channels ready: "
            f"{channel_summary}; env.step gates on ALL lock-step acks"
        )

    def _validate_action_layout(self, env) -> None:
        action_manager = getattr(env, "action_manager", None)
        if action_manager is None:
            return
        action_terms = sonic_host_action_terms(len(self.channels))
        active_terms = tuple(action_manager.active_terms)
        if active_terms[: len(action_terms)] != action_terms:
            raise ValueError(
                "host multi-robot action terms must be ordered as "
                f"{action_terms}, got {active_terms}"
            )
        nonzero_extras = [
            name
            for name in active_terms[len(action_terms):]
            if action_manager.get_term(name).action_dim != 0
        ]
        if nonzero_extras:
            raise ValueError(
                "host action layout allows only zero-dim auxiliary terms after "
                f"{action_terms}, got non-zero terms {nonzero_extras}"
            )
        expected = sum(ch._num_joints * 3 for ch in self.channels.values())
        if action_manager.total_action_dim != expected:
            raise ValueError(
                "host action dimension must contain q/dq/tau for every joint of all "
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
        # N 路 ack AND：任一侧未 ack 则整个 env 不 step，sample_seq 不前进；
        # 已 ack 的一侧下一轮对同一 tick 仍然匹配，锁步自愈。
        return all(channel.can_step_environment() for channel in self.channels.values())

    @property
    def control_started(self) -> bool:
        return any(channel.control_started for channel in self.channels.values())

    @property
    def fall_recovery_active(self) -> bool:
        return any(channel.fall_recovery_active for channel in self.channels.values())

    def begin_fall_recovery(self, **kwargs) -> None:
        # 复位是整场景语义：所有机器人一起回默认位，每条通道都要走恢复事务。
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
