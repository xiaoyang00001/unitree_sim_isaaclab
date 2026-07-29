# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
A layered robot control system
"""

import time
from typing import Optional, Dict, Any
import torch
from dataclasses import dataclass
from action_provider.action_base import ActionProvider


@dataclass
class ControlConfig:
    """minimal control configuration"""
    step_hz: int = 500  # the frequency of the low-level execution
    replay_mode: bool = False
    use_rl_action_mode: bool = False
    # SONIC GUI 晚渲染:env.step 只跑物理(cfg.sim.render_interval 已被 sim_main
    # 设成不可达),渲染由本控制器在 env.step 返回后调用——此时 lowstate 已发布,
    # C++ 推理与渲染并行,下一轮 get_action 的 ack 等待被渲染时间掩盖。
    late_render: bool = False
    # 每 N 个控制循环渲染一次(N=2 即 GUI 25Hz、物理仍 50Hz),摊薄渲染成本。
    late_render_interval: int = 1
    # 每次渲染窗口连渲 M 帧(实验旋钮):世界状态相同但头显姿态逐帧刷新。
    # 实测 XR 下 M=2 是负优化(背靠背第二帧成本 14.7→18.5ms/帧,
    # 物理掉到 17-18Hz 而 AR 仅 ~36fps),保留作诊断用,默认 1。
    late_render_repeat: int = 1


class RobotController:
    """robot controller
    """
    
    def __init__(self, env, config: ControlConfig):
        self.env = env
        self.config = config
        self.action_provider: Optional[ActionProvider] = None
        self.is_running = False

        if config.step_hz <= 0:
            raise ValueError(f"step_hz must be positive, got {config.step_hz}")

        # Deadline-based frequency control.  The previous implementation saved
        # the timestamp taken before sleeping, so every second loop skipped its
        # sleep and a requested 100 Hz ran at about 120 Hz.  A monotonic deadline
        # keeps the environment step cadence stable and skips missed deadlines
        # without issuing catch-up bursts.
        self._step_interval = 1.0 / config.step_hz
        self._next_step_time = 0.0
        
        # Action dimensions are task-defined and are not necessarily equal to
        # the articulation joint count.  The SONIC bridge, for example, carries
        # q/dq/tau as three fields per joint.
        action_dim = env.action_manager.total_action_dim
        self._last_action = torch.zeros((env.num_envs, action_dim), device=env.device)
        
        
        # minimal statistics
        self.step_count = 0
        self.wait_count = 0
        self._start_time = 0.0

        # 画面帧率统计:主循环 Hz 与 GUI fps 在隔圈渲染下不是一回事,
        # 单独累计 sim.render() 的次数与耗时,由 sim_main 的 stats 窗口取走。
        self._render_count = 0
        self._render_time_total = 0.0
        
        # minimal performance analysis
        self._profile_counter = 0
        self._profile_interval = 2000  # reduce the printing frequency
        self._render_loop_counter = 0
        
        # cache the function reference (reduce the lookup overhead)
        self._perf_counter = time.perf_counter
        self._time_sleep = time.sleep
        
        print(f"  - control frequency: {config.step_hz}Hz")
    
    def set_action_provider(self, provider: ActionProvider):
        """set the action provider"""
        if self.action_provider:
            self.action_provider.stop()
            self.action_provider.cleanup()
        
        self.action_provider = provider
        print(f"[SimpleController] set the action provider: {provider.name}")
    
    def start(self):
        """start the controller"""
        if self.is_running:
            return
        
        self.is_running = True
        self._start_time = time.time()
        self._next_step_time = self._perf_counter() + self._step_interval
        
        # start the action provider
        if self.action_provider:
            self.action_provider.start()
        
        print("[SimpleController] the controller is started")
    
    def stop(self):
        """stop the controller"""
        if not self.is_running:
            return
            
        self.is_running = False
        
        if self.action_provider:
            self.action_provider.stop()
        
        print("[SimpleController] the controller is stopped")
    
    def step(self) -> bool:
        """minimal control step - zero thread competition"""
        if not self.is_running:
            return False
        
        # use the cached function reference
        perf_counter = self._perf_counter
        step_start = perf_counter()
        
        # 1. minimal action acquisition (synchronous, zero thread competition, pre-calculated strategy)
        action_start = perf_counter()
        action = None
        
        # try to get the action from the action provider
        if self.action_provider:
            action = self.action_provider.get_action(self.env)
            if action is not None:
                self._last_action = action
        
        # if no action is obtained, use the pre-calculated fallback strategy
        if action is None:
            action = self._last_action

        action_time = perf_counter() - action_start

        environment_step_ready = True
        if self.action_provider and hasattr(self.action_provider, "can_step_environment"):
            environment_step_ready = bool(self.action_provider.can_step_environment())

        # 2. direct environment step
        env_start = perf_counter()
        stepped = False
        with torch.inference_mode():
            if self.config.replay_mode or self.config.use_rl_action_mode:
                # These legacy modes intentionally render/step elsewhere.  Do
                # not report them as SONIC synchronization stalls.
                stepped = True
                # self.env.sim.render()
            elif environment_step_ready:
                self.env.step(action)
                stepped = True
            env_time = perf_counter() - env_start

            if stepped:
                self.step_count += 1
            else:
                self.wait_count += 1
        
        # 2.5 late render: lowstate 已随 env.step 末尾的观测发布出门,这里渲染
        # 与 C++ 推理并行。HOLD(未 step)时也渲染,GUI 在锁步等待期不冻屏。
        # 按 late_render_interval 隔圈渲染(计循环数而非物理步数,HOLD 期照常)。
        render_time = 0.0
        if self.config.late_render:
            self._render_loop_counter += 1
            if self._render_loop_counter >= self.config.late_render_interval:
                self._render_loop_counter = 0
                render_start = perf_counter()
                for _ in range(self.config.late_render_repeat):
                    self.env.sim.render()
                render_time = perf_counter() - render_start
                self._render_count += self.config.late_render_repeat
                self._render_time_total += render_time

        # 3. deadline-based frequency control
        sleep_start = perf_counter()
        current_time = perf_counter()
        sleep_needed = self._next_step_time - current_time
        if sleep_needed > 0.0:
            self._time_sleep(sleep_needed)

        current_time = perf_counter()
        missed_intervals = max(
            0,
            int((current_time - self._next_step_time) // self._step_interval),
        )
        self._next_step_time += (missed_intervals + 1) * self._step_interval
        sleep_time = perf_counter() - sleep_start
        
        # 4. minimal performance print
        self._profile_counter += 1
        if self._profile_counter >= self._profile_interval:
            total_time = perf_counter() - step_start
            render_part = (
                f"R:{render_time*1000:.1f}ms, " if self.config.late_render else ""
            )
            print(
                f"[Performance] A:{action_time*1000:.1f}ms, "
                f"E:{env_time*1000:.1f}ms, {render_part}"
                f"S:{sleep_time*1000:.1f}ms, "
                f"T:{total_time*1000:.1f}ms, stepped={int(stepped)}, "
                f"physics_steps={self.step_count}, sync_waits={self.wait_count}"
            )
            self._profile_counter = 0
        return stepped
    def pop_render_stats(self):
        """取走并清零本统计窗口内的渲染计数与累计耗时。

        返回 (frames, total_seconds)。late_render 关闭时渲染发生在 env.step
        内部,这里恒为 (0, 0.0),调用方应据此回退到"主循环 Hz / render_interval"。
        """
        frames = self._render_count
        total = self._render_time_total
        self._render_count = 0
        self._render_time_total = 0.0
        return frames, total

    def cleanup(self):
        """clean up the resources"""
        self.stop()
        if self.action_provider:
            self.action_provider.cleanup()
    
    def set_profiling(self, enabled: bool, interval: int = 2000):
        """set the performance analysis"""
        if enabled:
            self._profile_interval = interval
        else:
            self._profile_interval = 999999999  # actually disable the printing
