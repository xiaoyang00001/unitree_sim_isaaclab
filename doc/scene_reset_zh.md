# Isaac/SONIC 场景复位说明

本文统一说明 `Isaac-G1-29DoF-Sonic*` 任务中的三类 reset 入口、共同执行语义、
安全恢复步骤、Pico 权限边界和已知风险。

## 1. 三类入口

| 入口 | 触发位置 | 默认条件 | 是否依赖 GR00T 改动 |
|---|---|---|---|
| 自动倒地复位 | Ubuntu `sim_main.py` | Root 倾角或高度持续越界 | 否 |
| Ubuntu F12 | 本地 Isaac/Kit 窗口 | 聚焦窗口后按一次 F12 | 否 |
| Pico#1 左 X | GR00T manager#1 | X 独占长按 2 秒且输入有效 | 是 |

三者最终都让 Ubuntu 权威端调用：

```text
trigger_robot_reset("reset_all_self", reason)
```

因此这里的 reset 不是“只把倒地机器人扶起来”，而是把任务注册的**整场景**写回默认
状态。Conveyor 双机器人 host 中，任意一台确认倒地、F12 或 Pico#1 发出请求，都会复位
两台机器人以及场景中的动态物体，并向镜像 viewer 广播新的 reset id。

## 2. 共同的安全恢复事务

`trigger_robot_reset()` 对 SONIC 任务按以下顺序执行：

1. 增加 Isaac reset epoch，使 deploy 能区分 reset 前后的状态代次；
2. reset 前开启 LowState grace，默认 1 秒内保留实时位置和 IMU，但把发布的关节
   `dq`、`tau_est` 临时置零，避免位姿瞬移误触发 `abs(dq) > 35 rad/s`；
3. 调用 `action_provider.begin_fall_recovery()`，恢复默认控制目标并拒绝 reset 前的
   LowCmd/HandCmd；
4. 触发 `reset_all_self`，随后执行 `env.reset()`，写回默认 Root、关节、物体状态并
   清理 Isaac 侧 action/observation/history；
5. `env.reset()` 返回后重新开启 LowState grace，覆盖首批 PhysX 瞬态样本；
6. 默认站姿固定 1 秒，只接受 reset 后新到达的 CONTROL 包，再用 1 秒平滑混合回
   SONIC 目标；倒地监控额外冷却 2 秒。

完整倒地恢复参数和历史故障注入结果见
[SONIC 29DoF 阶段交接文档](sonic_g1_29dof_phase1_handoff_zh.md#81-终端-a启动-43dof-载体上的-29dof-sonic-闭环)。

## 3. 自动倒地复位

自动检测默认只在有 SONIC DDS action provider 的权威仿真端启用；纯镜像 viewer 不启用。
只有对应控制通道已经进入 CONTROL 后才开始监控，避免初始化站姿固定阶段误判。

默认判据为以下任一条件连续保持 `0.5s`：

- Root 倾角 `>= 60°`；
- Root 高度 `base_z <= 0.35m`；
- Root 位姿出现 NaN、Inf 或无法计算的姿态。

双机器人 host 对两台机器人分别监控，任意一台确认倒地都会触发同一个整场景 reset。

可调参数：

```text
--fall_reset_tilt_deg 60
--fall_reset_min_base_height 0.35
--fall_reset_debounce_seconds 0.5
--fall_reset_hold_seconds 1.0
--fall_reset_blend_seconds 1.0
--fall_reset_lowstate_grace_seconds 1.0
--fall_reset_cooldown_seconds 2.0
```

测试 `kneel`、`crawl`、`IDLE_LYING_FACE_DOWN` 等故意低姿态动作时应增加：

```text
--no_auto_reset_on_fall
```

关闭自动检测不会关闭 F12、DDS 或其他手动 reset 入口。

## 4. Ubuntu F12

F12 只使用 Isaac 仓库，不需要 GR00T 或 Pico。启用条件为：

- Ubuntu/Linux 权威端；
- 有本地、非 headless 的 Kit 窗口；
- 不是 `--no_render`、replay 或 scene-sync viewer；
- 按键前先把焦点切到 Isaac/Kit 窗口。

`--hide_ui` 可以使用，只要本地 Kit 窗口仍存在。纯 SSH 的 headless / `--no_render`
没有本地键盘事件，不能使用 F12。

按键回调只把请求放入队列，不会在 Kit render 消息泵中直接执行 `env.reset()`；主循环
随后复用原有安全 reset 路径。正常日志为：

```text
[keyboard] Isaac/Kit window shortcut: F12 = full scene reset
[keyboard] F12 pressed: full scene reset queued
[reset_input] Ubuntu keyboard F12: full scene reset
```

连续 F12/交互请求受默认 `2s` 冷却约束。当前精简实现没有把 DDS category 2 与 F12
做跨来源冷却合并，因此不要同时按 F12 和 Pico X，否则可能连续执行两次 reset。

## 5. Pico#1 左 X

控制链为：

```text
Pico GameLink
  -> GR00T XrClient（完整按键、四轴、真实收包时间）
  -> GR00T manager#1（X 长按安全门）
  -> DDS rt/reset_pose/cmd，String_.data = "2"
  -> Ubuntu sim_main.py
  -> trigger_robot_reset("reset_all_self", ...)
```

manager#1 必须使用 `--enable_isaac_scene_reset` 显式启用；默认关闭。双 Pico 时只有
manager#1 有全局 reset publisher，manager#2 刻意无权限。单 Pico manager 可以作为唯一
权威端启用。

DDS category 2 的“整场景”含义以当前 SONIC pipeline（`enable_wholebody_dds=false`）为
前提，不能直接推广到所有旧 Wholebody 任务。Isaac 消费后会把本地 reset 命令槽写回
`-1`；这不是 manager 可见的网络 ACK。

一次有效长按必须同时满足：

- 只按左 X，A/B/Y 全部未按；
- 左右摇杆四轴都在默认死区 `±0.15` 内；
- GameLink 输入样本年龄不超过默认 `0.5s`；
- 连续保持 `2s`。

触发后只发布一次，必须收到明确、有效的 X 松开帧才重新武装。缺字段、NaN/Inf、摇杆未
回中、样本过期、未来时间或 GameLink 断流都会 fail-closed；断流不能伪装成松开。
`A+X`、`X+Y` 和 `A+B+X+Y` 不会误触发 reset，四键急停仍按原始按键状态工作。

manager#1 手工启动时至少需要：

```bash
UNITREE_DDS_DOMAIN=1 \
UNITREE_DDS_INTERFACE=lo \
XROBO_TRANSPORT=udp \
XROBO_UDP_PORT=63901 \
.venv_teleop/bin/python gear_sonic/scripts/pico_manager_thread_server.py \
  --manager \
  --no_auto_pose \
  --enable_isaac_scene_reset \
  --port 5556
```

成功初始化和触发时会看到：

```text
[SceneReset] DDS publisher ready: topic=rt/reset_pose/cmd domain=1 interface=lo
[SceneReset] published full-scene reset: topic=rt/reset_pose/cmd category=2
```

manager#2 的启动命令不得包含 `--enable_isaac_scene_reset`。

## 6. 分支与仓库边界

- Isaac `feat/ubuntu-f12-scene-reset`：F12-only；不要求 GR00T 改动；
- Isaac `feat/pico-x-scene-reset`：包含同一套 F12 功能，并增加 manager#1/manager#2
  的 bringup 权限接线；
- GR00T `feat/pico-x-scene-reset`：包含 X 长按状态机、GameLink 新鲜度快照和 DDS
  publisher；
- `backup/pico-scene-reset-controls-combined-20260805`：早期合并版本，仅用于回退和
  对照，不应作为精简 F12 基线。

本地分支可能因其他场景工作继续前移；需要复现时应同时核对提交图和实际 diff，不能只看
分支名。切换到 Pico X 版本后必须重启 sim 和 manager，运行中的 Python 进程不会热加载。

## 7. 已知风险与排查

整场景 reset 会清理 Isaac 环境和本地 action provider 状态，但**不会清空外部
manager/planner 输入缓冲**。PLANNER 模式 reset 后若出现原地踉跄或不位移：

1. 查看 deploy 的 `Planner Model` 耗时；
2. 健康路径通常 `>90ms`，已知退化指纹约为 `36–79ms`；
3. 重启对应 deploy；必要时同时重启对应 manager。

重要行走演示期间不要把整场景 reset 当作无风险归位键。

常见无效原因：

| 现象 | 优先检查 |
|---|---|
| F12 无日志 | 是否聚焦 Kit；是否 headless/`--no_render`/replay/viewer；sim 是否已重启 |
| X 长按无日志 | GR00T 是否切到 X 分支；manager#1 是否带 enable 参数；DDS domain/interface 是否一致 |
| X 计时中断 | A/B/Y、四轴死区、GameLink 样本新鲜度和 UDP 连通性 |
| manager#2 长按无效 | 预期行为；manager#2 没有全局 reset 权限 |
| reset 后不能正常行走 | 检查 planner 退化指纹并重启 deploy |

## 8. 已完成验证

- Isaac F12 与 bringup 权限测试：当前分支全套 `37/37` 通过；
- GR00T scene reset 与 UDP freshness 测试：`13/13` 通过；
- Python 编译、`bash -n tools/pipeline_pico_bringup.sh` 和两仓 `git diff --check` 通过；
- DDS category 2 在隔离 domain/loopback 上完成回环验证。

静态测试不能代替 Kit GUI 和真实 Pico 实机测试；实机验收时应保存上述触发日志。
