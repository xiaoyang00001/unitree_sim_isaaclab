# Gear SONIC → Isaac Lab G1 29DoF 非 VR 阶段交接与验证记录

> 文档日期：2026-07-23
> 当前系统：Ubuntu 22.04、Isaac Sim 5.1、Isaac Lab
> Isaac 桥接仓库：`/home/nolovr/Documents/unitree_sim_isaaclab`
> GR00T/SONIC 仓库：`/home/nolovr/GR00T-WholeBodyControl`
> Isaac Lab：`/home/nolovr/IsaacLab`
> 当前结论：非 VR 的 G1 本体 29DoF 闭环已经完成键盘慢走、停步、0.8→0.5 m 分级深蹲、0.5→0.8 m 分级起身和正常退出验证，阶段一要求的“键盘控制行走和蹲下并验证整条链路”已经通过。深蹲 0.5 m 时保持稳定但 reference tracking 的 P95 仍高于本文候选优良门槛，因此还需要扩大动作矩阵、改进深蹲跟踪并做长时间稳定性测试，才能认定“全身关节控制特别好”。OpenXR/PICO 按键和夹爪映射尚未验证，明确留到后续阶段。

## 1. 本阶段需求与边界

当前优先级不是 OpenXR，也不是 Dex3 夹爪，而是先把 Gear SONIC 对 G1 本体 29 个电机关节的闭环控制在 Isaac Lab/PhysX 中验证好。

本阶段必须满足：

- SONIC 独占控制 G1 本体 29DoF：双腿 12、腰部 3、双臂和双腕 14。
- Isaac Lab 不再叠加内部 locomotion policy，不与 SONIC 抢腿部或腰部控制权。
- Floating Root 只由 PhysX 动力学和地面接触决定；进入正式控制后不能逐帧写 Root pose，也不能做 teleport 式动作回放。
- 启动期间可以暂时固定 Root，避免 DDS 发现、模型加载和初始姿态插值期间机器人自由倒地；收到 SONIC 的正式 CONTROL 标记后必须释放 Root。
- 非 VR 验证阶段不接入 Dex3 14 个手指关节，不验证 PICO/OpenXR 按键，不验证夹爪映射。
- 控制频率、URDF、初始姿态、执行器参数、摩擦和自碰撞设置应尽量与 SONIC 发布训练环境一致。
- 所有安全检查不能因为“仿真模式”而整体失效；尤其是绝对关节速度、LowState 新鲜度和命令超时检查。

当前非 VR 数据链路为：

```text
键盘 / SONIC Planner / Reference Motion
    → Gear SONIC C++ Encoder / Policy
    → Unitree DDS rt/lowcmd
    → unitree_sim_isaaclab SonicDDSActionProvider
    → Isaac Lab JointPositionAction
    → G1 29DoF URDF / PhysX
    → Unitree DDS rt/lowstate + rt/secondary_imu
    → Gear SONIC C++ 下一控制周期
```

后续 PICO 数据链路预计为：

```text
OpenXR / PICO
    → pico_manager_thread_server.py
    → ZMQ manager input
    → Gear SONIC C++ Encoder / Policy
    → 同一套 DDS / Isaac Lab 29DoF 闭环
```

第二条链路本轮没有进行端到端验证，不能把当前结论表述为“OpenXR 已通过”。

## 2. 当前任务和机器人资产

### 2.1 本体 29DoF 基线任务

本阶段应使用：

```text
Isaac-G1-29DoF-Sonic
```

它直接加载 SONIC 发布训练环境使用的 URDF：

```text
/home/nolovr/GR00T-WholeBodyControl/
gear_sonic/data/assets/robot_description/urdf/g1/main.urdf
```

默认也可以通过环境变量覆盖 GR00T 根目录：

```bash
export GR00T_WBC_ROOT=/home/nolovr/GR00T-WholeBodyControl
```

实际运行诊断结果：

- 29 个本体关节；
- 30 个刚体；
- 总质量约 34.394 kg；
- 自由基座；
- 训练 URDF 的自碰撞开启；
- Root 初始位置 `(0, 0, 0.76)`；
- Root 初始四元数 `(1, 0, 0, 0)`，顺序为 `wxyz`。

### 2.2 Dex3 任务只保留给后续阶段

仓库仍注册：

```text
Isaac-G1-29DoF-Dex3-Sonic
```

该任务使用 29DoF 本体加 14DoF Dex3 的 whole-body USD，手指默认保持张开/零位。它没有作为本轮 29DoF 基线结论的依据，原因包括：

- Dex3 USD 比训练 URDF 多出手部刚体和质量；
- 整体质量、惯量、自碰撞开销和接触行为不再与训练环境完全一致；
- 本轮需求已经明确先验证纯 29DoF，再进入 OpenXR 和夹爪映射阶段。

因此，当前启动命令不要把 Dex3 任务当成“更完整的第一阶段任务”。

## 3. 与 SONIC 训练环境对齐的动力学配置

### 3.1 仿真时序

```text
physics dt = 0.005 s
decimation = 4
environment/control dt = 0.020 s
control frequency = 50 Hz
```

`sim_main.py` 会检查 SONIC 任务的墙钟控制频率是否与 `sim.dt × decimation` 一致。默认额外目标插值和单步限幅均关闭：

```text
sonic_ramp_seconds = 0.0
sonic_max_target_step = 0.0
```

原因是 SONIC C++ 自己已经执行 INIT 初始姿态过渡；Isaac 侧再加 2 秒 ramp 或每步 0.1 rad 限制会改变策略实际看到的闭环响应。

### 3.2 初始关节姿态

```text
左右 hip pitch       -0.312 rad
左右 knee             0.669 rad
左右 ankle pitch     -0.363 rad
左右 elbow            0.600 rad
left shoulder pitch   0.200 rad
left shoulder roll    0.200 rad
right shoulder pitch  0.200 rad
right shoulder roll  -0.200 rad
其余本体关节          0.000 rad
```

### 3.3 执行器参数

参数与 `gear_sonic/envs/manager_env/robots/g1.py` 和 C++ `policy_parameters.hpp` 使用同一组公式：

```text
natural frequency = 10 Hz × 2π
damping ratio = 2.0（过阻尼，不是临界阻尼）
stiffness = armature × natural_frequency²
damping = 2 × damping_ratio × armature × natural_frequency
```

基础电机参数为：

| 电机参数组 | Armature | Stiffness | Damping |
|---|---:|---:|---:|
| 5020 | 0.003609725 | 14.250623 | 0.907223 |
| 7520-14 | 0.010177520 | 40.179238 | 2.557890 |
| 7520-22 | 0.025101925 | 99.098428 | 6.308802 |
| 4010 | 0.004250000 | 16.778327 | 1.068142 |

脚踝以及腰部 roll/pitch 按发布训练配置使用 5020 参数的 2 倍。各关节 effort/velocity limit、armature、stiffness 和 damping 也按发布配置分组设置。

### 3.4 PhysX 和接触配置

- 训练 URDF 自碰撞开启；
- articulation solver position iterations 为 8；
- articulation solver velocity iterations 为 4；
- 刚体线性/角阻尼为 0；
- `max_depenetration_velocity = 1.0`；
- 地面静摩擦和动摩擦均为 1.0；
- friction/restitution combine mode 均为 `multiply`；
- 不再保留旧桥接场景中额外覆盖的 bounce threshold 和 friction correlation distance。

## 4. 29DoF DDS 协议顺序

LowCmd 和 LowState 的前 29 个 motor slot 统一使用 Unitree G1 硬件顺序。Isaac articulation 内部顺序不能直接假定等于 DDS 顺序，必须通过关节名显式映射。

```text
00 left_hip_pitch_joint
01 left_hip_roll_joint
02 left_hip_yaw_joint
03 left_knee_joint
04 left_ankle_pitch_joint
05 left_ankle_roll_joint

06 right_hip_pitch_joint
07 right_hip_roll_joint
08 right_hip_yaw_joint
09 right_knee_joint
10 right_ankle_pitch_joint
11 right_ankle_roll_joint

12 waist_yaw_joint
13 waist_roll_joint
14 waist_pitch_joint

15 left_shoulder_pitch_joint
16 left_shoulder_roll_joint
17 left_shoulder_yaw_joint
18 left_elbow_joint
19 left_wrist_roll_joint
20 left_wrist_pitch_joint
21 left_wrist_yaw_joint

22 right_shoulder_pitch_joint
23 right_shoulder_roll_joint
24 right_shoulder_yaw_joint
25 right_elbow_joint
26 right_wrist_roll_joint
27 right_wrist_pitch_joint
28 right_wrist_yaw_joint
```

当前实现由 `robots/g1_joint_order.py` 提供唯一 Python 协议定义，并同时用于：

- `rt/lowcmd` → Isaac articulation position target；
- Isaac articulation joint state → `rt/lowstate.motor_state[0:29]`。

启动日志会逐个输出 `motor[i] -> joint[j] name`，并要求最终显示 `Body mapping 29/29`。若任何关节缺失或重复，ActionProvider 会直接报错，而不是静默串关节。

## 5. DDS、IMU 和闭环频率

### 5.1 Isaac 专用 DDS profile

本轮为 GR00T `deploy.sh` 新增了 `isaac` profile：

```text
接口          lo
DDS domain    1
LowState CRC  开启
生命周期标记  开启
```

原有 profile 保持语义：

| Profile | 典型用途 | DDS domain | CRC |
|---|---|---:|---|
| `sim` | MuJoCo | 0 | 关闭 |
| `isaac` | 本项目 Isaac Lab | 1 | 开启 |
| `real` | 真机 | 0 | 开启 |

连接 Isaac 时必须使用命令末尾的 `isaac`，不能继续使用原来的 `sim`。使用 `sim` 会导致 domain、CRC 和生命周期协议不匹配，Isaac Root 也不会收到 A2 CONTROL 标记。

### 5.2 状态发布

Isaac 侧发布：

- `rt/lowstate`：29 个关节位置、速度、估计/应用力矩以及 pelvis IMU；
- `rt/secondary_imu`：torso link 的四元数、角速度和加速度；
- LowState CRC：由 Unitree Python SDK 的 CRC 实现计算；
- DDS 发布线程目标频率：100 Hz；
- PhysX/状态样本更新频率：50 Hz。

SONIC policy 实际使用的关键基座观测包括：

- pelvis/root 四元数；
- pelvis/root body-frame 角速度；
- 由四元数计算的重力方向历史；
- 29 个关节状态和上一动作历史。

四元数在 Isaac、Unitree HG 消息和 SONIC 中统一按 `[w, x, y, z]` 处理。

## 6. 启动生命周期和安全行为

### 6.1 生命周期标记

在 `isaac` profile 中，C++ 把桥接状态写入 LowCmd 的 `mode_machine`：

| 标记 | 数值 | 含义 | Isaac 行为 |
|---|---:|---|---|
| INIT | `0xA0` | C++ 初始姿态过渡 | 接收关节目标，但固定初始 Root |
| WAIT | `0xA1` | 等待操作者进入控制 | 继续固定 Root |
| CONTROL | `0xA2` | 正式闭环控制 | 永久释放 Root 给 PhysX |

这解决了启动时的循环依赖：C++ 必须先收到 LowState 才能执行 INIT，而 Isaac 若提前释放自由基座，会在模型初始化完成前倒地。

### 6.2 命令和状态安全检查

当前保护包括：

- Isaac LowCmd 默认超时为 0.10 秒；超时后保持最后一个安全 position target；
- LowCmd 少于 29 个 position 或 kp 值时拒绝使用；
- LowCmd 包含 NaN/Inf 时拒绝使用；
- SONIC 正常退出时会发送 `kp=0` 的 damping-only packet；Isaac 识别该数据包并保持最后目标，不把其中的 `q=0` 误解释为“29 个关节全部打到零位”；
- C++ 对 LowState 和 secondary IMU 都执行存在性与 500 ms 新鲜度检查；
- C++ 对所有关节执行 `abs(dq) > 35 rad/s` 安全检查，此检查不再因为关闭 CRC 而失效；
- 正式 CONTROL 后不再固定 Root；Root 释放是单向状态转换。即使 SONIC 进程随后重启并重新发送 INIT/WAIT 标记，Isaac 也只保持最后安全关节目标，不会再次写 Root pose 或把机器人传送回出生点；只有新的 CONTROL 标记到达后才恢复接收目标。

### 6.3 退出顺序

推荐测试结束时：

1. Planner 模式下按 `r`，清除移动动量并回到 IDLE；
2. 等机器人恢复稳定；
3. 按 `o` 正常退出 GR00T C++；
4. 在 Isaac 终端按 `Ctrl+C`；
5. 确认日志出现 ActionProvider、Controller、DDS publish thread 和 simulation application 的正常清理信息。

不要依赖旧实现中按进程名批量 `SIGKILL` 的清理方式；当前退出逻辑集中在正常 `finally` 路径中。

## 7. 关键代码实现

### 7.1 Isaac 桥接仓库

| 文件 | 作用 |
|---|---|
| `tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py` | 注册纯 29DoF 和后续 Dex3 两个任务；纯 29DoF 任务加载发布训练 URDF，并对齐时序、执行器、摩擦和自碰撞；显式复位时同时清除旧关节目标 |
| `action_provider/action_provider_sonic_dds.py` | 将 29 个 SONIC LowCmd position target 显式映射到 articulation；处理生命周期、命令数值校验、超时、damping-only packet、Root 单向释放、倒地复位后的安全再接管和运行指标 |
| `robots/g1_joint_order.py` | Python 侧唯一的 G1 29DoF DDS 硬件顺序 |
| `tasks/common_observations/g1_29dof_state.py` | 按同一协议回传 29DoF LowState，并生成 pelvis/torso IMU |
| `dds/g1_robot_dds.py` | LowCmd CRC 接收、LowState CRC 发布、secondary IMU 发布和频率统计 |
| `dds/dds_master.py` | 可配置 domain/interface、100 Hz 发布调度和正常清理 |
| `layeredcontrol/robot_control_system.py` | 基于 monotonic deadline 的稳定墙钟限频 |
| `sim_main.py` | SONIC 任务默认 50 Hz、domain 1/lo、无相机服务、真正关闭渲染的 `--no_render`、自动倒地检测/复位以及集中式信号退出 |
| `tools/diagnose_sonic_model.py` | 输出关节数、刚体数、质量、惯量和关键 frame 诊断 |
| `tools/monitor_sonic_tracking.py` | 订阅 C++ ZMQ debug 流，测量参考动作与 PhysX 实测关节的误差、频率、丢帧和滞后 |

### 7.2 GR00T/SONIC 仓库

| 文件 | 作用 |
|---|---|
| `gear_sonic_deploy/deploy.sh` | 新增 `isaac` profile、DDS domain/CRC/init duration/initial motion/frame 参数 |
| `g1_deploy_onnx_ref.cpp` | 可配置 DDS domain、生命周期标记、双状态流新鲜度检查、绝对关节速度安全检查和显式初始 motion/frame |
| `keyboard_handler.hpp` | 只有控制状态机实际进入 CONTROL 后才允许开启 planner；输出 motion set、mode、速度和高度变更日志 |
| `localmotion_kplanner.hpp` / `localmotion_kplanner_onnx.hpp` | 提供 0–26 planner 协议值的唯一名称映射；修正旧日志把 mode 4 错写为 BOXING 的问题；重新初始化前释放 alias 输入 buffer 的 Ort tensor，修复 planner 二次启用 |
| `motion_data_reader.hpp` | 对 motion 文件夹排序，使默认索引和初始动作可重复 |
| `CMakeLists.txt` | GoogleTest 改为可选，默认构建不再依赖联网下载测试框架 |

## 8. 推荐启动流程

建议先启动 Isaac，再启动 SONIC C++。C++ 首次加载 TensorRT/ONNX 模型可能持续一段时间；期间 Isaac 输出“等待 LowCmd”是正常的，不表示 DDS 一定失败。

### 8.1 终端 A：启动纯 29DoF Isaac 基线

```bash
cd /home/nolovr/Documents/unitree_sim_isaaclab

UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
/home/nolovr/IsaacLab/isaaclab.sh -p sim_main.py \
  --task Isaac-G1-29DoF-Sonic \
  --robot_type g129 \
  --action_source sonic_dds \
  --device cpu \
  --no_render \
  --stats_interval 5 \
  --profile_interval 250
```

当前单机器人配置推荐 `--device cpu`：PhysX 使用 CPU，GPU 留给 C++ TensorRT/ONNX Runtime，实测更容易稳定保持 50 Hz。

关键启动日志应包括：

```text
[DDS Config] domain=1, interface=lo
[sim] control timing: physics_dt=0.005000s, decimation=4, ... step_hz=50
[sonic_dds] Body mapping 29/29
[fall_reset] enabled: tilt>=60.0deg or base_z<=0.350m ...
```

纯 29DoF SONIC 任务默认开启自动倒地恢复。默认判据和恢复时序为：Root 倾角达到 60°，或 Root 高度低于等于 0.35 m，连续保持 0.5 秒后触发；复位到默认站姿后固定 Root 1 秒，只接受复位后新到达的 `0xA2` CONTROL 包，再用 1 秒从默认关节姿态平滑混合回 SONIC 目标；整个恢复过程之后还有 2 秒额外检测冷却。

可调参数如下：

```text
--fall_reset_tilt_deg 60
--fall_reset_min_base_height 0.35
--fall_reset_debounce_seconds 0.5
--fall_reset_hold_seconds 1.0
--fall_reset_blend_seconds 1.0
--fall_reset_cooldown_seconds 2.0
```

`kneel`、`crawl`、`IDLE_LYING_FACE_DOWN` 等动作会主动进入低高度或大倾角姿态，测试这些动作时必须显式增加：

```bash
--no_auto_reset_on_fall
```

关闭自动检测不会破坏手动 reset；SONIC 任务的手动 reset 仍会清除旧目标并走安全站姿恢复流程。

### 8.2 终端 B：启动非 VR 键盘 SONIC

本轮验证使用了显式 motion 和 frame，避免文件系统顺序或默认第 0 帧改变启动姿态：

```bash
cd /home/nolovr/GR00T-WholeBodyControl/gear_sonic_deploy

bash deploy.sh \
  --input-type keyboard \
  --output-type zmq \
  --initial-motion walking_quip_360_R_002__A428 \
  --initial-frame 24 \
  isaac
```

看到 `Init Done` 后：

```text
]       进入 CONTROL，Isaac 收到 A2 后释放 Root
Enter   开启/关闭 planner
1       在 standing motion set 中选择 SLOW_WALK
w       前进；当前最小速度会被限制为 0.2 m/s
r       Planner 紧急停步并清除移动动量
n       从 standing 切换到 squat motion set，默认 IDLE_SQUAT、高度 0.8 m
-       每次降低目标高度 0.1 m，最低限制为 0.2 m
=       每次升高目标高度 0.1 m，最高限制为 0.8 m
p       从 squat 返回 standing motion set
o       正常退出程序
```

重要安全顺序：先按 `]`，等待 C++ 日志确认进入 CONTROL、Isaac 日志确认收到 A2 并释放 Root，再按 Enter 开 planner。代码检查的是状态机已经实际进入 CONTROL，而不只是“曾经按过 `]`”。蹲下和起身建议每次只按一次 `-` 或 `=`，待机器人稳定后再进入下一高度；本轮实际按 0.8、0.7、0.6、0.5 m 和反向顺序完成验证。

### 8.3 参考动作跟踪监控

当前 `config/g1_udp_network.env` 的本机机器人 ID 为 1，因此 deploy 输出 topic 为 `g1_1_debug`。如果以后切换机器人 ID 或手动覆盖 topic，应以 `deploy.sh` 启动摘要里的 `ZMQ Output` 为准。

```bash
cd /home/nolovr/Documents/unitree_sim_isaaclab
source /home/nolovr/GR00T-WholeBodyControl/.venv_teleop/bin/activate

python -u tools/monitor_sonic_tracking.py \
  --host localhost \
  --port 5557 \
  --topic g1_1_debug \
  --warmup 2 \
  --duration 30 \
  --report-interval 5 \
  --timeout 10 \
  --max-lag-frames 10
```

该工具比较：

```text
body_q_target   = C++ 当前 reference motion 的 29DoF 目标
body_q_measured = Isaac/PhysX 反馈回 C++ 的 29DoF 实测位置
```

它与 Isaac 日志中的 `pd_target_mae` 不是同一个指标。`pd_target_mae` 是实际关节位置到 policy 虚拟 PD 平衡点的位移；机器人为了支撑重量和平衡，尤其在脚踝处需要非零弹簧位移，因此不能把它当作 reference tracking error。

### 8.4 后续 PICO/ZMQ 输入启动方式

用户提供的 PICO manager 流程应把原来的 `sim` profile 改为 `isaac`：

终端 B：

```bash
cd /home/nolovr/GR00T-WholeBodyControl/gear_sonic_deploy
bash deploy.sh --input-type zmq_manager --zmq-host localhost isaac
```

终端 C：

```bash
cd /home/nolovr/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5556
```

这组命令只记录为下一阶段入口。本轮没有连接 OpenXR/PICO 设备，也没有验证手柄按键、手指数据或夹爪映射。

## 9. 本轮实际验证结果

### 9.1 静态和构建检查

- Isaac 修改文件通过 Python AST 语法检查；
- 两个仓库均通过 `git diff --check`；
- `deploy.sh` 通过 `bash -n`；
- GR00T `just build` 全量构建通过；
- 任务实际加载为 29 joints / 30 bodies；
- 启动日志确认全部 29 个 LowCmd/LowState 关节映射正确；
- Planner 在同一进程中执行“启用 → 禁用 → 再启用”，第二次初始化成功，验证了 Ort alias buffer 的清理修复；
- ActionProvider 状态机独立测试通过：A2 后再次收到 A0 时保持最后安全目标且不再写 Root；NaN `kp`、超时命令和 damping-only 命令均被拒绝或安全保持；
- 最终补丁版端到端冒烟约 366 秒：CONTROL 前按 Enter 得到 `Wait for the CONTROL state...` 并拒绝 planner；A2 后 planner 正常初始化，随后完成 0.2 m/s 前进、`r` 停步、0.7 m 蹲下、恢复站立和正常退出；
- 最终冒烟第一次启动曾在 Kit 启动约 0.28 秒、尚未加载任务代码时，于 Isaac Sim telemetry/crashreporter 原生插件内发生一次段错误；确认无残留进程和资源不足后立即重试成功，后续 366 秒运行正常。该现象暂记为一次未复现的 Isaac Sim 启动瞬态，不作为 SONIC 控制链故障计数。

### 9.2 频率和 DDS

一次完整键盘回归持续约 780 秒：

- Isaac 主控制循环 overall average：50.00 Hz；
- 最近 100 帧 moving average：约 50.00 Hz；
- LowState 发布：约 98.8–99.2 Hz；
- secondary IMU 发布：约 98.8–99.2 Hz；
- C++ 持续收到 LowState 和 torso IMU，没有触发丢失状态安全停止；
- 120 秒混合状态 ZMQ debug 监控收到 6001 个样本，50.00 Hz，`missing_index_steps=0`；
- 25 秒深蹲专项监控收到 1251 个样本，50.00 Hz，`missing_index_steps=0`。

### 9.3 启动和站立

- INIT `0xA0`、WAIT `0xA1`、CONTROL `0xA2` 顺序正确；
- A2 前 Root 保持初始状态；
- A2 后 Root 释放给 PhysX；
- 启动瞬态最大倾角约 5.08°，最低 base z 约 0.732 m，随后恢复；
- 不同回归窗口的稳态站立倾角约 1.2–2.1°，base z 稳定在约 0.785–0.787 m；
- 完成深蹲并起身后，倾角稳定在约 2.0°，base z 约 0.785–0.786 m。

### 9.4 低速前进行走

测试动作：Planner `SLOW_WALK`，前进速度 0.2 m/s。

Isaac/PhysX 侧：

- 未跌倒；
- 动态窗口最大倾角约 6.25°；
- 动态窗口最低 base z 约 0.774 m；
- 停步后先恢复到倾角均值 1.36°、最大 2.61°，随后稳定到约 1.18°；
- base z 恢复到约 0.787 m。

C++ reference tracking 8 秒窗口：

```text
samples                  401
receive rate             50.00 Hz
missing index steps      0
joint MAE                0.0675 rad
joint RMSE               0.0931 rad
joint P95                0.2010 rad
joint max                0.3567 rad

left leg MAE             0.0528 rad
right leg MAE            0.0598 rad
waist MAE                0.0670 rad
left arm MAE             0.0824 rad
right arm MAE            0.0718 rad

base orientation error   2.23° mean / 4.59° max
best lag                 2 frames，约 40 ms
lag-corrected MAE        0.0671 rad
```

窗口前半段的 reference target activity 均值约 0.225 rad/s，完整 8 秒均值约 0.122 rad/s，最大约 3.816 rad/s，单关节覆盖范围最大约 0.902 rad。这说明窗口包含实际目标变化，不是纯静态站立数据。

覆盖站立、前进、停步和部分蹲下过渡的 120 秒混合窗口结果为：

```text
samples                  6001
receive rate             50.00 Hz
missing index steps      0
joint MAE                0.0768 rad
joint RMSE               0.1119 rad
joint P95                0.2305 rad
joint max                0.6244 rad
best lag                 2 frames，约 40 ms
lag-corrected MAE        0.0767 rad
```

### 9.5 IDLE_SQUAT 分级深蹲和起身

实际键盘序列：

```text
n     standing → squat motion set，IDLE_SQUAT (4)，height=0.8 m
-     height=0.7 m
-     height=0.6 m
-     height=0.5 m
=     height=0.6 m
=     height=0.7 m
=     height=0.8 m
p     squat → standing motion set，planner 输出回到 IDLE
```

日志中的 `IDLE_SQUAT (4)` 是 planner 协议的真实含义。旧 ONNX 日志只覆盖少数模式，曾错误地把数值 4 打印为 `BOXING`；本轮已把 0–26 的协议名称集中到唯一映射并在日志中同时输出名称和数值。

Isaac/PhysX 表现：

- 站立时 base z 约 0.785–0.787 m；
- 目标高度 0.7 m 时，base z 约 0.710–0.712 m，稳定倾角约 13.2–13.4°；
- 目标高度 0.6 m 时，base z 约 0.620–0.622 m，稳定倾角约 12–13°；
- 目标高度 0.5 m 时，base z 约 0.543–0.544 m，稳定倾角约 13.5°；
- 过渡过程最大倾角约 14.9°，最大关节速度约 1.6 rad/s；
- 0.5 m 深蹲可稳定保持，随后按 0.1 m 逐级起身，没有跌倒、NaN、DDS 丢失或安全停止；
- 起身回到 standing 后，base z 恢复到约 0.785–0.786 m，倾角约 2.0°。

25 秒深蹲专项 reference tracking：

```text
samples                  1251
receive rate             50.00 Hz
missing index steps      0
joint MAE                0.0940 rad
joint RMSE               0.1218 rad
joint P95                0.3102 rad
joint max                0.3832 rad

left leg MAE             0.0784 rad
right leg MAE            0.0840 rad
waist MAE                0.0912 rad
left arm MAE             0.1017 rad
right arm MAE            0.1093 rad

best lag                 1 frame，约 20 ms
```

该结果表明深蹲功能和稳定性已经通过，整体 MAE、RMSE、分组 MAE 和响应滞后处于本文候选范围内；但 P95 0.3102 rad 高于候选门槛 0.25 rad。因此不能把本次深蹲结果描述为“全部关节跟踪已经特别好”。误差主要来自深蹲平衡状态下的腰部和双臂参考姿态偏差，继续优化时应先区分 policy 为维持平衡主动偏离 reference 与桥接/执行器误差，不能直接通过随意提高增益破坏训练动力学对齐。

### 9.6 正常停止和安全降级

- `r` 成功把 planner 切回 IDLE 并清除 movement momentum；
- `o` 后 C++ 发送 damping-only packet 并正常退出；
- Isaac 连续识别到 `damping-only rt/lowcmd`，保持最后安全目标，没有把关节打到零位；
- Isaac `Ctrl+C` 后 ActionProvider、Controller、DDS 发布线程、共享内存和 simulation application 均正常清理；
- 退出后未发现残留的 `sim_main.py`、`g1_deploy_onnx_ref`、monitor 或 PICO server 进程。

### 9.7 自动倒地复位验证

2026-07-24 在隔离的 DDS domain 50、loopback 接口上完成了自动倒地恢复的端到端故障注入。测试发布器持续发送 CRC 正确、`mode_machine=0xA2`、但故意不稳定的 29DoF 目标，使机器人在 PhysX 中真实失稳；没有连接 PICO、真实机器人或日常测试 domain 1。

验证结果：

- Isaac 收到首个 A2 后正常释放 floating Root；
- 第一次在 `base_z=0.243 m`、倾角约 50°时，由低高度判据确认倒地；后续循环在倾角约 85–87°、`base_z≈0.075 m` 时同时命中倾角和高度判据；
- 每次确认倒地后均写回默认 Root pose/velocity、默认 joint position/velocity，并清除旧 joint position/velocity target；
- `env.reset(env_ids=...)` 成功清空 action、observation、termination、episode/history 等运行缓存并执行 simulation forward；
- 恢复期间旧 LowCmd 不会被重新应用，Root 保持默认站姿；只有复位开始时间之后新到达的有效 A2 才能重新释放 Root；
- 新 A2 到达后按 1 秒完成默认姿态到 SONIC 目标的连续混合；
- 因故障注入目标持续保持不稳定，系统按“保持 + 混合 + 冷却 + debounce”周期连续完成 4 次自动复位，没有卡死、NaN、串关节或控制循环中断；
- 最后一次复位后的观测为 `base_z≈0.682 m`、倾角约 1.9–2.5°，证明机器人实际恢复为直立状态，而不只是清除了内部状态标志；
- 故障注入期间主循环仍稳定在约 50.0 Hz。

该测试证明倒地检测、物理状态复位、控制缓存复位、旧命令隔离、站姿保持和 SONIC 平滑再接管已经形成闭环。它不代表故意不稳定的测试目标本身可以站立；持续发送同一故障目标时再次倒地并再次复位是预期行为。

## 10. 当前结论不能覆盖的内容

以下内容尚未验证：

- OpenXR/PICO 真实端到端链路；
- PICO 手柄按键；
- 夹爪或 Dex3 关节映射；
- `Isaac-G1-29DoF-Dex3-Sonic` 的动态性能；
- 后退、转向、侧移、不同速度行走；
- kneel、crawl、boxing、styled walking 等其余 planner 模式；
- 多个 reference motion 文件和不同初始 frame；
- 20–30 分钟长时间 soak；
- 故意断开 LowCmd、LowState 或 secondary IMU 的故障注入；
- GPU PhysX 模式下与 TensorRT 同时运行的性能稳定性。

另有两个指标解释限制：

1. C++ debug 流中的 `base_trans_measured` 目前仍是输出可视化层的固定占位值，不是真实 Isaac Root translation。因此本轮只使用 joint tracking 和 base quaternion error，不能用该字段评估位置跟踪。
2. `pd_target_mae` 是虚拟 PD 平衡点位移，不是 reference motion MAE；脚踝出现较大的稳态位移并不等同于脚踝 reference 跟踪失败。

所以准确表述应是：

> 纯 29DoF、非 VR 的阶段一功能验收已经完成：键盘可控制低速前进、停步、分级深蹲和分级起身，DDS/IMU/策略/PhysX 回路与安全退出均闭环通过。当前结果仍不能扩展解释为全部 planner 动作都已通过，也不能宣称 OpenXR 或夹爪控制已经完成；0.5 m 深蹲的 P95 跟踪质量还应继续改进。

## 11. 建议的下一步验收计划

### 11.1 先把监控变成可重复的验收

建议下一步先扩展 `monitor_sonic_tracking.py`：

- 支持 JSON/CSV 结果输出；
- 支持命令行阈值和非零退出码；
- 记录每个关节、每个分组、base tilt/z、数据新鲜度和恢复时间；
- 每个动作保存 motion、frame、planner mode、速度、持续时间和 Git commit；
- 形成一次命令可重复执行的非 VR 回归表。

可先采用以下“候选阈值”，跑完动作矩阵后再根据数据确认或调整：

| 指标 | 候选门槛 |
|---|---:|
| 控制平均频率 | 49.5–50.5 Hz |
| ZMQ missing index steps | 0 |
| reference joint MAE | ≤ 0.10 rad |
| reference joint RMSE | ≤ 0.13 rad |
| reference joint P95 | ≤ 0.25 rad |
| 每个大关节分组 MAE | ≤ 0.12 rad |
| 最佳响应滞后 | ≤ 3 帧 / 60 ms |
| 正常站立稳态倾角均值 | ≤ 2° |
| 普通行走最大倾角 | ≤ 10° |
| 停步后恢复到倾角 ≤ 3° | ≤ 2 秒 |
| 跌倒、NaN、CRC、安全停止 | 0 次 |

这些阈值是基于当前基线提出的工程建议，不是已经由产品需求确认的最终标准。

### 11.2 非 VR 动作矩阵

建议按风险递增执行：

1. 站立 5 分钟，不开启 planner；
2. reference motion 播放：选择 3–5 个不同动作和不同初始 frame；
3. SLOW_WALK：前进、后退、adjust left/right、heading left/right、侧移；
4. 速度阶梯：0.2、0.4、0.6 m/s，每档至少 60 秒；
5. squat set：`IDLE_SQUAT` 的 0.8→0.5→0.8 m 已通过；下一步测试 kneel，并对 0.5 m 深蹲的腰臂 P95 误差做专项分析；
6. boxing set：重点观察腰、肩、肘、腕；
7. styled walking：选择低风险样式开始；
8. 每种动作执行 `r` 停步，测量恢复时间；
9. 全部通过后进行 20–30 分钟混合动作 soak；
10. 最后做 LowCmd、LowState、secondary IMU 暂停/恢复的故障注入。

### 11.3 进入 OpenXR/夹爪阶段的门槛

只有在以下条件满足后，才建议开启阶段 9：

- 29DoF 动作矩阵全部无跌倒、无串关节、无异常安全退出；
- 站立、行走、腰臂动作和停步恢复均达到确认后的阈值；
- 长时间 soak 通过；
- 断流保护验证通过；
- Dex3 资产的额外质量、惯量、自碰撞和 50 Hz 性能单独重新评估。

然后再做：

1. 启用 PICO/ZMQ manager 输入但暂不控制手；
2. 验证 OpenXR 按键事件和状态机；
3. 定义夹爪开合或 Dex3 关节的唯一控制所有权；
4. 增加手部命令限位、速度限制、超时和 emergency open；
5. 最后做 OpenXR → PICO server → SONIC → DDS → Isaac 的端到端夹爪映射验证。

## 12. 常见问题排查

### Isaac 一直显示等待 LowCmd

- 确认 C++ 最终进入模型运行阶段；首次模型加载可能较慢；
- 确认使用 `isaac`，不是 `sim`；
- 两边均确认 `domain=1`、`interface=lo`；
- 检查 C++ 是否持续收到 `rt/lowstate` 和 `rt/secondary_imu`。

### 关节已经动，但 Root 一直被固定

- 查看 LowCmd `mode_machine` 是否进入 `0xA2`；
- 在键盘模式下需要看到 `Init Done` 后按 `]`；
- 如果 C++ 用 `sim` profile 启动，不会发布 Isaac 生命周期标记。

### monitor 没有数据

- 查看 `deploy.sh` 摘要中的 `ZMQ Output`；
- 当前 robot ID 1 的默认 topic 是 `g1_1_debug`；
- 端口默认 5557；
- monitor 必须在 GR00T `.venv_teleop` 中运行，确保 `pyzmq/msgpack/numpy` 可用。

### 仿真只有约 25 Hz

- 使用 `--no_render`；
- 不要同时指定非零 `--livestream_type`；
- 单机器人基线优先使用 `--device cpu`；
- 确认没有误用 43DoF Dex3 自碰撞任务作为纯 29DoF性能基线。

### 任务找不到 SONIC URDF

- 检查 `/home/nolovr/GR00T-WholeBodyControl` 是否存在；
- 或设置 `GR00T_WBC_ROOT`；
- 纯 29DoF 任务实际实例化时，Isaac URDF importer 会报告所选绝对路径。

## 13. Git 和工作区注意事项

两个仓库当前都已经由 Git 管理，但修改尚未在本文档中假定已经提交。

特别需要保留的用户现有改动：

- `/home/nolovr/GR00T-WholeBodyControl/config/g1_udp_network.env` 中 robot ID 已从 2 改为 1；
- `teleimager` 子模块的 dirty 状态；
- 运行 Isaac 后产生或更新的 `__pycache__` 文件。

这些内容不属于本轮 29DoF 实现本身，不应在整理提交时被误删或整体回退。`/home/nolovr/IsaacLab` 本轮没有代码修改。
