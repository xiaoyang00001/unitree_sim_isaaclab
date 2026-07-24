# Gear SONIC → Isaac Lab G1 本体 29DoF / 43DoF 载体非 VR 阶段交接与验证记录

> 文档日期：2026-07-24
> 当前系统：Ubuntu 22.04、Isaac Sim 5.1、Isaac Lab
> Isaac 桥接仓库：`/home/nolovr/Documents/unitree_sim_isaaclab`
> GR00T/SONIC 仓库：`/home/nolovr/GR00T-WholeBodyControl`
> Isaac Lab：`/home/nolovr/IsaacLab`
> 当前结论：非 VR 的 G1 本体 29DoF 闭环已经完成键盘慢走、停步、0.8→0.5 m 分级深蹲、0.5→0.8 m 分级起身和正常退出的历史验证。默认任务仍为“29 个 SONIC 本体关节 + 14 个保持默认张开姿态的 Dex3 关节”的 43DoF articulation；SONIC LowCmd 的 `mode/q/dq/tau/kp/kd` 全部接入 Isaac；手动/自动倒地复位及 reset 后 deploy 存活保护继续保留。当前代码已经回到 MuJoCo 第 3～7 项对齐之前的动力学/接触基线：左右脚恢复训练 URDF 的 7 个 cylinder/capsule 碰撞条，不再保留单平底 box；上肢/腰/踝 MuJoCo armature、frictionloss 映射、六 hip 统一 88 Nm、`enable_external_forces_every_iteration=True` 和 MuJoCo torso 合成惯量均未保留。PICO manager 保持 GR00T Git 原版本，不额外加入“允许更钝来换稳定”的新滤波；C++ policy/reference 和 GUI 均保持约 50 Hz，`LowState.tick` 只用于诊断。OpenXR 按键和夹爪映射仍留到后续阶段。

## 1. 本阶段需求与边界

当前优先级不是 OpenXR，也不是 Dex3 夹爪，而是先把 Gear SONIC 对 G1 本体 29 个电机关节的闭环控制在 Isaac Lab/PhysX 中验证好。

本阶段必须满足：

- SONIC 独占控制 G1 本体 29DoF：双腿 12、腰部 3、双臂和双腕 14。
- Isaac Lab 不再叠加内部 locomotion policy，不与 SONIC 抢腿部或腰部控制权。
- Floating Root 只由 PhysX 动力学和地面接触决定；进入正式控制后不能逐帧写 Root pose，也不能做 teleport 式动作回放。
- 启动期间可以暂时固定 Root，避免 DDS 发现、模型加载和初始姿态插值期间机器人自由倒地；收到 SONIC 的正式 CONTROL 标记后必须释放 Root。
- 非 VR 验证阶段在模型中保留 Dex3 14 个可动手指关节，但不接收手部命令，始终保持配置的默认张开姿态；不验证 PICO/OpenXR 按键和夹爪映射。
- 控制频率、URDF、初始姿态、执行器参数、摩擦和自碰撞设置应尽量与 SONIC 发布训练环境一致。
- 所有安全检查不能因为“仿真模式”而整体失效；尤其是绝对关节速度、LowState 新鲜度和命令超时检查。

当前非 VR 数据链路为：

```text
键盘 / SONIC Planner / Reference Motion
    → Gear SONIC C++ Encoder / Policy
    → Unitree DDS rt/lowcmd
    → unitree_sim_isaaclab SonicDDSActionProvider
    → Isaac Lab JointPositionAction + JointVelocityAction + JointEffortAction
      并动态写入 PhysX joint stiffness/damping
    → G1 43DoF articulation / PhysX（SONIC 只拥有其中 29 个本体关节）
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

### 2.1 默认 43DoF 载体任务

当前非 VR 主任务仍使用原任务名：

```text
Isaac-G1-29DoF-Sonic
```

任务名中的 `29DoF` 表示 SONIC/DDS 控制所有权仍是 G1 本体 29 个电机关节；Isaac articulation 本身现在包含 43 个可动关节：29 个本体关节和 14 个 Dex3 关节。阶段一的 14 个手指目标始终写回默认零位，不消费 PICO、OpenXR 或其他手部命令。

43DoF 适配模型以以下 URDF 为运动学、惯量、外观和 Dex3 可动关节来源：

```text
/home/nolovr/GR00T-WholeBodyControl/
gear_sonic/data/robots/g1/g1_29dof_with_hand_rev_1_0.urdf
```

外观网格全部解析到：

```text
/home/nolovr/GR00T-WholeBodyControl/
gear_sonic/data/robot_model/model_data/g1/meshes/
```

为避免牺牲已经验证过的本体控制表现，适配器同时从 SONIC 发布训练 URDF 原样复制 29 个身体关节的 effort/velocity limit 和身体碰撞几何：

```text
/home/nolovr/GR00T-WholeBodyControl/
gear_sonic/data/assets/robot_description/urdf/g1/main.urdf
```

生成器会严格检查两个源模型的 29 个身体关节父子链、关节原点、转轴和所有有惯量 link 的质量/惯量完全一致。生成结果为确定性临时文件，不修改 GR00T 的源 URDF：

```text
/tmp/unitree_sim_isaaclab_sonic_<uid>/
g1_29dof_with_hand_rev_1_0_sonic_isaaclab.urdf
```

当前模型特征：

- 43 个可动关节，其中 29 个由 SONIC DDS 控制，14 个 Dex3 关节保持默认姿态；
- 生成 URDF 在 PhysX 中的运行时总质量约 `34.39423 kg`，保持 43DoF 源 URDF/SONIC 训练模型的原始惯量，不再人为凑到 MuJoCo 的总质量；
- 固定辅助 link 合并后的运行时 `torso_link` 质量约 `7.81700 kg`，保持源 URDF 的质量、质心和惯量；此前实验性的 MuJoCo `9.598 kg` torso 合成惯量已经撤回；
- 自由基座，Root 初始位置 `(0, 0, 0.76)`；
- Root 初始四元数 `(1, 0, 0, 0)`，顺序为 `wxyz`；
- 使用训练 URDF 的身体碰撞几何；每只脚保留 7 个 cylinder，Isaac importer 在加载时把它们转换成 capsule；
- 阶段一移除手部碰撞，避免尚未验证的手指接触影响 29DoF 本体闭环；
- `Isaac-G1-29DoF-Dex3-Sonic` 保留为同一 43DoF 配置的兼容任务名。

默认也可以通过环境变量覆盖 GR00T 根目录：

```bash
export GR00T_WBC_ROOT=/home/nolovr/GR00T-WholeBodyControl
```

### 2.2 精确 29DoF 训练模型回归任务

原来的精确训练 URDF 没有删除，改为独立 A/B 回归任务：

```text
Isaac-G1-29DoF-Training-Sonic
```

该任务仍直接加载 SONIC 发布训练环境的 `main.urdf`，运行时为 29 个可动关节、无可动手指。因为每个关节都同时承载 `q/dq/tau`，ActionManager 总动作维度为 `29 × 3 = 87`。它用于定位“问题来自 43DoF 载体改造，还是来自 SONIC/PhysX 本体闭环”，不再是默认启动任务。

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

当前保持任务原始渲染间隔：每 4 个物理步渲染一次，即目标约 50 Hz。为准确回到第 3～7 项对齐前的代码，`rt/sim_state` 也恢复原有的每主循环发布行为，没有保留后来加入的 2 Hz 稳定化开关。`--no_render` 仍保留，只用于区分渲染负载与控制/动力学问题，不作为默认运行方式。

默认 43DoF 适配载体继续关闭整机 self-collision。该项不增加输入延迟，并能减少额外手部刚体带来的碰撞计算以及手臂靠近躯干时的离散接触冲量；精确 29DoF 训练 A/B 任务保持其训练设置。

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

启动/无命令时的 stiffness、drive damping 和 armature 按发布训练配置分组设置；收到有效 LowCmd 后，29 个本体关节的 stiffness/damping 会由该包的 `kp/kd` 动态覆盖。当前最终配置为：

- hip pitch、hip roll、knee 使用 7520-22 armature `0.025101925`；hip yaw 和 waist yaw 使用 7520-14 armature `0.010177520`；
- ankle pitch/roll 与 waist pitch/roll 使用发布训练配置的两倍 5020 armature，即约 `0.00721945`，不再统一改成 MuJoCo 的 `0.01`；
- shoulder、elbow、wrist roll 使用 5020 armature `0.003609725`，wrist pitch/yaw 使用 4010 armature `0.00425`；
- 29 个本体关节的 PhysX static/dynamic/viscous joint friction 均保持 `0`，不再直接把 MuJoCo `frictionloss` 数值套到 PhysX；
- 运行时 effort limit 为 hip pitch/roll `139 Nm`、hip yaw `88 Nm`、knee `139 Nm`；这与 SONIC 发布训练执行器配置和 C++ action scale 保持一致；
- 生成 URDF 仍逐项复制训练 `main.urdf` 的文件级 limit（其中 hip pitch 为 88 Nm），但 Isaac 运行时最终生效的是 `ImplicitActuatorCfg.effort_limit_sim`；真实 PhysX 诊断已经确认 hip pitch/roll 实际均为 139 Nm，因此不能只看生成 URDF 判断运行时上限；
- 默认 43DoF 任务和 `Isaac-G1-29DoF-Training-Sonic` 使用同一套本体执行器基线，差异集中在载体、手部和脚底接触，便于做有效 A/B；
- 目前仍未向本体关节额外加入 MuJoCo `joint damping=0.05`，避免把多个被动参数再次捆绑修改。

### 3.4 PhysX 和接触配置

- 任务基线保留训练 URDF 的自碰撞设置；默认稳定优先运行模式仅对 43DoF 适配载体在 spawn 前关闭整机 self-collision，29DoF 精确训练 A/B 任务仍保持开启；
- articulation solver position iterations 为 8；
- articulation solver velocity iterations 为 4；
- 默认 43DoF 任务和精确 29DoF 训练回归任务均保持 `enable_external_forces_every_iteration=False`；该开关是 PhysX/TGS 数值选项，不是 MuJoCo Newton solver 的等价配置，当前现场结果没有证明启用它有收益；
- 没有把 MuJoCo Newton 的 100 iterations 生硬映射成 PhysX iteration 数；两个求解器的一次迭代不等价，因此继续保留已经验证的 articulation `8/4`；
- `enable_stabilization=False`，因为当前物理步长仅 5 ms，Isaac Lab 不建议在这种小步长下用额外 stabilization 掩盖接触问题；
- 刚体线性/角阻尼为 0；
- `max_depenetration_velocity = 1.0`；
- 地面静摩擦和动摩擦均为 1.0；
- friction/restitution combine mode 均为 `multiply`；
- 默认 43DoF 任务恢复训练 URDF 每只脚的 7 个 cylinder 接触条；由于 URDF importer 使用 `replace_cylinders_with_capsules=True`，Isaac 运行时对应 7 个 capsule，不再使用 MuJoCo 单平底 box；
- 不再保留旧桥接场景中额外覆盖的 bounce threshold 和 friction correlation distance。

### 3.5 Dex3 阶段一配置

从 `g1_29dof_sonic_model12.yaml` 和 `g1_29dof_with_hand.xml` 选择性采用与 Isaac articulation 有直接对应关系的参数：

- 左右 thumb-0 effort limit 为 2.45 Nm；其余 12 个手指关节在阶段一使用更保守的 0.7 Nm 执行器限幅，源 URDF 的物理极限仍保留为 1.4 Nm；
- thumb-0 velocity limit 为 3.14 rad/s，其余手指为 12 rad/s；
- armature 为 0.01；
- MuJoCo `damping=0.05` 对应到 Isaac 的 viscous friction 0.05；
- MuJoCo `frictionloss=0.1` 对应到静/动态摩擦系数 0.1；
- 手指 position drive 采用保守的 stiffness 1.5、damping 0.1，只负责保持默认张开姿态。

没有照搬 YAML/XML 中与当前闭环不兼容的设置，例如固定基座、DDS domain 0、MuJoCo elastic band、MOTOR_KP/KD 或旧 Isaac USD 的 300 Nm 手部限幅。

### 3.6 第 3～7 项现场反馈与最终取舍

现场反馈表明：第 3/4 项后行走本身不再明显牵动身体，也没有动作变慢或关节粘滞，但双臂运动造成的上身晃动没有改善，停步后的回弹反而偏钝；继续加入第 5/6/7 项后，出现高速行走时制动不足、向前小步找平衡的问题，双臂带动身体仍没有改善。双臂问题在这些修改前已经存在，不能归因于新增配置，但第 4～7 项也没有证明能解决它。

最终逐项处理如下：

| 项目 | 当前处理 | 原因 |
|---|---|---|
| 3. 每只脚 7 个 capsule 改为单个平底 box | 撤回 | 按用户要求精确回到第 3～7 项对齐之前，恢复训练 URDF 原始脚底接触几何 |
| 4a. 上肢/腰/踝 armature 统一对齐 MuJoCo | 撤回 | 会偏离策略训练使用的等效电机惯量，未改善双臂反作用问题；恢复逐电机训练参数更可控 |
| 4b. 本体 `frictionloss` 数值直接映射为 PhysX joint friction | 撤回 | MuJoCo 与 PhysX 参数语义和求解方式并非一一等价，并且现场停步回弹变钝与该被动阻力高度相关 |
| 5. 六个 hip 全部限制为 88 Nm | 撤回 | hip pitch/roll 的训练值为 139 Nm；统一降到 88 Nm 后制动和平衡余量变差，与高速行走“刹不住”的反馈一致 |
| 6. `enable_external_forces_every_iteration=True` | 撤回 | 不是 MuJoCo solver 的等价项，未观察到明确收益，恢复默认 False 以减少额外变量 |
| 7. torso 合成质量/惯量改为 MuJoCo 值 | 撤回 | 破坏了 43DoF 源 URDF 与训练 URDF 的一致惯量，且没有改善双臂牵动身体；恢复运行时 torso 约 7.817 kg |

因此下一轮不再继续“整包复制 MuJoCo 参数”，而应在当前干净基线上结合完整 LowCmd 指标，分别判断手臂参考动作幅度、`kp/kd`、力矩饱和、腰部补偿和 torso 姿态响应。

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

- `rt/lowcmd.motor_cmd[0:29]` 的 `mode/q/dq/tau/kp/kd` → Isaac articulation 的 motor enable、position target、velocity target、feed-forward effort、stiffness 和 damping；
- Isaac articulation joint state → `rt/lowstate.motor_state[0:29]`。

ActionManager 使用 field-major 动作张量：

```text
[全部关节 q][全部关节 dq][全部关节 tau]
```

因此默认 43DoF 任务的 action dimension 为 `43 × 3 = 129`，训练回归任务为 `29 × 3 = 87`。`kp/kd` 不是普通 action 元素，而是只在数值变化时写入 PhysX joint drive，并同步 Isaac Lab implicit actuator 的 stiffness/damping 缓冲。执行器的等效命令为：

```text
tau_total = kp * (q_target - q) + kd * (dq_target - dq) + tau_ff
```

每个 motor 的 `mode=0` 会将该 motor 的 `tau/kp/kd` 置零，从而禁用驱动；`mode=1` 执行完整命令。这里的 motor `mode` 与桥接生命周期 `mode_machine=A0/A1/A2` 是两个不同字段。

当前 SONIC C++ active policy 会发布非零 `q/kp/kd`，但按现有实现把 `dq_target=0`、`tau_ff=0`；这两个字段为零是发布端策略设计，不是 Isaac 丢弃。Isaac 对非零 `dq/tau` 的映射与执行已有独立回归测试。

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

`rt/lowstate` 的 100 Hz 是网络刷新频率，不代表 PhysX 每 10 ms 都前进一次。当前约定为：

- 每个 50 Hz `env.step()` 完成后，观测端生成一个唯一 `sample_seq`；
- `sample_seq` 写入 `LowState.tick`；
- DDS 线程可以在两个物理样本之间重复发布最新状态，但重复包保持相同 tick；
- SONIC 任务的观测端不再使用“墙钟间隔至少 20 ms”的限速判断。该判断在调度器提前几百微秒时会跳过当前样本，实测曾把 50 Hz 混叠为约 31～32 Hz；现在 SONIC 每个环境步必定写入一个新样本。

`sample_seq/LowState.tick` 当前只用于诊断 100 Hz DDS 包中哪些是新环境状态、哪些是重复发布。现场验证表明，用 tick 直接门控 C++ policy/history/reference 会在 Isaac 稍慢或 PICO 流持续前进时积累明显延迟，因此该门控已经删除。C++ 恢复固定 50 Hz 控制循环；后续若需要解决状态重复问题，必须采用有界、可测量且不会让实时输入队列持续落后的方案。

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

- Isaac LowCmd 默认超时为 0.10 秒；超时后保持最后一个 `q/kp/kd`，但将可能继续驱动运动的旧 `dq/tau` 清零；
- LowCmd 的 `mode/q/dq/tau/kp/kd` 任一字段少于 29 项时拒绝使用；
- LowCmd 任一命令字段包含 NaN/Inf 时拒绝使用；motor mode 必须为整数 0/1，`kp/kd` 不能为负数；
- SONIC 正常退出时生成 `q=0,dq=0,tau=0,kp=0,kd=8` 的 damping-only packet；Isaac 会真实执行该包。由于 `kp=0`，`q=0` 不产生位置力矩，`kd=8` 对 `dq_target=0` 提供阻尼制动，而不是把 29 个关节主动打到零位；
- C++ 对 LowState 和 secondary IMU 都执行存在性与新鲜度检查；真机阈值仍为 500 ms，只有 `--isaac-sim` 对明确的 reset/渲染卡顿提供有界 5 秒容忍；
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
| `robots/g1_sonic_urdf.py` | 从 43DoF 源 URDF、训练 URDF 和指定 STL 目录生成确定性的 Isaac 适配 URDF；校验 29+14 关节集合、身体运动学/源惯量、限位和网格路径，并原样复制训练 URDF 的本体碰撞几何，不再改写脚底或 torso 惯量 |
| `tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py` | 注册默认 43DoF 载体任务和精确 29DoF 回归任务；对齐 SONIC 时序、训练执行器、自碰撞和阶段一 Dex3 参数；提供按 `q/dq/tau` 排列的三类 action term，并为 SONIC 状态观测显式关闭墙钟限速，保证每个环境步生成一个新 LowState tick |
| `action_provider/action_provider_sonic_dds.py` | 将 29 个 SONIC LowCmd `mode/q/dq/tau/kp/kd` 显式映射到 articulation；动态应用 PhysX drive gains，处理生命周期、完整命令校验、超时、damping-only packet、Root 单向释放、倒地复位后的安全再接管和运行指标 |
| `robots/g1_joint_order.py` | Python 侧唯一的 G1 29DoF DDS 硬件顺序 |
| `tasks/common_observations/g1_29dof_state.py` | 按同一协议回传 29DoF LowState，生成 pelvis/torso IMU，并为每个新 PhysX/环境步生成单调 `sample_seq` |
| `dds/g1_robot_dds.py` | LowCmd CRC 接收并完整保留 motor `mode/q/dq/tau/kp/kd`，发布 LowState CRC、secondary IMU 和频率统计；将 `sample_seq` 写入 `LowState.tick`，分别统计新 PhysX 样本与 100 Hz 重复发布；在明确执行 Isaac reset 的短暂窗口内抑制非物理的 `dq/tau_est` 瞬态 |
| `dds/dds_master.py` | 可配置 domain/interface、100 Hz 发布调度和正常清理 |
| `layeredcontrol/robot_control_system.py` | 基于 monotonic deadline 的墙钟限频；fallback action 按 ActionManager 总维度创建，支持 43DoF/129 维和 29DoF/87 维动作 |
| `sim_main.py` | SONIC 任务默认 50 Hz、domain 1/lo、无相机服务、基于 monotonic deadline 的不补帧调度、任务原始渲染频率、真正关闭渲染的 `--no_render`、自动倒地检测/复位以及集中式信号退出 |
| `tools/diagnose_sonic_model.py` | 输出关节数、刚体数、总质量、关键刚体质量、关节 effort/armature/friction 和 PhysX 外力更新开关 |
| `tools/monitor_sonic_tracking.py` | 订阅 C++ ZMQ debug 流，测量参考动作与 PhysX 实测关节的误差、频率、丢帧和滞后 |
| `tests/test_g1_sonic_urdf.py` | 验证适配模型严格为 29+14 可动关节、左右脚底为精确尺寸/位置的 MuJoCo box、手部阶段一无碰撞、限位正确、网格路径正确且生成过程不修改源文件 |
| `tests/test_g1_robot_dds_reset_grace.py` | 验证 reset 保护窗口只将 LowState 的 `dq/tau_est` 置零，关节位置和 IMU 仍实时发布，窗口结束后自动恢复实时速度和力矩 |
| `tests/test_sonic_dds_full_lowcmd.py` | 验证乱序 articulation 下 `q/dq/tau/kp/kd` 仍按关节名正确映射，motor mode 在混合期间仍硬禁用，超时会清除旧 `dq/tau`，并确认 damping-only packet 会执行而不是被 position-only 逻辑拦截 |

### 7.2 GR00T/SONIC 仓库

| 文件 | 作用 |
|---|---|
| `gear_sonic_deploy/deploy.sh` | 新增 `isaac` profile、DDS domain/CRC/init duration/initial motion/frame 参数 |
| `g1_deploy_onnx_ref.cpp` | 可配置 DDS domain、生命周期标记、双状态流新鲜度检查、绝对关节速度安全检查和显式初始 motion/frame；当前恢复固定 50 Hz policy/reference 推进，不再由 `LowState.tick` 降速 |
| `gear_sonic/scripts/pico_manager_thread_server.py` | 恢复仓库稳定版本：用相邻 PICO 样本对 50 Hz 目标时刻做姿态/关节插值，每次新源样本最多推进一个 streamed frame；不使用后来实验的多帧追赶式重采样 |
| `gear_sonic/utils/mujoco_sim/base_sim.py` / `unitree_sdk2py_bridge.py` | 输出 MuJoCo physics/viewer/fresh LowState/LowCmd/RTF/超期统计，明确区分“画面帧率低”与“物理或状态更新低” |
| `keyboard_handler.hpp` | 只有控制状态机实际进入 CONTROL 后才允许开启 planner；输出 motion set、mode、速度和高度变更日志 |
| `localmotion_kplanner.hpp` / `localmotion_kplanner_onnx.hpp` | 提供 0–26 planner 协议值的唯一名称映射；修正旧日志把 mode 4 错写为 BOXING 的问题；重新初始化前释放 alias 输入 buffer 的 Ort tensor，修复 planner 二次启用 |
| `motion_data_reader.hpp` | 对 motion 文件夹排序，使默认索引和初始动作可重复 |
| `CMakeLists.txt` | GoogleTest 改为可选，默认构建不再依赖联网下载测试框架 |

## 8. 推荐启动流程

建议先启动 Isaac，再启动 SONIC C++。C++ 首次加载 TensorRT/ONNX 模型可能持续一段时间；期间 Isaac 输出“等待 LowCmd”是正常的，不表示 DDS 一定失败。

### 8.1 终端 A：启动 43DoF 载体上的 29DoF SONIC 闭环

```bash
cd /home/nolovr/Documents/unitree_sim_isaaclab
source /home/nolovr/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab

UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
/home/nolovr/IsaacLab/isaaclab.sh -p sim_main.py \
  --task Isaac-G1-29DoF-Sonic \
  --robot_type g129 \
  --action_source sonic_dds \
  --device cpu \
  --stats_interval 5 \
  --profile_interval 250
```

当前单机器人配置推荐 `--device cpu`：PhysX 使用 CPU，GPU 留给 C++ TensorRT/ONNX Runtime。GUI 已恢复任务原始约 50 Hz 目标刷新，不再默认降为 12.5 Hz。只有在排查渲染负载时才临时增加 `--no_render` 做 A/B；该参数会取消画面，但不会改变物理步长。

关键启动日志应包括：

```text
[DDS Config] domain=1, interface=lo
[sim] control timing: physics_dt=0.005000s, decimation=4, ... step_hz=50
[sim] rendering: render_interval=4 physics steps (~50.00 Hz), self_collisions=False
[sonic_dds] Control ownership: articulation=43, G1 body=SONIC DDS (29), Dex3=default hold (14), root=PhysX
[sonic_dds] LowCmd execution: mode/q/dq/tau/kp/kd enabled; environment action=129 (q/dq/tau field-major)
[sonic_dds] Body mapping 29/29
[fall_reset] enabled: tilt>=60.0deg or base_z<=0.350m ...
```

运行后每约 5 秒应看到 Isaac DDS 状态诊断：

```text
[g1_robot] DDS publish: lowstate≈99Hz, secondary_imu≈99Hz, fresh_physx≈50Hz, repeats≈49Hz, ...
```

`lowstate` 高于 `fresh_physx` 是预期行为：前者是 DDS 保活/刷新，后者才是新环境状态。`LowState.tick` 目前只服务这组诊断，C++ 不再据此暂停 policy/reference。

如果 43DoF 载体出现异常，需要与训练模型做 A/B 对比，只把任务名替换为：

```text
Isaac-G1-29DoF-Training-Sonic
```

该回归任务的启动日志应显示 `articulation=29`、`Dex3=not articulated (0)` 和 action dimension 87。

所有专用 SONIC 任务（默认 43DoF 载体和 29DoF 回归任务）都默认开启自动倒地恢复。默认判据和恢复时序为：Root 倾角达到 60°，或 Root 高度低于等于 0.35 m，连续保持 0.5 秒后触发；复位到默认站姿后固定 Root 1 秒，只接受复位后新到达的 `0xA2` CONTROL 包，再用 1 秒从默认关节姿态平滑混合回 SONIC 目标；整个恢复过程之后还有 2 秒额外检测冷却。

可调参数如下：

```text
--fall_reset_tilt_deg 60
--fall_reset_min_base_height 0.35
--fall_reset_debounce_seconds 0.5
--fall_reset_hold_seconds 1.0
--fall_reset_blend_seconds 1.0
--fall_reset_lowstate_grace_seconds 1.0
--fall_reset_cooldown_seconds 2.0
```

Isaac 对 articulation 执行 reset 时会离散写入 Root/关节状态，这属于仿真状态跳转，不是电机真实运动。默认在 reset 前开启 1 秒 LowState 保护，并在 `env.reset()` 返回后重新开启 1 秒：关节位置、pelvis IMU 和 torso IMU 保持实时，`dq` 与 `tau_est` 临时发布为零。这样即使 reset 本身耗时较长，reset 返回后的首批 PhysX 样本仍受保护，可避免 pose teleport 的单帧速度瞬态误触发 C++ 的 `abs(dq) > 35 rad/s` 保护；正常运行时的 35 rad/s 阈值没有修改。启动日志会显示 `lowstate_grace=1.00s`，每次 reset 后会报告窗口样本数和窗口内观测到的原始最大关节速度。

reset 期间 Python/PhysX 还可能短暂停止发布整包 LowState/secondary IMU。C++ 在真机模式仍严格使用 500 ms 断流阈值；只有显式 `--isaac-sim` 模式把该阈值放宽到有界的 5 秒，并在 500 ms 时打印暂停告警、恢复时打印恢复日志。若 Isaac 确实退出或断流超过 5 秒，deploy 仍会执行安全退出。

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

PICO manager 当前与 GR00T Git 提交中的原文件一致，没有额外保留后来“允许更钝来换稳定”的修改。该原版本本身使用前后源样本对目标时刻做插值并维护 5 帧窗口；这是第 3～7 项对齐之前已有的行为，不属于本轮新加稳定化逻辑。

每约 5 秒应看到：

```text
[PoseLoop] FPS: ...，Step: ...
```

manager 只在设备时间戳推进时生成下一帧；退出并重新进入 pose 模式时会清空窗口并重新预填充 5 帧。这里保持 Git 原行为，没有追加新的断流重同步策略。

这组命令仍需要重新做真实 PICO 动态验证；尚未验证手柄按键、手指数据或夹爪映射。

## 9. 本轮实际验证结果

### 9.1 静态和构建检查

2026-07-24 完成第 3～7 项回退和完整 LowCmd 接入后的检查：

- 回退后关键 Python 文件通过语法检查；模型生成、DDS/reset 和完整 LowCmd 共 14/14 通过；GR00T C++ `just build` 全量构建通过，两个仓库 `git diff --check` 通过；固定时间网格重采样器及其 4 个测试已随高延迟方案一起删除；
- 真实 PhysX 审计确认：hip pitch/roll 为 139 Nm、hip yaw 为 88 Nm、knee 为 139 Nm；本体 armature 恢复 SONIC 训练分组，所有本体关节 static/dynamic/viscous friction 为 0；
- `enable_external_forces_every_iteration=False`，solver iterations 保持 8/4；运行时 `torso_link` 质量约 7.81700 kg，整机总质量约 34.39423 kg，均来自源 URDF，不再使用实验性的 MuJoCo 合成惯量；
- 默认 `Isaac-G1-29DoF-Sonic` 在真实 Isaac Lab 中完成创建、reset 和控制循环：43 joints、44 runtime bodies、129 维 action、14 个可动手指、29DoF 状态观测 87 维；
- `Isaac-G1-29DoF-Training-Sonic` 同样完成创建和诊断：29 joints、30 runtime bodies、87 维 action、0 个可动手指；
- 完整 `sim_main.py + SonicDDSActionProvider + DDS` 在隔离 domain 91 上运行，ActionManager 确认三个 action term 顺序严格为 `joint_pos/joint_vel/joint_effort`，29/29 身体映射和 14/14 Dex3 保持映射正确；
- 使用真实 `deploy.sh --input-type zmq_manager ... isaac` 联调时，Isaac 收到 C++ 实际发布的完整 LowCmd：`enabled=29/29`、`max|dq_target|=0`、`max|tau_ff|=0`、`kp=[14.2506,99.0984]`、`kd=[0.9072,6.3088]`；这验证了真实 DDS/CRC/字段传输以及动态 gain 写入；
- 该次 PICO manager 没有收到人体跟踪数据，C++ 停留在 A0/A1，未进入 A2，因此这不是默认 43DoF 动态动作矩阵验收；
- 曾在隔离 DDS domain 124 上验证过 tick 门控实验的 A2 CONTROL 和短时站立，但用户现场测试确认该方案没有改善平衡、行走或摆臂扰动，并产生明显延迟；该结果只作为已否决实验记录，不能代表当前回退版本的性能；
- 非零 `dq/tau`、motor mode 禁用和 `kp=0/kd=8` damping-only 执行使用独立测试包验证；当前 C++ active policy 的 `dq/tau` 本来就是零；
- 没有 SONIC 发布端时持续显示 `HOLD: waiting for the first rt/lowcmd`，这是启动保护的预期行为；
- Isaac/GR00T 两个源 URDF 的 SHA-256 在生成测试前后保持不变，STL 源目录也没有被修改；
- URDF importer 会对固定且带惯量的传感器、外观辅助 link 输出 merge deprecation warning；这是 Isaac Sim 5.1 的导入提示，不是关节数或 PhysX 创建失败。

以下条目和第 9.2–9.7 节是 43DoF 载体升级前、精确 29DoF 训练 URDF 的历史闭环基线，仍用于后续 A/B 回归：

- Isaac 修改文件通过 Python AST 语法检查；
- 两个仓库均通过 `git diff --check`；
- `deploy.sh` 通过 `bash -n`；
- GR00T `just build` 全量构建通过；
- 当时的训练回归任务实际加载为 29 joints / 30 bodies；
- 启动日志确认全部 29 个 LowCmd/LowState 关节映射正确；
- Planner 在同一进程中执行“启用 → 禁用 → 再启用”，第二次初始化成功，验证了 Ort alias buffer 的清理修复；
- 当时的 position-only ActionProvider 状态机独立测试通过：A2 后再次收到 A0 时保持最后安全目标且不再写 Root；NaN `kp`、超时命令和 damping-only 命令均被拒绝或安全保持。当前实现已升级为完整 LowCmd，damping-only 不再被保持逻辑拦截；
- 最终补丁版端到端冒烟约 366 秒：CONTROL 前按 Enter 得到 `Wait for the CONTROL state...` 并拒绝 planner；A2 后 planner 正常初始化，随后完成 0.2 m/s 前进、`r` 停步、0.7 m 蹲下、恢复站立和正常退出；
- 最终冒烟第一次启动曾在 Kit 启动约 0.28 秒、尚未加载任务代码时，于 Isaac Sim telemetry/crashreporter 原生插件内发生一次段错误；确认无残留进程和资源不足后立即重试成功，后续 366 秒运行正常。该现象暂记为一次未复现的 Isaac Sim 启动瞬态，不作为 SONIC 控制链故障计数。

### 9.2 频率和 DDS

一次完整键盘回归持续约 780 秒：

- Isaac 主控制循环 overall average：50.00 Hz；
- 最近 100 帧 moving average：约 50.00 Hz；
- LowState 发布：约 98.8–99.2 Hz；
- secondary IMU 发布：约 98.8–99.2 Hz；
- 修正 SONIC 观测墙钟混叠后，新 PhysX 状态样本：约 49.9–50.1 Hz；100 Hz LowState 中其余约 49 Hz 为同 tick 重复发布；
- 曾测试过的 C++ tick 门控实验为约 50 Hz 调度、48.2–49.8 Hz 新 tick 消费；该实验因现场延迟过大且稳定性无收益已经回退，当前 C++ 恢复每个 50 Hz 控制周期推进 policy/reference；
- C++ LowCmd writer：约 500 Hz；CONTROL 后 LowCmd 内容更新约 49 Hz；
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
- `o` 后 C++ 生成 damping-only packet 并正常退出；历史 position-only 桥接会识别后保持最后安全目标；
- 当前完整 LowCmd 桥接改为执行 `kp=0,kd=8,dq_target=0,tau_ff=0`，其中 `q=0` 因 `kp=0` 不产生位置力矩；该语义已由单元测试覆盖，真实 C++ 退出包仍需在下一轮动态联调中留出发送窗口并抓取确认；
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

#### 9.7.1 reset 瞬态触发 C++ 速度保护的修复回归

用户现场曾在倒地 reset 后看到：

```text
Error: abs(body_dq[14]) = 35.4468 > 35.
Error: Failed to gather robot state to logger in the middle of the control loop!
Stopping control system.
```

这里的 `body_dq[14]` 是 C++ 的 MuJoCo/URDF 顺序索引；经 `mujoco_to_isaaclab[14] == 10` 映射后，实际读取的是 DDS/硬件槽位 10，即 `right_ankle_pitch_joint`，不是腰部关节。根因是 Isaac reset 的离散状态写入可能在 LowState 中形成一个非物理的速度瞬态，而 C++ 无法仅凭该样本判断它是 teleport 还是实际关节超速，因此按设计执行了安全退出。

修复没有改动 GR00T C++ 的 35 rad/s 阈值。Isaac 侧只在明确进入手动或自动 reset 事务时，在 reset 前后各开启默认 1 秒的 LowState 保护窗口；正常站立、行走、动作播放和真实失稳期间仍发布实时 `dq/tau_est`，原速度安全检查完整保留。

后续现场又捕获到第二个独立边界：reset 期间整包状态暂停，deploy 最后一个正常周期的 LowState age 已达到约 352 ms，随后跨过原 500 ms 阈值并报告 `Lost LowState data connection`。因此新增了仅对 `--isaac-sim` 生效的 5 秒有界断流容忍；真机仍为 500 ms。该改动解决的是仿真 reset 的发布空窗，不会放宽真机状态断流保护。

2026-07-24 完成以下回归：

- 单元测试注入 29 个关节 `dq=42 rad/s`、`tau_est=7`：保护窗口内实际发布 `dq=0`、`tau_est=0`，位置和双 IMU 保持实时；窗口到期后 `dq=2`、`tau_est=3` 立即恢复实时发布；
- 隔离 DDS domain 95 上，真实 SONIC reference motion 运行期间关节速度达到约 13.76 rad/s 后执行手动 reset，完成站姿保持、新 A2 接管和平滑恢复，`gear_sonic_deploy` 始终存活；
- 隔离 DDS domain 96 上，将倾角阈值临时降为 1°进行自动路径故障注入；部署端已经处于 CONTROL 时，自动检测在倾角约 6.7°处触发，LowState 保护窗口覆盖约 49 个发布样本，随后恢复实时状态；部署端没有出现 `abs(body_dq)>35`，继续稳定运行并最终通过 `o` 正常退出；
- 隔离 DDS domain 97 上复核当前补丁：自动 reset 在 reset 前后重新计时，单次窗口覆盖约 98 个 LowState 样本并恢复实时 `dq/tau_est`；随后用 `SIGSTOP` 暂停 Isaac 主进程 1.2 秒，deploy 在 age=520 ms 时分别对 LowState 和 secondary IMU 打印一次 5 秒容忍告警，最大观测 age 约 1161 ms，恢复后两路 age 回到约 9 ms，deploy 保持 CONTROL 且最终通过 `o` 正常退出；
- 上述联调中 Isaac 主循环约 50 Hz，LowState/secondary IMU 约 99 Hz；测试结束后部署端和 Isaac 均按正常清理流程退出。

domain 96 的 1°阈值只用于强制覆盖自动 reset 分支，没有写入默认配置；正常启动仍使用 60°倾角、0.35 m 高度和 0.5 秒 debounce。

## 10. 当前结论不能覆盖的内容

以下内容尚未验证：

- OpenXR/PICO 真实端到端链路；
- PICO 手柄到 Isaac Dex3 的实际控制；
- 夹爪或 Dex3 关节映射的限位、超时和 emergency open；
- 默认 43DoF 载体已完成真实 C++ policy 的 A2 站立短测，但尚未重新验收 PICO 流、行走/停步、双臂大幅挥动等完整动态动作矩阵；
- 当前发布端策略固定为零的 `dq_target/tau_ff` 尚未做“C++ 实际发布非零值”的端到端注入；Isaac 执行路径已通过非零测试包验证；
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

> 精确 29DoF 训练模型上的非 VR 历史基线已经完成：键盘可控制低速前进、停步、分级深蹲和分级起身，DDS/IMU/策略/PhysX 回路与安全退出均闭环通过。当前默认 43DoF 载体保留 129 维 `q/dq/tau` 动作链、动态 `kp/kd`、完整 LowCmd 和倒地复位；动力学/接触已经回到 MuJoCo 第 3～7 项对齐之前，PICO 也保持仓库原代码。下一步需要在这个明确的回退版本上重新跑站立、行走、停步和双臂大幅挥动动作矩阵；OpenXR 和夹爪控制尚未完成。

## 11. 建议的下一步验收计划

### 11.1 当前重新判断：先解决控制时序，不再继续整包调动力学参数

现场结果已经否定“降低 GUI 刷新率、让 C++ 等待新 `LowState.tick`、在 PICO 端一次补齐多个固定网格帧”这一组合。当前没有保留这些修改，也没有在 PICO Git 原版本之上追加新的钝化/稳定滤波。

当前需要重点验证的结构差异是：

- MuJoCo 的 `SIMULATE_DT=0.005`，每 5 ms 读取一次最新 LowCmd，按当前关节状态重新计算完整 `tau_ff + kp(q_des-q) + kd(dq_des-dq)`，随后执行一次 `mj_step`；viewer 独立按 `VIEWER_DT=0.02` 更新。因此画面约 50 Hz 时，物理、执行器和新状态仍是 200 Hz；
- Isaac 同样使用 5 ms PhysX 步长，但当前 `decimation=4`。`SonicDDSActionProvider.get_action()` 只在每个 20 ms 环境步开始时读取一次最新 LowCmd，目标 `q/dq/tau/kp/kd` 随后保持 4 个 PhysX 子步；PhysX drive 仍会在每个子步依据当前状态计算反馈力矩，所以不能简单描述为“力矩只算 50 Hz”，但新目标、前馈力矩、动态增益和对外新状态的更新粒度目前只有 50 Hz；
- C++ policy/reference 本身仍应保持发布设计的 50 Hz。下一步不是把 policy 提到 200 Hz，而是让 Isaac 的执行器桥和状态采样能够每 5 ms 使用最新数据，使 50 Hz policy 输出以更小的附加等待进入物理闭环；
- `rt/sim_state` 当前按对齐前代码恢复为每主循环生成和发布；其开销继续由现有统计单独报告，不再作为本轮稳定化变量。

### 11.2 第一步：确认低延迟回退基线并测清端到端延迟

先不改动力学参数，只在当前回退版本上重复四个最能暴露问题的动作：站立、开始行走、行走中停止、双臂大幅上下摆动。每个动作同时记录以下单调时钟时间点和序号：

1. PICO 原始样本产生时间；
2. manager 完成 SMPL 转换和 ZMQ 发送时间；
3. C++ 收到/合并 streamed frame、实际消费的全局 frame；
4. policy 开始/结束时间与 LowCmd 内容更新时间；
5. Isaac DDS 接收 LowCmd 的时间和序号；
6. ActionProvider 读取该命令、目标写入 PhysX 的时间；
7. 对应 PhysX 子步后的关节状态、脚底接触和 Root 姿态；
8. GUI 实际显示该状态的时间。

必须分开报告 P50、P95、最大值，不能只看平均频率。特别要新增两个 backlog 指标：`最新 ZMQ frame - C++ 当前消费 frame`，以及 `最新 LowCmd - Isaac 当前应用 LowCmd`。这样才能区分延迟来自 PICO/stream merger、policy、DDS、Isaac 命令保持还是单纯画面刷新。

回退基线的最低确认项为：

- 启动日志不再出现按新 tick 门控 policy 的提示；
- PICO 使用稳定版单帧插值，不存在一次生成多个追赶帧；
- GUI 默认 `render_interval=4`，目标约 50 Hz；
- 用户体感延迟恢复到本轮稳定优先实验之前；
- 完成站立、开始走、停止和摆臂各 3 次记录，作为后续 A/B 的同一基线。

### 11.3 第二步：验证 200 Hz Isaac 执行器/状态桥假设

先做一个短时、可完全回退的 A/B 原型：

- PhysX 仍为 200 Hz、`dt=0.005`；
- C++ policy/reference 仍为 50 Hz，LowCmd DDS writer 保持原频率；
- Isaac 临时使用 `decimation=1`、控制器 200 Hz，每 5 ms 读取最新 LowCmd 并推进一个 PhysX 步；
- GUI 每 4 个物理步渲染一次，仍约 50 Hz，不把画面频率与控制频率绑在一起；
- `rt/sim_state` 在该 A/B 中关闭或降到 1–2 Hz；奖励、调试打印和不必要的数据导出也要限频；
- LowState/secondary IMU 保持现有 DDS 对外频率也可以，但每次发布必须取最新 5 ms PhysX 状态，不能重复旧的 20 ms 环境样本。

这个原型的目标不是直接作为最终架构，而是验证“缩短最新状态/最新目标进入物理闭环的等待”能否改善停步收脚、脚底抖动和摆臂时下肢稳定。当前电脑若无法承受 200 Hz 的完整 Python manager 调用，则用 `--no_render` 做短时动力学 A/B，只判断假设，不据此评估画面。

若原型有明确收益，再实现低开销正式版本：外层仍维持 50 Hz manager bookkeeping，但在内部 4 个 PhysX 子步前读取最新 LowCmd、更新目标和动态增益，并在每个子步后刷新给 DDS 的最新机器人状态。这样保留 200 Hz 执行器桥，又避免把 observation/reward/termination/场景序列化等 Python 管理逻辑全部放大到 200 Hz。

若 200 Hz A/B 对停止和下肢抖动没有可重复收益，则立即撤回，不继续围绕该假设调参。

### 11.4 第三步：单独处理 streamed motion 的积压，不在生产端等待未来帧

PICO 人体跟踪约 35–45 Hz，而 C++ reference 消费为 50 Hz；当前每包还携带 5 帧窗口，`StreamedMotionMerger` 保留历史帧并有 catch-up 逻辑。这里可能造成“新姿态已经到达，但 policy 仍消费旧全局 frame”的额外延迟。

下一步先只增加 lag 观测，不立刻改变合并算法。如果确认长期积压，再在 C++ 消费端采用有界滞后策略，例如把允许落后限制在 1–2 个 reference frame，并在超限时受控快进。当前保留的生产端相邻样本插值每次最多生成一帧；不要恢复一次 manager 循环补发多个过期帧。任何快进策略都必须同时限制单帧关节目标变化，避免用降低延迟换来上肢冲击。

### 11.5 第四步：时序明确后再做单变量动力学 A/B

每次只改一个变量，并用相同 motion、初始 frame、动作时间和速度重复测试：

1. 先记录脚底切向速度、左右接触力/压力中心、Root pitch/roll、关节目标/实测和实际力矩饱和率；
2. 判断停步问题是 reference 仍在前进、hip/knee/ankle 力矩饱和、脚底滑动，还是上身角动量没有被腰腿补偿；
3. 只在有证据时分别测试 torso 质量/惯量、被动关节阻力、单个电机组 effort limit 或接触参数；
4. 已经失败的“整组 armature/friction/hip limit/torso inertia 一次性复制”不得重新引入；
5. 当前训练脚底碰撞和 43DoF 关闭整机自碰撞作为回退基线保持不变；任何重新对齐都必须单变量进行。

### 11.6 完成上述两类问题后再跑非 VR 动作矩阵

稳定方案确定后，扩展 `monitor_sonic_tracking.py` 支持 JSON/CSV、命令行阈值和非零退出码，并按风险递增执行：

1. 站立 5 分钟；
2. SLOW_WALK 的前进、后退、转向和侧移；
3. 速度阶梯和每档停止恢复；
4. 双臂大幅上下摆动、boxing 和腰部动作，重点确认上肢扰动不会触发下肢连续小步或跌倒；
5. squat、kneel 和起身；
6. 3–5 个不同 reference motion 和初始 frame；
7. 20–30 分钟混合动作 soak；
8. LowCmd、LowState、secondary IMU 暂停/恢复和 reset 故障注入。

候选门槛仍包括：控制平均频率 49.5–50.5 Hz、ZMQ 不丢索引、reference joint MAE ≤ 0.10 rad、普通行走最大倾角 ≤ 10°、停步后 2 秒内回到倾角 ≤ 3°，并且跌倒、NaN、CRC 和异常安全停止均为 0。对于新增的 200 Hz 执行器桥，应另外验收 PhysX 子步频率、目标应用延迟 P95 和 streamed-frame lag，不能只用外层 50 Hz 平均值判定。

### 11.7 进入 OpenXR/夹爪阶段的门槛

只有在以下条件满足后，才建议开启阶段 9：

- 29DoF 动作矩阵全部无跌倒、无串关节、无异常安全退出；
- 站立、行走、腰臂动作和停步恢复均达到确认后的阈值；
- 长时间 soak 通过；
- 断流保护验证通过；
- 默认 43DoF 载体重新完成与精确 29DoF 基线相同的动作矩阵、跟踪阈值和 50 Hz 长时间验证。

然后再做：

1. 启用 PICO/ZMQ manager 输入但暂不控制手；
2. 验证 OpenXR 按键事件和状态机；当前 PICO server 已读取 trigger/grip 并生成 7DoF 手部目标，但 `generate_finger_data()` 的实际闭合判据只使用 `trigger > 0.5`，`grip` 目前仅被采集/透传，进入验收前必须明确最终映射；
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
- 用 `Isaac-G1-29DoF-Training-Sonic` 做 29DoF A/B 对比，区分载体差异和系统负载问题；当前 43DoF 默认任务在无 LowCmd 的完整启动检查中可稳定保持约 50 Hz。

### 任务找不到 SONIC URDF

- 检查 `/home/nolovr/GR00T-WholeBodyControl` 是否存在；
- 或设置 `GR00T_WBC_ROOT`；
- 检查 43DoF 源 URDF、训练 `main.urdf` 和 `model_data/g1/meshes` 是否同时存在；
- 默认任务会在 `/tmp/unitree_sim_isaaclab_sonic_<uid>/` 生成适配 URDF，源文件缺失时 Isaac URDF importer 会报告最终选择的绝对路径；
- 精确 29DoF 回归任务仍直接加载训练 `main.urdf`。

## 13. Git 和工作区注意事项

两个仓库当前都已经由 Git 管理，但修改尚未在本文档中假定已经提交。

特别需要保留的用户现有改动：

- `/home/nolovr/GR00T-WholeBodyControl/config/g1_udp_network.env` 中 robot ID 已从 2 改为 1；
- `teleimager` 子模块的 dirty 状态；
- 运行 Isaac 后产生或更新的 `__pycache__` 文件。

这些内容不属于本轮 29DoF 实现本身，不应在整理提交时被误删或整体回退。`/home/nolovr/IsaacLab` 本轮没有代码修改。
