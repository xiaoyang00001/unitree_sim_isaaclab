# Gear SONIC → Unitree Isaac Lab G1-29DoF 第一阶段交接记录

> 文档日期：2026-07-23  
> 当前开发环境：Windows 10、Isaac Sim 5.1、Isaac Lab  
> 后续目标环境：Ubuntu  
> 项目目录：`unitree_sim_isaaclab`  
> 状态：第一阶段代码骨架已实现并通过静态检查，尚未完成 Ubuntu 运行时、DDS 和端到端控制验证。

## 1. 最终确认的需求

本项目使用 `unitree_sim_isaaclab` 替换 Gear SONIC 原有的 MuJoCo 仿真后端。

第一阶段只实现 G1 本体 29DoF 的自由基座全身控制，完整输入与控制链路为：

```text
OpenXR 串流 APK
    → RobotKit
    → pico_manager_thread_server.py
    → SMPL 人体全身运动参考
    → Gear SONIC C++ / TensorRT
    → Unitree DDS rt/lowcmd
    → unitree_sim_isaaclab
    → G1-29DoF-Dex3 Wholebody USD
    → Isaac Lab / PhysX
    → Unitree DDS rt/lowstate
    → Gear SONIC
```

这里需要区分两类关节数据：

- OpenXR/RobotKit 传输的是人体追踪骨架数据；
- SMPL 表示人体全身运动参考；
- Gear SONIC 将人体参考转换为 G1 的 29 个电机关节命令；
- Isaac Lab 不直接使用 RobotKit/SMPL 数据控制机器人；
- Isaac Lab 第一阶段只消费 Gear SONIC 通过 `rt/lowcmd` 发出的 G1 29DoF 控制。

第一阶段的控制所有权：

| 对象 | 数量 | 唯一控制源 |
|---|---:|---|
| G1 左腿 | 6DoF | Gear SONIC DDS |
| G1 右腿 | 6DoF | Gear SONIC DDS |
| G1 腰部 | 3DoF | Gear SONIC DDS |
| G1 左臂和左腕 | 7DoF | Gear SONIC DDS |
| G1 右臂和右腕 | 7DoF | Gear SONIC DDS |
| 左 Dex3 | 7DoF | 第一阶段保持 USD 默认姿态 |
| 右 Dex3 | 7DoF | 第一阶段保持 USD 默认姿态 |
| Floating Root | 6DoF | PhysX |

禁止以下控制方式：

- 不允许 Isaac Lab 内部 locomotion RL policy 控制双腿；
- 不允许把腰部固定为默认值而忽略 SONIC；
- 不允许从 OpenXR、RobotKit 或 SMPL 直接覆盖机器人 Root；
- 不允许每帧写 Root pose 形成 teleport 式动作回放；
- 第一阶段不允许 SONIC 或 Dex3 DDS 修改 14 个手指关节；
- 第一阶段不启用 Isaac Lab OpenXR 手柄 retargeter。

## 2. 指定机器人资产

第一阶段必须使用以下自由基座 wholebody USD：

```text
assets/robots/g1-29dof_wholebody_dex3/
    g1_29dof_with_dex3_rev_1_0.usd
```

完整工作区路径：

```text
F:\ISAACWholeBody\unitree_sim_isaaclab\
assets\robots\g1-29dof_wholebody_dex3\
g1_29dof_with_dex3_rev_1_0.usd
```

Ubuntu 上应通过项目相对路径加载，不应保留 Windows 绝对路径。

机器人配置复用：

```python
G1RobotPresets.g1_29dof_dex3_wholebody(...)
```

底层配置为：

```python
G129_CFG_WITH_DEX3_WHOLEBODY
```

不能替换为：

```text
g1_29dof_with_dex3_base_fix.usd
```

因为本需求需要自由基座站立、移动、脚底接触和 PhysX 动力学结果。

## 3. 第一阶段场景定义

新增任务：

```text
Isaac-G1-29DoF-Dex3-Sonic
```

场景只包含：

```text
World
├── GroundPlane
├── DomeLight
└── G1-29DoF-Dex3 Wholebody Robot
```

不包含：

- 桌子；
- 方块；
- 圆柱；
- 仓库场景；
- 相机任务；
- PickPlace；
- reward 业务逻辑；
- termination 业务逻辑；
- curriculum；
- 内部 locomotion policy。

严格意义上的“完全空场景”没有地面，机器人会自由落体，因此保留 GroundPlane。

## 4. 本轮已经修改和新增的文件

### 4.1 新增 SONIC 29DoF ActionProvider

```text
action_provider/action_provider_sonic_dds.py
```

职责：

- 从已注册的 `G1RobotDDS` 获取 `rt/lowcmd`；
- 校验 motor 数量至少为 29；
- 按 Unitree G1 硬件 motor order 映射全部 29 个本体关节；
- 不运行内部 RL policy；
- 让 Dex3 的 14 个关节保持 USD/ArticulationCfg 默认姿态；
- 从默认姿态平滑渐入首个 SONIC 目标；
- 检查 NaN/Inf；
- 检查 LowCmd 超时；
- 限制每控制步最大目标变化；
- 输出启动时 29/29 和 14/14 映射日志。

当前实现输出的是完整 43 关节绝对位置目标：

```text
29 body joints ← SONIC LowCmd.q
14 Dex3 joints ← robot.data.default_joint_pos
```

### 4.2 新增共享的 G1 DDS 硬件顺序

```text
robots/g1_joint_order.py
```

该文件定义：

```python
G1_29DOF_DDS_JOINT_ORDER
```

顺序已经对照 Gear SONIC：

```text
GR00T-WholeBodyControl/
gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/
robot_parameters.hpp
```

以及其中的：

```cpp
enum G1JointIndex
```

确认真实 DDS/硬件顺序为：

```text
0   left_hip_pitch_joint
1   left_hip_roll_joint
2   left_hip_yaw_joint
3   left_knee_joint
4   left_ankle_pitch_joint
5   left_ankle_roll_joint

6   right_hip_pitch_joint
7   right_hip_roll_joint
8   right_hip_yaw_joint
9   right_knee_joint
10  right_ankle_pitch_joint
11  right_ankle_roll_joint

12  waist_yaw_joint
13  waist_roll_joint
14  waist_pitch_joint

15  left_shoulder_pitch_joint
16  left_shoulder_roll_joint
17  left_shoulder_yaw_joint
18  left_elbow_joint
19  left_wrist_roll_joint
20  left_wrist_pitch_joint
21  left_wrist_yaw_joint

22  right_shoulder_pitch_joint
23  right_shoulder_roll_joint
24  right_shoulder_yaw_joint
25  right_elbow_joint
26  right_wrist_roll_joint
27  right_wrist_pitch_joint
28  right_wrist_yaw_joint
```

特别注意：

现有 Isaac Lab action 数组中曾出现左右交错的内部顺序，不能直接作为 Unitree `LowCmd.motor_cmd[i]` 顺序。若不做显式映射，会导致腿、腰和双臂串关节。

### 4.3 修改 LowState 状态反馈映射

```text
tasks/common_observations/g1_29dof_state.py
```

原实现使用固定 articulation 下标列表拼接 `rt/lowstate`。本轮改为：

```text
Articulation joint_names
    → name-to-index
    → G1_29DOF_DDS_JOINT_ORDER
    → LowState.motor_state[0:29]
```

这样 LowCmd 正向映射和 LowState 反向映射使用同一个协议定义。

### 4.4 修改 G1 DDS 接收数据

```text
dds/g1_robot_dds.py
```

每次收到并通过 CRC 检查的 LowCmd 后，增加：

```python
"receive_time_monotonic": time.monotonic()
```

用于识别 shared memory 中残留的过期命令。

### 4.5 修改 ActionProvider 工厂

```text
action_provider/create_action_provider.py
```

新增：

```text
action_source = sonic_dds
```

映射到：

```python
SonicDDSActionProvider
```

### 4.6 新增任务配置

```text
tasks/g1_tasks/g1_29dof_dex3_sonic/__init__.py
tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py
```

动作配置使用：

```python
JointPositionActionCfg(
    asset_name="robot",
    joint_names=[".*"],
    scale=1.0,
    use_default_offset=False,
    preserve_order=True,
)
```

`use_default_offset=False` 是必要条件，因为 SONIC `LowCmd.q` 是绝对目标位置，不能再次叠加 USD default pose。

### 4.7 修改 G1 task 注册

```text
tasks/g1_tasks/__init__.py
```

注册新模块：

```text
g1_29dof_dex3_sonic
```

### 4.8 修改启动入口

```text
sim_main.py
```

新增：

```text
--action_source sonic_dds
--sonic_lowcmd_timeout
--sonic_ramp_seconds
--sonic_max_target_step
```

如果任务是：

```text
Isaac-G1-29DoF-Dex3-Sonic
```

且 action source 保持默认 `dds`，程序会自动改用 `sonic_dds`，不会进入旧的 `dds_wholebody` RL policy 路径。

## 5. 当前控制参数

默认值：

```text
sonic_lowcmd_timeout = 0.10 s
sonic_ramp_seconds = 2.0 s
sonic_max_target_step = 0.10 rad/control-step
simulation dt = 0.002 s
decimation = 1
```

第一轮真实联调建议更保守：

```text
sonic_max_target_step = 0.02 rad/control-step
```

## 6. 当前控制模式及边界

当前只实现了第一阶段位置目标模式：

```text
LowCmd.q
    → 29DoF绝对位置目标
    → Isaac Lab implicit actuator
    → PhysX
```

尚未实现最终显式 PD 力矩模式：

```text
tau_cmd =
    LowCmd.tau
    + LowCmd.kp × (LowCmd.q - measured_q)
    + LowCmd.kd × (LowCmd.dq - measured_dq)

tau_cmd
    → set_joint_effort_target
    → PhysX
```

在位置模式完成真实 DDS、29DoF 映射和自由站立验证前，不建议直接切换力矩模式。

显式 PD 模式还需要检查：

- USD actuator 是否仍配置 stiffness/damping；
- 避免外部 PD 与 implicit actuator 形成双重 PD；
- 每关节 effort limit；
- kp/kd 上下限；
- dq 和 tau 的实际协议单位；
- PhysX solver iteration；
- 关节 damping、armature 和 friction；
- 控制频率与仿真频率。

## 7. Windows 上已经完成的验证

已通过：

```text
Python py_compile：PASS
git diff --check：PASS
指定 wholebody Dex3 USD 存在：PASS
Gear SONIC G1JointIndex 顺序检查：PASS 29/29
新任务注册代码：已加入
sonic_dds ActionProvider 工厂：已加入
LowCmd 接收时间戳：已加入
LowState 动态关节名映射：已加入
Dex3 第一阶段默认保持：已加入
```

检查输出：

```text
DDS_ORDER_CHECK PASS 29/29
USD_EXISTS True
```

## 8. Windows 上未完成的验证及原因

尝试使用 Isaac Sim 5.1 Python 启动任务注册/配置解析时，现有环境缺少：

```text
unitree_sdk2py
```

错误为：

```text
ModuleNotFoundError: No module named 'unitree_sdk2py'
```

失败发生在项目已有的：

```text
dds/dds_master.py
```

导入阶段，不是新代码的 Python 语法错误。

经确认后决定：

- 不在 Windows 上安装或编译 `unitree_sdk2py`；
- 提交当前代码和本文档；
- 在 Ubuntu 环境安装官方依赖并继续运行时验证。

因此当前不能宣称以下内容已经成功：

- 新任务已经完成环境构造；
- USD 实际加载后确认是 43 个 articulation joints；
- `rt/lowcmd` 已经被收到；
- 29DoF 已经实际运动；
- `rt/lowstate` 已经被 SONIC 消费；
- 机器人已经自由站立；
- OpenXR 到 Isaac Lab 已经端到端运行。

## 9. Ubuntu 环境准备

建议目录结构：

```text
workspace/
├── IsaacLab
├── unitree_sim_isaaclab
├── unitree_sdk2_python
└── GR00T-WholeBodyControl
```

按照 Unitree 官方项目说明安装 `unitree_sdk2_python`：

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

如果出现：

```text
Could not locate cyclonedds.
Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```

先编译 CycloneDDS 0.10.x：

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

然后：

```bash
export CYCLONEDDS_HOME=/absolute/path/to/cyclonedds/install
export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH
cd /absolute/path/to/unitree_sdk2_python
pip install -e .
```

必须确保 `unitree_sdk2py` 安装到运行 Isaac Lab 的同一个 Python 环境，而不是另一个系统 Python 或 Conda 环境。

验证：

```bash
python -c "import unitree_sdk2py; print(unitree_sdk2py.__file__)"

python -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber; print('channel import OK')"

python -c "from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_; print('G1 HG IDL OK')"

python -c "from unitree_sdk2py.utils.crc import CRC; print('CRC OK')"
```

## 10. Ubuntu 上的下一步执行顺序

### 步骤 1：检查提交内容

进入 `unitree_sim_isaaclab`：

```bash
git status
git diff --check
```

确认以下文件存在：

```text
action_provider/action_provider_sonic_dds.py
robots/g1_joint_order.py
tasks/g1_tasks/g1_29dof_dex3_sonic/__init__.py
tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py
doc/sonic_g1_29dof_phase1_handoff_zh.md
```

### 步骤 2：验证任务注册和配置解析

在 Isaac Lab Python 环境中执行一个最小脚本：

```python
from isaaclab.app import AppLauncher

app = AppLauncher(headless=True).app

import gymnasium as gym
import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

task = "Isaac-G1-29DoF-Dex3-Sonic"
assert task in gym.registry

cfg = parse_env_cfg(task, device="cuda:0", num_envs=1)
print(cfg.scene.robot.spawn.usd_path)
print(cfg.actions.joint_pos.use_default_offset)

app.close()
```

预期：

```text
任务已注册
USD路径指向 g1_29dof_with_dex3_rev_1_0.usd
use_default_offset=False
sim.dt=0.002
```

### 步骤 3：只构造环境，不启动 SONIC

目标：

- USD 能加载；
- Floating Root 未固定；
- GroundPlane 正常；
- articulation joint 数量符合预期；
- 29 个 body joint 全部存在；
- 14 个 Dex3 joint 全部存在；
- 机器人初始姿态不穿透地面。

必须记录：

```text
robot.data.joint_names
robot.num_joints
robot.data.body_names
root initial pose
```

预期总关节数：

```text
29 body + 14 Dex3 = 43
```

### 步骤 4：验证启动时映射日志

启动：

```bash
cd unitree_sim_isaaclab

python sim_main.py \
  --task Isaac-G1-29DoF-Dex3-Sonic \
  --robot_type g129 \
  --action_source sonic_dds \
  --step_hz 500 \
  --sonic_lowcmd_timeout 0.10 \
  --sonic_ramp_seconds 2.0 \
  --sonic_max_target_step 0.02
```

第一阶段不要添加：

```text
--enable_dex3_dds
```

预期日志：

```text
[sonic_dds] Control ownership: G1 body=SONIC DDS (29), Dex3=default hold (14), root=PhysX
[sonic_dds] motor[00] -> ... left_hip_pitch_joint
...
[sonic_dds] motor[28] -> ... right_wrist_yaw_joint
[sonic_dds] Body mapping 29/29, Dex3 hold mapping 14/14
[sonic_dds] HOLD: waiting for the first rt/lowcmd
```

### 步骤 5：验证无 SONIC 时的安全行为

不启动 Gear SONIC，只运行 Isaac Lab：

- 机器人保持 default joint target；
- Dex3 保持默认姿态；
- 不出现 action buffer 清零导致的手指抽动；
- 不出现内部 RL policy 输出；
- 不直接写 Root；
- 等待 LowCmd 时日志最多每秒提示一次。

注意：

自由基座机器人能否仅依靠 default pose 站立取决于 actuator、接触和初始高度。若机器人在无 SONIC 时缓慢倒下，不应立即修改 Root；先确认其是否只是没有平衡控制输入。

### 步骤 6：先验证 DDS，本阶段不应用大动作

分别验证：

```text
Gear SONIC publisher：rt/lowcmd
Isaac Lab subscriber：rt/lowcmd
Isaac Lab publisher：rt/lowstate
Gear SONIC subscriber：rt/lowstate
```

确保 DDS 两端统一：

- domain/channel；
- network interface；
- `unitree_hg` IDL；
- topic name；
- CRC；
- QoS；
- 29 motor count。

当前 `sim_main.py` 提示仿真侧使用 channel/domain 1。Gear SONIC 必须使用同一 domain。

确认原 MuJoCo bridge 已停止，不能同时存在两个：

```text
rt/lowstate publisher
```

### 步骤 7：打印 LowCmd 统计但暂不驱动

建议临时或通过调试日志记录：

```text
receive timestamp
message frequency
motor count
mode_pr
mode_machine
q min/max
dq min/max
tau min/max
kp min/max
kd min/max
CRC result
```

先确认数据合理，再让 ActionProvider 应用动作。

### 步骤 8：29个关节逐一小幅映射验证

对每个 motor 发送小幅位置变化，例如：

```text
+0.02 rad
```

按顺序验证 0–28：

- 只有目标关节运动；
- 左右不交换；
- pitch/roll/yaw 不交换；
- ankle 不映射到 shoulder；
- 腰部3轴正确；
- 反馈的 `LowState.motor_state[i]` 对应同一个物理关节。

不要直接用完整 PICO 大动作作为第一项测试。

### 步骤 9：验证 Dex3 第一阶段保持

在29DoF测试过程中确认：

- 14个Dex3关节保持 default pose；
- `rt/lowcmd` 不写入Dex3；
- 没有启动 `rt/dex3/left/cmd` 或 `rt/dex3/right/cmd` 控制；
- 手指不随 SONIC 变化。

### 步骤 10：验证 LowState 闭环

必须证明：

```text
PhysX measured q/dq/tau
    → G1 DDS hardware order
    → rt/lowstate
    → Gear SONIC consumed
```

不要只看 `publisher.Write()` 返回或 DDS 初始化成功。

建议在两端同时记录 sequence/timestamp，测量：

- LowCmd 发布频率；
- Isaac 接收延迟；
- PhysX step 延迟；
- LowState 发布频率；
- SONIC 状态反馈延迟；
- 陈旧包和丢包。

### 步骤 11：自由基座动作验证

依次测试：

1. 静止站立；
2. 轻微手臂动作；
3. 腰部小幅旋转；
4. 膝盖小幅弯曲；
5. 原地踏步；
6. 单步前进；
7. 连续行走；
8. 转向；
9. 上下肢同时动作。

必须确认：

- Root 由 PhysX 运动；
- 没有 root teleport；
- 脚底接触有效；
- 地面摩擦合理；
- 没有关节爆炸；
- 没有明显穿透；
- LowCmd 超时后不会继续执行陈旧行走命令。

### 步骤 12：位置模式稳定后再实现显式 PD

位置模式真实运行稳定后，新增控制模式，例如：

```text
--sonic_control_mode position
--sonic_control_mode effort
```

effort 模式计算：

```python
tau_cmd = (
    tau_ff
    + kp * (q_des - q)
    + kd * (dq_des - dq)
)
```

然后：

```python
robot.set_joint_effort_target(tau_cmd, joint_ids=body_indices)
```

实现时必须禁用或规避 actuator 双重 PD，并增加：

- torque clamp；
- kp/kd clamp；
- position limit；
- velocity limit；
- NaN/Inf fault；
- fault state；
- timeout damping；
- reset 后重新 ramp-in。

## 11. 第二阶段范围：Isaac Lab OpenXR → Dex3

第一阶段完成后，第二阶段才处理 Dex3。

第二阶段数据流：

```text
Isaac Lab OpenXR Device
    → CONTROLLER_LEFT / CONTROLLER_RIGHT
    → Trigger / Squeeze
    → Dex3 retargeter
    → 7 + 7 hand joint targets
```

参考文件：

```text
D:\Omniverse\IsaacLab\source\isaaclab\isaaclab\devices\openxr\
retargeters\humanoid\unitree\trihand\
g1_upper_body_motion_ctrl_retargeter.py
```

Ubuntu 对应使用 Isaac Lab 仓库内同一路径文件。

参考映射：

```text
Trigger → index finger
Squeeze → middle finger
max(Trigger, Squeeze) → thumb flexion
Trigger - Squeeze → thumb base rotation
```

第二阶段只复用：

- motion controller input extraction；
- Trigger/Squeeze 映射；
- 左右镜像；
- 14手指关节输出重排。

不复用其 wrist pose 去覆盖 G1 手腕。

第二阶段最终所有权：

```text
G1 29DoF body → SONIC DDS
G1 wrists → SONIC DDS
Dex3 14DoF fingers → Isaac Lab OpenXR
Floating Root → PhysX
```

## 12. 第一阶段验收标准

只有同时满足以下条件，才能把第一阶段标记为完成。

### 资产和场景

- 加载指定 `g1_29dof_with_dex3_rev_1_0.usd`；
- Floating Root 未固定；
- 一个机器人、一个地面、一个灯光；
- 29 body joints + 14 Dex3 joints；
- 无任务物体。

### 控制权

- 29DoF 全部由 SONIC DDS 控制；
- 没有内部 RL policy；
- 腰部没有默认值覆盖；
- Dex3 保持默认姿态；
- Root 只由 PhysX 更新。

### DDS

- 持续收到有效 `rt/lowcmd`；
- 持续发布有效 `rt/lowstate`；
- 两边使用相同 domain 和 IDL；
- 29 motor order 正确；
- 原 MuJoCo bridge 已停止；
- `rt/lowstate` 只有一个权威发布者。

### 物理

- 自由基座机器人能在 SONIC 控制下站立；
- 能响应全身动作；
- Root 没有被直接覆盖；
- 脚底接触有效；
- 无明显爆炸、穿透或瞬移；
- 超时保护有效。

### 端到端证据

必须能够追踪：

```text
OpenXR frame
→ RobotKit frame
→ PICO manager frame
→ SMPL frame
→ SONIC inference/control sequence
→ rt/lowcmd sequence
→ Isaac applied sequence
→ PhysX state
→ rt/lowstate sequence
→ SONIC consumed state
```

仅有以下证据不足以宣称完成：

- Python 能导入；
- Isaac 能启动；
- DDS 初始化成功；
- `publisher.Write()` 没报错；
- 机器人 USD 可见；
- 单个关节偶尔运动。

## 13. 建议的第一次 Ubuntu 启动命令

安装依赖并确认任务能构造后：

```bash
cd /path/to/unitree_sim_isaaclab

python sim_main.py \
  --task Isaac-G1-29DoF-Dex3-Sonic \
  --robot_type g129 \
  --action_source sonic_dds \
  --step_hz 500 \
  --sonic_lowcmd_timeout 0.10 \
  --sonic_ramp_seconds 2.0 \
  --sonic_max_target_step 0.02
```

第一轮建议：

- 不加 `--enable_dex3_dds`；
- 不直接启动大幅人体动作；
- 不立即切 effort 控制；
- 先记录真实 articulation joint names；
- 先验证等待 LowCmd 时的默认保持；
- 再做单关节映射。

## 14. 提交前建议

提交前执行：

```bash
git status
git diff --check
git diff --stat
```

建议提交说明：

```text
feat: add Gear SONIC DDS control scene for floating G1 29DoF
```

不要在提交说明中写：

```text
end-to-end control completed
```

因为当前还没有完成 Ubuntu 运行时和真实 DDS 端到端验证。

