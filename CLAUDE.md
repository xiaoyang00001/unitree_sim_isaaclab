# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概览

基于 Isaac Lab 的 Unitree G1 / H1-2 仿真环境。核心特点：**仿真器收发与真机完全相同的 DDS 话题**
（`rt/lowcmd`、`rt/lowstate`、`rt/dex3/*` 等），因此可直接对接 `xr_teleoperate` 做数据采集、回放、
数据增强生成，也可以接入外部控制器（当前分支 `xiaoyang01` 的重点是 Gear SONIC 全身控制闭环）。

⚠️ 运行仿真时会往网络里发真机同名 DDS 话题；同一网段有真机时务必用 `UNITREE_DDS_DOMAIN` /
`UNITREE_DDS_INTERFACE` 隔离。

## 本机环境（已实测）

| 项目 | 路径 / 值 |
|---|---|
| conda 环境 | `env_isaaclab`（Python 3.11），`conda activate env_isaaclab` |
| `isaaclab` 包实际来源 | `/home/nolo/xiaoyang_IssacLab/IsaacLab`（可编辑安装，分支 `07241`）—— **不是** `/home/nolo/IsaacLab` |
| `isaacsim` | pip 安装在 conda 环境内，故可直接 `python sim_main.py`，无需 `isaaclab.sh -p` |
| GR00T/SONIC 仓库 | `/home/nolo/GR00T-WholeBodyControl` |
| `assets/` | 符号链接到外部盘；资产由 `fetch_assets.sh`（HuggingFace + git-lfs）下载 |

要改 Isaac Lab 本身的行为时，改的是 `/home/nolo/xiaoyang_IssacLab/IsaacLab` 下的源码。

## 常用命令

```bash
conda activate env_isaaclab

# 遥操作（普通任务）：--enable_dex1_dds / --enable_dex3_dds / --enable_inspire_dds 三选一
python sim_main.py --device cpu --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds --robot_type g129

# 数据回放 / 数据生成
python sim_main.py --device cpu --enable_cameras --task Isaac-Stack-RgyBlock-G129-Dex1-Joint \
  --enable_dex1_dds --robot_type g129 --replay_data --file_path <数据集目录> \
  [--generate_data --generate_data_dir ./data2] [--modify_light] [--modify_camera]

# SONIC 43DoF 闭环（另一终端跑 GR00T 的 deploy.sh isaac profile）
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl \
UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
python sim_main.py --task Isaac-G1-29DoF-Sonic --robot_type g129 \
  --action_source sonic_dds --device cpu --stats_interval 5 --profile_interval 250

# 无渲染 A/B（排除渲染负载，物理步长不变），或 WebRTC 直播（二者互斥）
--no_render     |     --livestream_type 1|2 [--public_ip x.x.x.x]

# 非 AR GUI 画面帧率（默认已是"画面 = 物理 = 50 fps"，下面是回退/加码旋钮）
--full_kit                  # 回退 Isaac Lab 原版 experience，要 Stage 树/Property 面板时用
--hide_ui                   # 只留 viewport，再省 ~0.7ms/帧（conveyor 靠它稳定 50/50）
--late_render_interval 2    # 隔圈渲染（画面 25fps），留给场景更重、余量吃紧的情况。
                            # ⚠️ XR 下改它救不了卡顿（两套刚性时钟打拍 + 全链无重投影），
                            # 保留只为不静默覆盖用户输入，见 doc/xr_ar_judder_zh.md

# AR/XR：消卡顿必须走 CloudXR（SteamVR+NOLO 那条链全程无重投影，必抖）
python -m isaacteleop.cloudxr --accept-eula --host-client   # 另一终端，先起 runtime
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
  --action_source sonic_dds --device cpu --teleop_device motion_controllers \
  --xr_runtime cloudxr        # 自动注入环境+kit 设置；runtime 没起会直接报错而非静默降级
# 头显浏览器开 https://<PC的IP>:48322/client/ ，接受自签证书后点 CONNECT（不用装 App）

# SONIC 资产/跟踪诊断
python tools/diagnose_sonic_model.py --task Isaac-G1-29DoF-Sonic [--summary-only]
python tools/monitor_sonic_tracking.py     # 订阅 C++ ZMQ debug 流比对参考动作与实测关节

# AR 卡顿：逼 SteamVR compositor 退出直通的探针（需 SteamVR 已起 + Isaac 已 --xr 接入）
# 客观判据不用戴头显:comp_gpu 从 ~0.0x 跳起来 = compositor 不再直通。见 doc/xr_ar_judder_zh.md §4.1
python tools/xr_overlay_probe.py --mode none    # 基线
python tools/xr_overlay_probe.py --mode tiny    # 3cm overlay,测零代价解
```

任务名清单见 `README_zh-CN.md` 的表格；带 `Wholebody` 的任务支持移动（配 `send_commands_8bit.py` /
`send_commands_keyboard.py` 发速度指令）。

## 测试

测试是纯 unittest + mock，**不启动 Isaac Sim**，跑得很快（<1s）：

```bash
python -m unittest discover -s tests                       # 全部（23 用例）
python -m unittest tests.test_sonic_dds_full_lowcmd -v     # 单个文件
python -m unittest tests.test_g1_robot_dds_reset_grace.G1RobotDDSResetGraceTest.test_reset_grace_can_be_rearmed_after_a_slow_reset

# URDF 相关 6 个用例默认因缺 GR00T 源资产而 skip，需显式指路径
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl python -m unittest tests.test_g1_sonic_urdf
```

测试里 DDS 固定用 `UNITREE_DDS_DOMAIN=91` / `lo` 与真实运行隔离，新增 DDS 测试请沿用。

## 架构要点

### 控制主循环（单线程渲染 + 后台 DDS 线程）

```
sim_main.py main() 主循环（必须在主线程，Isaac 渲染要求）
  └─ RobotController.step()          layeredcontrol/robot_control_system.py
       ├─ ActionProvider.get_action(env)   同步调用，拿到 action tensor
       ├─ [可选] can_step_environment()    返回 False 则本轮跳过 env.step（SONIC 锁步握手）
       ├─ env.step(action)
       └─ 基于 monotonic deadline 的定频休眠（--step_hz，SONIC 默认 50，其他 100）
```

`ActionProvider`（`action_provider/action_base.py`）另有一个后台线程 `_run_loop` 做接收/解码，
`get_action` 只做取值，避免与主线程抢锁。拿不到新动作时沿用 `_last_action`。

### DDS 层：共享内存双向桥

`dds/dds_master.py` 的 `DDSManager` 是全局单例，按名字注册对象（`g129`/`dex3`/`dex1`/`inspire`/
`run_command`/`reset_pose`/`sim_state`/`rewards`），有独立发布线程按对象各自频率（默认 100 Hz）调度。
每个 `DDSObject` 持有两块共享内存：

- **Isaac → DDS（状态）**：`tasks/common_observations/*.py` 里的观测函数在被 `ObsTerm` 调用时，
  **顺带把状态写进 `input_shm`**（如 `g1_robot_dds` 的 `isaac_robot_state`），发布线程再读出来发 DDS。
  ⚠️ 这些"观测函数"有副作用，不是纯函数——删掉某个 ObsTerm 会直接让对应话题停发。
- **DDS → Isaac（命令）**：订阅回调把命令写进 `output_shm`（如 `dds_robot_cmd`），
  ActionProvider 通过 `dds_manager.get_object("g129").get_robot_command()` 读取并组装 action。

话题：`rt/lowcmd`(订)、`rt/lowstate`+`rt/secondary_imu`(发)、`rt/dex3/{left,right}/{cmd,state}`、
`rt/dex1/{left,right}/{cmd,state}`、`rt/inspire/{cmd,state}`、`rt/reset_pose/cmd`(订)、
`rt/run_command/cmd`(订)、`rt/sim_state`、`rt/rewards_state`。

### action_source 会被 sim_main.py 自动改写

`--action_source` 显式传 `dds` 时仍可能被覆盖，排查行为时注意启动日志：
SONIC 任务 → `sonic_dds`；任务名含 `Wholebody` 或 `--enable_wholebody_dds` → `dds_wholebody`
（并打开 `use_rl_action_mode`）；`--replay_data` → `replay`。
provider 在 `action_provider/create_action_provider.py` 里按需惰性导入。

### 任务注册与配置复用

- `tasks/__init__.py` 用 `import_packages` 递归导入注册 gym 环境，但黑名单含 `pick_place`，
  **新任务必须同时加进 `tasks/g1_tasks/__init__.py` 的显式 import 和 `__all__`**，否则 `gym.make` 找不到。
- 每个任务目录：`__init__.py`(gym.register) + `<name>_env_cfg.py`(场景/动作/观测/事件) + `mdp/`。
  `mdp/__init__.py` 做 `from isaaclab.envs.mdp import *` 再叠加本地 `observations/terminations/rewards`
  的 re-export，于是 env_cfg 里统一写 `mdp.xxx`，官方项与自定义项混用无缝。
- 横向复用层：`tasks/common_scene/`（除机器人外的场景）、`tasks/common_config/`
  （`G1RobotPresets` / `H12RobotPresets` / `CameraPresets`）、`tasks/common_observations/`、
  `tasks/common_termination/`（物体越界判定）、`tasks/common_event/`（reset 事件）。
- 机器人本体 `ArticulationCfg` 与 USD 路径集中在 `robots/unitree.py`。
- 新增任务的完整步骤见 `README_zh-CN.md` §3.2。

### SONIC 43DoF 桥接（`Isaac-G1-29DoF-{Sonic,Dex3-Sonic,Training-Sonic,Sonic-Conveyor}`）

- **载体**：29 本体关节 + 14 个 Dex3 关节。URDF 由 `robots/g1_sonic_urdf.py` 在运行时从 GR00T 源
  资产合成到 `/tmp/unitree_sim_isaaclab_sonic_<uid>/`，再经 `UrdfFileCfg(force_usd_conversion=True)`
  转 USD。GR00T 根目录默认硬编码为 `/home/nolovr/GR00T-WholeBodyControl`（另一台机器的路径），
  **本机必须设 `GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl`**，否则静默 fallback 到旧的
  tmp 产物或在 URDF 导入时才报缺文件。
- **action 布局**：三个 action term（`joint_pos`/`joint_vel`/`joint_effort`）field-major 拼接 ⇒
  43×3 = 129 维；`RobotController` 的 fallback action 按 `action_manager.total_action_dim` 分配，
  不要假设 action 维度 == 关节数。
- **锁步握手**：LowState 带 `sample_seq`/reset epoch，C++ 侧用 LowCmd reserve 回 ack；ack 未匹配时
  `can_step_environment()` 返回 False，PhysX 暂停而非重复推进旧动作（日志里 `sync_waits`）。
- **倒地复位**：`SonicFallResetMonitor`（倾角/根高 + 去抖 + 冷却）→ `trigger_robot_reset()`：
  开 reset epoch → 抑制非物理 dq/tau 的 grace 窗口 → 事件复位 → `env.reset` → 重新武装 grace。
- **SONIC 任务的默认值与普通任务不同**（`sim_main.py` 里按 `is_sonic_task` 分支）：step_hz 50、
  DDS domain 1 + `lo`、`sim_state` 导出降到 5 Hz、自动倒地复位开、跳过图像服务（无相机）、
  强制 Dex3 DDS。改这些默认值前先读 `doc/sonic_g1_29dof_phase1_handoff_zh.md`。

### SONIC 场景内容：四个任务不是同一个场景

| 任务 id | EnvCfg / SceneCfg | 场景内容 |
|---|---|---|
| `Isaac-G1-29DoF-Sonic` | `G129SonicEnvCfg` / `G129SonicSceneCfg` | 地面 + 43DoF 机器人 + **操作台 + 3 个 5cm 方块** + XR 配置 |
| `Isaac-G1-29DoF-Dex3-Sonic` | 同上——**两个 id 注册到同一个 EnvCfg** | 同上 |
| `Isaac-G1-29DoF-Sonic-Conveyor` | `G129SonicConveyorEnvCfg`（继承前者） | 上面全部 + 传送带 |
| `Isaac-G1-29DoF-Training-Sonic` | `G129TrainingSonicEnvCfg` / `G129TrainingSonicSceneCfg` | **只有地面 + 精确 29DoF 训练模型**，干净的 A/B 回归基线 |

- 桌子用 Nucleus 官方 `Props/PackingTable/packing_table.usd`（kinematic）。**别换回仓库自带的旧副本**：
  它缺 3 个金属材质贴图，XR 模式下会刷 RTX/MDL 错误。
- **要调布局请转桌子，不要转机器人。** 参考 locomanipulation 场景的机器人初始 yaw 是 +90°，而 SONIC
  机器人必须保持 identity 朝向以维持 policy 的世界系约定，所以这里是把桌子绕 Z 转 -90° 来补偿的。
- 方块 5 cm / 0.08 kg / 静动摩擦 1.2 与 0.9 / restitution 0；初始高度由 `SONIC_TABLE_TOP_Z = 0.6996`
  （世界系桌面高度，不是 prim 原点）推出。
- 复位：`EventsCfg` 是空的，倒地/手动复位统一走 sim_main 注册的 `reset_scene_to_default`，
  **方块会跟着一起归位**，不需要再加物体重置事件。
- ⚠️ SONIC 行走 policy 不感知桌子。桌子是 kinematic 的且就在正前方 0.55 m，跑行走类动作会撞上去——
  这是场景约束，不是 bug。需要纯行走验证时用 `Training-Sonic`。

### OpenXR：只接管视角，不接管机器人

`--teleop_device motion_controllers`（仅上表前三个任务可用；会自动置 `--xr`；与 `--no_render` 互斥）。

- **`retargeters=[]` 是刻意留空的**：OpenXR 只提供视角锚定和一个 recenter 按键，机器人关节命令
  100% 仍来自 SONIC DDS。别指望 VR 手柄能操作机器人，排查动作异常时也不必怀疑到它头上。
- 位置锚是 `torso_link/head_link`——URDF importer 把固定的 head link 并到了 torso 下，层级与参考实现的
  `Robot_1/head_link` 不同，照抄参考路径会找不到 prim。
- 旋转锚单独指向 `pelvis` + `FOLLOW_PRIM_SMOOTHED`：只跟 pelvis 的 yaw，**避免躯干 roll/pitch 把 XR
  世界带歪**。
- 右手柄 B 键**松开**时重新对正视角 yaw（`recenter_yaw_button_event="release"`），是视角 recenter，
  不是环境 reset。
- 退出时必须显式调 `teleop_interface.__del__()` 再 `gc.collect()`：Isaac Lab 的 `OpenXRDevice` 至今没有
  公开的 `close()`，XR 消息总线和按钮订阅会反向持有设备回调，只丢引用回收不掉。

### AR 卡顿：真因是没有重投影，不是帧率

**权威文档 `doc/xr_ar_judder_zh.md`，改 XR 相关参数前必读**，能省下重跑一遍已被否决实验的时间。

- **根因**：SteamVR + NOLO XrLink 这条链**全程没有重投影/ATW**（vrcompositor 统计
  `228920 presents / 0 reprojected`，且每次启动都打印
  `Async support disabled by user setting or a direct mode driver`）。应用帧率只要低于面板
  刷新率，旧帧就被原样重发、头姿不更新 → 头一转就"粘一下跳一下"。**这是结构性的，
  帧率和帧节奏优化只能减轻，不能消除。**
- **`--xr_runtime cloudxr` 是解**：CloudXR 头显端客户端自带深度重投影。本机 runtime 已装好、
  Isaac Sim 已实连验证。不需要手工 source，runtime 没起会直接报错。
- ⚠️ **观测陷阱**：SteamVR dashboard（菜单）由 compositor 按面板刷新率逐帧用最新头姿渲染，
  **菜单永远不抖**，会掩盖应用层 judder。2026-07-28 那条"36fps 匀速就流畅"的结论就是这么
  被污染的。做 XR 主观评测前先确认 dashboard 已关。
- ⭐ **但 dashboard 模式下连 Isaac 画面也不抖**（2026-07-30 用户实测，且此时机器人照常走动
  ⇒ 应用仍在正常提交新帧）。这说明 **PC 侧每帧按新头姿重新变换的能力本来就有，
  SteamVR compositor 自己在做，只是不对正常 scene layer 路径启用**——正常模式
  `Compositor Time GPU: 0.007ms` 是在**直通**。据此的假设：直通的前提是单一 layer，
  只要存在一个可见 overlay 就可能逼它转入合成模式，**代价近乎为零**（不牺牲立体、
  不改 Isaac 一行代码）。探针已就绪：`tools/xr_overlay_probe.py`，有不需要戴头显的客观
  判据（`m_flCompositorRenderGpuMs` 是否从 ~0 跳起来）。步骤与三种结果的分支见
  `doc/xr_ar_judder_zh.md` §4.1。
  ⚠️ 别拿 `0 reprojected` 去否证它——那个计数器只统计 async 补帧路径，不含 compositor 自绘。
  ⚠️ 也别把 §7 的"NOLO 驱动内做不到"外推成"PC 侧做不到"；但反过来，§4.1 即使成功也只补
  PC 内那一段，编码+网络+解码的几十毫秒延迟仍只有头显端 ATW 能补。
- ⚠️ **kit 的 `/persistent/xr/...` 设置跨进程残留**且优先级高于 experience 文件里的
  `runtime = "system"`。所以三种 `--xr_runtime` 模式都**显式**写回自己要的值——否则跑过一次
  CloudXR 之后，普通 `--xr` 会继续去连 CloudXR 并失败在 `xrCreateInstance`。
- ⚠️ `--xr_runtime` 的 `add_argument` **必须在 `AppLauncher.add_app_launcher_args()` 之后**：
  该函数内部先跑一次 `parse_known_args()` 探测，那时 `--xr` 还没注册，argparse 会把 `--xr`
  当成 `--xr_runtime` 的缩写，把所有现有 `--xr` 命令行打断。`tests/test_xr_runtime_cli.py` 守着。
- **NOLO 那条路 PC 侧改不了**（闭源二进制 + 驱动内根本没有图形管线 + direct-mode 之后再无 warp
  阶段），只能向 NOLO 提需求；好消息是协议已经够了（每帧视频已带渲染时头姿四元数 + 时钟同步
  + 高频 tracking 上报），只差客户端实现。细节见 `doc/xr_ar_judder_zh.md` §7。

## 约定与陷阱

- **关节顺序**：`robots/g1_joint_order.py` 的 `G1_29DOF_DDS_JOINT_ORDER` 是 Python 侧唯一的 DDS
  硬件顺序真源。Isaac articulation 的关节顺序与之**不同**，任何映射都必须按关节名建索引，
  不能按下标假设。
- **`__pycache__` 曾被误跟踪**（87 个 `.pyc`，因 `.gitignore` 规则是后加的），已用
  `git rm -r --cached` 移除跟踪，磁盘文件保留。之后若再在 diff 里看到 `.pyc`，说明有人用
  `git add -f` 强加了，应当剔除而不是提交。
- `sim_main.py` 大量用 `try/except` + `print` 包裹可选功能（材质、相机、physx 参数、奖励等），
  **失败会静默降级**。排查问题先核对启动日志里的关键行：`[DDS Config]`、`[sim] control timing`、
  `[sonic_dds] Body mapping`、`[fall_reset] enabled`。
- 手部 DDS 三选一互斥（`--enable_dex1_dds` / `--enable_dex3_dds` / `--enable_inspire_dds`）。
- `--device cpu` 是单机器人场景的推荐值（尤其 SONIC：PhysX 走 CPU，把 GPU 留给外部
  TensorRT/ONNX 推理进程）。
- `teleimager` 是 git submodule，图像服务在运行时把 `teleimager/src` 插入 `sys.path` 后导入。
- 首次启动会加载/转换资产，等待时间较长属正常；GUI 里需点 PerspectiveCamera → Cameras →
  PerspectiveCamera 才能看到主视图（**注意**：`--hide_ui` 下没有菜单栏，这一步做不了，
  得靠任务自带的开局机位 `viewer.eye/lookat`）。
- **SONIC GUI 默认走精简 experience** `apps/isaaclab.sonic.kit`（去掉资产浏览器/示例机器人/
  合成数据链路等 71 个扩展 + 关地面网格与选中轮廓）。模板里的 `@ISAACLAB_APPS@` /
  `@ISAACLAB_SOURCE@` 由 `sim_main.py` 在启动前按实际安装位置替换后写进 `/tmp`——
  kit 的 `${app}` 指 experience 文件所在目录，模板放本工程会让扩展目录全部失效。
  XR 与 `--enable_cameras` 各有专属 experience，这两种模式下自动跳过精简版。
- **帧率账本**（20ms 预算 = A 等ack + E 物理 + R 渲染 + S 睡眠余量）：主场景
  A≈0.9/E≈6.2/R≈6.4 有 5-6ms 余量；conveyor E≈10、R≈7 就贴着预算跑，S≈0。
  判读：`S` 长期为 0 说明已经超支，主循环会掉出 50Hz。渲染成本 R 里约 6ms 是
  **kit CPU 地板**（与场景内容无关），所以降分辨率/`rendering_mode=performance` 都无效
  （已复验否决），真正有效的是减扩展与减每帧重绘的 UI。

## Windows 部署（win2，跨机闭环）

仿真端可以跑在 Windows 上、GR00T 留在 Ubuntu，两端走原生 DDS 跨机。本工程侧的
Windows 适配已合入，下面是**只在 Windows 上才需要知道**的部分。

启动一律用工程根的 `run_win.bat`（它封装了两个必需的环境变量）：

```bat
run_win.bat --task Isaac-G1-29DoF-Sonic --robot_type g129 ^
    --action_source sonic_dds --device cpu --dds-interface <本机IP>
REM AR：先起 NOLO Link 或 ALVR 拉起 SteamVR，再双击 run_win_ar.bat（必须桌面双击，
REM ssh 起的进程够不到 OpenXR runtime）
```

对端 Ubuntu：`./deploy.sh isaac:enp4s0`（`isaac:<iface>` 是跨机专用模式，传裸网卡名
会被判成 real 而**静默丢掉 `--isaac-sim`**、domain 从 1 掉到 0）。两边都不要显式传 domain。

Windows 特有的坑（每条都有对应提交，`git log --grep="(win)"`）：

| 现象 | 根因 | 对策 |
|---|---|---|
| 启动即 `AttributeError` | `os.getuid` 不存在 | 已修：POSIX 用 uid、Windows 用登录名 |
| `UnicodeEncodeError: 'gbk'` 崩进程 | 中文控制台编不了日志里的 emoji | `PYTHONUTF8=1` + `chcp 65001`（在 bat 里） |
| `import tasks` 阶段就死 | SONIC 任务在模块级合成 URDF | `GR00T_WBC_ROOT` 必设，**与跑哪个任务无关** |
| GUI 报 TOML 转义错、连锁 `No module named omni.kit.usd` | 精简 kit 里的 `D:\...` 被 TOML 当转义 | 已修：`as_posix()`。⚠️ headless 测不出来 |
| GUI/XR 报 `DLL load failed importing _errors` | h5py 的 hdf5.dll 被扩展抢先加载 | 已修：kit 启动前预加载 h5py |
| `--dds-interface lo` 失败 | Windows 没有 lo 网卡 | 用 `auto` 或本机 IP，注意选项名是**连字符** |
| 锁步只有 0.4Hz、`Init Done` 等 6 分钟 | 纯 Python CRC 打满回调线程 | 已修：CRC 抽样校验（启动日志有提示行） |

环境侧的两个前提（不在代码里，装机时要做）：

- **cyclonedds 必须是 0.10.x**，且只能自编（无 cp311 wheel）。⚠️ **千万别用 11.0.1**：
  它会在 discovery 广播 XTypes TypeObject，让 GR00T 的 C++（CycloneDDS 0.10.2）
  **段错误**。装完还要手改 `cyclonedds/__library__.py` 的路径转义，每次重装都要改。
- **XR 需要 `isaacsim-extscache-{kit,kit-sdk,physics}`**。装之前先开 Windows 长路径，
  否则解包失败留下半装残渣，会让普通仿真也崩在 h5py 上。

## 参考文档

- `README_zh-CN.md` / `README.md`：任务清单、环境安装（`auto_setup_env.sh 4.5|5.0|5.1 <env_name>`）、
  docker 构建、新增任务步骤。
- `doc/sonic_g1_29dof_phase1_handoff_zh.md`：SONIC 阶段一的权威交接记录——动力学对齐取舍、
  DDS 协议顺序、生命周期与安全降级、启动顺序、已验证结论与**明确未覆盖的范围**。
  改 SONIC 相关动力学/时序参数前必读，避免重跑已被否决的实验。
- `doc/xr_ar_judder_zh.md`：**AR 卡顿的权威文档**——根因（全链无重投影）、已被实测否决的
  方案清单、CloudXR 通路操作手册与判读口径、NOLO 侧逆向结论。动 XR 参数前必读。
- `doc/isaacsim{4.5,5.0,5.1}_install_zh.md`：分版本手工安装步骤。
