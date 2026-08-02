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
--late_render_interval 2    # 隔圈渲染（画面 25fps），留给场景更重、余量吃紧的情况

# SONIC 资产/跟踪诊断
python tools/diagnose_sonic_model.py --task Isaac-G1-29DoF-Sonic [--summary-only]
python tools/monitor_sonic_tracking.py     # 订阅 C++ ZMQ debug 流比对参考动作与实测关节
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

### 流水线双机器人 host/viewer（conveyor 任务的三种身份）

**权威文档 `doc/pipeline_dual_robot_host_viewer_zh.md`，动这套前必读**（启动手册/键盘序列/
帧率账本/五个实测坑全在里面）。身份由 `sync_identity.py` **单一真源**解析（sim_main 与
env cfg 共用，进程环境变量优先于 `configs/scene_sync.env`——别在任何一侧重新实现判定）：

- `ISAACLAB_LOCAL_ROBOT_ID=1/2`：对等端（既有双机模式，机器人互为镜像）；
- `ISAACLAB_LOCAL_ROBOT_ID=0`：**viewer 纯镜像**——只收不发、双镜像体、本机 robot 退化为
  场外 ghost、自动切 `hold` 动作源（不挂锁步，50Hz 满帧）、DDS 默认落 domain 9；
- `ID=1` + `ISAACLAB_HOST_BOTH_ROBOTS=1`：**host 双机器人**——robot_1+robot_2 双全动力学，
  各一套 deploy（第二套 `G1_LOCAL_ROBOT_ID=2 ./deploy.sh isaac` 自动走 `rt/r2/*` 话题 +
  `g129_r2`/`_r2` shm），动作源自动切 `sonic_dds_host`（258 维、双 ack AND 锁步）。

⚠️ 三条高频坑：host 的两套 deploy **必须并行启动**（串行等 Init Done 会在双 ack 门下自锁）；
测前 `pgrep -fa g1_deploy_onnx_ref` 必须为 0（残留实例 kHz 级轰 ack）；host 建议
`UNITREE_SKIP_LOWSTATE_CRC=1` + `UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50` + 两套 deploy
`--disable-crc-check`（**Linux 的 unitree CRC 也是纯 Python**，双通道 1000 包/秒下占 35% GIL）。

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

<!-- BEGIN win-port -->
## Windows 部署与帧率（win2，跨机闭环）

仿真端可以跑在 Windows、GR00T 留在 Ubuntu，两端走原生 DDS 跨机。启动一律用工程根的
`run_win.bat`（封装了两个必需环境变量）；对端 `./deploy.sh isaac:enp4s0`
（`isaac:<iface>` 是跨机专用模式，传裸网卡名会被判成 real 而**静默丢掉 `--isaac-sim`**）。

### 🔥 跑性能测试前必做：关掉 XR 常驻服务

**这是 Windows 侧掉帧最大的隐形杀手，比所有代码优化加起来都重要。**

`XRLink`（NOLO 串流）常驻会吃 **245% CPU**（2.45 核），`vrserver`+`vrcompositor`
再吃 **92%**。即使 Isaac 完全不用 XR，它们照样满载抢核——而 kit 主循环是单线程的。

```powershell
Get-Process XRLink,vrserver,vrmonitor,vrcompositor -EA SilentlyContinue | Stop-Process -Force
```

同一台机、同一份代码、同样跨机 WiFi，仅此一项差异的实测对比：

| | A(等ack) | S(余量) | T max | 达标率 |
|---|---|---|---|---|
| XRLink 在跑 | 8.2ms（p90 27.3） | 0.0ms | 47.6ms | 49% |
| **XRLink 关掉** | **2.1ms** | **5.8ms** | **20.2ms** | **~100%** |

⚠️ **曾错误归因于 WiFi**：看到 `A` 尾部尖峰（p90 27.3ms）就判定是跨机 WiFi 抖动，
还写进过文档。实际关掉 XRLink 后网络条件完全没变，`A` 却降到 2.1ms、抖动归零。
**"跨机 = 网络背锅"是个容易上钩的假设，先排除主机侧常驻负载。**

### 帧率账本（headless 跨机锁步，i5-14600KF + RTX 4070 Ti，与 Linux 同款 CPU）

| 阶段 | A | E | S | T | 频率 |
|---|---|---|---|---|---|
| 基线 | 11.9 | 15.6 | 0.0 | 27.6ms | 36.2Hz |
| + 跳过 LowState CRC | 9.4 | 12.7 | 0.0 | 22.6ms | 44.2Hz |
| + `timeBeginPeriod(1)` | 8.2 | 10.4 | 0.4 | 20.0ms | 50.0Hz |
| **+ 关 XR 常驻服务** | **2.1** | **11.6** | **5.8** | **19.9ms** | **50.4Hz** ✅ |
| Linux 本机参考 | 0.9 | 6.2 | 5-6 | — | 50.00Hz |

两个代码修复（均已默认生效，CRC 那刀需显式开环境变量）：

1. **`timeBeginPeriod(1)`**（`sim_main.py` 顶部）——Windows 系统定时器周期默认 15.625ms，
   `threading.Event.wait` 与带 timeout 的锁全被量化，而 DDS 发布线程正用
   `_wake_event.wait()` 排下次发布（`dds_master.py`）⇒ lowstate 迟发 ⇒ ack 迟到。
   🪤 **`time.sleep` 不受影响**（py3.11+ 用高精度 waitable timer）：
   `time.sleep(0.2ms)` 0.50→0.50ms 无变化，而 `Event.wait(0.2ms)` **15.50→1.46ms**。
   只测 `time.sleep` 会得出"`timeBeginPeriod` 无效"的错误结论——本轮踩过。
2. **`UNITREE_SKIP_LOWSTATE_CRC=1`**——纯 Python 算 `LowState_` 的 CRC 要 2.15ms/包，
   102.8Hz 发布 = 22% GIL 且跑在发布线程里抢主循环。
   ⚠️ 必须同时给 deploy 传 `--disable-crc-check`，否则对端丢弃每一帧。

### 已被实测否决（别重试）

| 方向 | 实测 | 为什么不行 |
|---|---|---|
| 钉 P 核 + High 优先级 | 39.8Hz（比默认 44.2 差） | Linux 的"钉 P 核"不能照搬：200+ 线程挤进 12 逻辑核加剧争抢，E 核的并行容量是净收益 |
| `sys.setswitchinterval(0.5ms)` | **17.6Hz**（差 2.5 倍） | 切换过频，上下文切换开销压倒尾延迟收益，A/E 同时恶化 2 倍 |
| 优化 lowcmd 解析 / json | 各 0.02ms | CRC 抽样后回调仅占 2.4% GIL，不是瓶颈 |
| 降分辨率 / GPU 侧 | GPU 仅 11-23% | 卡在 CPU 提交路径 |

### 口径与测量纪律

- **`--xr` 是另一条链路**：走专属 experience、帧节奏由 compositor 和 `xrWaitFrame` 决定，
  上表数据全部不适用。启动日志里 action source 不是 `sonic_dds`、或加载的是
  `isaaclab.python.xr.openxr.kit`，就说明跑的不是这条路。
- **跨机 A/B 必须两侧同时重启**。只重启一侧会让 tick/epoch 失配，特征是
  `A ≡ 250ms`（等于 `sonic_sync_wait_timeout`）、`sync_waits` 每周期整百递增、
  `physics_steps=0`。健康时 `sync_waits` 应该几乎不涨（实测 1）。
- `just run` 起的 deploy 被 timeout 打断会留残余进程，多实例同时发 ack 会让数据
  完全不可信，测前先确认进程数为 0。
- headless 的 `[Performance]` 行没有 `R` 项；GUI 才有。**headless 通过 ≠ GUI 通过**
  （精简 kit 的 TOML 转义、h5py DLL 抢占两个坑都只在 GUI 暴露）。
<!-- END win-port -->

## 参考文档

- `README_zh-CN.md` / `README.md`：任务清单、环境安装（`auto_setup_env.sh 4.5|5.0|5.1 <env_name>`）、
  docker 构建、新增任务步骤。
- `doc/sonic_g1_29dof_phase1_handoff_zh.md`：SONIC 阶段一的权威交接记录——动力学对齐取舍、
  DDS 协议顺序、生命周期与安全降级、启动顺序、已验证结论与**明确未覆盖的范围**。
  改 SONIC 相关动力学/时序参数前必读，避免重跑已被否决的实验。
- `doc/isaacsim{4.5,5.0,5.1}_install_zh.md`：分版本手工安装步骤。
- `doc/windows_deployment_zh.md`：**从零部署一台新 Windows 机器的完整任务书**——系统前置、
  conda/Isaac Sim、cyclonedds 自编 0.10.5（含 MSVC 与 `__library__.py` 两个必踩坑）、
  该拉哪些分支、GR00T 资产子集、冒烟测试与判读。上面「Windows 部署」章节是**已部署机器**的
  速查与帧率账本，这份是**新机器**的施工顺序，互补不重复。
