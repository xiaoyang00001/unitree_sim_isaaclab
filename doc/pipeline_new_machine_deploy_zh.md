# 流水线多机场景——新机器部署速查（交接用）

> 目标：在一台新 Ubuntu 机器上跑起 **host 全链**（生产默认 Isaac 双机器人仿真 + 两套
> GR00T deploy + 可选 Pico VR 控制；三/四/五路可显式启用），并可选配 Windows AR viewer。
> 本文只写施工顺序与必改配置；原理、判读、坑的展开见文末指路。

## 0. 环境组成与机器角色

| 角色 | 机器 | 跑什么 | 必需？ |
|---|---|---|---|
| host | Ubuntu + NVIDIA GPU | sim（默认 robot_1/2 两台动力学，robot_3..5 站位为空）+ deploy#1/2 + pico_manager | ✅ |
| viewer | Windows + NVIDIA GPU | IsaacLab 纯镜像（只收不发）+ SteamVR AR | 可选 |
| 操作端 | Pico 4 Ultra | GameLink app（全身追踪+手柄 → host） | 可选（keyboard 可替代调试） |

参考硬件：host 实测 i5-14600KF/i9 级 CPU + RTX 4070Ti；AR viewer 显存 ≥12GB。

## 1. 仓库与分支（全部在远程，可直接 clone）

| 仓库 | URL | 分支 |
|---|---|---|
| 仿真工程 | `github.com/xiaoyang00001/unitree_sim_isaaclab` | `feat/conveyor-v61-all-robots-sonic` |
| GR00T（deploy） | `github.com/muojie/GR00T-WholeBodyControl` | `fix/pico-invalid-body-frame`，至少包含 `122c947` |
| IsaacLab fork | `github.com/xiaoyang00001/IsaacLab` | `07241`（Linux）/ `0703`（Windows） |

⚠️ **不在 git 里、必须从现有机器拷**：GR00T 的模型文件
`gear_sonic_deploy/policy/release/*.onnx` 与 `planner/target_vel/V2/planner_sonic.onnx`
（拷 `.onnx` 即可，**`.trt` 是 GPU 专属缓存别拷**，新机首跑自动重编）。
大文件走局域网直传，别走代理。

## 2. Ubuntu host 部署步骤

```bash
# ① 系统前置：NVIDIA 驱动 + CUDA、git、git-lfs、miniconda、just、cmake
# ② 仿真工程 + conda 环境（脚本自动装 isaacsim 5.1 / pytorch / cyclonedds 0.10.x / 子模块）
git clone -b feat/conveyor-v61-all-robots-sonic https://github.com/xiaoyang00001/unitree_sim_isaaclab
cd unitree_sim_isaaclab
bash auto_setup_env.sh 5.1 env_isaaclab
#   ⚠️ 必做：脚本克隆的是官方 IsaacLab，必须切到 fork 的 07241 分支——官方版未在本
#   流水线验证过，且缺 fork 上的 SONIC 修复（如 G1 43-DoF USD 查找顺序）。
#   editable 安装指向源码目录，切分支即生效，通常无需重跑 --install：
cd <IsaacLab目录>
git remote add xiaoyang https://github.com/xiaoyang00001/IsaacLab
git fetch xiaoyang 07241 && git checkout -b 07241 xiaoyang/07241
#   漏做的指纹：sim 启动 ~4.7s 报 No module named 'isaaclab_physx'（isaaclab_tasks
#   扩展加载失败 → app ready 后 SIGSEGV）。07241 里没有这个包也没有这行 import——
#   见到它 = IsaacLab 停在官方/更新版本上。别按报错去补装 isaaclab_physx（那是把
#   未验证的版本修得能跑），切分支才是正解。
bash fetch_assets.sh          # HuggingFace 资产（assets/ 可放外部盘做符号链接）

# ③ GR00T deploy
git clone -b fix/pico-invalid-body-frame https://github.com/muojie/GR00T-WholeBodyControl
#   Pico 多机一键 HandCmd 100 Hz 要求至少包含参数提交 0f4e0b4，且必须包含
#   定时校准修复 122c947；仅有前者会把配置的 100 Hz 实际量化到约 84–85 Hz。
#   拷入 §1 说的 .onnx 模型 → 按仓库 README 构建 deploy（CMake/just，需 CUDA+TensorRT）
#   首次务必【单实例】跑一遍 deploy.sh 预热 .trt 缓存（双实例并发冷启会争写缓存）
#   仅 Pico 控制需要：bash install_scripts/install_pico.sh   # 创建 .venv_teleop
#   （udp 模式不需要 XRoboToolkit PC Service，不用装 .deb）

# ④ 环境变量（写进 ~/.bashrc）
export GR00T_WBC_ROOT=/绝对路径/GR00T-WholeBodyControl   # 必设！缺了会静默 fallback
```

## 3. 必改配置（换机器 = 换 IP，就这几处）

| 配置点 | 在哪 | 改成什么 |
|---|---|---|
| bringup 脚本路径 | 环境变量 `PIPELINE_SIM_DIR` / `GR00T_WBC_ROOT` / `PIPELINE_SIM_PY` | 新机实际路径（脚本内有默认值） |
| viewer 连谁 | win 侧 `run_pipeline_viewer*.bat` 的 `PIPELINE_HOST_IP` 默认值（或设同名系统环境变量） | 新 host 的 IP |
| Pico 追踪发给谁 | 机器人专用强刷版改 APK 内置 `assets/XrLinkConfig.json` 后重打包；旧版才直接改头显 files 副本。安装、签名切换与回读规则见 `doc/pipeline_pico_vr_deployment_zh.md` §1.1 | 新 host 的 IP |
| 防火墙 | host 入站 | TCP 15555/15556（viewer）、UDP 63901（Pico#1）；双 Pico 再放行 UDP 63902 |

DDS 全程走 `lo` + domain 1（sim 与 deploy 同机），跨机不需要任何 DDS 配置；
⚠️ 同网段有真机时保持 domain/interface 隔离（工程 CLAUDE.md 开头的告警）。

## 4. 运行命令

### 4.1 一键（推荐，日志落 `/tmp/pipeline_pico/`）

```bash
cd <仿真工程>
PIPELINE_SIM_DIR=$PWD GR00T_WBC_ROOT=<GR00T路径> \
PIPELINE_SIM_PY=$(conda run -n env_isaaclab which python) \
bash tools/pipeline_pico_bringup.sh

# 双 Pico 控制（头显#2 必须已按 Pico 任务书配置为 UDP 63902）：
PIPELINE_DUAL_PICO=1 PIPELINE_SIM_DIR=$PWD GR00T_WBC_ROOT=<GR00T路径> \
PIPELINE_SIM_PY=$(conda run -n env_isaaclab which python) \
bash tools/pipeline_pico_bringup.sh
```

默认拉起 sim + deploy#1(zmq_manager) + deploy#2(keyboard) + pico_manager 全套，并对
每个 Isaac deploy 显式传 `--isaac-handcmd-hz 100`；robot_3..5 站位为空，只有显式
`PIPELINE_SONIC_ROBOT_COUNT=3|4|5` 才增加对应真身和 deploy。LowCmd、锁步 ACK、Control 和 Planner
仍保持 500 Hz。外仓 standalone Isaac 默认及非 Isaac/实机路径也仍为 500 Hz。
一键脚本的生产默认同时为 `PIPELINE_SONIC_MERGE_ACTUATORS=1`、
`PIPELINE_SONIC_VALIDATE_ACTUATORS=0`：host 使用 43 关节单组执行器，正常启动不做
GPU→CPU tensor 校验。核心 EnvCfg 的 merge 回退值仍为 `0`，但在线多机器人 Host
未显式覆盖时 `sim_main.py` 会选择 `1`；validate 仍默认 `0`。
本地 Host 预览默认每 4 个控制圈渲染一次，不节流 ZMQ scene-state；
需要每圈本地画面时显式设 `PIPELINE_HOST_LATE_RENDER_INTERVAL=1`。
`PIPELINE_DUAL_PICO=1` 把 deploy#2 换成 zmq_manager 并追加 manager#2，端口矩阵与实机
gate 见 `doc/pipeline_pico_vr_deployment_zh.md` §5。
**无头显也能跑**（channel#1 停在等发车属正常，不影响物理与锁步）。

四/五路能力测试必须完整重启 bringup，例如：

```bash
PIPELINE_SONIC_ROBOT_COUNT=5 PIPELINE_SIM_DIR=$PWD GR00T_WBC_ROOT=<GR00T路径> \
PIPELINE_SIM_PY=$(conda run -n env_isaaclab which python) \
bash tools/pipeline_pico_bringup.sh
```

当前 clean-load 选择依据：四机基线 `2705 / 120.28 = 22.489192 Hz`，空闲纸箱 velocity
写入跳过候选 `2732 / 120.21 = 22.726895 Hz`，只提升 1.057%，未达到 5% 采用门槛；
历史三机 clean-load 配置 `3929 / 120.15 = 32.700791 Hz`，比四机基线提升 45.41%，三路
timeout/stale/`sync_waits` 均为 0 且姿态健康。因此生产不打开空闲写候选，核心
`ISAACLAB_CONVEYOR_SKIP_IDLE_VELOCITY_WRITES` 默认仍为 `0`；
`ISAACLAB_CONVEYOR_CONTACT_HISTORY_LENGTH` 默认仍为 `4`。

新机首次或升级 GPU/Isaac Lab 后，做一次带 tensor 契约门禁的完整启动；验收通过后下一次
完整启动恢复默认 `validate=0`。若单组有兼容性回归，以 `merge=0` 完整重启即可恢复原始
6 组，不能在运行中热切换：

```bash
# 一次性验收
PIPELINE_SONIC_VALIDATE_ACTUATORS=1 PIPELINE_SIM_DIR=$PWD GR00T_WBC_ROOT=<GR00T路径> \
PIPELINE_SIM_PY=$(conda run -n env_isaaclab which python) \
bash tools/pipeline_pico_bringup.sh

# 完整回滚原始 6 组
PIPELINE_SONIC_MERGE_ACTUATORS=0 PIPELINE_SONIC_VALIDATE_ACTUATORS=0 \
PIPELINE_SIM_DIR=$PWD GR00T_WBC_ROOT=<GR00T路径> \
PIPELINE_SIM_PY=$(conda run -n env_isaaclab which python) \
bash tools/pipeline_pico_bringup.sh
```

遇到兼容性问题时，用下面命令重启完整 bringup 回滚 HandCmd 流量；
`PIPELINE_ISAAC_HANDCMD_HZ` 不是运行期热更新：

```bash
PIPELINE_ISAAC_HANDCMD_HZ=500 PIPELINE_SIM_DIR=$PWD GR00T_WBC_ROOT=<GR00T路径> \
PIPELINE_SIM_PY=$(conda run -n env_isaaclab which python) \
bash tools/pipeline_pico_bringup.sh
```

⚠️ 三个易错点（新机首跑都踩过）：

1. **三个变量必须与命令同一行**（如上，行尾 `\` 续行）或先 `export`——分行裸赋值
   不会传给子进程，脚本会改用当前 checkout、`$HOME/GR00T-WholeBodyControl` 和
   `$HOME/miniconda3/envs/env_isaaclab/bin/python`；自定义安装路径会因此跑偏；
2. **变量别指串**：`PIPELINE_SIM_DIR`=仿真工程目录，`GR00T_WBC_ROOT`=GR00T 目录，
   `PIPELINE_SIM_PY`=conda 环境的 python；
3. **GUI 形态需要活动的 X 桌面会话**（脚本默认 `--hide_ui`+DISPLAY）。纯 ssh /
   无显示器的机器加 `PIPELINE_HEADLESS=1` 前缀（切 `--no_render`，host 本地无画面，
   物理/锁步/viewer 不受影响）。脚本已内置 X 预检，拿不到会直接报错并给修法。

### 4.2 手动分步（两机排查用，四个终端；deploy 频率等价于一键脚本）

```bash
# 终端① host sim
conda activate env_isaaclab && cd <仿真工程>
GR00T_WBC_ROOT=<GR00T路径> UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
ISAACLAB_LOCAL_ROBOT_ID=1 ISAACLAB_HOST_BOTH_ROBOTS=1 \
ISAACLAB_SONIC_ROBOT_COUNT=2 \
ISAACLAB_SONIC_MERGE_ACTUATORS=1 \
UNITREE_SKIP_LOWSTATE_CRC=1 UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50 \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
  --action_source sonic_dds --device cpu --hide_ui --stats_interval 10 \
  --late-render-interval 4

# 终端②③ 两套 deploy —— ⚠️ 必须并行起（错峰 ~20s），串行等 Init Done 会自锁
cd <GR00T路径>/gear_sonic_deploy
bash deploy.sh --disable-crc-check --input-type zmq_manager \
  --isaac-handcmd-hz 100 isaac                                      # 终端② deploy#1（Pico）
# keyboard 验证形态把上行的 zmq_manager 换成 keyboard
G1_LOCAL_ROBOT_ID=2 bash deploy.sh --disable-crc-check --input-type keyboard \
  --isaac-handcmd-hz 100 isaac                                      # 终端③ deploy#2

# 终端④ pico_manager（仅 Pico 控制需要；两个参数一个都不能丢）
cd <GR00T路径>
XROBO_TRANSPORT=udp PYTHONUNBUFFERED=1 .venv_teleop/bin/python \
  gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose --port 5556
```

手动 host 中的 `ISAACLAB_SONIC_MERGE_ACTUATORS=1` 是可复现的生产显式值；
在线多机器人 Host 未传时 `sim_main.py` 也会选择 `1`。
首次验收可再加 `ISAACLAB_SONIC_VALIDATE_ACTUATORS=1`，常态/回滚规则与 §4.1 相同，均需
完整停止并重新创建 host 场景。

### 4.3 控制与验收

```bash
# keyboard 通道（deploy 终端里直接敲，或 bringup 的键管道）：
#   激活序列 ]（开始控制）→ 回车（开 planner）→ 2（行走模式）→ w/s/a/d 移动
printf 'w' >> /tmp/pipeline_pico/dk_r2     # 一键版给 robot_2 发"前进"
# Pico 通道（头显上）：A+B+X+Y 发车（站直了按）→ 左摇杆行走/右摇杆转向；
#   A+B 升档、X+Y 降档；A+X 切全身跟随；再按 A+B+X+Y 急停
```

验收判据（按序核对）：
1. 两 deploy 日志出现 `Init Done`、`Dex3 HandCmd Rate: 100 Hz` 和
   `Isaac Dex3 HandCmd publish rate: 100 Hz`；
2. sim 日志 `physics_steps` 持续涨、`sync_waits` 基本不涨（涨=锁步卡了）；
3. keyboard：`w` 后 robot_2 行走（起步有 ~10s 慢加速斜坡，别当没响应）；
4. Pico：`pico_manager.log` 停止刷 `waiting for body data` → 头显发车后 sim 出现
   `[sonic_dds] CONTROL marker received` → 摇杆推动 robot_1 行走。

### 4.4 Windows viewer（可选）

基础环境按 `doc/windows_deployment_zh.md`（⚠️ cyclonedds 必须自编 0.10.x），流水线
增量按权威文档 §6；拷工程根的 `run_pipeline_viewer*.bat` 三个脚本、把里面
`PIPELINE_HOST_IP` 的默认值改成新 host IP（或设同名系统环境变量），桌面双击运行
（AR 版不能走 ssh）：

```
run_pipeline_viewer.bat            # 纯镜像（无 AR）
run_pipeline_viewer_ar.bat         # AR，视角跟 robot_1（操作者#1）
run_pipeline_viewer_ar_robot2.bat  # AR，视角跟 robot_2（操作者#2）
```

AR 视角语义（2026-08-04 定稿）：**位置**跟机器人头（平滑 0.15s，
`ISAACLAB_XR_ANCHOR_POS_SMOOTHING` 可调）；**旋转**只听操作者自己的头，
不跟机器人转身（跟转实测头晕）——与机器人朝向错位时按 **右手柄 B（松开触发）
或 win 键盘 F9** 对正，启动时自动对正一次。`ISAACLAB_XR_ANCHOR_ROT_FOLLOW=1`
恢复 yaw 跟随模式。

## 5. 高频坑（每条都踩过，逐条核对）

1. **两套 deploy 必须并行启动**（错峰 ~20s）——串行等 Init Done 会在双 ack 门下自锁 4Hz；
2. 测前 `pgrep -fa g1_deploy_onnx_ref` **必须为 0**——残留实例 kHz 级轰 ack，数据全不可信；
3. `GR00T_WBC_ROOT` 忘设 → URDF 静默 fallback 旧产物或导入才报错；
4. CRC 三件套别丢：sim 侧 `UNITREE_SKIP_LOWSTATE_CRC=1` + `UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50`、
   deploy 侧 `--disable-crc-check`（缺一侧 = 对端丢帧或白算 CRC，帧率立掉）；
5. pico_manager **必带 `--no_auto_pose`**（否则头显数据一到机器人立刻全身跟随）
   和 `PYTHONUNBUFFERED=1`（否则日志恒空）——bringup 脚本已内置，手动起别丢；
6. 行走演示期间**别发整场景 reset**——deploy planner 会进退化态，唯一恢复=重启 deploy。
7. **ssh 会话直接跑 GUI 形态 → sim 启动 ~5s 段错误**：kit 拿不到 X（日志指纹
   `GLFW initialization failed`×3 → RTX `carbOnPluginStartup` SIGSEGV）。修法=
   本地桌面终端跑，或 `PIPELINE_HEADLESS=1`。**`--device cpu` 与此无关**
   （只切 PhysX 后端、不关渲染），别为排查这个去掉它——GPU 要留给两套 TensorRT deploy。
8. **`No module named 'isaaclab_physx'` → SIGSEGV**：IsaacLab 没切到 fork 07241
   分支（§2 那步漏了）。切分支即修，**别补装 isaaclab_physx**。
9. **AR 视角跟不上机器人/晃动、B 键 recenter 无效**：viewer 机的 IsaacLab fork
   `devices/openxr` 三件套（openxr_device / xr_anchor_utils / xr_cfg）是旧版——
   缺 pelvis 旋转锚分离与 recenter 绑定（sim 日志打印 rotation_anchor= 只是 cfg
   属性回显，**不证明 fork 在用**）。与 Linux 07241 同步三件套即修；B 键回传
   **已实测可达**（2026-08-04 松开 B 对正生效；此前"不回传"是误判——零触发只因
   旧代码没绑定），另有启动自动对正 + GUI 窗口 **F9** 双兜底；抖动旋钮 `ISAACLAB_XR_ANCHOR_POS_SMOOTHING`（默认 0.15s）。

## 6. 深入阅读

- `doc/pipeline_dual_robot_host_viewer_zh.md` — 权威文档：三身份、启动手册、帧率账本、坑集
- `doc/pipeline_pico_vr_deployment_zh.md` — Pico 链部署任务书：判读表、双 Pico 预案
- `doc/windows_deployment_zh.md` — Windows 机器从零部署
- 工程根 `CLAUDE.md` — 全部速查入口
