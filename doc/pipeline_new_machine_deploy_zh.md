# 流水线双机场景——新机器部署速查（交接用）

> 目标：在一台新 Ubuntu 机器上跑起 **host 全链**（Isaac 双机器人仿真 + 两套 GR00T
> deploy + 可选 Pico VR 控制），并可选配 Windows AR viewer。本文只写施工顺序与
> 必改配置；原理、判读、坑的展开见文末指路。

## 0. 环境组成与机器角色

| 角色 | 机器 | 跑什么 | 必需？ |
|---|---|---|---|
| host | Ubuntu + NVIDIA GPU | sim（robot_1+robot_2 双动力学）+ deploy#1/#2 + pico_manager | ✅ |
| viewer | Windows + NVIDIA GPU | IsaacLab 纯镜像（只收不发）+ SteamVR AR | 可选 |
| 操作端 | Pico 4 Ultra | GameLink app（全身追踪+手柄 → host） | 可选（keyboard 可替代调试） |

参考硬件：host 实测 i5-14600KF/i9 级 CPU + RTX 4070Ti；AR viewer 显存 ≥12GB。

## 1. 仓库与分支（全部在远程，可直接 clone）

| 仓库 | URL | 分支 |
|---|---|---|
| 仿真工程 | `github.com/xiaoyang00001/unitree_sim_isaaclab` | `feat/pipeline-pico-vr-control` |
| GR00T（deploy） | `github.com/muojie/GR00T-WholeBodyControl` | `feat/isaac-state-sync` |
| IsaacLab fork | `github.com/xiaoyang00001/IsaacLab` | `07241`（Linux）/ `0703`（Windows） |

⚠️ **不在 git 里、必须从现有机器拷**：GR00T 的模型文件
`gear_sonic_deploy/policy/release/*.onnx` 与 `planner/target_vel/V2/planner_sonic.onnx`
（拷 `.onnx` 即可，**`.trt` 是 GPU 专属缓存别拷**，新机首跑自动重编）。
大文件走局域网直传，别走代理。

## 2. Ubuntu host 部署步骤

```bash
# ① 系统前置：NVIDIA 驱动 + CUDA、git、git-lfs、miniconda、just、cmake
# ② 仿真工程 + conda 环境（脚本自动装 isaacsim 5.1 / pytorch / cyclonedds 0.10.x / 子模块）
git clone -b feat/pipeline-pico-vr-control https://github.com/xiaoyang00001/unitree_sim_isaaclab
cd unitree_sim_isaaclab
bash auto_setup_env.sh 5.1 env_isaaclab
#   ⚠️ 脚本克隆的是官方 IsaacLab；为与已验证环境一致，把它换成上表 fork 的 07241 分支
#   再 ./isaaclab.sh --install（官方版未在本流水线验证过）
bash fetch_assets.sh          # HuggingFace 资产（assets/ 可放外部盘做符号链接）

# ③ GR00T deploy
git clone -b feat/isaac-state-sync https://github.com/muojie/GR00T-WholeBodyControl
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
| viewer 连谁 | 环境变量 `ISAACLAB_SCENE_SYNC_PEER_IP`（win 侧 bat 里设） | 新 host 的 IP |
| Pico 追踪发给谁 | 头显 `/sdcard/Android/data/com.Nolo.CloudVR/files/XrLinkConfig.json` 的 `wholeBodyTracking.serverHost`（adb 改，重启 app 生效） | 新 host 的 IP |
| 防火墙 | host 入站 | TCP 15555/15556（viewer）、UDP 63901（Pico） |

DDS 全程走 `lo` + domain 1（sim 与 deploy 同机），跨机不需要任何 DDS 配置；
⚠️ 同网段有真机时保持 domain/interface 隔离（工程 CLAUDE.md 开头的告警）。

## 4. 启动与验收（先 keyboard 后 Pico，分两步走）

```bash
# ① keyboard 版验证（不依赖头显，先证明 host 全链健康）：
#    直接跑下面的一键脚本即可——它无头显也能完整拉起（channel#1 停在等发车属正常，
#    不影响物理与锁步），robot_2 走键盘通道：
bash <工程>/tools/pipeline_pico_bringup.sh
printf 'w' >> /tmp/pipeline_pico/dk_r2     # robot_2 前进 = 全链健康
#    验收：两 deploy "Init Done"；sim 日志 physics_steps 持续涨、sync_waits 基本不涨。
#    （手动分步启动、双通道全键盘的形态见权威文档 §2 启动手册）

# ② Pico 版（同一脚本，接上头显侧，日志在 /tmp/pipeline_pico/）
#    验收：pico_manager.log 停止刷 "waiting for body data"（头显数据到）→
#    头显按 A+B+X+Y 发车 → sim 日志出现 "[sonic_dds] CONTROL marker received" →
#    左摇杆推动 robot_1 行走
```

Windows viewer：基础环境按 `doc/windows_deployment_zh.md`（⚠️ cyclonedds 必须自编
0.10.x），流水线增量按 `doc/pipeline_dual_robot_host_viewer_zh.md` §6；拷
`run_pipeline_viewer*.bat` 三个脚本改 PEER_IP 即可（AR 版桌面双击，不能走 ssh）。

## 5. 高频坑（每条都踩过，逐条核对）

1. **两套 deploy 必须并行启动**（错峰 ~20s）——串行等 Init Done 会在双 ack 门下自锁 4Hz；
2. 测前 `pgrep -fa g1_deploy_onnx_ref` **必须为 0**——残留实例 kHz 级轰 ack，数据全不可信；
3. `GR00T_WBC_ROOT` 忘设 → URDF 静默 fallback 旧产物或导入才报错；
4. CRC 三件套别丢：sim 侧 `UNITREE_SKIP_LOWSTATE_CRC=1` + `UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50`、
   deploy 侧 `--disable-crc-check`（缺一侧 = 对端丢帧或白算 CRC，帧率立掉）；
5. pico_manager **必带 `--no_auto_pose`**（否则头显数据一到机器人立刻全身跟随）
   和 `PYTHONUNBUFFERED=1`（否则日志恒空）——bringup 脚本已内置，手动起别丢；
6. 行走演示期间**别发整场景 reset**——deploy planner 会进退化态，唯一恢复=重启 deploy。

## 6. 深入阅读

- `doc/pipeline_dual_robot_host_viewer_zh.md` — 权威文档：三身份、启动手册、帧率账本、坑集
- `doc/pipeline_pico_vr_deployment_zh.md` — Pico 链部署任务书：判读表、双 Pico 预案
- `doc/windows_deployment_zh.md` — Windows 机器从零部署
- 工程根 `CLAUDE.md` — 全部速查入口
