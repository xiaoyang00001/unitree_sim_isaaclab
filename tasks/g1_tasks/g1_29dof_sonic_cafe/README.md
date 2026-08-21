# 双 SONIC 咖啡场景

`Isaac-G1-29DoF-Sonic-Cafe` 是基于
`feat/conveyor-v61-dual-robot-simultaneous-pick` 双机器人流水线架构实现的咖啡交接场景。
本任务只替换 KitchenRoom 背景、咖啡杯、双机器人站位和交接任务锚点，不复制或改写
Conveyor 的机器人、DDS、Viewer、场景同步和复位实现。

实跑报告见 [ACCEPTANCE_2026-08-21.md](./ACCEPTANCE_2026-08-21.md)，非阻塞后续项见
[TODO_sonic_cafe_acceptance.md](../../../TODO_sonic_cafe_acceptance.md)。

## 机器人拓扑约束

- Host 只创建 `robot` 和 `robot_2` 两台权威物理 G1。
- Viewer 只创建 `peer_robot` 和 `peer_robot_2` 两台同步镜像 G1。
- Cafe 不创建 robot 3-5，也不创建 Conveyor 的三个 standby 显示机器人。
- 本机机器人直接复用 Conveyor 的 `_make_local_robot_cfg()` 和 `_make_additional_local_robot_cfg(2)`。
- Viewer 直接复用 Conveyor 的 `_make_peer_scene_cfg()` 和 `_make_additional_peer_scene_cfg(2)`。
- Viewer 固定使用 `articulation`，不得设置 `ISAACLAB_PEER_ROBOT_MODE=visual_lod`。
- 两台机器人均为完整 43-DoF G1，包含 29 个本体关节和 14 个 Dex3 手部关节。
- DDS 通道保持流水线约定：robot 使用 `rt/*`，robot 2 使用 `rt/r2/*`。
- ZMQ 场景帧只包含 `robot_1`、`robot_2` 和 `cup`。

## 外部资产

KitchenRoom 不在仓库中，需要：

```text
<Lightwheel根目录>/Locomotion/KitchenRoom/KitchenRoom.usd
```

启动前设置：

```bash
export LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR=<Lightwheel根目录>
export GR00T_WBC_ROOT=<GR00T-WholeBodyControl路径>
```

G1 参考外观资产已纳入当前分支：

```text
assets/robots/g1-29dof_wholebody_dex3/g1_29dof_with_dex3_rev_1_0.usd
SHA256: 01677e6ab1d321e72533dc42393500c17655584d7f272b328857ab5080ef7c98
```

该 USD 是单层自包含资产，仅使用 Isaac 内置 `OmniPBR.mdl`。它是 Conveyor/Cafe
恢复相同白、银、黑分色所必需的材质来源。

## KitchenRoom 物理模式

默认 `ISAACLAB_CAFE_KITCHEN_MODE=proxy_background`：

- 保留完整 KitchenRoom 视觉背景。
- 移除背景中的动态刚体、articulation、joint 和 298 个复杂碰撞体。
- 使用两个简单静态碰撞代理承托机器人和杯子：地面与岛台。
- KitchenRoom 自带灯光保留，父场景的额外 DomeLight 被禁用。

可选模式：

```text
proxy_background  默认模式，适合当前咖啡交接任务
static_background 保留 KitchenRoom 静态碰撞
legacy_dynamic    保留第三方资产原始物理配置，仅用于回归
```

## 启动 Linux Host

以下是 2026-08-21 验收使用的配置。DDS domain `72` 用于隔离本次实跑，可按现场统一调整，
但 Isaac 与两套 deploy 必须完全一致。

```bash
cd <Cafe仓库工作树>

env \
  -u ISAACLAB_PEER_ROBOT_MODE \
  -u ISAACLAB_SCENE_SYNC_BIND_ENDPOINT \
  -u ISAACLAB_SCENE_SYNC_CONNECT_ENDPOINT \
  PYTHONUNBUFFERED=1 \
  GR00T_WBC_ROOT=/home/nolovr/GR00T-WholeBodyControl \
  LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR=/home/nolovr/Lightwheel_OpenSource \
  UNITREE_DDS_DOMAIN=72 \
  UNITREE_DDS_INTERFACE=lo \
  ISAACLAB_LOCAL_ROBOT_ID=1 \
  ISAACLAB_HOST_BOTH_ROBOTS=1 \
  ISAACLAB_SCENE_SYNC=1 \
  ISAACLAB_SCENE_SYNC_PEER_IP=127.0.0.1 \
  ISAACLAB_SCENE_SYNC_PORT_BASE=17555 \
  ISAACLAB_CAFE_KITCHEN_MODE=proxy_background \
  taskset -c 0-15 \
  python -u sim_main.py \
    --task Isaac-G1-29DoF-Sonic-Cafe \
    --robot_type g129 \
    --action_source sonic_dds \
    --device cpu \
    --dds-domain 72 \
    --dds-interface lo \
    --late_render_interval 4 \
    --stats_interval 5 \
    --profile_interval 250 \
    --sim-state-export-hz 0
```

Host 必须出现：

```text
[sonic_cafe] topology=host: robot + robot_2 authoritative physics; peer_mode=articulation
[sonic_dds] Initial PhysX state seeded for LowState lock-step
[sonic_dds:r2] Initial PhysX state seeded for LowState lock-step
[sonic_dds_host] 2-robot channels ready
```

## 启动两套 deploy

两套 deploy 应在 20 秒内并行启动。出现确认提示时输入 `y`，两套均 `Init Done` 后分别按 `]`
进入 CONTROL。

robot 1：

```bash
cd "$GR00T_WBC_ROOT/gear_sonic_deploy"
UNITREE_DDS_DOMAIN=72 UNITREE_DDS_INTERFACE=lo \
bash deploy.sh --disable-crc-check --input-type keyboard --dds-domain 72 isaac
```

robot 2：

```bash
cd "$GR00T_WBC_ROOT/gear_sonic_deploy"
UNITREE_DDS_DOMAIN=72 UNITREE_DDS_INTERFACE=lo G1_LOCAL_ROBOT_ID=2 \
bash deploy.sh --disable-crc-check --input-type keyboard --dds-domain 72 isaac
```

控制建立后必须出现：

```text
[sonic_dds] CONTROL marker received
[sonic_dds:r2] CONTROL marker received
[sonic_dds] First complete LowCmd applied: enabled=29/29
[sonic_dds:r2] First complete LowCmd applied: enabled=29/29
```

## 启动 Win129/Win130 AR Viewer

正式 Viewer 使用 OpenXR/AR，不运行 Unitree DDS，只通过 ZMQ 接收场景状态和 reset 事件。
先启动 NOLO Link 或 ALVR，等待 SteamVR 显示头显 ready。

Win129 作为 operator 1，OpenXR anchor 跟随 `robot_1`，从交互桌面双击：

```bat
D:\Isaac\unitree_sim_isaaclab-cafe-v61\run_cafe_viewer_ar_win129.local.bat
```

Win130 作为 operator 2，OpenXR anchor 跟随 `robot_2`，从交互桌面双击：

```bat
D:\Isaac\unitree_sim_isaaclab-cafe-v61\run_cafe_viewer_ar_win130.local.bat
```

两个本机脚本都固定连接 Ubuntu Host `192.168.1.131:17555`，并分别将 OpenXR
anchor 绑定到对应机器人镜像。它们最终调用同一个通用入口：

```bat
run_cafe_viewer_ar.bat
```

operator 1 或其他 Host 可直接设置环境变量后调用通用入口：

```bat
cd /d D:\Isaac\unitree_sim_isaaclab-cafe-v61
set PIPELINE_HOST_IP=<Ubuntu Host IP>
set ISAACLAB_XR_ANCHOR_ROBOT_ID=1
call run_cafe_viewer_ar.bat
```

`run_cafe_viewer.bat` 是非 AR 诊断入口，不作为正式 Viewer 启动方式。

Viewer 必须出现：

```text
[viewer] Unitree DDS disabled; ZMQ scene sync only
[sonic_cafe] topology=viewer: peer_robot + peer_robot_2 synchronized mirrors; peer_mode=articulation
[g1_materials] reference appearance applied: white=26, dark=22, logo=1
```

如果看到 `reference G1 visual USD is missing` 或机器人通体浅灰，应停止验收；这表示参考外观资产
没有进入当前工作树。不要改用 `visual_lod` 掩盖问题。

## 复位

在 Linux Isaac 窗口按 `F12`：

- 两台物理机器人与杯子恢复默认位姿。
- 两路 DDS reset epoch 同时递增。
- 两套 deploy 清空旧观测历史并重新建立 ACK。
- Viewer 接收新的 ZMQ reset ID 和后续场景帧。

ZMQ reset sequence 从 `0` 开始，因此第一次 F12 的 ZMQ ID 后缀是 `:0`；DDS 第一次复位的
epoch 是 `1`。验收应判断 reset ID 是否发生变化，不应要求两个协议的显示编号相同。
