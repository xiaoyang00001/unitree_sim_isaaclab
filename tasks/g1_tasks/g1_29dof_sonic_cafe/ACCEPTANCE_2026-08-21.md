# 双 SONIC 咖啡场景实跑验收报告

## 结论

**部分通过。**

本次验收不是代码静态检查，也不是仅确认 GUI 打开。Linux Host 同时接入两套真实 deploy，
完成双路 LowCmd/LowState 锁步、60 秒计时、独立键盘输入、F12 reset 和 Win130 articulation
Viewer 场景同步。修复后的 Cafe Viewer 与目标 Conveyor Viewer 使用同一完整 G1 几何和同一
白、银、黑参考外观。正式操作端要求通过 OpenXR/AR 启动；该补充验收阻塞在 Pico/SteamVR
runtime session，尚未到达 XR anchor、材质和 AR 画面门禁，因此不能给出最终“通过”。

## 版本

| 项目 | 值 |
|---|---|
| 目标基线分支 | `feat/conveyor-v61-dual-robot-simultaneous-pick` |
| 目标基线提交 | `87e1f4e326ad45e216b7c1b25039b9b0140048f7` |
| Cafe 分支 | `feat/cafe-dual-sonic-on-conveyor-v61` |
| 核心验收提交 | `f12abe2674c2f09a48653ff0b1528980952542d3` |
| Win130 AR 脚本提交 | `454dbd44c6d6ece1b22a8c90d3e5da4510b0a2ce` |
| 验收日期 | `2026-08-21` |

当前 Cafe 分支只新增 Cafe 场景、任务接入、启动器、测试和参考外观资产。Conveyor 场景、机器人
factory、DDS/provider、ZMQ 协议及共享材质绑定逻辑均未修改。

## 机器与网络

| 角色 | 系统 | CPU | GPU/驱动 |
|---|---|---|---|
| Linux Host + 两套 deploy | Ubuntu 24.04.4 | Intel Core i7-14790F | NVIDIA GeForce RTX 5080, 580.173.02 |
| Win130 Viewer, `192.168.1.130` | Windows 10 Pro 22H2 | Intel Core i7-14790F | NVIDIA GeForce RTX 5080, 581.80 |

| 通道 | 配置 |
|---|---|
| Unitree DDS | domain `72`, interface `lo` |
| robot 1 | `rt/*` |
| robot 2 | `rt/r2/*` |
| ZMQ 场景同步 | Linux `192.168.1.131:17555` -> Win130 |

Viewer 明确禁用 Unitree DDS，只接收 ZMQ 场景状态。

## 实际启动命令

Linux Host：

```bash
cd /tmp/unitree_sim_isaaclab-cafe-v61
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
  /home/nolovr/miniconda3/envs/env_isaaclab/bin/python -u sim_main.py \
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

robot 1 deploy：

```bash
cd /home/nolovr/GR00T-WholeBodyControl/gear_sonic_deploy
UNITREE_DDS_DOMAIN=72 UNITREE_DDS_INTERFACE=lo \
bash deploy.sh --disable-crc-check --input-type keyboard --dds-domain 72 isaac
```

robot 2 deploy：

```bash
cd /home/nolovr/GR00T-WholeBodyControl/gear_sonic_deploy
UNITREE_DDS_DOMAIN=72 UNITREE_DDS_INTERFACE=lo G1_LOCAL_ROBOT_ID=2 \
bash deploy.sh --disable-crc-check --input-type keyboard --dds-domain 72 isaac
```

Win130 Viewer：

```bat
cd /d D:\Isaac\unitree_sim_isaaclab-cafe-v61
set ISAACLAB_SCENE_SYNC_PEER_IP=192.168.1.131
set ISAACLAB_SCENE_SYNC_PORT_BASE=17555
call run_cafe_viewer.bat
```

## 单元测试

命令：

```bash
python -m unittest -v \
  tests.test_sonic_cafe_scene \
  tests.test_sonic_dds_host_provider \
  tests.test_sonic_dds_full_lowcmd
```

结果：

```text
Ran 47 tests in 0.038s
OK
```

目标流水线已扩展到五机器人通用 provider，因此当前相关测试集为 47 项，而不是旧 Cafe 分支的
25 项。

## 启动与双路接入日志

```text
[sonic_cafe] topology=host: robot + robot_2 authoritative physics; peer_mode=articulation
[sonic_cafe] KitchenRoom mode=proxy_background payload_roots=44 deinstanced=0 usd_rigid_removed=154 physx_rigid_removed=64 usd_articulation_removed=4 physx_articulation_removed=4 joints_disabled=85 collisions_preserved=0 collisions_removed=298
[sonic_dds] Initial PhysX state seeded for LowState lock-step
[sonic_dds:r2] Initial PhysX state seeded for LowState lock-step
[sonic_dds_host] 2-robot channels ready: robot<-g129(rt/*), robot_2<-g129_r2(rt/r2/*); env.step gates on ALL lock-step acks
```

```text
[sonic_dds:r2] First complete LowCmd applied: enabled=29/29, max|dq_target|=0.0000, max|tau_ff|=0.0000, kp=[14.2506, 99.0984], kd=[0.9072, 6.3088]
[sonic_dds] First complete LowCmd applied: enabled=29/29, max|dq_target|=0.0000, max|tau_ff|=0.0000, kp=[14.2506, 99.0984], kd=[0.9072, 6.3088]
[sonic_dds] CONTROL marker received: releasing the floating base to PhysX
[sonic_dds:r2] CONTROL marker received: releasing the floating base to PhysX
```

## 60 秒双路锁步

外部墙钟窗口：

| 指标 | 结果 |
|---|---:|
| 实际时长 | `60.003574848 s` |
| `physics_steps` | `2926 -> 5926` |
| steps 增量 | `3000` |
| 实际物理/控制频率 | `49.997021 Hz` |
| `stepped=1` | `13/13`, `100%` |
| `sync_waits` | `74 -> 74`, 增量 `0` |
| robot 1 timeouts | `0` |
| robot 2 timeouts | `0` |
| GUI FPS | Linux 约 `12.35-12.59`, Win130 约 `12.40-12.60` |

GUI FPS 与 50 Hz 控制/物理频率分开统计，没有用 GUI FPS 代替锁步频率。

13 条 Performance 样本：

| 指标 | 均值 | 最大值 |
|---|---:|---:|
| A | `2.446 ms` | `3.9 ms` |
| E | `12.223 ms` | `14.4 ms` |
| R | `3.523 ms` | `7.2 ms` |
| S | `2.223 ms` | `6.4 ms` |
| T | `20.408 ms` | `23.7 ms` |

首尾 Performance：

```text
[Performance] A:2.0ms, E:11.5ms, R:6.1ms, S:0.0ms, T:19.7ms, stepped=1, physics_steps=2926, sync_waits=74
[Performance] A:2.2ms, E:14.4ms, R:7.2ms, S:0.0ms, T:23.7ms, stepped=1, physics_steps=5926, sync_waits=74
```

robot 1 首尾同步窗口：

```text
[sonic_dds][sync] expected=0:2998, ack=0:2998, matched=251, timeouts=0, stale_variants=29, wait=0.14/2.19ms(mean/max)
[sonic_dds][sync] expected=0:6006, ack=0:6006, matched=251, timeouts=0, stale_variants=33, wait=0.18/3.28ms(mean/max)
```

robot 2 首尾同步窗口：

```text
[sonic_dds:r2][sync] expected=0:2998, ack=0:2998, matched=251, timeouts=0, stale_variants=19, wait=0.10/2.17ms(mean/max)
[sonic_dds:r2][sync] expected=0:6006, ack=0:6006, matched=251, timeouts=0, stale_variants=31, wait=0.17/2.24ms(mean/max)
```

两路 `expected` 和 `ack` 均增长 `3008`，所有窗口 ACK lag 为 `0`，每个统计周期
`matched=250..251`。未出现约 250 ms 的单路长期等待、`physics_steps=0` 或锁步死锁。

## ZMQ Viewer 同步

F12 后连续采样 10 秒：

| 指标 | 结果 |
|---|---:|
| 墙钟时长 | `10.016107 s` |
| 唯一帧 | `502` |
| `frame_id` | `23153 -> 23654` |
| 丢帧 | `0` |
| 发布/接收频率 | `50.0194 Hz` |
| 平均帧间隔 | `19.992 ms` |
| p95 帧间隔 | `22.336 ms` |
| 最大帧间隔 | `28.048 ms` |
| validation errors | `0` |

502 帧均只包含 `robot_1`、`robot_2`、`cup`。两台机器人始终为 root state 13、
joint position 43、joint velocity 43；杯子 root state 为 13，所有数值 finite。

## Viewer 外观 A/B

初次 Cafe Viewer 虽然创建了完整 articulation G1，但参考外观 USD 被旧 `.gitignore`
排除，没有进入独立 worktree，导致两台机器人保留通体浅灰 DefaultMaterial。该问题不按“几何一致”
掩盖处理，而是补齐目标 Conveyor 实际使用的权威参考资产并重新实跑。

修复后日志：

```text
[sonic_cafe] topology=viewer: peer_robot + peer_robot_2 synchronized mirrors; peer_mode=articulation
[g1_materials] reference appearance applied: white=26, dark=22, logo=1
```

`missing reference` 出现次数为 `0`。每台机器人均为 49 个 Mesh、43 个关节，材质分区与
Conveyor 完全相同：头罩、颈部、Dex3 双手、髋部外壳和双脚为深色，躯干和四肢为银白，
UNITREE 标识一致。Cafe 画面只有两台 G1 和杯子，没有 robot 3-5 或 standby 机器人。

本机证据文件：

| 证据 | 文件 | SHA256 |
|---|---|---|
| Conveyor articulation A/B | `/tmp/conveyor-v61-articulation-viewer-20260821.png` | `225f8bab05ebc58b7b1a6f905b8a6ab88c30234a995e00a91de2f5f74960f7e0` |
| Cafe 修复后 articulation | `/tmp/cafe-v61-articulation-viewer-fixed-20260821.png` | `4778c1941e5e44aa3b78fce9a083c7b70d266213a8a46658d876cd407f666e4d` |
| Cafe F12 后画面 | `/tmp/cafe-v61-articulation-viewer-post-reset-20260821.png` | `9ee80c52f054d489bcfeeb7d25571a91b9baf4f3b3911105a7b3b9e217e2469f` |

## 键盘输入隔离

robot 1 deploy 单独输入 `q`：

```text
Delta heading left: 0.261799 rad
```

robot 2 deploy 单独输入 `e`：

```text
Delta heading right: -0.261799 rad
```

两条日志分别只出现在对应 deploy 文件中，DDS topics 分别保持 `rt/*` 和 `rt/r2/*`，未发现串线。

## F12 reset

Host 日志：

```text
[keyboard] F12 pressed: full scene reset queued
[reset_input] Ubuntu keyboard F12: full scene reset
[g1_robot] Isaac reset epoch -> 1 (Ubuntu keyboard F12)
[g1_robot_r2] Isaac reset epoch -> 1 (Ubuntu keyboard F12)
[fall_reset] reset #1 complete: default standing state restored; waiting for safe SONIC re-entry
[sonic_dds] FALL RECOVERY complete: normal SONIC control restored
[sonic_dds:r2] FALL RECOVERY complete: normal SONIC control restored
```

reset 后首个完整双路同步窗口：

```text
[sonic_dds][sync] expected=1:9714, ack=1:9714, matched=251, timeouts=0, stale_variants=28, wait=0.15/2.34ms(mean/max)
[sonic_dds:r2][sync] expected=1:9714, ack=1:9714, matched=251, timeouts=0, stale_variants=14, wait=0.08/2.30ms(mean/max)
```

ZMQ 首次 reset ID 为 `d9a6dd1230b24509b764c0ec838f3ba6:0`。其 session 与初始场景 ID 不同，
符合 Conveyor 协议从 sequence 0 开始的定义。

复位后的实测状态：

| 实体 | 根位置 |
|---|---|
| robot 1 | `(0.171240, -0.371775, 0.786133)` |
| robot 2 | `(0.264757, 0.752070, 0.786136)` |
| cup | `(0.019904, 0.039838, 0.906918)` |

机器人相对配置出生位的最大偏差约 4.82 cm，来自落地和站姿收敛；杯子偏差约 3.1 mm。
Win130 视觉确认两台机器人双脚着地、分别位于岛台两侧，杯子回到台面，无掉落、重叠或多余机器人。

## Win130 OpenXR/AR 补充验收

正式 Win130 入口：

```bat
D:\Isaac\unitree_sim_isaaclab-cafe-v61\run_cafe_viewer_ar_131.local.bat
```

该脚本参考 Win130 现有 `run_pipeline_viewer_ar_131.local.bat`：连接 Ubuntu Host
`192.168.1.131:17555`，将 XR anchor 绑定 `robot_2`，再调用 Cafe 通用 OpenXR 入口。

已经通过的启动门禁：

```text
[ext: omni.kit.xr.system.openxr-107.3.109] startup
Loading experience file: ...isaaclab.python.xr.openxr.kit
[viewer] Unitree DDS disabled; ZMQ scene sync only
[sonic_cafe] ... peer_mode=articulation
```

随后 OpenXR runtime 创建实例失败：

```text
Error [GENERAL | xrCreateInstance | OpenXR-Loader] : LoaderInstance::CreateInstance chained CreateInstance call failed
Error [GENERAL | xrCreateInstance | OpenXR-Loader] : xrCreateInstance failed
[24.016s] Simulation App Shutting Down
```

运行环境诊断：

| 项目 | 结果 |
|---|---|
| Active OpenXR runtime | SteamVR `steamxr_win64.json` |
| SteamVR | `vrserver`、`vrcompositor`、`vrmonitor`、`vrdashboard` 均在 Session 2 |
| NOLO | `XRLink` 在 Session 2，虚拟 `NOLO-HMD-1` 处于 standby |
| ALVR | 未安装，无 Dashboard 或运行进程 |
| Pico `192.168.1.190` | ping 超时/目标不可达，ARP 无记录 |
| AR sim_main | 失败后正常退出，无残留 Python 进程 |

由于 Pico client 未连接，未满足重试门禁。以下项目没有到达，不能用普通 Viewer 结果替代：

```text
XR anchor -> PeerRobot2
[g1_materials] reference appearance applied: white=26, dark=22, logo=1
OpenXR mirror window / headset image
```

失败证据：

| 证据 | 文件 | SHA256 |
|---|---|---|
| Cafe AR 完整失败日志 | `/tmp/cafe-v61-ar-viewer-20260821.log` | `1fbb8dce2aa2de43ca814ff862e6764e607a087322452e9aa6ef4e12ad0d06c4` |
| SteamVR 状态截图 | `/tmp/win130-steamvr-status-20260821.png` | `691d253090753b0d07eeb5b96c5793837526cf2aebf6afcdc19fc7707c2f99c1` |
| XRLink 状态截图 | `/tmp/win130-xrlink-status-20260821.png` | `cd5966627de6547da3037b8236da4a3c8d5658c430fed49c2ae41787df23d3bc` |

结论：AR 启动脚本和 OpenXR kit 路由已经接通，但外部头显链路未 ready。本项写入 TODO，
Pico 恢复在线并在 XRLink 或 ALVR 明确显示 client connected 后，必须重新实跑才能将总结果升级为“通过”。

## 非阻塞警告

- Lightwheel KitchenRoom 缺少 `Materials/Textures/3d66Model-7443619-files-8.jpg` 法线贴图。
- 两套 deploy 检测到 TensorRT engine 跨设备复用警告。
- `proxy_background` 只保留地面与岛台两个任务必需碰撞代理，不提供完整柜体和墙体交互。

这些非阻塞项目没有影响本次双路锁步、咖啡杯承托、普通 Viewer 外观、F12 reset 或 50 Hz 结论，已记录到
[TODO_sonic_cafe_acceptance.md](../../../TODO_sonic_cafe_acceptance.md)。
