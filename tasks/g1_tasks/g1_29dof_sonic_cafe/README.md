# 双 SONIC 递咖啡场景

本任务把 `~/xiaoyang_IssacLab/IsaacLab` 的 `scene-caffe` 分支中已调过站位的
Lightwheel KitchenRoom 场景接到本仓库既有双 SONIC host 控制链。

场景资产默认读取：

```text
/home/nolo/Lightwheel_OpenSource/Locomotion/KitchenRoom/KitchenRoom.usd
```

其他机器可用 `LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR` 指定 Lightwheel 根目录。任务不会
复制这份约 170 MB 的第三方资产，也不会改写源资产。

启动 Isaac 端：

```bash
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl python sim_main.py \
  --task Isaac-G1-29DoF-Sonic-Cafe \
  --robot_type g129 \
  --action_source sonic_dds
```

此任务固定启用两个本机 SONIC 通道：

- `robot`：`rt/*`
- `robot_2`：`rt/r2/*`

因此两套 SONIC deploy 都必须启动；主循环会等待两路 LowCmd 锁步确认。第二路 deploy
的启动参数、DDS 前缀和进程隔离方式沿用现有 conveyor 双机器人 host 方案。

场景保留 `RobotSpawnA`、`RobotSpawnB`、`CupSpawn`、`HandoverZone`、`ServeZone`、
`ViewerAnchor` 六个逻辑锚点。当前任务用于 SONIC 手动协作验证，不引入源分支的 IK、
OpenXR 遥操作、ZMQ 杯子同步或阶段终止逻辑；F12/DDS reset 会把两台机器人和杯子
一起恢复到默认位姿。

Lightwheel 原始 `KitchenRoom.usd` 在当前资产包内仍有两个缺失的
`metricsAssembler` 子层、一个写死为 `/Downloads/Table049/...` 的 payload，以及部分
嵌套刚体 schema 警告。Isaac Sim 5.1 能完成场景和两台机器人的创建，但 Table049
可能缺失，背景里部分可动电器的物理行为不保证可靠；本任务不改写这份第三方 USD。
