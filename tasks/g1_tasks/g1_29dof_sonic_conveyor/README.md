# Isaac-G1-29DoF-Sonic-Conveyor

SONIC DDS 控制的 G1 + warehouse 流水线场景 + ZMQ 双机场景同步。

场景与流水线驱动移植自 IsaacLab 分叉 `feat/conveyor-loop-totes-wip`
（tip 17e71c0a0）的 `pick_place` 任务；机器人与 DDS/XR/观测链路沿用本工程
`g1_29dof_dex3_sonic` 的 SONIC 底座。方案与勘查记录见该分叉仓库对话存档。

## 启动

```bash
cd ~/unitree_sim_isaaclab
conda activate env_isaaclab

# 每台机器先跑一次：生成无碰撞镜像机器人 USD（缺失时任务启动会 fail-fast 提示）
python tools/build_peer_robot_usd.py

# 1 号机（物体权威 + 复位权威，流水线驱动在这端跑）
UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
GR00T_WBC_ROOT=$HOME/GR00T-WholeBodyControl \
ISAACLAB_LOCAL_ROBOT_ID=1 ISAACLAB_SCENE_SYNC_PEER_IP=<对端IP> \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor \
  --robot_type g129 --action_source sonic_dds --device cpu \
  --teleop_device motion_controllers --xr

# 2 号机（镜像端）：同命令，改 ISAACLAB_LOCAL_ROBOT_ID=2 与对端 IP
```

单机自测（不建 socket）：`ISAACLAB_SCENE_SYNC=0`。
持久配置写 `configs/scene_sync.env`（进程环境变量永远优先）。

## 双机形态（对等/混合权威）

| 项 | 权威 | 说明 |
|---|---|---|
| 本机机器人 `robot` | 各自机器 | 各自的 SONIC deploy 经 `rt/lowcmd` 驱动 |
| 对端机器人 `peer_robot` | 对端 | scene_state 帧驱动的无碰撞无重力镜像体 |
| 场景物体（7 件） | 固定 ID=1 | ID=2 侧 spawn 成 kinematic 纯跟随 |
| 流水线驱动 | ID=1 | 镜像端强制关（本地驱动会和同步打架） |
| 整环境复位 | ID=1 | 广播 reset_id，ID=2 跟随；ID=2 本地复位不回传 |

端口：ID=1 绑 `15555`、ID=2 绑 `15556`（base+id-1），双方互连对方端口。
发布节流默认每 4 个物理步一帧（50 Hz）。

## 已知限制 / 待实测

- **只有 ID=1 侧机器人能与物体发生物理交互**（镜像体无碰撞；物体在 ID=2 侧是 kinematic）。
- ID=1 出生朝向是 yaw 180°（面对面布局）；SONIC 底座任务刻意保持 identity 朝向，
  **deploy 行走在 180° 出生下是否正常需实测**，异常先设 `ISAACLAB_ROBOT_YAW_IDENTITY=1` 兜底。
- 同步收发挂在 ActionTerm 上，跟随 SONIC 锁步：**deploy 停发 lowcmd 时本机 env.step
  停摆，同步（含镜像画面）随之冻结**。主循环挂载版（方案 b）在另一分支。
- 镜像端仅限 `--device cpu`：GPU pipeline 下 tensor API 写位姿驱不动 kinematic 体，
  镜像物体会静默冻结（启动时有告警）。
- 筐在带上被拖拽滑行（碰撞板静止 + μd=0.6），实测平均速度 ≈0.25 m/s 且会缓慢自转（源分支已知）。
- 原布局（`ISAACLAB_TOTES_ON_CONVEYOR=0`）的作业闭环在源分支就未实跑过。

## scene_assets/ 资产来源

| 文件 | 来源 | 说明 |
|---|---|---|
| warehouse-simple6_v48.usd | 分叉 git-LFS tip 版（**入本仓库 git**） | 工位平移/镜像已烘入；**内含 59 条绝对 S3 URL**，新机首载需联网或预热 `~/.cache/ov` |
| ConveyorBelt02.usd (46.7MB) | 分叉工作区手拷 | 被 warehouse USD 以 `./ConveyorBelt02.usd` 相对引用，**必须同目录**；体积过大已在本仓库 .gitignore 排除，换机需手拷 |
| peer_robot/g1_43dof_peer.usd | `tools/build_peer_robot_usd.py` 生成（derived，不入 git） | 无碰撞镜像机器人产物；缺失时任务启动 fail-fast |
| nolo_label.png | 分叉 git | warehouse USD 相对引用的地面贴花 |
| props/pushcart_physics.usda | 分叉工作区手拷（未入 git） | 引用 Nucleus 5.1 SM_PushcartA_02 |
| props/cart_box_d05_physics.usda | 分叉 git-LFS tip 版 | 已含关 CCD 修复（ae9118a2e） |
| props/tote_b04_physics.usda | 分叉 git-LFS | 内嵌 2.0/1.6 combine=min 高摩擦材质（抓取配方一部分） |
