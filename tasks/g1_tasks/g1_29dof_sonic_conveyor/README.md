# Isaac-G1-29DoF-Sonic-Conveyor

SONIC DDS 控制的 G1 + warehouse 流水线场景 + ZMQ 双机场景同步。

场景物理、USD 轻量化、资源选型、分阶段施工顺序和验收指标统一记录在
[Isaac G1 双机器人流水线场景优化路线与验收基线](../../../doc/conveyor_scene_optimization_roadmap_zh.md)。
该文档是后续优化的执行入口；其中待办项不表示已经实施。

场景与流水线驱动移植自 IsaacLab 分叉 `feat/conveyor-loop-totes-wip`
（tip 17e71c0a0）的 `pick_place` 任务；机器人与 DDS/XR/观测链路沿用本工程
`g1_29dof_dex3_sonic` 的 SONIC 底座。源仓库中“流水线两筐”和“推车两大筐”的分支、
坐标及后续双布局开关关系见
[IsaacLab 双机器人抓筐场景分支对照](../../../doc/isaaclab_scene_branch_map_zh.md)。

## 启动

```bash
cd ~/unitree_sim_isaaclab
conda activate env_isaaclab

# articulation 产物已随仓库提供；仅在缺失或上游 URDF 变化后重新生成
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

镜像机器人默认继续使用已验过的无碰撞 articulation。需要先试用不进入 PhysX 的
43-DoF 纯显示 LOD，可在接收镜像的进程增加：

```bash
ISAACLAB_PEER_ROBOT_MODE=visual_lod
```

该模式按 `scene_state` 的 base pose + 43 关节角执行 USD Xform/FK，ID=0 viewer 的
`PeerRobot`、`PeerRobot2` 均支持；不是静态模型或隐藏 articulation。实现、协议兼容、
资产统计和后置动态验收清单见
[流水线镜像机器人纯显示 LOD](../../../doc/conveyor_peer_visual_lod_zh.md)。删除变量或设为
`articulation` 即回退。

## 场景布局切换

同一任务通过 `ISAACLAB_TOTES_ON_CONVEYOR` 切换布局，修改后需重启仿真：

| 值 | 布局 |
|---|---|
| `1`（默认） | 5 个纸箱排在工位**上游**、挡停放行流向工位；机器人分站流水线两侧 |
| `0` | 两个原尺寸塑料筐叠放在入料口推车上；机器人面对面站在推车两侧 |

例如：`ISAACLAB_TOTES_ON_CONVEYOR=0 python sim_main.py ...`。

默认的 `ISAACLAB_CONVEYOR_PROPS=layout` 会真正不生成当前布局用不到的道具：

| 布局 | 实际生成的任务道具 |
|---|---|
| 流水线 | `belt_box_1` … `belt_box_5` |
| 推车 | `pushcart_2` + `cart2_tote1` + `cart2_tote2` |
| `legacy_props`（任一布局） | 旧的 10 个道具，**不含**纸箱 |

两种布局都不再继承原点的 packing table/cubes，也不生成第一组推车、纸箱和
`test_box`；scene-sync 默认清单与实际生成清单共用同一解析结果。排障或视觉 A/B 可设
`ISAACLAB_CONVEYOR_PROPS=legacy_props` 恢复旧的全量道具——它是**纯回退**，不叠加纸箱
队列。两者不能共存：塑料筐的流水线出生位 (-5.35, 17.4) / (-5.89, 18.0) 与纸箱队列的第
4、5 个同 y 且同在带上，筐沿 X 半宽 0.15 + 纸箱 0.125 = 0.275 > 车道距中线的 0.27，
开局就会三轴互穿 5 mm。

背景默认使用 `warehouse-simple6_v61_visual_only.usda` 强覆盖层：保留 v61 外观，但在 PhysX
解析前删除 10 个流水线纸箱和 5 个 KLT 料箱的刚体/碰撞语义，避免装饰物参与物理、复位和
双端同步；这 15 个装饰物**同时被 `active=false` 整体摘除**——它们的摆位是 v48→v61 换版
遗留（纸箱底 z=0.633 对带面顶 0.772，陷进带里 14 cm；料箱悬空 12~22 cm），而任务现在自己
在同一条中线上放真箱子，留着只会视觉打架。A/B 或排障时可用
`ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61` 临时恢复原始 v61；无效值会在启动时直接报错，
不会静默换场景。

真正精简的 `conveyor_workcell_lite.usd` 已生成，但功能集成期仍保持 opt-in：

```bash
ISAACLAB_CONVEYOR_BACKGROUND=workcell_lite python sim_main.py ...
```

它用 63 个顶层 Prim 白名单取代完整 v61 的 3,017 个根子节点，不组合相机、
NavMesh、Render 设置、额外 PhysicsScene、货架、纸箱堆、推车与原背景灯光。可复现生成、
组合统计和远程依赖说明见
[`scene_assets/conveyor_workcell_lite_zh.md`](scene_assets/conveyor_workcell_lite_zh.md)。

### 料筐碰撞与 ContactReport

料筐默认使用 `ISAACLAB_TOTE_COLLIDER=compound`：视觉仍引用同一份 SimReady
`Tote_B04`，物理碰撞改为底板加四壁的 5 个 box，保持开口容器语义并避免运行时
`convexDecomposition`。需要抓取 A/B 时可回退
`ISAACLAB_TOTE_COLLIDER=convex_decomposition`。

机器人 ContactReport 通过 `ISAACLAB_CONVEYOR_CONTACT_REPORT` 选择：

| 值 | 行为 |
|---|---|
| `ankles`（默认） | 只给左右 `ankle_roll_link` 添加 ContactReport，保留足底诊断 |
| `off` | 不创建 ContactReport 和足底 ContactSensor，供生产模式使用 |
| `all` | 恢复历史全身 reporter，供诊断 A/B 使用 |

环境变量拼写错误会直接报错，不会静默回退到高成本模式。

### 纸箱队列的挡停放行

流水线布局的作业对象是 5 个纸箱（`SM_CardBoxD_01`，0.38×0.25×0.1487 m，与 v61 背景
`ConveyorBelt_Box_XX` 同款视觉，物理封装见 `props/cart_box_d01_physics.usda`）。它们只
排在工位 `y_stop=14.148` **上游**那一段带面上，默认出生 y 为
`15.6 / 16.2 / 16.8 / 17.4 / 18.0`，车道 `x=-5.62`、`z=0.775`。`belt_box_1` 是队首。

每个箱子的停止线是 `max(y_stop, 前车 y + queue_pitch)`，"前车"取下游方向上**仍在带面**
的最近一个箱子（默认 `queue_pitch=0.45`，即箱长 0.38 + 7 cm 间隙）。于是：

* 队首没有前车 → 流到工位停住等抓取；
* 后车被前车顶住 → 在上游排队，一次只有一个箱子在工位；
* 机器人把工位那个拎走 → 它掉出带面窗口、不再是任何人的前车 → 下一个自动补位。

"抓走后放行下一个"因此没有任何状态机：语义完全由当前帧的位置推出来，天然可复位、
可断点续跑、双机一致。判据在 `conveyor_queue.queue_drive_mask`（不依赖 Isaac，有单测）。

驱动只在箱底位于带面高度 `z=0.772±0.15`，且原点进入 `x=[-6.17,-5.07]`、
`y=[10.19,18.22]` 时生效；箱子被举离带面、掉到地上或放到范围外都不会被强行拖动。
驱动只在物体权威端运行，viewer/镜像端通过场景同步看到相同运动。

`ISAACLAB_CONVEYOR_Y_STOP<=0` 切到**循环模式**：没有工位停止线，箱子只受前车约束一路流到
出料端，到 `y_recycle=10.6` 后被传回 `y_respawn=18.0` 继续循环（回收由额外挂上的
`recycle_surface_totes` 事件负责——纸箱的驱动函数只管排队和限速，不搬运）。

推车布局（`ISAACLAB_TOTES_ON_CONVEYOR=0`）仍是两塑料筐的老路径：机器人把筐放上带面
松手后被施加沿 `-Y` 的 `0.3 m/s` 目标速度，送到出料段 `y=11.5` 后交给摩擦停住；筐之间
没有队列语义。

可调项：

| 环境变量 | 默认 | 说明 |
|---|---|---|
| `ISAACLAB_BELT_BOX_COUNT` | `5` | 纸箱数量；上限 5（SceneCfg 字段是显式声明的，超限 fail-fast） |
| `ISAACLAB_BELT_BOX_SPAWN_Y_LEAD` | `15.6` | 队首出生 y，同时是复位落点 |
| `ISAACLAB_BELT_BOX_SPAWN_PITCH` | `0.6` | 出生间距（只决定初始队形） |
| `ISAACLAB_BELT_BOX_QUEUE_PITCH` | `0.45` | 停稳后的队列中心距，不得小于箱长 |
| `ISAACLAB_BELT_BOX_LANE_X` / `_SPAWN_Z` | `-5.62` / `0.775` | 车道与出生高度 |
| `ISAACLAB_BELT_BOX_MASS` | `1.0` | 单箱质量（kg） |
| `ISAACLAB_CONVEYOR_SPEED` / `_Y_STOP` / `_ENABLED` | `0.3` / `14.148` / `1` | 速度、工位、总开关 |

排到带面外、队距小于箱长、队首压在工位上等非法组合都会在启动时报错，不会等到运行时
才看见箱子悬空或互相穿模。

验收（`--pick-lead-at` 会在指定步把队首搬离带面，模拟机器人取件）：

```bash
python tools/smoke_conveyor_scene.py --steps 900 --sync 0 --device cpu
python tools/smoke_conveyor_scene.py --steps 900 --sync 0 --device cpu --pick-lead-at 500
```

## 双机形态（对等/混合权威）

| 项 | 权威 | 说明 |
|---|---|---|
| 本机机器人 `robot` | 各自机器 | 各自的 SONIC deploy 经 `rt/lowcmd` 驱动 |
| 对端机器人 `peer_robot` | 对端 | 默认无碰撞 articulation；可显式切纯 USD Xform/FK 的 visual_lod |
| 场景物体（随布局生成） | 固定 ID=1 | ID=2 侧 spawn 成 kinematic 纯跟随 |
| 流水线驱动 | ID=1 | 镜像端强制关（本地驱动会和同步打架） |
| 整环境复位 | ID=1 | 广播 reset_id，ID=2 跟随；ID=2 本地复位不回传 |

端口：ID=1 绑 `15555`、ID=2 绑 `15556`（base+id-1），双方互连对方端口。
发布节流默认每 4 个物理步一帧（50 Hz）。

## Dex3 夹爪控制数据流

当前所谓“夹爪”实际是两只 Dex3 灵巧手，每只手有 7 个电机关节。控制输入来自 Pico
左右控制器，不使用 Pico 手部骨骼追踪，也不经过 129/130 Viewer：Viewer 只显示 131
通过 `scene_state` 同步过去的 43 关节机器人状态。

### 双机器人链路

```text
Pico 左右控制器 trigger
  ├─ robot_1: UDP :63901 → manager ZMQ PUB :5556 → deploy#1 → DDS rt/dex3/*
  └─ robot_2: UDP :63902 → manager ZMQ PUB :5566 → deploy#2 → DDS rt/r2/dex3/*
                                                               ↓
                    Isaac Dex3DDS → SonicDDSActionProvider → Dex3 PhysX 执行器
                                                               ↓
                              HandState（实际 q/dq/tau）DDS 回传 deploy
```

两套 manager 均以 `--manager --no_auto_pose` 启动；两套 deploy 均使用
`--input-type zmq_manager`。robot_1 的 `G1_LOCAL_ROBOT_ID=1`、DDS 前缀为 `rt`；
robot_2 的 `G1_LOCAL_ROBOT_ID=2`、DDS 前缀为 `rt/r2`，命令、状态和共享内存完全隔离。

| 项目 | robot_1 | robot_2 |
|---|---|---|
| Pico UDP | `63901` | `63902` |
| manager → deploy ZMQ | `5556` | `5566` |
| 左手命令 | `rt/dex3/left/cmd` | `rt/r2/dex3/left/cmd` |
| 右手命令 | `rt/dex3/right/cmd` | `rt/r2/dex3/right/cmd` |
| 左手状态 | `rt/dex3/left/state` | `rt/r2/dex3/left/state` |
| 右手状态 | `rt/dex3/right/state` | `rt/r2/dex3/right/state` |

### 手柄输入与模式

manager 读取左右控制器的 `trigger` 和 `grip`，但当前 `generate_finger_data()` 实现只使用
`trigger`，并在 `0.5` 处二值化：

- `trigger <= 0.5`：对应手发送 7 维全零张开目标；
- `trigger > 0.5`：直接发送完整 middle-close 目标，左手为
  `[0, 0.7, 0.7, -1.0, -1.5, -1.0, -1.5]`，右手为镜像值；
- `grip` 当前不参与夹爪目标计算；左 `grip+A/B` 用于数据采集/放弃，不是夹爪控制；
- 因此当前是“左右 trigger 分别控制对应手的开/闭”，不是模拟量连续闭合，也不是手势追踪。

| manager 模式 | 夹爪行为 |
|---|---|
| `POSE` | 约 50 Hz 读取 trigger、生成并发送左右手 7 维目标 |
| `PLANNER_VR_3PT` | 约 20 Hz 读取 trigger、生成并发送左右手 7 维目标 |
| `PLANNER_FROZEN_UPPER_BODY` | 发送进入模式时从反馈保存的左右手目标，不实时跟随 trigger |
| 普通 `PLANNER` | 不发送 hand 字段；deploy 当前回退到 InputInterface 的预设闭合姿态 |
| `OFF` / `POSE_PAUSE` | 不产生新的实时夹爪目标 |

### deploy、DDS 与 PhysX

manager 把左右手目标编码为 ZMQ `pose`/`planner` 消息中的
`left_hand_joints: f32[7]` 和 `right_hand_joints: f32[7]`。deploy 解码后绕过身体策略，
直接写入 `Dex3Hands` 命令缓存；500 Hz command writer 随 `LowCmd` 同频重发两手
`HandCmd`。每个 HandCmd 的 7 个 motor slot 都包含 `mode/q/dq/tau/kp/kd`，当前目标
主要使用 `q`，默认 `dq=0`、`tau=0`、`kp=1.5`、`kd=0.1`。

`Dex3Hands` 根据 Isaac 回传的实际手指位置，把每次发布的目标差限制到 `±0.25 rad`，
并应用最大闭合比例。Isaac 的 [`Dex3DDS`](../../../dds/dex3_dds.py) 订阅左右手命令，
[`SonicDDSActionProvider`](../../../action_provider/action_provider_sonic_dds.py) 在每个
50 Hz 环境步读取最新快照，按关节名映射到 43 关节 articulation：

- `q/dq/tau` 进入 position、velocity、effort 三个 ActionTerm；
- `kp/kd` 由 provider 直接写入 PhysX stiffness/damping；
- 新目标在一个 20 ms 环境步内保持 4 个 5 ms PhysX 子步，驱动求解仍为 200 Hz；
- 每步结束后，[`dex3_state.py`](../../common_observations/dex3_state.py) 按相同顺序采集
  实际 `q/dq/applied_torque`，经 `HandState` DDS 回传 deploy，形成闭环。

Isaac 会拒绝长度不是 7、NaN/Inf、负 `kp/kd`、错误 motor ID 或越界的命令。
HandCmd 默认超时为 `0.20 s`；超时后保持最后安全的 `q/kp/kd`、清除 `dq/tau`，
但不会暂停身体的 LowState/LowCmd 锁步控制。131 host 模式最终把两台机器人的
`[q(43), dq(43), tau(43)]` 拼成 258 维动作，两个身体 LowCmd ack 都匹配后才推进环境。

### 当前已知断点

- `grip` 未接入夹爪目标，`trigger` 又被二值化，尚不支持按压深度连续控制闭合程度；
- 普通 `PLANNER` 没有实时手字段，会落到预设闭合姿态，不能在该模式下用 trigger 控手；
- manager 的 `FeedbackReader` 仍按 ZMQ `localhost:5557/g1_debug` 读取，而当前两套 deploy
  配置为 UDP `g1_1_debug:5557` / `g1_2_debug:5567`。因此冻结姿态和 VR3PT 重校准拿不到
  这路网络反馈；它不影响 `POSE`/`PLANNER_VR_3PT` 的 trigger → HandCmd 主路径，也不影响
  Dex3 的 DDS HandState 平滑反馈闭环。

## 已知限制 / 待实测

- **只有 ID=1 侧机器人能与物体发生物理交互**（镜像体无碰撞；物体在 ID=2 侧是 kinematic）。
- ID=1 出生朝向是 yaw 180°（面对面布局）；SONIC 底座任务刻意保持 identity 朝向，
  **deploy 行走在 180° 出生下是否正常需实测**，异常先设 `ISAACLAB_ROBOT_YAW_IDENTITY=1` 兜底。
- 同步挂载模式（`ISAACLAB_SCENE_SYNC_MAINLOOP`，本分支默认 1）：主循环挂载下
  deploy 断连/锁步暂停时镜像仍活着；置 0 退回 ActionTerm 挂载（方案 a——
  deploy 停发 lowcmd 时 env.step 停摆，同步随之冻结）。
- 镜像端仅限 `--device cpu`：GPU pipeline 下 tensor API 写位姿驱不动 kinematic 体，
  镜像物体会静默冻结（启动时有告警）。
- 物体在带上被拖拽滑行（碰撞板静止 + μd=0.6），实测平均速度 ≈0.25 m/s 而不是设定的
  0.3 m/s，且会缓慢自转（源分支已知）。
- 纸箱停位比理论槽位各偏小 8~33 mm（越线后驱动关闭、靠摩擦停住），队列越靠上游累计偏差
  越大；smoke 的容差取 50 mm。需要更准的节距就调 `queue_pitch`，不要指望停在整数槽位上。
- **纸箱队列尚未做机器人实抓验收**：`--pick-lead-at` 是把队首直接搬离带面来模拟取件，
  验的是"队列会不会正确放行下一个"，不等于 Dex3 真能抓起 1.0 kg 的纸箱。
- 流水线布局的 robot_2 仍是偏展示的站位，离带面较远，实际可达性尚未完全收口。
- 原布局（`ISAACLAB_TOTES_ON_CONVEYOR=0`）的作业闭环在源分支就未实跑过。
- `surface_velocity` 后端下纸箱队列靠"后车撞前车"物理涌现，没有走
  `queue_drive_mask`；该组合尚未实测，上游带面会持续挤压排队的箱子。

## scene_assets/ 资产来源

| 文件 | 来源 | 说明 |
|---|---|---|
| warehouse-simple6_v61.usd | 分叉 git-LFS tip 版（**入本仓库 git**） | 原始 v61，作为 `ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61` 的 A/B 回退；2026-08-07 对根层 Sdf reference list-op 的静态审计记录约 1,805 个唯一资产路径，不等于传递依赖或实际下载数 |
| warehouse-simple6_v61_visual_only.usda | 本仓库任务专用强覆盖层 | 默认背景入口；引用原始 v61，对 10 个纸箱和 5 个 KLT 料箱删除刚体/碰撞 API 并显式禁用物理，**再用 `active=false` 整体摘出组合**（摆位与带面对不上，且与任务的真箱子队列重叠）；其余视觉保持不变 |
| conveyor_workcell_lite.usd | `tools/build_conveyor_workcell_lite.py` 生成的 ASCII USD 薄层 | opt-in 背景；白名单引用 63 个 v61 根 Prim，当前静态审计为 27,802→1,052 active Prim、1,818→13 used layer |
| conveyor_workcell_lite.manifest.json | 本仓库可复现生成清单 | 锁定源哈希、保留规则、必须存在/缺席的 Prim 和组合降幅门槛 |
| ConveyorBelt02.usd (46.7MB) | 分叉工作区拷入（**已入本仓库 git**） | 被 warehouse USD 以 `./ConveyorBelt02.usd` 相对引用，必须与 warehouse 层保持可解析的相对路径；后续 visual-only 派生层不直接重写这个二进制源资产 |
| peer_robot/g1_43dof_peer.usd | `tools/build_peer_robot_usd.py` 生成（已入 git） | 默认 articulation 模式的无碰撞镜像机器人；缺失时任务启动 fail-fast |
| peer_robot/g1_43dof_visual_lod.usda | `tools/build_peer_visual_lod_usd.py` 生成（入 git） | 43-DoF 纯显示镜像；47 个解析 Gprim、3 份共享材质、无 PhysX schema |
| nolo_label.png | 分叉 git | warehouse USD 相对引用的地面贴花 |
| props/pushcart_physics.usda | 分叉工作区手拷（未入 git） | 引用 Nucleus 5.1 SM_PushcartA_02 |
| props/cart_box_d05_physics.usda | 分叉 git-LFS tip 版 | 已含关 CCD 修复（ae9118a2e） |
| props/cart_box_d01_physics.usda | 本仓库任务专用物理层 | 流水线纸箱队列用；与 d05 同构但引用 `SM_CardBoxD_01`（= v61 `ConveyorBelt_Box_XX` 的视觉源），删掉原资产的 triangle-mesh 碰撞、另挂 0.38×0.25×0.149 m 的 convexHull，原点在箱底面 |
| props/tote_b04_compound_physics.usda | 本仓库任务专用物理层 | 默认料筐碰撞；复用 SimReady 视觉，以底板+四壁 5-box compound 保持开口语义 |
| props/tote_b04_physics.usda | 分叉 git-LFS | 历史 convex decomposition 回退；内嵌 2.0/1.6 combine=min 高摩擦材质 |
