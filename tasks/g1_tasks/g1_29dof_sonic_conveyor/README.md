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

## 多机器人完整真实场景变体

基于同一套 Conveyor 多机器人配置，新增了四个可直接选择的任务 ID：

| 任务 ID 后缀 | 完整背景 | 说明 |
|---|---|---|
| `Conveyor-Warehouse` | Simple Warehouse 完整房间 | 只有仓库地面/墙体/货架/灯光，无 ConveyorBelt、箱队列和流水线事件 |
| `Conveyor-Apartment` | `Apartment/scene_04.usd` | 室内北侧开放房间，多机器人站位已避开横墙 |
| `Conveyor-Staircase` | `2-StoryStaircase.usd` | 带楼梯/Loft 的真实建筑场景，机器人组放在开阔地面 |
| `Conveyor-Office` | Isaac Sim Office `office.usd` | 官方 Office 完整场景；首次加载可能较慢 |

例如只看 Apartment 画面（不接 SONIC deploy）：

```bash
ISAACLAB_SCENE_SYNC=0 \
ISAACLAB_CONVEYOR_VISIBLE_ROBOTS=first \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor-Apartment \
  --action_source hold --device cpu
```

这四个变体都保留 Conveyor 的机器人角色字段：对等模式是本机 `robot` + 对端
`peer_robot`，`ISAACLAB_HOST_BOTH_ROBOTS=1` 时再实例化第二台全动力学 `robot_2`，viewer
模式则有 `peer_robot` + `peer_robot_2`；三台 `standby_robot_*` 始终是纯显示资产。
默认显示策略是只渲染全局 `robot_1`。隐藏只改变 USD/Imageable 可见性，不会因为隐藏
而删除已实例化机器人的动力学、DDS 或 scene-sync 配置。
需要检查完整编队时可重启并设置：

```bash
ISAACLAB_CONVEYOR_VISIBLE_ROBOTS=all
```

也可用逗号选择（如 `robot_1,robot_2,standby_robot_1`）。原有的
`Isaac-G1-29DoF-Sonic-Warehouse/Apartment/Staircase/Office` 任务保持单机器人语义；
要看这里的多机器人版本，请使用上表中带 `Conveyor-` 的任务 ID。这些完整房间变体
都会显式关闭输送带道具和 ConveyorEvents，避免把流水线坐标硬拼到真实场景 USD；
只有原始 `Isaac-G1-29DoF-Sonic-Conveyor` 任务保留流水线作业布局。
这些新变体会无条件保留三台 standby 外观；后文关于
`ISAACLAB_TOTES_ON_CONVEYOR=0` 不生成 standby 的说明只适用于原始
`Isaac-G1-29DoF-Sonic-Conveyor` 基线。

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

| 值 | 布局 | 关键世界坐标 |
|---|---|---|
| `1`（默认） | 17 个箱/包沿**西拐入口弯道路径**以 pitch 0.75 排在工位上游、挡停放行流向工位；第二机器人及其西侧工作台、分拣箱沿 `+Y` 错开 `0.75 m`，队首两箱成组到位后供两机器人同时抓取；另有 3 台纯显示 G1 分散站位 | 第一/第二机器人工位分别为 `y=14.398 / 15.148`（`x=-4.54 / -6.7`）；新增站位 `(-12.70,18.9534)` 位于最后一个物体南侧偏东 1 m 并朝西北正对它、`(-6.70,17.95)` 朝 `+X`、`(-9.50,21.15)` 朝 `-Y`；出生槽位 s=`11.06 − 0.75k (k=0..16)`；`y_stop=14.398` |
| `0` | 两个原尺寸塑料筐叠放在入料口推车上；机器人面对面站在推车两侧 | 作业组 `(-5.62, 19.0)`（机器人 `x=-4.82 / -6.42`）；`y_stop=11.75` |

例如：`ISAACLAB_TOTES_ON_CONVEYOR=0 python sim_main.py ...`。

流水线布局新增的三台站位机器人使用 `g1_43dof_standby_visual_only.usda`：它引用原两台
同源的完整 G1 网格，烘焙相同的 SONIC 默认关节姿态，并复用同一套白/黑分区和 Logo
涂装；资产的 `Physics/Robot/Sensor` 三组 variant 全部选为 `None`。因此组合后外形与
原机一致，但没有 articulation、关节、刚体、碰撞、执行器或接触传感器，也不参与
`scene_state` 同步。切到 `ISAACLAB_TOTES_ON_CONVEYOR=0` 时不会生成这三台。

默认的 `ISAACLAB_CONVEYOR_PROPS=layout` 会真正不生成当前布局用不到的道具：

| 布局 | 实际生成的任务道具 |
|---|---|
| 流水线 | `belt_box_1` … `belt_box_17` |
| 推车 | `pushcart_2` + `cart2_tote1` + `cart2_tote2` |
| `legacy_props`（任一布局） | 旧的 10 个道具，**不含**纸箱 |

两种布局都不再继承原点的 packing table/cubes，也不生成第一组推车、纸箱和
`test_box`；scene-sync 默认清单与实际生成清单共用同一解析结果。排障或视觉 A/B 可设
`ISAACLAB_CONVEYOR_PROPS=legacy_props` 恢复旧的全量道具——它是**纯回退**，不叠加纸箱
队列。两者不能共存：塑料筐的流水线出生位 (-5.35, 17.65) / (-5.89, 18.25) 与纸箱队列
在带上重叠，筐沿 X 半宽 0.15 + 纸箱 0.125 = 0.275 > 车道距中线的 0.27，
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

### 流水线的头尾（整体北移 Δ=0.25）

带体是三段 ConveyorBelt_A08 视觉资产，实测世界 `y[10.438, 18.472]`（碰撞板
`y[10.44,18.47]`），车道中线 `x=-5.62`、带面 `z≈0.772`。**整条流水线（主线+弯道+
X 支线+机器人工位，=0 布局作业组同理）整体北移 Δ=0.25 m**：这是 X 支线越过货架
排 B 北侧所需的最小量（支线机身南缘 `19.4779` 对排 B 端护板北缘 `19.3946` 净距
`83.3 mm`，最小可行 Δ=0.2167 取 50 mm 整倍数；约束绑定项是端护板，叉车货叉尖
`19.321` 净距 `156.9 mm` 不绑定），换来支线向西延到**五段**、把出生/回生点真正
藏到货架（和叉车）后面。**货架与叉车都没有动**——动的只有流水线。
`conveyor_drive.CONVEYOR_NORTH_SHIFT_Y=0.25` 是唯一真源（`scene_layout` 抄一份、
测试交叉断言；`endless_intake` 用绝对坐标 +Δ 重钉、关系锁在测试层），几何落地在
clean wrapper 的 `over "ConveyorBelt"` 组变换 + 15 件带面装饰 + 3 条地贴（世界
y +Δ ≡ 背景局部 x +Δ）。⚠️ `ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61` 的裸 v61
不含位移 override，Δ≠0 期间与代码常量错位 0.25 m，仅可视觉参考、不可作功能
A/B 基线。安检机时代的 Δ=3.55 方案早已回退（箱子在带上露顶），与本次北移无关。

### 看不到头的入料端（西拐弯道 + X 支线，复用已有货架）

`ISAACLAB_CONVEYOR_ENDLESS`（`auto`=默认开 | `on` | `off`，非法值报错）在带头接一段
**A02 低位滚筒弯道向西（-X）拐 90°**，再接**五段 A05 短直段**成 X 支线，箱子"从西侧
货架后面拐出来"。**零新增货架/遮挡件**（用户硬要求）——遮挡只依赖背景 v61 既有的
西侧满载货架排（Δ=0.25 北移让支线从排 B 北侧擦过；遮挡结论见「已知限制」）。
几何（世界系，全部离线 pxr 实测）：

| 件 | 摆位 | 占位 |
|---|---|---|
| A02 弯道（`ConveyorBelt_A02_PR_NVD_01`，scale=0.01） | pos `(-7.1129, 20.5548, 0.003)`，yaw `-90°` | `x[-7.113,-5.042] y[18.457,20.558]`，连续结构顶 `z=0.801`、门柱 `1.169` |
| A05 支线 ×5（`ConveyorBelt_A05_PR_NVD_01`） | t.x `-7.0979 / -9.0830 / -11.0681 / -13.0532 / -15.0383`（步进 1.9851），同 yaw/z | 合计 `x[-17.038,-7.098] y[19.478,20.629]`，滚筒可用带西端 `-17.034` |

插接：弯道**南向母口套主线带头公头 15 mm**（带头实测面 `y=18.4723`、公头中心
`x=-5.617`）；弯道西公头插段 1 母口、段间公插母各 15 mm。周边间隙（mm，全部离线
mesh 级实测）：支线机身南缘 `19477.9` − 排 B 端护板北缘 `19394.6` = **83.3**
（Δ 的约束绑定项）、− 架板北缘 132.2、− 钢架柱北缘 170.6；− 叉车货叉尖 `19321.0`
= **156.9**（段 5 与货叉 x 重叠段）；Cone_4 南缘 − 支线北缘 = 596.9；+Y 墙面 −
支线北缘 = 2976.7；柱 1473 落地柱身南缘 − 支线北缘 = 1367.4（该柱 y<20.9 部分全在
z≥7.96 的屋面桁架，走廊无碰撞）；均无碰撞。

物理托面：默认 `legacy` 后端使用 `props/conveyor_path_support_physics.usda`
提供一张连续静态三角托面，一体覆盖主线 `y[10.44,18.47]`、拐角
`x[-7.02,-5.17] y[18.47,20.5034]` 和支线
`x[-17.034,-7.02] y=20.0534±0.45`。顶面是单个连通 Mesh，没有旧主线/拐角/支线 Cuboid 对接处的
内部竖直端面，避免平底包裹或大箱在接缝处被挂住。实验性 `surface_velocity`
后端仍保留可独立施加表面速度的主线 Cuboid 和两块扩展补板，且不驱动支线/弧段。
A02/A05 本体纯视觉（coll=0/rigid=0），全部 `AssetBaseCfg` → `scene.extras`，
不进 scene_props/同步清单。

驱动换**沿路径距离 s** 的两段式（判据在 `conveyor_queue`，零 Isaac 依赖有单测）：
支线 +X → R=1.4 圆角弧（圆心 `(-7.02,18.6534)`，由两条车道中线导出）→ 主线 -Y；
`path_progress` 无分支无同步，整带节拍直接复用 `queue_drive_mask`（喂 `-s` 翻转
下游方向，零第二实现）；on-belt 判据改 L 形并集（主线矩形 ∪ 拐角/支线矩形，各放宽
0.10）；箱子**不旋转**（只写线速度沿航向，双机同步/镜像端零改动）。关键 s 值
（s 参数化对 Δ 平移**不变**，工位/回收线的 s 与 Δ=0 时相同）：工位
`y=14.398 ⇒ s=12.1945`、回收线 `10.85 ⇒ s=15.7425`、`s_min=-4.27`（支线滚筒
西端）。生成门 = `TOTES_ON_CONVEYOR=1` 且 `ISAACLAB_CONVEYOR_PROPS=layout`：
=0 拖车组（约 83%）与 legacy_props 空车（约 99%）都落在弯道占位内，被门拦下时
启动日志自曝原因并回退直线带头。

### 纸箱队列的整带节拍启停

流水线布局的作业对象是 17 个箱/包（直线回退默认 5），可从
C01/C02、D01~D05 和三种软包裹中混排：

| key | 来源 | 尺寸（沿带 × 横向 × 高） | 质量 | 资产 |
|---|---|---|---|---|
| `d01` | v61 背景 `ConveyorBelt_Box_XX` 同款纸箱 | 0.250 × 0.380 × 0.1487 m | 0.1 kg | `props/cart_box_d01_physics.usda` |
| `d02` | Simple Warehouse 顶部压皱纸箱，任务层蓝色覆盖 + 白色顶标 | 0.250 × 0.380 × 0.1663 m | 0.1 kg | `props/cart_box_d02_physics.usda` |
| `c01` | Simple Warehouse 方形 C01 大箱 | 0.500 × 0.500 × 0.250 m | 1.5 kg | `props/cart_box_c01_physics.usda` |
| `c02` | Simple Warehouse 加高 C02 大箱 | 0.500 × 0.500 × 0.311 m | 1.5 kg | `props/cart_box_c02_physics.usda` |
| `d03` | Simple Warehouse 高型不规则 D03 纸箱 | 0.2606 × 0.4126 × 0.2656 m | 1.5 kg | `props/cart_box_d03_physics.usda` |
| `d04` | Simple Warehouse 印刷 D04 纸箱 | 0.250 × 0.380 × 0.149 m | 1.5 kg | `props/cart_box_d04_physics.usda` |
| `d05` | Simple Warehouse D05 纸箱 | 0.250 × 0.380 × 0.149 m | 1.5 kg | `props/cart_box_d05_physics.usda` |
| `parcel_a01` | 程序化快递软包裹（灰） | 0.301 × 0.402 × 0.080 m | 0.35 kg | `props/parcel_soft_a01.usda` |
| `parcel_a02` | 程序化快递软包裹（白，面单朝上） | 0.352 × 0.453 × 0.097 m | 0.5 kg | `props/parcel_soft_a02.usda` |
| `parcel_a03` | 程序化快递软包裹（粉） | 0.241 × 0.322 × 0.060 m | 0.22 kg | `props/parcel_soft_a03.usda` |

Simple Warehouse 中真正独立的 C/D 视觉资产只有 C01、C02 和 D01~D05。场景树里
看到的 C03~C09 都复用 C01 网格，D06/D07/D08/D10/D11 也只是 D01/D03/D05 的
别名（D09 不存在），因此不把这些别名重复注册成“新箱型”。

箱/包保持资产原始朝向，让短边沿流水线 Y 方向；机器人从流水线侧面抱取时，双臂跨距由
沿带尺寸决定。默认队首 `d01` / `d02` 仍都是 0.25 m，类型和近侧偏移坐标不变。

⚠️ 软包裹是**刚体不是软体**：枕形鼓包、热封边、褶皱、顶面白色面单都只是视觉造型，物理
上与纸箱同一套约定（根挂 RigidBody+Mass、凸包碰撞、原点在袋底）——CPU pipeline 不支持
deformable，本场景固定 `--device cpu`。资产取自 IsaacLab 分叉 `feat/pickplace-parcel-assets`
的程序化生成器，入本仓库时做了两处流水线适配：①抓取摩擦从 1.6/1.2 `combine=max` 归一为
纸箱同款 1.4/1.1 `multiply`（max 会压过带面摩擦，legacy 驱动的写入间隙靠摩擦减速，μd 近
两倍 ⇒ 包裹比纸箱慢 ~35%、整带停时短停 0.24~0.28 m）；②碰撞从枕形 Bag 凸包换成平底
"雪橇"盒（枕形凸包滑动时轻微摇晃耗能，仍慢 ~3%/程，混排时纸箱逐程追上包裹）。两处
适配后实测五件位移 0.809~0.817 m/程，差 <1%。

未设 `ISAACLAB_BELT_BOX_PATTERN` 时，`belt_box_1/2` 固定为 D01/D02，不参与
随机；后 15 件从 `c01,c02,d01,d02,d03,d04,d05,parcel_a01,parcel_a02,parcel_a03`
十种资产池中生成**带 seed 的平衡随机序列**。每轮先将十种各放一次再开始
下一轮，轮边界若与前一件同型会稳定换位，因此不会因普通抽样漏掉某种外观。
排序使用 SHA-256，不依赖 Python `hash` 或系统熵；相同 seed 在权威端和 viewer
上会得到相同 USD 槽位。默认 seed `20260811` 的完整序列是：

```text
d01,d02,parcel_a01,d04,parcel_a03,d05,d02,c02,parcel_a02,d03,d01,c01,d04,parcel_a02,c02,parcel_a01,d01
```

改 `ISAACLAB_BELT_BOX_RANDOM_SEED` 只重排第 3~17 件，不改队首两件。显式设置
`ISAACLAB_BELT_BOX_PATTERN` 时则保留历史的**完整覆盖**语义：忽略 random seed，
从第 1 件开始按给定 pattern 循环，所以它也可以显式改掉队首两件；写单个 key
即全用一种。**默认（弯道形态）**箱子按**显式槽位序列**
排在工位 `s_stop=12.1945`（`y_stop=14.398`）上游，以队首 `s=11.06` 为锚、pitch 0.75
向上游排列：`s = 11.06 − 0.75k (k=0..16)`——主线 5 箱 + 弧上 3 箱 + X 支线
9 箱，队尾槽位 `s=-0.94 (-13.70,20.05)`；`z=0.775`。默认 seed 的队尾是
D01，按路径半长 0.19 核算，队尾缘距支线滚筒可用端 `s_min=-4.27`
仍有 **3.14 m**；即使其他 seed 让最大 C 型箱落在队尾，也有不少于 **3.08 m**。循环模式的
独立回生点保持 `s=-3.90`，比默认出生队尾再向上游 2.96 m，不随槽位序列改变。
`belt_box_1` 是队首。显式给出 `S_LEAD`/`Y_LEAD`/`PITCH` 任一旋钮时回退等距排布；
`COUNT<17` 先按 seed 生成完整 17 项再取前缀，不会因改 COUNT 而重排已有槽位。
`ISAACLAB_CONVEYOR_ENDLESS=off` / `legacy_props` 回退直线形态：沿主车道 `x=-5.62`
直排，**默认 5 箱**（上游带面只有 ~4.07 m 装不下 17），出生 y
`15.20 / 15.95 / 16.70 / 17.45 / 18.20`。

默认 `legacy` 的连续无缝托面已取代上述三块 Cuboid，因此随机序列中的软包裹和
C/D 箱可以位于支线或弧段，不再靠“只把 parcel 放在 3~5 号槽”规避内部竖直缝。
`surface_velocity` 后端仍没有这项完整 L 路径保证，随机混排的弯道动线应使用默认
`legacy` 后端。

弯道形态下默认 17 箱的等距 pitch 上限为 0.94625（队尾缘顶
到支线可用端 `s_min=-4.27`）；显式减 COUNT 才能加大 pitch（如 `COUNT=5` 时最大约
3.792）。队首默认行程 `s 11.06→12.1945 ≈ 1.135 m`（实测带速约 0.244 m/s 时约
4.6 s 流到工位）。直线回退形态维持
旧账：上游只有 `18.47 - 14.398 ≈ 4.07 m`，`(count-1)×pitch + 端部半长 + 队首行程
≤ 4.07`，超了启动即报"队尾悬出带面"。

运动语义是**整条带一起启停**，不是逐个积放：

```text
in_corridor[i]         = 箱根 XY 投影在主线矩形 ∪ 拐角矩形 ∪ 支线矩形内（忽略 Z）
on_belt[i]             = in_corridor[i] & 箱底位于带面高度窗口
departure_completed[i] |= ~in_corridor[i]
transfer_complete      = all(on_belt[i] | departure_completed[i])
belt_running           = (新队首尚未到 y_stop) & transfer_complete
drive[i]               = on_belt[i] & belt_running & 没顶到前车
```

* 队首没到工位 → 整带都在走；
* 队首一压到工位 → **整条带立刻停**，后面的箱子原地保持当前间距，不会继续往前挤；
* 机器人只把工位箱竖直抬高 → 该箱不再受带速驱动，但 XY 仍在线上方 →
  **整带继续停止**；
* 箱根 XY 横向偏出流水线通道 → 偏离完成位立即锁存 → 整带重新启动，一起前进到
  新队首也压到工位为止；不要求进入蓝箱，也不检查速度或驻留时间。

于是每完成一次"取件→横移出线"，整列前进一格（≈ 出生间距）。只改变高度（包括仍在
通道投影内掉到地上）都不会提前放行；深藏队尾也同格推进，要连续完成多次搬离才逐渐
拐出货架后面
——这正是"看不到头"的补货观感。

第三个因子是**防撞保底**，正常跑不到：整带同起同停时相对间距恒定，而出生间距
远大于任何一对箱子的最小净距。它只兜住摩擦/质量差异带来的缓慢漂移，约束是
`ys[i] > 前车尾部 + 自己半长 + queue_gap`（默认 `queue_gap=0.07`）。⚠️ 这里恒定的是
**净空隙**而不是中心距。当前十种箱/包会混排，防撞判定按每件自己的
`max(沿带尺寸, 横向尺寸) / 2` 保守占位计算，不会把 C 型大箱当成 D01/D02。

偏离完成按“每个箱子 × 每个 env”锁存，F12/DDS/env reset 时清零。锁存避免已经偏出的
箱子因抓取轨迹回摆或边界抖动让流水线中途反悔；同时下一箱被抬起仍会形成新的未完成
缺口，所以首次搬离不会永久解锁后续周期。平面几何和整带判据都在
`conveyor_queue`（不依赖 Isaac，有单测）；状态只由物体权威端维护，viewer/镜像端继续
通过场景同步看到相同结果，无需新增 wire 字段。

驱动只在箱底位于带面高度 `z=0.772±0.15`，且原点进入主线矩形 `x=[-6.17,-5.07]`
`y=[10.44,18.47]` **或**弯道形态的两块附加矩形（拐角/支线，各放宽 0.10）时生效；
箱子被举离带面或掉到地上都不会被强行拖动。放行另用同一组矩形的纯 XY 判据：只抬高
或掉低仍保持停线，根位置真正偏出矩形并集才立即放行。该门控只用于默认停止式
`legacy` 后端；循环模式及 `surface_velocity` 组合维持原有行为。

`ISAACLAB_CONVEYOR_Y_STOP<=0` 切到**循环模式**：没有工位停止线，整带长跑不停（只剩防撞
保底），箱子到 `y_recycle=10.85` 后被传回回生点继续循环（回收由额外挂上的
`recycle_surface_totes` 事件负责——纸箱的驱动函数只管启停和限速，不搬运）。弯道形态
的回生点在 X 支线最深处 `s=-3.90 (-16.66, 20.0534)`（藏在货架排 B 后面：E2 眼位按
"东上角"判据全遮、E1 有缝隙残余，见「已知限制」；比默认出生队尾槽位 `-0.94`
再向上游 2.96 m）；直线回退形态维持主线 `y_respawn=18.25`。
**环路容量已复核并 fail-fast**：环路等价周长 = `s(y_recycle) − RESPAWN_S ≈ 19.64 m`，
17 箱队列跨度 12.0 m，回绕缺口 ≈7.64 m ≫ 首尾防撞下限（按最大 C 型箱计也只有
半长和 + queue_gap = 0.25+0.25+0.07 = 0.57 m）
——整带同速循环不会追尾；以后加箱/调 pitch/收 y_recycle 越限会在启动时报错
（`conveyor_env_cfg` 运行时检查 + `test_conveyor_scene_layout` 锁默认值）。

推车布局（`ISAACLAB_TOTES_ON_CONVEYOR=0`）仍是两塑料筐的老路径：机器人把筐放上带面
松手后被施加沿 `-Y` 的 `0.3 m/s` 目标速度，送到出料段 `y=11.75` 后交给摩擦停住；筐之间
没有队列语义。

可调项：

| 环境变量 | 默认 | 说明 |
|---|---|---|
| `ISAACLAB_CONVEYOR_ENDLESS` | `auto` | 入口弯道三态开关（`auto`/`on`/`off`）；非法值报错。=0 布局与 legacy_props 下即使 `on` 也被布局门拦下（日志自曝原因） |
| `ISAACLAB_BELT_BOX_RANDOM_SEED` | `20260811` | 未显式设 PATTERN 时，稳定重排第 3~17 件的平衡随机序列；第 1/2 件始终为 D01/D02，相同 seed 跨进程/平台得到同一序列 |
| `ISAACLAB_BELT_BOX_PATTERN` | 未设（使用上述固定队首 + seed 平衡随机） | 显式设置时完整覆盖默认序列并忽略 RANDOM_SEED，从第 1 件起按 pattern 循环；10 种 key 可选，未知 key 直接报错 |
| `ISAACLAB_BELT_BOX_COUNT` | `17`（弯道）/ `5`（直线回退） | 纸箱数量；上限 17（SceneCfg 字段是显式声明的，超限 fail-fast）；<17 取槽位序列前缀（从队首往上游数） |
| `ISAACLAB_BELT_BOX_SPAWN_S_LEAD` | 无（默认走槽位序列） | 弯道形态队首出生的沿路径距离（优先于 Y_LEAD）；**显式给出 S_LEAD/Y_LEAD/PITCH 任一都会回退等距排布** |
| `ISAACLAB_BELT_BOX_SPAWN_Y_LEAD` | `15.20`（直线） | 队首出生 y（世界系），同时是复位落点；弯道形态只接受主线段 y（<18.6534）并自动换算成 s |
| `ISAACLAB_BELT_BOX_SPAWN_PITCH` | `0.75` | 等距排布的箱间距（弯道形态=沿路径距离）；整带节拍下**也就是**停稳后的队列间距 |
| `ISAACLAB_BELT_BOX_QUEUE_GAP` | `0.07` | 防撞保底的**净空隙**（不是中心距）；整带节拍下正常不触发 |
| `ISAACLAB_BELT_BOX_LANE_X` / `_SPAWN_Z` | `-5.62` / `0.775` | 车道与出生高度；LANE_X 仅直线回退形态生效（弯道形态 x 由路径决定） |
| `ISAACLAB_BELT_BOX_MASS_<KEY>` | 各箱型默认 | 按箱型覆写质量（kg），如 `_D01` / `_PARCEL_A02` |
| `ISAACLAB_CONVEYOR_SPEED` / `_Y_STOP` / `_ENABLED` | `0.3` / `14.398` / `1` | 速度、工位、总开关；弯道形态要求 y_stop 在主线段（<18.6534） |

下列非法组合都会在启动时报错，不会等到运行时才看见箱子悬空或互相穿模：出生位排到
带面/支线外、**相邻两箱按各自半长算放不下**、队首压在工位上、工位不在主线段、
Y_LEAD 排到弧段/支线（须换 S_LEAD）、箱型比带面还宽、未知箱型 key。

验收（`--pick-lead-at` 会在指定步只抬高队首、保持原 XY，等待
`--depart-lead-after` 步确认停线，再横向移出通道并检查下一格补位；旧参数名
`--place-lead-after` 仍兼容；弯道形态队首行程约 1.135 m，默认 1600 步还为
整列稳定和取件补位留出时间）：

```bash
python tools/smoke_conveyor_scene.py --steps 1600 --sync 0 --device cpu
python tools/smoke_conveyor_scene.py --steps 1600 --sync 0 --device cpu --pick-lead-at 1000 --depart-lead-after 50
```

2026-08-11 本组合实测：193 项相关单测通过；默认 seed 的 1600-step 停位与
pick@1000/depart50 补位均 PASS，队首/补位误差约 6.4 mm；seed=8 的随机顺序同样
PASS。另将 `c01,c02,d03,d04,d05,parcel_a01,parcel_a02,parcel_a03` 八种候选从支线
`s_lead=5.6` 排起做 1200-step 接缝回归，逐件沿路径移动 5.25~5.57 m、全部仍在带面，
确认连续托面能让这些箱/包通过原支线→拐角接缝。该结果只验输送与节拍，不等价于
Dex3 对 C 型 0.5 m 大箱的实抓验收。

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
- **17 箱队列的物理成本**（2026-08-10 改版）：动态刚体从 5 → 17。按既有实测口径
  （每箱边际 ~0.22ms/步，碰撞形状主导而非箱数线性外推的观测），预估 env.step 的
  E 增量 ~2.6ms；headless smoke 实测 env_hz 见提交信息，**GUI 50Hz 帧率账本待用户
  实测复核**（conveyor 本就贴着 20ms 预算跑，S 余量可能被吃掉）。
- 停位有 0.06~10 mm 的散布（越线后驱动关闭、靠摩擦停住，各箱摩擦略有差异），smoke 容差
  取 50 mm；整带节拍下这个误差不累计，队列间距长期维持在出生值附近（实测 0.748~0.760，
  出生 0.75），但**不要指望它精确恒定**。
- **直线回退形态**的队首行程只有 0.80 m（约 3.3 s）。这是为拉大箱间距付出的
  代价——带面上游总长固定 4.07 m，间距、行程、数量三者只能三选二。
- **纸箱队列尚未做机器人实抓验收**：`--pick-lead-at` 是把队首原地抬高、等待后再
  横向移出通道来模拟取放，验的是"只抬高保持停线、XY 偏出后正确放行"，不等于 Dex3
  真能抓起并投放箱子。队首固定的 `d01`/`d02` 为 0.25×0.38 m / 0.1 kg，
  但默认随机池中的 `c01`/`c02` 均为 0.50×0.50 m / 1.5 kg；C01 过去就因
  0.5 m 抱取跨度过大而从默认箱型中移除，C02 还更高。它们现在会随队列最终走到
  工位，Dex3 的可达跨度、载荷和投放稳定性均未验；实抓前不应把 headless 输送
  smoke 视为 C 型抓取通过。
- 流水线布局的 robot_2 仍是偏展示的站位，离带面较远，实际可达性尚未完全收口。
- 原布局（`ISAACLAB_TOTES_ON_CONVEYOR=0`）的作业闭环在源分支就未实跑过。
- `surface_velocity` 后端下纸箱队列靠"后车撞前车"物理涌现，没有走
  `queue_drive_mask`，也不支持本节的 XY 偏离放行门；该组合尚未实测，上游带面会持续挤压
  排队的箱子。
- **回生点遮挡：既有高度下 E2 达成全遮，C02 待复核；E1 几何上无全遮解（缝隙残余）**。Δ=0.25 北移 +
  五段支线把出生/回生点深藏到 `(-16.66,20.05)`（排 B 后面、近端还有叉车），
  离线 mesh 级射线核算（眼位 E1=(-4.54,14.698,1.6)/E2=(-6.7,14.698,1.6)，箱顶
  历史核算最高只覆盖到 C01；当前随机池的 C02 箱顶约 `z=1.086`，比 C01
  再高 61 mm，因此下述 E2 全遮结论对 C02 需重做射线复核）：
  - **E2（robot_2 侧）**：既有 C01 及以下高度按"东上角"判据**全遮**——顶面 25 点 + 四顶角 + 顶心全部
    被排 B 首层架板板面实体挡住（遮挡源稳健）；全遮边界 `s≤-3.81`，`RESPAWN_S=-3.90`
    留 90 mm 裕量。残余=东立面中段 `z≈0.86-0.94` 有 4/20 点经架板下缝可见。
  - **E1（robot_1 侧）无全遮解**：mesh 级复测推翻了"排 B 首层=实心遮挡体"的 AABB
    级假设——地面货堆顶 `z=0.960` 与架板板面底 `z=1.176` 之间有 **216 mm 通视水平缝**
    贯穿排 B 全长，E1 视线在排 B 北面出口高度 `z≈1.099` 恰落在缝内；加段数、更深、
    换矮箱 d01 都无效。残余=箱顶条带经缝可见（顶面 68% 可见、距 13.25 m、缝的角高
    约 1.2°），底角 4/4 全遮。
  - 相比旧三段方案（回生点两眼**全裸露**、距 8-9.6 m）是大幅弱化；"双眼全遮"严格
    目标只达成 E2。若要消掉 E1 缝隙残余，只能另议时序/材质类弱化手段（缝是背景
    货架自身的结构，不动货架填不上）。
  - 弧段/主线 slot 维持刻意揭示（箱底沿被 0.801 挡边遮，"绕弯而来"）。
- **弧段拖滑比直线慢**（同款物理，弯道形态新增账目）：弧段实测拖滑 ~0.218 m/s
  低于直线 ~0.244（东拐版实测，同族资产/同套托面适用），前车先出弧提速时队距
  最多被拉伸 ≈ 弧长 2.2 × (0.244/0.218−1) ≈ 0.26（出生 0.75 的对，实测停稳
  可到 ~1.01）；反向：前车在弧上、后车还在支线直线段时间距收缩，防撞保底
  （中心距下限 = 该对半长和 + queue_gap；最大 C/C 组合为 0.57 m）会让后车临时
  停走——17 箱长队列在
  弧口会出现走走停停的"手风琴"瞬态，属预期。"停稳间距=出生间距"对拖滑物理
  不成立，smoke 判据用"队首停位 + 每对间距落在
  [max(该对真实半长和+queue_gap, 出生间距−0.45), 出生间距+0.35]"。
- `surface_velocity` 后端只驱动主线碰撞面（-Y）：支线/弧段上的箱子不会被接触
  驱动，该组合未验证（启动日志有警告），弯道形态建议 legacy 后端。
- 弯道压住背景红色地贴 `FloorZone_Robot`（随 Δ 同移后北缘 19.14）约 0.68 m：
  纯视觉贴纸 z=0.01 无碰撞，观感是"设备摆进了机器人红区"，暂接受未收缩（相对
  几何与 Δ=0 相同）。
- 弯道/支线的 GUI 目视验收（穿模/连续托面/材质）尚未做——本次几何全部为
  离线 pxr 实测 + smoke 动线验证；北移后的三条地贴（S3 远程资产）离线取不到包围盒，
  同样待 Kit 目视复核。
- `ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61` 的裸 v61 回退档不含 Δ=0.25 位移
  override，与代码常量错位 0.25 m：仅可视觉参考，**不可作功能 A/B 基线**
  （回退档要重新可用需把 Δ 归零并同步拆 wrapper override）。

## scene_assets/ 资产来源

| 文件 | 来源 | 说明 |
|---|---|---|
| warehouse-simple6_v61.usd | 分叉 git-LFS tip 版（**入本仓库 git**） | 原始 v61，作为 `ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61` 的 A/B 回退；2026-08-07 对根层 Sdf reference list-op 的静态审计记录约 1,805 个唯一资产路径，不等于传递依赖或实际下载数 |
| warehouse-simple6_v61_visual_only.usda | 本仓库任务专用强覆盖层 | 默认背景入口；引用原始 v61，对 10 个纸箱和 5 个 KLT 料箱删除刚体/碰撞 API 并显式禁用物理，**再用 `active=false` 整体摘出组合**（摆位与带面对不上，且与任务的真箱子队列重叠）；同时是**整体北移 Δ=0.25 的几何落地处**（`over "ConveyorBelt"` 组变换 + 15 件装饰 + 3 条地贴，真源 `conveyor_drive.CONVEYOR_NORTH_SHIFT_Y`）；其余视觉保持不变 |
| conveyor_workcell_lite.usd | `tools/build_conveyor_workcell_lite.py` 生成的 ASCII USD 薄层 | opt-in 背景；白名单引用 63 个 v61 根 Prim，当前静态审计为 27,802→1,052 active Prim、1,818→13 used layer |
| conveyor_workcell_lite.manifest.json | 本仓库可复现生成清单 | 锁定源哈希、保留规则、必须存在/缺席的 Prim 和组合降幅门槛 |
| ConveyorBelt02.usd (46.7MB) | 分叉工作区拷入（**已入本仓库 git**） | 被 warehouse USD 以 `./ConveyorBelt02.usd` 相对引用，必须与 warehouse 层保持可解析的相对路径；后续 visual-only 派生层不直接重写这个二进制源资产 |
| peer_robot/g1_43dof_peer.usd | `tools/build_peer_robot_usd.py` 生成（已入 git） | 默认 articulation 模式的无碰撞镜像机器人；流水线布局的三台站位机器人也复用其完整网格；缺失时任务启动 fail-fast |
| peer_robot/g1_43dof_visual_lod.usda | `tools/build_peer_visual_lod_usd.py` 生成（入 git） | 43-DoF 纯显示镜像；47 个解析 Gprim、3 份共享材质、无 PhysX schema |
| peer_robot/g1_43dof_standby_visual_only.usda | `tools/build_standby_robot_visual_only_usd.py` 生成（入 git） | 静态站位机器人专用；引用完整 G1 网格、烘焙 SONIC 默认姿态，三组物理 variant 全关，无活动 Physics schema |
| nolo_label.png | 分叉 git | warehouse USD 相对引用的地面贴花 |
| props/pushcart_physics.usda | 分叉工作区手拷（未入 git） | 引用 Nucleus 5.1 SM_PushcartA_02 |
| props/cart_box_d05_physics.usda | 分叉 git-LFS tip 版 | D05 平底刚体封装，已含关 CCD 修复（ae9118a2e）；现已注册到默认随机池 |
| props/cart_box_d02_physics.usda | 本仓库任务专用物理层 | 流水线压皱箱；引用 `SM_CardBoxD_02`，增加蓝色材质覆盖和白色顶标，使用 0.38×0.25×0.1663 m 平底 convexHull |
| props/cart_box_d01_physics.usda | 本仓库任务专用物理层 | 流水线纸箱队列的第一种箱型；与 d05 同构但引用 `SM_CardBoxD_01`（= v61 `ConveyorBelt_Box_XX` 的视觉源），删掉原资产的 triangle-mesh 碰撞、另挂 0.38×0.25×0.149 m 的 convexHull，原点在箱底面 |
| props/cart_box_c01_physics.usda | 本仓库任务专用物理层 | C01 0.50×0.50×0.25 m 大箱封装；重新注册到默认随机池，但 Dex3 实抓尚未验收 |
| props/cart_box_c02_physics.usda | 本仓库任务专用物理层 | 新增 C02 0.50×0.50×0.311 m 加高大箱封装；删除官方 triangle-mesh 碰撞，改用平底 convexHull |
| props/cart_box_d03_physics.usda | 本仓库任务专用物理层 | 新增 D03 高型不规则纸箱封装；用 0.4126×0.2606×0.2656 m 对称平底碰撞盒保守包住偏心视觉网格 |
| props/cart_box_d04_physics.usda | 本仓库任务专用物理层 | 新增 D04 印刷纸箱封装；轮廓为 0.38×0.25×0.149 m，使用平底 convexHull |
| props/parcel_soft_a01/a02/a03.usda | IsaacLab 分叉 `feat/pickplace-parcel-assets`（LFS）拷入 | 程序化快递软包裹（刚体），自包含无外部引用；入库时摩擦归一 1.4/1.1 multiply、碰撞换平底雪橇盒（原枕形凸包滑动摇晃耗能，混排时比纸箱慢） |
| props/conveyor_path_support_physics.usda | 本仓库任务专用物理层 | `legacy` 默认的完整 L 路径连续静态托面；单个连通顶面 Mesh，无主线/拐角/支线对接处的内部竖直面 |
| props/tote_b04_compound_physics.usda | 本仓库任务专用物理层 | 默认料筐碰撞；复用 SimReady 视觉，以底板+四壁 5-box compound 保持开口语义 |
| props/tote_b04_physics.usda | 分叉 git-LFS | 历史 convex decomposition 回退；内嵌 2.0/1.6 combine=min 高摩擦材质 |
