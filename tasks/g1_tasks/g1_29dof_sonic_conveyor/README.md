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

| 值 | 布局 | 关键世界坐标（已含北移 Δ=3.55） |
|---|---|---|
| `1`（默认） | 5 个纸箱排在工位**上游**、挡停放行流向工位；机器人分站流水线两侧 | 工位 `y=17.698`（机器人 `x=-4.75 / -6.7`）；队首出生 `y=18.50`（间距 0.75 向上游排）；`y_stop=17.698` |
| `0` | 两个原尺寸塑料筐叠放在入料口推车上；机器人面对面站在推车两侧 | 作业组 `(-5.62, 22.30)`（机器人 `x=-4.82 / -6.42`）；`y_stop=15.05` |

例如：`ISAACLAB_TOTES_ON_CONVEYOR=0 python sim_main.py ...`。

两种布局用**同一个** Δ，不按布局分叉：`=0` 的作业组本来就站在入料端之北，北移后
拖车占位 `y[21.888,22.712]`，离带端仍是原来的 `0.116 m`、离 +Y 墙面 `23.606` 还有
`0.894 m`（墙给的上限约 `Δ≤4.24`，远大于 3.55），离线实测与背景全部叶子 0 冲突。
但 `=0` 的遮挡罩**钉死在 `scale=1.0` 的出料端摆法**，不跟着入料端一起放大靠墙——
理由见下面「端口遮挡罩」。

默认的 `ISAACLAB_CONVEYOR_PROPS=layout` 会真正不生成当前布局用不到的道具：

| 布局 | 实际生成的任务道具 |
|---|---|
| 流水线 | `belt_box_1` … `belt_box_5` |
| 推车 | `pushcart_2` + `cart2_tote1` + `cart2_tote2` |
| `legacy_props`（任一布局） | 旧的 10 个道具，**不含**纸箱 |

两种布局都不再继承原点的 packing table/cubes，也不生成第一组推车、纸箱和
`test_box`；scene-sync 默认清单与实际生成清单共用同一解析结果。排障或视觉 A/B 可设
`ISAACLAB_CONVEYOR_PROPS=legacy_props` 恢复旧的全量道具——它是**纯回退**，不叠加纸箱
队列。两者不能共存：塑料筐的流水线出生位 (-5.35, 20.95) / (-5.89, 21.55) 与纸箱队列的第
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

### 端口遮挡罩（料筐的来处 / 去处）

流水线两端原本是断头，料筐在带面上凭空出现/消失。`ISAACLAB_CONVEYOR_SHROUD` 放一台
**纯视觉**的隧道式 X 光安检机（DigitalTwin `SecurityBaggageScanner_A01_01`，全资产包里
唯一自带隧道洞口 + 铅胶条帘的件），让带面看起来是从设备里穿出来 / 钻进去的：

| 值 | 行为 |
|---|---|
| `auto`（默认） | 生成遮挡罩，端口随 `ISAACLAB_TOTES_ON_CONVEYOR` 切换 |
| `on` | 同 `auto`，显式打开 |
| `off` | 不生成；启动日志仍打印它本该在的位置 |

拼写错误直接报错。`ISAACLAB_CONVEYOR_SHROUD_FACE_Y=<float>` 覆盖**机身 +Y 端面**的
世界 y（两种布局同一语义），整体沿 Y 平移罩子。

#### 入料端（`=1`，默认）：靠墙 + 流水线贯穿

原来那套“贴在端口外侧、与带体首尾相接”的偏移逻辑已经**去掉**。现在机身**远端面贴
+Y 落地墙** `SM_WallA_6M16`（离线实测南面 `y=23.6055`，取 `23.606`，留 `0.05` 墙缝），
并放大 `SHROUD_INFEED_DESIGN_SCALE=1.40`；流水线整体北移 `3.55 m` 反过来插进隧道口，端头
断面藏在不透明机柜里。被带体吞掉的那半段外露滚筒床在薄层里直接置
`visibility = "invisible"`。

| 项 | 值 |
|---|---|
| 机身 pos / yaw / scale | `(-5.620, 21.77206, -0.42990)` / `0°` / `0.014`（= 0.01 × 1.40） |
| 机身占位 | `x[-6.210,-5.030]` `y[19.988,23.556]` 可见顶 `z=1.673`，埋地 `0.430` |
| 机身尺寸 | `1.181 (W) × 3.568 (L) × 2.103 (H)`，宽度略宽于流水线实测最宽处 `1.151` |
| 隧道条帘 | `y[21.358,22.186]`，洞口 `z[0.768,1.446]`、净宽 `0.884` |
| 落地机柜 | `y[21.282,22.263]`（不透明，带端断面藏在这里） |
| 带体入料端 | `y=21.772` —— 越过近端条帘 `0.415`，离远端条帘还差 `0.414` |
| 隐藏子网格 | `roller/rollercover 10..18` 共 18 个 Mesh（局部 `y[-1.214,-0.643]`） |

**为什么必须放大到 1.40**：两条筐道在 `x=-5.35 / -5.89`、半尺寸筐宽 `0.30`，横向总跨
`0.84 m`（README 旧稿写的 `0.69` 是把筐宽当成 0.15，已更正）。原尺寸条帘净开口只有
`0.632`，根本装不下；`1.40` 撑到 `0.884`，单边余量 `22 mm`。等宽解其实是 `1.365`，但
流水线最宽处是每 2 m 一组的支腿站（实测整线 x 跨度 `1.15114`），`1.365` 反而窄 0.9 mm，
所以取整数档 `1.40`。代价是整机下沉 `0.4299`：机脚/脚轮/底座全埋进地面，机柜直接落地，
观感反而比原来那圈脚轮更干净；可见高度 `1.673` 远低于墙高 `3.10`。

**为什么再多压 10 mm**（`SHROUD_ZFIGHT_SINK`）：资产的 `cartframe` 顶面 native z 恰好
等于 `ASSET_ROLLER_TOP_Z`，只按滚筒面对齐会与流水线滚筒顶 `0.7723` 只差 0.3 mm，俯视
必然 z-fighting。压 10 mm 后车架顶落到 `0.762`（离线实测落差 `10.28 mm`），条帘底边
`0.76845` 比带面低 3.5 mm，帘子轻扎进带面。

**隐藏不了的那几件靠流水线本体吞掉**：`cartframe` / `joints` / `screws` / `feets` /
`feetbase` / `belt` / `caps` 是贯穿全长的单一 Mesh，没法只藏一半。离线点级核对（机柜
以南 10 432 个顶点）显示它们**全部**落在流水线 x 轮廓 `[-6.1925,-5.0414]` 之内、且
全部低于带面 `0.7723`，所以侧视被侧裙挡住、俯视被带面盖住。

#### 出料端（`=0`）：维持老摆法，只随北移平移

| 项 | 值 |
|---|---|
| 机身 pos / yaw / scale | `(-5.620, 11.93576, -0.08935)` / `180°` / `0.01`（design_scale 1.0） |
| +Y 端面 / 占位 | `13.210` / `y[10.662, 13.210]` |

⚠️ **=0 不能套用“放大 + 靠墙”**：放大后半长 `1.784`，贴带尾放会整块插进
`blue_sorting_bin_03`（北移后 `x[-6.221,-4.555] y[12.307,13.642] z[0.003,0.420]`）。
落地的隧道机柜（`z` 从 0.078 到 1.498，占满机身中段 `y ±0.350`）必须整体避开它，于是
+Y 端面上限 = `12.307 + 0.924 ≈ 13.231`，取 `13.21` 留 `0.0204 m` 余量——这条约束随
北移刚体平移，与北移前逐毫米一致。这段 `0.53 m` 空隙视觉上正好被那堆料箱填住。

出料端另用一份**不带隐藏清单**的薄层 `outfeed_scanner_visual.usda`：它的滚筒床要留着
当“料筐的去处”，转 180° 让**完整**的那半段滚筒床朝着流水线（近端那半在入料端薄层里
被隐藏了）。两份薄层引用同一个 S3 源资产。

零物理：走 `AssetBaseCfg` + `UsdFileCfg`，不传 `collision_props` / `rigid_props`，
被引用资产本身 coll=0 / rigid=0（97 个 Prim 已离线审计）。它落在 `scene.extras` 的
XformPrimView 上，不进 PhysX、不参与复位，也不进 `scene_props` 的 scene-sync 清单。
源资产是 cm 原生（自带 `metersPerUnit=0.01` 且无补偿缩放），与 `Tote_B04` 一样由任务侧
用 `UsdFileCfg.scale` 补换算因子。

### 流水线整体北移 Δ = 3.55 m

`conveyor_drive.CONVEYOR_NORTH_SHIFT_Y` 是**唯一真源**（`shroud_config` /
`scene_layout` 各抄一份，测试交叉断言三处一致）。推导：

```
机身中心 y_c = 墙面 23.606 − 墙缝 0.05 − 放大后半长 1.27424×1.40 = 21.772064
Δ = y_c − 北移前带体入料端 18.2223 = 3.54966   →   取 3.55
```

上限约 `3.58`：流水线每 2 m 一组的支腿站外偏 `0.576 > ` 洞口半宽，再往北会扎穿机柜
侧板。几何位移写在背景 clean wrapper `warehouse-simple6_v61_visual_only.usda` 的
`over "ConveyorBelt"`——**世界 y +Δ ≡ 背景局部 x +Δ**（背景挂载时绕 Z 转了 +90°：
`世界x = -4.68 − 局部y`、`世界y = 14.39363 + 局部x`）。四种背景模式经 reference /
subLayer 自动继承（离线实测 Δy 精确 `+3.550000`、Δz 精确 `0`、Δx `-6.8e-5` 是背景
init_state 四元数未归一的既有残差）。

跟着走的东西分两类：`ConveyorBelt` **组内**的三段带体 + 两张打包桌 +
`blue_sorting_bin_01/02/03` + `warehouse_extras_01` 由组变换一起带走；`/Root` 下的
**兄弟** prim 必须各自 +Δ，本层显式覆盖了 `ConveyorBelt_Box_00..09`、`KLT_Bin_00..04`
（带面上的装饰货物）与 `FloorZone_KeepClear` / `FloorZone_Robot` / `Stripe_Conv1`
（地面标线）。

⚠️ `ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61` 的裸 v61 **不含**这个位移（既有约定是
不重写二进制源资产），它的带体会与代码常量差 3.55 m。该回退档从本次改动起只能用于
“看看原始 v61 长什么样”，不能再当功能 A/B 基线。

### 纸箱队列的整带节拍启停

流水线布局的作业对象是 5 个箱/包，**两种交错排布**（默认小纸箱 + 白色软包裹）：

| key | 来源 | 尺寸（沿带 × 横向 × 高） | 质量 | 资产 |
|---|---|---|---|---|
| `d01` | v61 背景 `ConveyorBelt_Box_XX` 同款纸箱 | 0.38 × 0.25 × 0.1487 m | 1.0 kg | `props/cart_box_d01_physics.usda` |
| `c01` | v61 背景 `KLT_Bin_XX` 同款大纸箱 | 0.50 × 0.50 × 0.25 m | 1.5 kg | `props/cart_box_c01_physics.usda` |
| `parcel_a01` | 程序化快递软包裹（灰） | 0.402 × 0.301 × 0.080 m | 0.35 kg | `props/parcel_soft_a01.usda` |
| `parcel_a02` | 程序化快递软包裹（白，面单朝上） | 0.453 × 0.352 × 0.097 m | 0.5 kg | `props/parcel_soft_a02.usda` |
| `parcel_a03` | 程序化快递软包裹（粉） | 0.322 × 0.241 × 0.060 m | 0.22 kg | `props/parcel_soft_a03.usda` |

⚠️ 软包裹是**刚体不是软体**：枕形鼓包、热封边、褶皱、顶面白色面单都只是视觉造型，物理
上与纸箱同一套约定（根挂 RigidBody+Mass、凸包碰撞、原点在袋底）——CPU pipeline 不支持
deformable，本场景固定 `--device cpu`。资产取自 IsaacLab 分叉 `feat/pickplace-parcel-assets`
的程序化生成器，入本仓库时做了两处流水线适配：①抓取摩擦从 1.6/1.2 `combine=max` 归一为
纸箱同款 1.4/1.1 `multiply`（max 会压过带面摩擦，legacy 驱动的写入间隙靠摩擦减速，μd 近
两倍 ⇒ 包裹比纸箱慢 ~35%、整带停时短停 0.24~0.28 m）；②碰撞从枕形 Bag 凸包换成平底
"雪橇"盒（枕形凸包滑动时轻微摇晃耗能，仍慢 ~3%/程，混排时纸箱逐程追上包裹）。两处
适配后实测五件位移 0.809~0.817 m/程，差 <1%。

默认排布是 `d01, parcel_a02, d01, parcel_a02, d01`（`ISAACLAB_BELT_BOX_PATTERN` 循环，
写单个 key 就是全用一种；`d01,c01` 切回双纸箱形态）。箱子只排在工位 `y_stop=17.698` **上游**那一段带面上，默认出生 y 为
`18.50 / 19.25 / 20.00 / 20.75 / 21.50`（间距 0.75，相邻箱子净空隙 0.31 m，含北移 Δ=3.55），车道
`x=-5.62`、`z=0.775`。`belt_box_1` 是队首。

⚠️ **间距、行程、数量此消彼长**。整带节拍下停稳间距 = 出生间距，而工位上游只有
`21.77 - 17.698 ≈ 4.07 m`（北移前 18.22 - 14.148，跨度不变）：

```text
(count-1) × pitch + 端部半长  +  队首行程 (y_lead - y_stop)  ≤ 4.07
```

当前默认吃掉 3.0 + 0.19，只剩 0.80 m 给队首行程（约 3.3 s 流到工位）。想再拉大间距就得
同时下调 `ISAACLAB_BELT_BOX_SPAWN_Y_LEAD` 或减少 `ISAACLAB_BELT_BOX_COUNT`，否则启动直接
报"队尾悬出带面"。几个可用组合：

| 目标 | 命令片段 |
|---|---|
| 更大间距，保 5 箱（行程缩到 0.5 m） | `ISAACLAB_BELT_BOX_SPAWN_PITCH=0.8 ISAACLAB_BELT_BOX_SPAWN_Y_LEAD=18.20` |
| 大间距 + 长行程，减到 4 箱 | `ISAACLAB_BELT_BOX_COUNT=4 ISAACLAB_BELT_BOX_SPAWN_PITCH=0.95 ISAACLAB_BELT_BOX_SPAWN_Y_LEAD=18.65` |
| 回到密排（原默认） | `ISAACLAB_BELT_BOX_SPAWN_PITCH=0.6 ISAACLAB_BELT_BOX_SPAWN_Y_LEAD=19.05` |

运动语义是**整条带一起启停**，不是逐个积放：

```text
belt_running = (最下游那个仍在带面的箱子的 y) > y_stop
drive[i]     = on_belt[i] & belt_running & 没顶到前车
```

* 队首没到工位 → 整带都在走；
* 队首一压到工位 → **整条带立刻停**，后面的箱子原地保持当前间距，不会继续往前挤；
* 机器人把工位那个拎走 → 它掉出带面窗口、不再参与"队首是谁"的判定 → 整带重新启动，
  一起前进到新队首也压到工位为止。

于是每取走一件，整列前进一格（≈ 出生间距），就是真实节拍输送线的样子。实测取件后
四个箱子各前进 0.587 m 后同时停住。

第三个因子是**防撞保底**，正常跑不到：整带同起同停时相对间距恒定，而出生间距（0.6）
远大于任何一对箱子的最小净距。它只兜住摩擦/质量差异带来的缓慢漂移，约束是
`ys[i] > 前车尾部 + 自己半长 + queue_gap`（默认 `queue_gap=0.07`）。⚠️ 这里恒定的是
**净空隙**而不是中心距：两种箱型尺寸不同，统一中心距要么让大箱互相穿模、要么在小箱
之间留出突兀的空档。

整套判据没有任何状态机，也不需要知道谁被抓走了：语义完全由当前帧的位置推出来，天然
可复位、可断点续跑、双机一致。判据在 `conveyor_queue.queue_drive_mask`（不依赖 Isaac，
有单测）。

驱动只在箱底位于带面高度 `z=0.772±0.15`，且原点进入 `x=[-6.17,-5.07]`、
`y=[13.74,21.77]`（= 北移前 `[10.19,18.22]` + Δ=3.55）时生效；箱子被举离带面、掉到地上或放到范围外都不会被强行拖动。
驱动只在物体权威端运行，viewer/镜像端通过场景同步看到相同运动。

`ISAACLAB_CONVEYOR_Y_STOP<=0` 切到**循环模式**：没有工位停止线，整带长跑不停（只剩防撞
保底），箱子到 `y_recycle=14.15` 后被传回 `y_respawn=21.55` 继续循环（含北移）（回收由额外挂上的
`recycle_surface_totes` 事件负责——纸箱的驱动函数只管启停和限速，不搬运）。

推车布局（`ISAACLAB_TOTES_ON_CONVEYOR=0`）仍是两塑料筐的老路径：机器人把筐放上带面
松手后被施加沿 `-Y` 的 `0.3 m/s` 目标速度，送到出料段 `y=15.05` 后交给摩擦停住；筐之间
没有队列语义。

可调项：

| 环境变量 | 默认 | 说明 |
|---|---|---|
| `ISAACLAB_BELT_BOX_PATTERN` | `d01,parcel_a02` | 箱型循环序列（5 种可选）；未知 key 直接报错 |
| `ISAACLAB_BELT_BOX_COUNT` | `5` | 纸箱数量；上限 5（SceneCfg 字段是显式声明的，超限 fail-fast） |
| `ISAACLAB_BELT_BOX_SPAWN_Y_LEAD` | `18.50` | 队首出生 y（世界系，含北移），同时是复位落点；决定队首行程 |
| `ISAACLAB_BELT_BOX_SPAWN_PITCH` | `0.75` | 箱间距；整带节拍下**也就是**停稳后的队列间距 |
| `ISAACLAB_BELT_BOX_QUEUE_GAP` | `0.07` | 防撞保底的**净空隙**（不是中心距）；整带节拍下正常不触发 |
| `ISAACLAB_BELT_BOX_LANE_X` / `_SPAWN_Z` | `-5.62` / `0.775` | 车道与出生高度 |
| `ISAACLAB_BELT_BOX_MASS_<KEY>` | 各箱型默认 | 按箱型覆写质量（kg），如 `_D01` / `_PARCEL_A02` |
| `ISAACLAB_CONVEYOR_SPEED` / `_Y_STOP` / `_ENABLED` | `0.3` / `17.698` / `1` | 速度、工位、总开关 |

下列非法组合都会在启动时报错，不会等到运行时才看见箱子悬空或互相穿模：出生位排到带面
外、**相邻两箱按各自半长算放不下**、队首压在工位上、**停稳后的队列伸出带尾**、箱型比带面
还宽、未知箱型 key。

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
- 停位有 0.06~10 mm 的散布（越线后驱动关闭、靠摩擦停住，各箱摩擦略有差异），smoke 容差
  取 50 mm；整带节拍下这个误差不累计，队列间距长期维持在出生值附近（实测 0.748~0.760，
  出生 0.75），但**不要指望它精确恒定**。
- 队首行程只有 0.80 m（约 3.3 s）。这是为拉大箱间距付出的代价——带面上游总长固定
  4.07 m，间距、行程、数量三者只能三选二。
- **纸箱队列尚未做机器人实抓验收**：`--pick-lead-at` 是把队首直接搬离带面来模拟取件，
  验的是"队列会不会正确放行下一个"，不等于 Dex3 真能抓起箱子。尤其 `c01` 是
  0.5×0.5×0.25 m / 1.5 kg，跨度和重量都明显大于 `d01`，能否抓取完全未验。
- 流水线布局的 robot_2 仍是偏展示的站位，离带面较远，实际可达性尚未完全收口。
- 原布局（`ISAACLAB_TOTES_ON_CONVEYOR=0`）的作业闭环在源分支就未实跑过。
- `surface_velocity` 后端下纸箱队列靠"后车撞前车"物理涌现，没有走
  `queue_drive_mask`；该组合尚未实测，上游带面会持续挤压排队的箱子。
- **箱子出生瞬间只遮住了队尾（TODO）**：北移后队列出生 y 为
  `18.50 / 19.25 / 20.00 / 20.75 / 21.50`，只有队尾那只（`21.50`）落在不透明机柜
  `y[21.282,22.263]` 内部（真正被遮住），前四只都在机柜以南的可见带面上凭空出现。
  与合并前的双筐布局不同，现在复位是"整队重摆"，观感上是一排箱子瞬间出现——
  要全部藏进机柜物理上不可能（机柜只有 `0.98 m` 长）。可选的根治路径：
  ①复位后延迟逐只"从机柜里放出"（改 conveyor_queue 的放行逻辑，工作量大）；
  ②接受现状——复位是显式操作，操作者本来就知道场景重置了。横向尺寸已闭环：
  洞口净宽 `0.884` 大于最宽箱型 `c01` 的 `0.50` 与软包裹的 `0.35`。
- 遮挡罩与整体北移**只做过离线 pxr 静态核对，尚未在 Kit 里目视验收**。待验项：
  带体贯穿处的穿模、埋地 `0.43 m` 的观感、隐藏近端滚筒后机柜落地结构是否露怯、
  三条地面标线（`FloorZone_*` / `Stripe_Conv1`，引用 S3 资产离线取不到包围盒）
  是否真的跟着走了、以及 `warehouse_extras_01` 最北那只纸箱与 `SM_WallA_6M15`
  仅 `0.0248 m` 的间隙在渲染里是否穿帮。
- `ISAACLAB_CONVEYOR_PROPS=legacy_props` 下入料端会多出 `pushcart_2`
  （`x=-5.4, y=22.94363`），落在放大后的罩子占位 `y[19.988,23.556]`
  `x[-6.210,-5.030]` 内部；默认的 `layout` 道具模式在 `TOTES_ON_CONVEYOR=1` 时
  根本不生成 `pushcart_2`，所以只有做 legacy A/B 时才需要用
  `ISAACLAB_CONVEYOR_SHROUD_FACE_Y` 或 `=off` 避让。同模式下的
  `pushcart` / `cart_box*` / `test_box`（`x=-6.8, y=22.94363`）在机身 x 之外，
  离 +Y 墙还有 `0.25 m`，不必避让。
- `ISAACLAB_CONVEYOR_BACKGROUND=legacy_v61` 的裸 v61 不含北移，带体位置与代码常量
  差 `3.55 m`，该模式已不能作为功能 A/B 基线（见上面「流水线整体北移」）。

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
| props/cart_box_d01_physics.usda | 本仓库任务专用物理层 | 流水线纸箱队列的第一种箱型；与 d05 同构但引用 `SM_CardBoxD_01`（= v61 `ConveyorBelt_Box_XX` 的视觉源），删掉原资产的 triangle-mesh 碰撞、另挂 0.38×0.25×0.149 m 的 convexHull，原点在箱底面 |
| props/cart_box_c01_physics.usda | 本仓库任务专用物理层 | 第二种箱型；同构，引用 `SM_CardBoxC_01`（= v61 `KLT_Bin_XX` 的视觉源），convexHull 0.50×0.50×0.25 m |
| props/parcel_soft_a01/a02/a03.usda | IsaacLab 分叉 `feat/pickplace-parcel-assets`（LFS）拷入 | 程序化快递软包裹（刚体），自包含无外部引用；入库时摩擦归一 1.4/1.1 multiply、碰撞换平底雪橇盒（原枕形凸包滑动摇晃耗能，混排时比纸箱慢） |
| props/tote_b04_compound_physics.usda | 本仓库任务专用物理层 | 默认料筐碰撞；复用 SimReady 视觉，以底板+四壁 5-box compound 保持开口语义 |
| props/tote_b04_physics.usda | 分叉 git-LFS | 历史 convex decomposition 回退；内嵌 2.0/1.6 combine=min 高摩擦材质 |
| props/infeed_scanner_visual.usda | 本仓库任务专用视觉薄层 | **入料端**遮挡罩（带体贯穿）；以 S3 绝对 URL 引用 DigitalTwin `SecurityBaggageScanner_A01_01`（cm 原生，任务侧补 0.01 换算），本身不 author 任何物理，只用 `over` 把近端 18 个滚筒/滚筒盖置 `visibility=invisible` |
| props/outfeed_scanner_visual.usda | 同上，无隐藏清单 | **出料端**遮挡罩（贴在带尾外侧）；滚筒床要留着当“料筐的去处”，因此与入料端分成两份薄层 |
