# 流水线镜像机器人纯显示 LOD

## 目标与边界

`ISAACLAB_PEER_ROBOT_MODE=visual_lod` 把 `peer_robot`（viewer 模式还包括
`peer_robot_2`）从 Isaac Lab `Articulation` 换成纯 USD 显示资产。它仍逐帧消费
`scene_state` 中的 base pose 和全部 43 个关节角，通过本地 FK 更新各 link 的
`xformOp:orient`，不是静态整机，也不是把 articulation 隐藏起来。

默认值仍为 `articulation`。这让已有现场链路保持原行为，纯显示版本在完成动态
验收前必须显式开启。

本次只移除“镜像机器人”的物理开销。viewer 为兼容现有 SONIC EnvCfg 仍保留一个
场外、不可见的本机 ghost articulation；镜像场景物体仍是 kinematic body。因此
`visual_lod` 不等于整个 viewer 进程完全无 PhysX。

## 启用方式

更新发布端和接收端到含本功能的同一版本。接收镜像的进程增加：

```bash
ISAACLAB_PEER_ROBOT_MODE=visual_lod \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor ...
```

适用形态：

| 本机身份 | 纯显示对象 |
|---|---|
| ID=1/2 对等端 | 对端一台 `PeerRobot` |
| ID=0 viewer | `robot_1 → PeerRobot`、`robot_2 → PeerRobot2` |
| host 双真身 | 无镜像对象，开关不改变两台本机动力学机器人 |

回退只需删除该环境变量或显式设置：

```bash
ISAACLAB_PEER_ROBOT_MODE=articulation
```

无效拼写会直接报错，不会静默回退。

## 数据链路

```text
host/peer articulation
  └─ scene_state: root_state[13] + joint_pos[43] + joint_names[43]
       └─ visual_lod receiver
            ├─ 校验 joint_count、名称集合和 joint_order_hash
            ├─ 按名称映射 wire order（不假定数组位置）
            ├─ root pose → PeerRobot 根 Xform
            └─ 43 × URDF origin/axis FK → 各 link 局部 orient
```

协议 schema 仍是 `g1_peer_scene_state.v1`，新增的 `joint_names` 是向后兼容字段：旧
articulation 接收端会忽略它；纯显示接收端必须收到该字段，因为本机没有镜像
articulation 可用于推导 Isaac 的实际关节顺序。旧发布端连接纯显示接收端时，帧会
被明确拒绝并提示更新发布端，不会按猜测顺序驱动错误关节。

## 资产构成

入口资产：
`tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot/g1_43dof_visual_lod.usda`

- 47 个 link Xform，覆盖 43 个活动关节、pelvis、两只 palm 和 `head_link`；
- 47 个 Cube/Sphere/Capsule 解析显示 Prim，0 个三角 Mesh；
- 3 份共享 `UsdPreviewSurface` 材质；
- 0 个 `RigidBodyAPI`、`CollisionAPI`、`ArticulationRootAPI`、PhysX joint、actuator
  和 `ContactReportAPI`；
- 保留 `/PeerRobot/pelvis` 和 `/PeerRobot/.../torso_link/head_link`，viewer 的 XR
  锚定路径无需另做分支；
- 资产约 55 KiB，由 `tools/build_peer_visual_lod_usd.py` 确定性生成。

视觉造型是任务专用低面数替身，重点保留人体轮廓、四肢关节和 Dex3 手指开合可读性，
不追求近景 CAD 外观。需要高精度展示时回退 articulation 模式。

重新生成：

```bash
python3 tools/build_peer_visual_lod_usd.py
git diff --exit-code -- \
  tasks/g1_tasks/g1_29dof_sonic_conveyor/scene_assets/peer_robot/g1_43dof_visual_lod.usda
```

## 当前已完成的静态验证

普通 Python 的 FK、关节集合、乱序映射、wire hash、模式解析和资产可复现测试：

```bash
python3 -m unittest tests.test_peer_visual_lod -v
```

有 `pxr` 的环境可执行组合检查：把 USDA reference 到
`/World/envs/env_0/PeerRobot`，构造 `UsdVisualLodMirror`，写入非零 base pose 和
关节角后确认：102 个组合 Prim、47 个 Gprim、0 个刚体/碰撞 API，根位姿和关节
orient 均产生有效 authored override。

```bash
python tools/validate_peer_visual_lod_usd.py
```

## 动态验收后置项

按当前施工顺序，功能先合入，动态验收集中放到后续联合场景阶段：

1. host → 单 viewer 的 43 关节全范围回放，检查腿、腰、手臂和 14 个 Dex3 关节；
2. viewer 同时接收 `robot_1`、`robot_2`，检查两台镜像互不串写；
3. AR 分别锚定 robot 1/2，检查 `head_link`/`pelvis` 路径、平滑和 recenter；
4. reset/session 切换、乱序/缺帧/stale 恢复；
5. articulation 与 visual_lod 的 CPU、GPU、内存、渲染帧时间 A/B。

在这些动态项完成前，不把 `visual_lod` 改成默认模式，也不宣称已经达到某个性能
提升百分比。
