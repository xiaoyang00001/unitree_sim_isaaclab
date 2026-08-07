# Conveyor Workcell Lite 资产说明

`conveyor_workcell_lite.usd` 是一个约 10 KiB 的 ASCII USD 白名单层。它不复制或重写
v61 的几何，只对 `warehouse-simple6_v61_visual_only.usda` 中任务需要的顶层
Prim 建立显式 reference。因此未入白名单的货架、成千纸箱和运行时设置不会进入
组合 Stage，也不需要在 Git 里再提交一份大体积二进制资产。

## 保留与删除边界

保留 63 个 v61 顶层 Prim：

- `/Root/ConveyorBelt`：三段输送机及其工位内部视觉；
- `/Root/GroundPlane`：原场景地面碰撞；
- `SM_floor47` 和 `SM_floor58`：覆盖传送带和两台机器人工位的两块地面视觉；
- `FloorZone_*` 和 `Stripe_*`：工位安全与物流标识；
- `SM_WallA_*`：v61 根层中的仓库外墙实例，用作 XR/监控视角的空间参照。

以“不引用”的方式删除其余 2,954 个顶层 Prim，包括：

- `/Root/Camera`、所有原场景灯光与灯具；任务配置自己生成一盏 `sun`；
- `/PhysicsScene`、`/Render`（RenderProduct/Settings/Var）和 `/NavMesh`；
- 远处货架、纸箱堆、KLT 仓储区、叉车、推车、旧工作桌和其他装饰物。

## 可复现生成与检查

清单位于 `conveyor_workcell_lite.manifest.json`，锁定了源层 SHA-256、根节点数、白名单
规则、必须保留/删除的路径以及组合降幅门槛。必须在 Isaac Lab/Isaac Sim 5.1 环境中运行：

```bash
python tools/build_conveyor_workcell_lite.py
python tools/build_conveyor_workcell_lite.py --check
```

`--check` 不写文件，会同时检查：

- 源哈希和 3,017 个根子节点的审计前提是否变化；
- 仓库中的 USD 是否与生成器逐字节一致；
- 三段输送机、地面和墙体是否有效，相机/Render/NavMesh/PhysicsScene 等是否缺席；
- active Prim 和 used layer 的实际组合降幅。

2026-08-07 在 Isaac Sim 5.1 的静态组合结果：

| 指标 | v61 visual-only | workcell-lite | 降幅 |
|---|---:|---:|---:|
| 顶层根 Prim | 3,017 | 63 | 2,954 |
| active Prim | 27,802 | 1,052 | 26,750 |
| used layer | 1,818 | 13 | 1,805 |

上述是资产组合统计，不是 FPS/物理性能结论；本轮按要求不执行动态验收。

## 依赖和启用方式

该薄层仍依赖仓库内的 `warehouse-simple6_v61_visual_only.usda`、
`warehouse-simple6_v61.usd` 和 `ConveyorBelt02.usd`。组合 Stage 还保留 8 个 Isaac Sim 5.1 官方远程层：

- 地面标识 3 个：Keepclear、RecRed1X1、StripeFull_4m；
- 墙体 4 个：WallA/WallB 的 6M 和 InnerCorner；
- 地面 1 个：`SM_floor02.usd`。

所有远程路径锁定在
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/`；本轮未将 NVIDIA 资产
blob 复制进 Git，离线包和 resolver 切换仍属后续资产部署工作。

功能集成期暂不改默认背景，显式启用：

```bash
ISAACLAB_CONVEYOR_BACKGROUND=workcell_lite python sim_main.py ...
```

随时可回退到 `visual_only`（当前默认）或 `legacy_v61`。
