# ConveyorBelt02 visual-only 派生层

## 目标与边界

`ConveyorBelt02.usd` 是约 45 MiB 的原始 USD crate。它不只是三段输送机，默认
Prim `/Root` 下还包含三只分拣箱、两张工作台和一组装饰箱。任务已经用独立的
`ConveyorCollider` 提供承托碰撞，因此本派生层只保留原资产的视觉组成，并关闭
payload 内与任务 proxy 重复的刚体和碰撞；不重写二进制网格，也不改变原资产。

源资产 composed 审计结果由 `ConveyorBelt02_visual_only.manifest.json` 固定：

- 721 个 Prim，其中 440 个视觉 Prim；
- 6 个有效 `RigidBodyAPI`；
- 56 个有效 `CollisionAPI`；
- 源 crate SHA-256：`11c0c2f053f2c1c5afca4cd4148f5131ff498f51014665a1db3a314679031201`。

`ConveyorBelt02_visual_only.usda` payload 原 crate，并在这些审计路径上显式写入
`physics:rigidBodyEnabled = false` 或 `physics:collisionEnabled = false`。视觉网格、
材质和变换仍由原 crate 提供，原文件可随时作为回退。

## 构建与检查

在 Isaac Sim 5.1 / `env_isaaclab` 环境运行：

```bash
python tools/build_conveyor_visual_only_usd.py
python tools/build_conveyor_visual_only_usd.py --check
```

`--check` 会重新审计源文件并逐项验证：

- 生成层和 manifest 与源哈希、路径一致；
- 原始 payload 能解析；
- 派生层的视觉 Prim 路径及类型与源资产完全一致；
- composed visual-only stage 中有效刚体和碰撞均为 0；
- 仓库 adapter 下 `/Root/ConveyorBelt` 同样不存在有效刚体或碰撞。

默认 ``legacy`` 驱动不启用该变体。A/B 时显式设置：

```bash
ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET=1 \
python tools/smoke_conveyor_scene.py --steps 900
```

``surface_velocity`` 驱动会强制选用该变体，即使环境中显式写了
``ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET=0`` 也不会重新引入原生碰撞。

## 与第一优先级背景清理层组合

集成工作树已合入背景清理层，并重新运行了生成器。当前
`warehouse-simple6_v61_conveyor_visual_only.usda` 已 subLayer
`warehouse-simple6_v61_visual_only.usda`，不会绕过 10 个 `ConveyorBelt_Box`
和 5 个 `KLT_Bin` 的背景物理清理。

最终组成关系应为：

```text
warehouse-simple6_v61_conveyor_visual_only.usda
  -> warehouse-simple6_v61_visual_only.usda       # 清理 15 个根层装饰物物理
     -> warehouse-simple6_v61.usd                 # 原始仓库基线
  -> /Root/ConveyorBelt payload 替换为
     ConveyorBelt02_visual_only.usda
       -> ConveyorBelt02.usd                      # 仅复用视觉内容
```

开发阶段的静态检查确认：

- `/Root/ConveyorBelt` 子树有效刚体和碰撞均为 0；
- 10 个 `ConveyorBelt_Box` 与 5 个 `KLT_Bin` 不再具有有效动态物理；
- legacy 未开资产变体时可回退到原生 ConveyorBelt 物理；
- legacy 开启资产变体时使用 clean background + visual-only ConveyorBelt；
- Surface Velocity 总是使用上述 visual-only 组合，任务 proxy/侧导轨是唯一接触几何。

动态 smoke、停止误差和机器人站位属于后续验收阶段，本文档不把静态组合
检查当作功能验收结论。
