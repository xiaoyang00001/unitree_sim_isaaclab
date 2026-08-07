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

任务默认不启用该变体。A/B 时显式设置：

```bash
ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET=1 \
python tools/smoke_conveyor_scene.py --steps 900
```

## 与第一优先级背景清理层组合

本独立工作树基于提交 `9865bcf`，其中还没有第一优先级提交 `f46c157` 生成的
`warehouse-simple6_v61_visual_only.usda`。因此当前 standalone adapter
`warehouse-simple6_v61_conveyor_visual_only.usda` 暂时 subLayer 原始 v61。

合并 `f46c157` 后，**不能把任务默认背景直接改成当前未重新生成的 standalone
adapter**，否则会绕过第一优先级层并重新引入 10 个 `ConveyorBelt_Box` 和 5 个
`KLT_Bin` 的根层物理。

正确组合方式如下：

1. 先把两个提交合入同一工作树；
2. 重新运行 `python tools/build_conveyor_visual_only_usd.py`；
3. 生成器检测到 `warehouse-simple6_v61_visual_only.usda` 后，会让 conveyor
   adapter subLayer 该清理层，而不是原始 v61；
4. 未设置环境变量时，`asset_variants.py` 仍选择第一优先级清理层；只有设置
   `ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET=1` 时才选择组合后的 conveyor adapter；
5. 再运行 `--check`、单元测试和动态 smoke。

最终组成关系应为：

```text
warehouse-simple6_v61_conveyor_visual_only.usda
  -> warehouse-simple6_v61_visual_only.usda       # 清理 15 个根层装饰物物理
     -> warehouse-simple6_v61.usd                 # 原始仓库基线
  -> /Root/ConveyorBelt payload 替换为
     ConveyorBelt02_visual_only.usda
       -> ConveyorBelt02.usd                      # 仅复用视觉内容
```

集成验收至少确认：

- `/Root/ConveyorBelt` 子树有效刚体和碰撞均为 0；
- 10 个 `ConveyorBelt_Box` 与 5 个 `KLT_Bin` 不再具有有效动态物理；
- `ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET=0/1` 两种模式均能加载，且 `0` 可回退；
- 两筐在 smoke 中保持在代理板上并到达目标 `y≈14.148`。

当前独立分支的 900 步 smoke 已确认派生 payload 能加载、输送机子树没有物理解析
告警，但一只筐只到 `y≈16.99`，另一只筐横向漂出窄代理板后掉落。这个动态问题不
由视觉资产层单独解决，需要与根层障碍物清理、Surface Velocity 驱动和代理碰撞
尺寸调整一起复验。
