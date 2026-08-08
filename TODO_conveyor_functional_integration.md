# 流水线功能集成 TODO

更新时间：2026-08-08

当前集成分支：`feat/conveyor-functional-integration`

## 已合入但尚未做完整验收

- v61 动态装饰物纯视觉清理；
- `ConveyorBelt02_visual_only.usda` 与 Surface Velocity 实验驱动；
- `conveyor_workcell_lite.usd` 及按布局卸载无关道具；
- tote 底板加四壁的 5-box compound collider；
- `ankles/all/off` ContactReport 模式；
- 镜像机器人纯 USD FK/Xform 视觉 LOD（当前为 opt-in）。

## P0：补齐 workcell-lite 与纯视觉输送机的组合层（✅ 静态部分已完成）

分支 `feat/conveyor-workcell-visual-only-adapter` 已落地：

1. ✅ `conveyor_workcell_lite_conveyor_visual_only.usda` 由
   `tools/build_conveyor_visual_only_usd.py` 生成：subLayer
   `conveyor_workcell_lite.usd`；由于 lite 的 `/Root/ConveyorBelt` 是 reference
   （payload 在引用目标 layer stack 内，跨 arc 的 `delete payload` 无效），组合层
   把 reference 重定向到完整仓库 adapter 的 `</Root/ConveyorBelt>` 子树，payload
   替换在该 stack 内生效，v61 的 xform 与子 Prim 覆盖保留；
2. ✅ `asset_variants.py` 按 baseline 文件名路由两份 adapter；组合层缺失、
   subLayer 不是轻量工位、引用未替换、或被依赖的完整 adapter 过期时 fail-fast；
3. ✅ 生成器 `--check` 与 `tests/test_conveyor_workcell_visual_only_adapter.py`
   静态确认：ConveyorBelt 0 rigid/0 collision，仅存地面+地贴+外墙静态边界碰撞
   且地面/外墙碰撞必须健在，lite Prim/layer 门槛满足，Camera、Render、NavMesh、
   PhysicsScene、货架、额外灯光保持缺席，belt 视觉签名与本地变换均与完整
   adapter 一致，instanceable Prim 出现即报错拒绝（防审计链静默漏检）；
4. ✅ `CONVEYOR_VISUAL_ONLY_ASSET_ENABLED` 兼容两份 adapter，最终模式日志为
   `workcell_lite+conveyor_visual_only[(surface_forced)]`。

⚠️ 仍属 P1 的动态验收（Kit 启动、双端、抓取、长时、性能 A/B）未做；在此之前
不要把“workcell_lite + surface_velocity”当作轻量场景性能结果。

## P1：集成与动态验收

- 运行合并后的全量 pytest、compileall、USD 生成器 `--check` 和最小 Kit 启动；
- 验证默认/推车/legacy_props 三种实体清单、reset 与双端 scene-sync；
- 验证 5-box tote 的抓取、放置、900 步到站和多轮 reset；
- 验证 Surface Velocity 的停止段、举离脱驱动、侧导轨和权威 ID 门禁；
- host/viewer 回放 43 关节，检查两台 visual LOD、Dex3 手指、XR anchor、掉帧与重连；
- 对 articulation/visual_lod、完整背景/workcell-lite 做 CPU、GPU、内存和帧时间 A/B；
- 验收通过后再决定是否把 `workcell_lite`、`surface_velocity`、`visual_lod` 改为默认。

## P2：配置健壮性

- 校验 `ISAACLAB_SCENE_SYNC_OBJECTS` 是当前 `SCENE_PROPS.spawned_names` 的无重复
  子集，避免运行期 `KeyError`；
- 在双端 props/layout 不一致时明确报告 object inventory mismatch；
- 评估并移除 viewer 为兼容 SONIC EnvCfg 保留的场外隐藏 ghost articulation。
