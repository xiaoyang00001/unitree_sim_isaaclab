# 流水线功能集成 TODO

更新时间：2026-08-07

当前集成分支：`feat/conveyor-functional-integration`

## 已合入但尚未做完整验收

- v61 动态装饰物纯视觉清理；
- `ConveyorBelt02_visual_only.usda` 与 Surface Velocity 实验驱动；
- `conveyor_workcell_lite.usd` 及按布局卸载无关道具；
- tote 底板加四壁的 5-box compound collider；
- `ankles/all/off` ContactReport 模式；
- 镜像机器人纯 USD FK/Xform 视觉 LOD（当前为 opt-in）。

## P0：补齐 workcell-lite 与纯视觉输送机的组合层

当前 `asset_variants.py` 只有一份固定 subLayer 完整 clean-v61 的 conveyor
visual-only adapter。选择 `ISAACLAB_CONVEYOR_BACKGROUND=workcell_lite` 后再启用
`ISAACLAB_CONVEYOR_VISUAL_ONLY_ASSET=1` 或 `surface_velocity`，会切回完整 clean-v61，
从而丢失轻量工位效果。

后续需要：

1. 生成 `conveyor_workcell_lite_conveyor_visual_only.usda`，subLayer
   `conveyor_workcell_lite.usd`，并把 `/Root/ConveyorBelt` payload 替换成
   `ConveyorBelt02_visual_only.usda`；
2. 在 `asset_variants.py` 按 baseline 文件名路由两份 adapter，缺失或 subLayer
   不匹配时 fail-fast；
3. 扩展生成器和测试，确认组合后输送机为 0 rigid/0 collision，同时仍满足 lite
   Prim/layer 门槛，且 Camera、Render、NavMesh、PhysicsScene、货架等保持缺席；
4. 修正最终模式日志与 `CONVEYOR_VISUAL_ONLY_ASSET_ENABLED` 判断。

完成前不要把“workcell_lite + surface_velocity”当作轻量场景性能结果。

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
