# AR 画面卡顿（judder）根因与处置

> 2026-07-29 调查记录。**结论先行：卡顿的根因是 SteamVR + NOLO XrLink 这条链路上
> 全程没有任何重投影（reprojection / ATW）。这不是帧率问题，也不是本工程的渲染优化
> 失效。在无重投影的前提下，应用帧率只要低于头显面板刷新率就必然卡顿，帧率与帧节奏
> 优化只能减轻，不能消除。**

## 1. 症状

`--xr --teleop_device motion_controllers` 下，头显内画面"粘一下、跳一下"，头转动时尤其明显。
表现与应用帧率高低不完全相关：30fps 抖，36fps 也抖。

**极具误导性的现象：打开 SteamVR dashboard（菜单）时不抖。**
这一度让人误判"某个配置能修好它"。可以确定的部分：dashboard 菜单本身由 vrcompositor
以面板刷新率逐帧、用当前最新头姿渲染，所以**菜单永远不抖**；做主观评测时它会盖住应用层的
judder。

> ⚠️ 历史教训：2026-07-28 记录的"72Hz 全链对齐 + 应用匀速 36fps = 流畅"结论，事后
> 复盘确认当时多半开着 dashboard 观察，**该结论已被推翻**。做 XR 主观评测时必须确认
> dashboard 已关闭，否则测的是 compositor 而不是你的应用。

### ✅ 关键实证：dashboard 模式下**连应用画面也不抖**（2026-07-30 用户确认）

两条用户实测，合起来是本文最重要的一个发现：

1. dashboard 开着时 Isaac 画面**并没有定住**，能看到机器人正常走动 → 应用仍在持续提交新帧；
2. 此时**不只是菜单不抖，Isaac 画面本身也不抖**。

**结论：PC 侧"按最新头姿逐帧重新变换应用画面"的能力本来就存在，SteamVR 自己就在做，
只是不对正常的 scene layer 路径启用。** 内容仍是 30fps（机器人在动），几何变换却是 72Hz
的——这就是 ATW 的定义，只是 SteamVR 不这么叫、也不计入 `reprojected`。

这直接推翻了一个过强的表述："PC 侧无法做任何姿态校正"。准确的说法是：**NOLO 驱动内做不到**
（§7 的三条理由仍然成立），但 **SteamVR compositor 做得到，而且正在做**。

由此得到一个可检验的因果假设，见 §4.1：

> 直通是一个优化，前提是只有单一 layer。正常模式 `Compositor Time GPU: 0.007ms`
> 就是在直通；dashboard 一开，compositor 必须自己合成，于是顺带每帧重新变换。
> **若只要存在任何一个可见 overlay 就能逼它转入合成模式，则代价极小——
> 不牺牲立体、不改 Isaac 一行代码。**

⚠️ **不要用 `0 reprojected` 去否证这件事**（2026-07-30 犯过这个推理错误）：那个计数器统计的是
async/interleaved **补帧**路径的次数，dashboard 模式下 compositor 是主渲染源、不是在"补帧"，
其自绘帧不进这个计数。两者不在同一条统计路径上，`0 reprojected` 与"dashboard 在重新变换
应用画面"并不矛盾。

## 2. 根因证据链

| # | 证据 | 出处 |
|---|---|---|
| 1 | `Async support disabled by user setting or a direct mode driver` | `~/.steam/steam/logs/vrcompositor.txt`，每次启动必打印 |
| 2 | `Total.. 228920 presents. 0 dropped. **0 reprojected**` | 同上，会话结束统计。跑了 22 万帧，重投影次数为 0 |
| 3 | `Timed out.641 total.... 2433 presents. 0 reprojected` | 同上。应用没赶上刷新时 compositor **原样重发旧帧**，不做任何姿态校正 |
| 4 | 设 `steamvr.vrsettings` 的 `"enableLinuxVulkanAsync": true` 后重启 SteamVR，日志**仍**打印证据 1 | 2026-07-29 15:30 实测 |
| 5 | NOLO 驱动日志中 ATW / timewarp / reprojection 零命中 | `~/nolo_driver_deploy/log/*.log` |
| 6 | `Compositor Time........CPU: 0.209ms / GPU: 0.007ms` | 同上，6 次会话统计一致（GPU 0.007–0.011ms）。合成路径上 compositor **几乎不做 GPU 工作**——连畸变都由 direct-mode 驱动自己做，它只是把 layer 转手 |

证据 4 是关键的排除法：`user setting` 这一半已被排除，那么打印这行的原因只剩
`a direct mode driver` —— **NOLO 是 direct-mode 串流驱动，SteamVR 不对它启用 PC 侧异步重投影**。

⚠️ **证据 2/3 的适用范围**：`reprojected` 计数器只统计 async/interleaved **补帧**路径。
compositor 自绘（如 dashboard 模式）不进这个计数，因此不能用它去否证 §1 的解释 A。

### 附带发现：XR 帧率瓶颈 100% 在 CPU（2026-07-30）

同一批统计行里的 `Game Info` 一栏（`Target 72`，即 SteamVR 目标就是面板 72Hz）：

| 会话 | ApplicationTime CPU | ApplicationTime GPU |
|---|---|---|
| 07-29 15:02 | 8.4ms | 0.42ms |
| 07-29 15:28 | 18.5ms | 0.59ms |
| 07-29 14:34 | 23.6ms | 1.04ms |
| 07-29 18:05 | 25.0ms | 0.70ms |
| 07-30 00:13 | 33.3ms | 3.16ms |
| 07-30 00:14 | 34.2ms | 3.18ms |

**GPU 全程 0.4–3.2ms，对 13.9ms 的帧预算等于空转；CPU 8–34ms 才是唯一的限速环节。**
这条独立于卡顿根因，但它把提帧率的可行动作钉死了：一切降分辨率 / 降超采样 /
`rendering_mode=performance` 的想法都无效（与 §3 表格里那一行互为印证），要提帧率**只能压 CPU**。

### 机制

面板 72Hz，每 13.9ms 必须出一帧。应用只有 36fps 时，每个应用帧要在合成流里出现 2 次；
30fps 时在 2 次和 3 次之间摇摆。**没有重投影 = 第 2、3 次展示时用的还是那一帧渲染时刻的
头部姿态**。用户的头在这期间一直在动，于是画面相对头部"粘住"，等新帧到来再"跳"回正确位置。

这也解释了为什么 30fps 比 36fps 更难受：36fps 是均匀的"每帧重复 2 次"，30fps 是
2 次/3 次交替，抖动本身还带节奏变化。

## 3. 已被实测否决的方案（不要重跑）

| 方案 | 结果 | 原因 |
|---|---|---|
| `enableLinuxVulkanAsync = true` | ❌ 无效 | direct-mode 驱动旁路，日志无变化 |
| 关 SteamVR 帧率帽（`--disable_xr_frame_cap`） | ❌ 更差 | 平均帧率 21→34Hz 但帧间隔散乱；已降级为诊断开关 |
| `--late_render_repeat 2`（连渲两帧） | ❌ 负优化 | 第二帧成本涨 26%，物理掉到 20Hz 以下 |
| `--late_render_interval 2`（XR 下隔圈渲染） | ❌ 仍抖 | 见下 |
| 降分辨率 / `rendering_mode=performance` | ❌ 无效 | 渲染成本大头是 kit CPU 地板，与像素量无关 |
| 回滚 NVIDIA 驱动 | ❌ 与卡顿无关 | 驱动问题是另一回事（见 §5） |

### 为什么 XR 下隔圈渲染锁不住匀速档

隔圈渲染会让循环里同时存在**两套互不整除的刚性时钟**：

- 非渲染圈：由 `RobotController` 的 `step_hz` deadline 网格定拍（50Hz → 20ms）
- 渲染圈：由 `xrWaitFrame` 的帧槽网格定拍（72Hz → 13.9ms）

40ms 对 13.9ms 除不尽，每对循环相位漂移约 12ms，渲染提交仍在 2 帧槽与 3 帧槽之间游走。
**每圈渲染（interval=1）之所以能锁相，正是因为全环只剩 xrWaitFrame 一个时钟。**

因此 `--late_render_interval` 在 XR 下保持默认 1；该参数现在接受显式覆盖（不再静默丢弃
用户输入），但改它救不了卡顿。

## 4. 有效方向

按优先级：

1. **换用 CloudXR 通路（已实现，见 §6）** —— 唯一能**根治**的方向。CloudXR 的头显端
   WebXR 客户端自带深度重投影，本机 runtime 已装好且 Isaac Sim 已实连验证过。
   用 `--xr_runtime cloudxr` 一键切换。
2. **头显端 NOLO 客户端实现 ATW** —— 只能向 NOLO 提需求，PC 侧做不了（理由见 §7）。
3. **提高并稳住应用帧率** —— 治标。无重投影时 36fps（均匀重复 2 次）是能达到的最舒服状态，
   优于 30fps 的交替抖动。conveyor 场景物理比主场景重约 4ms，是它掉出 36fps 的原因。
   ⚠️ 但要注意：在这条无重投影的链上，**唯一真正不抖的应用帧率是跑满 72fps**，没有中间档
   （36fps 匀速那个"流畅"结论已被 §1 的 dashboard 污染推翻）。按 §2 的 CPU 账本，
   当前单帧 CPU 23–34ms，到 13.9ms 需要 2.4 倍以上的提升，**不乐观**。
4. **⭐ 让 compositor 待在 dashboard 模式** —— 见 §4.2。目前唯一被实测证明"留在
   SteamVR + NOLO 链上还不晃"的方向。（其近亲"仅靠存在 overlay 逼它合成"已被否决，见 §4.1.1。）

### 4.1 ❌ 已否决：靠存在 overlay 逼 compositor 退出直通

**假设**：`Compositor Time GPU: 0.007ms`（证据 6）说明正常模式下 compositor 在**直通**——
把应用的 swapchain 图像原样转手给 direct-mode 驱动，连畸变都不做。而直通的前提是只有单一
layer。**只要存在任何一个可见 overlay，compositor 就必须自己合成，可能顺带就把应用画面
也按最新头姿每帧重新变换了**（正是 §1 里 dashboard 表现出来的行为）。

若假设成立，代价近乎为零：不牺牲立体、不损失视野、不改 Isaac 一行代码，只需常驻一个
几乎不可见的小 overlay。**但它是错的。**

第一轮读数（Isaac 为 scene app 已确认，`panel` overlay 正显示在头显里）：

```
comp_gpu = 0.006–0.008ms   ← 与日志里 6 次会话的 0.007ms 完全一致
comp_cpu = 0.28–0.37ms
app_gpu  = 10–12ms
interval = 21.6–28.7ms     → 应用 ~35–46fps
presents = 1   reproj = none
```

**决定性的主观判读（用户实测）**：in-game overlay **和 Isaac 画面一起晃**——
> "这个和 isaac 是分开渲染的，问题一样，都会晃动。steamvr 的 dashboard 是和 isaac
> 合成为一个画面，在头显侧不晃动。"

**假设否决定案** —— 依据是上面这条主观判读。

> ⚠️ **撤回一条曾用来支撑它的证据（2026-07-30）**：原文写"客观上 comp_gpu 与基线毫无差别，
> 两个独立证据同向"。**那条支撑无效** —— 后来在 dashboard 开着（`dash=ON` 确认、画面确认
> 不晃）时采到 comp_gpu 均值 **0.0080ms**（12 样本），与直通基线一样。
> 所以 `m_flCompositorRenderGpuMs` 在 direct-mode 下**没有区分力**（overlay 应用拿到的
> 大概不是全局渲染开销，或 compositor 自绘不计入该字段）。
> **别再把它当判据**，它只是诊断读数；工具里已改为每次运行都打印这句提醒。

**错在哪**：我把"compositor 必须合成 overlay"等同于"compositor 每 vsync 重新渲染"。
实际上 compositor 只在**应用提交新帧时**把 overlay + scene layer 合成一张，然后整张原样
重发给 direct-mode 驱动。**合成 ≠ 每帧重新渲染**——所以 overlay 跟着 scene 一起带着过期
头姿被重复展示。

这也顺带说明 dashboard 的差别到底在哪：它让 compositor 变成**每 vsync 的渲染源**，
而不是"多了一个 layer 要合成"。转向见 §4.2。

**踩到的两个坑（已修，都会产出假结论）**：

1. 三轮跑完只看到探针图案、看不到 Isaac 画面 —— 探针是 overlay 应用，**不提供场景画面**。
   而且没有 scene app 时 compositor 要渲染 SteamVR 自己的环境，comp_gpu 必然非零，
   恰好长得像"假设成立"。现已改为**没有 scene app 就拒绝测量**（逃生门
   `--allow-no-scene`），并在启动时打印 `✅ scene app pid = ...`。
2. 读数全是 `nan` 却不报错 —— pyopenvr 的 `getFrameTiming()` 便捷封装返回的是
   `(result, timing)` **元组**而非 struct，且不设 `m_nSize`（openvr.h 明确要求）。
   原实现用 `getattr(..., default)` 兜底"防崩"，结果把这个错误静默成了 nan，白跑三轮。
   现已直接走 `function_table` 并自填 size；**测量工具里的静默兜底是负资产**，
   改为启动时核对字段、缺了就当场退出。

**同时否决的近亲写法**：通过 OpenXR 提交 `XrCompositionLayerQuad` —— 同样只是"多一个 layer
要合成"，而上面已经证明合成 ≠ 每帧重新渲染；kit 也没暴露这个开关。

**同时否决的退路**：把画面搬进 overlay（scene layer 提交纯黑、`IVROverlay` 持画面纹理）——
既然 overlay 自己就跟着晃，这条退路的前提就不存在了。

### 4.2 ⭐ 当前主攻：让 compositor 待在 dashboard 模式

**唯一被实测证明"留在 SteamVR + NOLO 链上还不晃"的方向。** 用户实测：dashboard 开着时
连 Isaac 画面都不晃，且机器人照常走动。机制是 compositor 成为**每 vsync 的渲染源**，
把 Isaac 画面逐帧按最新头姿重绘（§1）。

做法：`showDashboard(key)` 可以指定显示哪个 dashboard overlay。用
`createDashboardOverlay()` 建一个**极小的自有 overlay** 并指向它，就有机会既进入 dashboard
模式（拿到"不晃"）、又不被 SteamVR 主菜单挡住视野。

**探针**：`tools/xr_overlay_probe.py`（依赖 `pip install openvr`，纯 ctypes 绑定）。
它以 `VRApplication_Overlay` 身份接入，**不会**抢占 Isaac 的 scene app 身份。
模式：`dashboard`（当前主攻）、`none`（基线对照）、`tiny`/`panel`/`blink`（已否决，留档复核）。

**必须两个终端，Isaac 先起** —— 探针只提供一个 overlay、**不提供场景画面**：

```bash
# 终端 A：先起 Isaac，等头显里确实看到画面。注意要带 --teleop_device motion_controllers,
#         只给 --xr 不会建立视角锚定（anchor 由 OpenXRDevice 装配）。
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl \
UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
    --action_source sonic_dds --device cpu --teleop_device motion_controllers --xr

# 终端 B：确认 Isaac 画面已在头显里之后
python tools/xr_overlay_probe.py --mode dashboard                            # 默认 3cm overlay
python tools/xr_overlay_probe.py --mode dashboard --width 0.01 --alpha 0.15  # 更不挡
python tools/xr_overlay_probe.py --mode none                                 # 基线对照
```

启动时会打印 `✅ scene app pid = ...`；没有 scene app 会**直接拒绝测量**。
若有上一轮忘了退出的探针实例，也会列出来提醒（它们各自贴着一个 overlay，会污染判读）。

### 4.2.1 ✅ 首次实证：dashboard 开着确实不晃（2026-07-30）

跑了一次 `--mode dashboard`（进程随后就退出了），用户随即报告：

> "我看到的画面不晃动了，现在画面有 Isaac、steamvr dashboard、3cm overlay"

三点确认与一个新发现：

1. ✅ **不晃了** —— 这是第二次独立确认 dashboard 模式有效（第一次是 §1 手动开菜单）。
   而且此时 Isaac 画面**正常显示**、机器人照常动，不是定格。
2. ⚠️ **SteamVR 主菜单仍在显示** —— `showDashboard(自有 key)` 没能只显示我们那个小
   overlay。**遮挡问题未解决**，这是这条路能否变成正式方案的主要障碍。
3. ℹ️ 用户看到的 3cm overlay 是他自己那个 `--mode tiny` 实例（tiny 默认宽度正好 0.03m），
   与 dashboard 无关。
4. ⭐ **dashboard 状态在探针进程退出后仍然保持** —— 探针死了、它的 dashboard overlay 被
   destroy 了，dashboard 却还开着、画面还不晃。**意味着"打开一次"就够，不需要常驻进程**。
   代价是回落到显示 SteamVR 主菜单（因为自有 overlay 已经没了）。

同期采到的读数（`dash=ON`，Isaac 为 scene app）：

```
comp_gpu = 0.007–0.010ms   ← 与直通基线一样 ⇒ 这个字段没有区分力（见 §4.1.1 的撤回说明）
comp_cpu = 0.48–0.72ms
app_gpu / interval 时有时无（0.000 / 0.00 与 0.029 / 28–34ms 交替）
    ⇒ dashboard 模式下应用帧提交变得断续，但画面仍不晃
      —— 反过来印证 compositor 在自己按 vsync 出帧
```

**下一步要回答的就是遮挡**：让探针**持续运行**（我那次跑得太短就退出了），看它的 dashboard
overlay 保持活动时，主菜单会不会让位给那个小 overlay。

**判读三件事**（前两件只能戴头显看）：

1. **Isaac 画面还晃不晃** —— 唯一的成败判据；
2. **视野被挡多少** —— 我们的小 overlay 加上 SteamVR 自己的工具栏/背景；
3. `dash` 列是否为 `ON` —— 确认 dashboard 真的开着（⚠️ 不要看 comp_gpu，它无区分力）。

**已知代价与未决项**（决定它能不能变成正式方案）：

| 项 | 状态 |
|---|---|
| 消除 judder | ✅ **已实证两次**（§1 手动开菜单、§4.2.1 探针） |
| SteamVR 主菜单遮挡 | ⚠️ **当前主要障碍**：`showDashboard(自有 key)` 后主菜单仍在显示。待测"探针持续运行时主菜单是否让位" |
| 输入焦点被 dashboard 抢 | ⚠️ 右手柄 B 键 recenter 大概率失效。机器人控制不受影响（100% 来自 SONIC DDS），recenter 可改键盘/DDS 触发 |
| Isaac 画面是否仍为全视野立体 | ⏳ 待确认（会不会被降级成一个面板）。已知它**正常显示且机器人照常动** |
| 应用帧提交变断续 | ℹ️ 已观察到（`interval` 时有时无），但画面不晃 —— compositor 在自己按 vsync 出帧 |
| compositor 每帧渲染的额外开销 | ⏳ 待测。20ms 预算本来就紧，conveyor 场景 S≈0。⚠️ comp_gpu 量不出来（无区分力） |
| OpenXR session 变 `VISIBLE_UNFOCUSED` 的副作用 | 暂无异常（机器人照常走动） |
| 延迟 | ⚠️ **治不了**。它只补 PC 内那一段，编码 + 网络 + 解码的几十毫秒仍在（§7 第 3 条） |
| 是否需要常驻进程 | ✅ 不需要 —— dashboard 状态在进程退出后保持（§4.2.1） |

**这是 hack，不是正解。** 正解仍是 CloudXR（§6）或 NOLO 头显端 ATW（§7）——
但如果它挡得不厉害，就是当前链路上立刻可用的止痛药。

## 6. CloudXR 通路操作手册

### 为什么它治得了

| | SteamVR + NOLO XrLink | CloudXR |
|---|---|---|
| 重投影位置 | 无（PC 侧被 direct-mode 旁路，头显端不做） | **头显端 WebXR 客户端**（深度重投影网格 64×64） |
| 应用 36fps 时 | 旧帧原样重发，头姿过期 → 粘跳 | 客户端按最新头姿 warp 后再显示 |
| PC 侧可开关 | `enableLinuxVulkanAsync` 已试，无效 | 不需要 |

⚠️ **换 CloudXR 买的是重投影（治卡顿），不是帧率**。实测两条通路的渲染成本同一数量级，
应用帧率不会因此变高。

### 启动

runtime 必须先在跑（它是独立常驻进程，与本工程无关）：

```bash
python -m isaacteleop.cloudxr --accept-eula --host-client
# 用 GR00T 的 .venv_teleop 那份（1.3.132rc1）；--host-client 让 WebXR 客户端页面
# 从本机自托管，免得依赖头显能否访问外网
```

然后正常启动仿真，只需加 `--xr_runtime cloudxr`：

```bash
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
  --action_source sonic_dds --device cpu --teleop_device motion_controllers \
  --xr_runtime cloudxr
```

不需要手工 `source cloudxr.env`——`--xr_runtime cloudxr` 会自己读
`~/.cloudxr/run/cloudxr.env` 并注入环境，同时补上 kit 设置。**runtime 没在跑时它会直接
报错退出，不会静默降级**（静默跑回 SteamVR 是这条路最容易浪费半天的失败模式）。

头显端：浏览器打开 `https://<PC的IP>:48322/client/` → 先点页面里的证书链接、
「高级 → 继续前往」接受自签证书 → 回来点 CONNECT。**头显端不需要装任何 App。**

### 实现细节（改代码时要知道）

- 两个条件缺一不可：① 环境变量 `XR_RUNTIME_JSON` + `NV_CXR_RUNTIME_DIR`
  （后者用于定位 `ipc_cloudxr` unix socket）；② kit 设置
  `xr/system/openxr/runtime=custom` + `activeRuntimeJSON`。
  只设环境变量不够——`isaaclab.python.xr.openxr.kit` 里硬写了 `runtime="system"`，
  而命令行 `--/` 设置优先级更高。
- ⚠️ **绝不能写 `runtime=cloudxr`**：那个值指向 Isaac Sim **内置的 CloudXR 5.0.0** 并会自己
  拉起内置 service，与外部 6.2.0 抢同一套 IPC 与 49100/48322 端口。
- ⚠️ `--xr_runtime` 的参数定义必须放在 `AppLauncher.add_app_launcher_args()` **之后**，
  否则 argparse 会把 `--xr` 当成它的缩写，把所有现有 `--xr` 命令行打断。
  `tests/test_xr_runtime_cli.py` 锁死了这个行为。

### 判读口径（与 SteamVR 不同）

CloudXR **不导出** `presents`/`reprojected` 计数器，拿不到与 vrcompositor 同口径的对照。
可用的是 `~/.cloudxr/logs/cxr_server.*.log` 里的 report 块：

| 指标 | 含义 |
|---|---|
| `DevicePoseInterval` | 头显位姿上报间隔（实测 11.1ms = 90Hz） |
| `PredictEndToNextPredict` | 应用出帧间隔（这才是"应用帧率"） |
| `GpuEndToEncodeEnd` | NVENC 编码耗时（实测 11.3–11.9ms） |
| `LayerCommitToGpuEnd` | 渲染提交到 GPU 完成 |

**不要再用主循环 Hz 冒充头显帧率**——它们是四个不同的口径。

### 已知约束

- 本机 RTX 4070 Ti **不在 CloudXR 支持白名单**（日志会警告），目前能跑但 NVIDIA 不保证。
- 走 CloudXR 就完全不经过 SteamVR，NOLO driver 的一切能力同时失效（含全身追踪链路）。
  本工程 `retargeters=[]` 只用视角锚定 + B 键 recenter，不受影响。
- 控制器以 `bytedance/pico4_controller` 注册（不是 SteamVR 的 profile），B 键 recenter
  绑定可能失效；绑定失败只打 warning 不致命，视角锚定照常。
- SONIC 硬门槛（`fresh_physx`≥48Hz、`lowcmd_changes`≥48Hz、`timeouts`=0）在 CloudXR 下
  尚未验证，NVENC 编码是全新变量，20ms 预算账本需重测。

## 7. 为什么 NOLO 那部分代码 PC 侧改不了

用户问过"能否直接实现 NOLO 那部分代码"。答案是**不能**，三条独立理由：

1. **不可为**：`driver_nolo.so` / `libnolo-link-encoder.so` / `libTouPing.so` 全是闭源二进制，
   只有 `.symtab`、无任何 `.debug_*` 段——够逆向读结构，无法重建。
2. **物理上做不到**：驱动里 SPIR-V magic 出现 **0 次**，`FrameRender::RenderFrame` 只有
   `vkCmdPipelineBarrier` + `vkCmdCopyImageToBuffer`。**它根本没有图形管线**，
   做 warp 要从零写一整套渲染管线再注入闭源二进制。
3. **位置根本就错**：NOLO 是 direct-mode 驱动，compositor 之后 PC 侧再无 warp 阶段；
   而 ATW 必须用「扫描输出时刻」的最新头姿做，PC 侧在编码前 warp 只能补 PC 内几毫秒，
   补不了编码（11.3–11.9ms）+ 网络 + 解码 + 显示这几十毫秒。

> ⚠️ **这三条说的是"NOLO 驱动内做不到"，不要外推成"PC 侧做不到任何姿态校正"。**
> §1 的实证表明 SteamVR compositor 在 dashboard 模式下就在做每帧重新变换。区别在于：
> compositor 在 NOLO 驱动**之前**、且它有完整的图形管线。这条差别正是 §4.1 的立足点。
>
> 但要注意第 3 条对 §4.1 同样成立：即使逼出了 compositor 的每帧重新变换，它补的仍只是
> **PC 内**那一段。编码 + 网络 + 解码那几十毫秒仍然只有头显端 ATW 能补。所以 §4.1 若成功，
> 预期是"judder 消除、但延迟仍在"（延迟表现为均匀的跟手慢，而非粘跳）。

**能做的是向 NOLO 提需求**，且论据很硬——协议已经够了，只差客户端实现：

- 每帧视频**已经携带**渲染时的头姿四元数
  （`SubmitLayer` → `HmdMatrix_MatToQuat` → `FrameEncoder::CopyToStaging` → `SendVideo`）；
- 已有 `NOLO_PACKET_TYPE_TIME_SYNC` 时钟同步；
- 已有 9236 端口的高频 tracking 上报。

三要素齐全，客户端做 ATW 不需要改协议、也不需要改 PC 侧任何代码。

顺带可报两个 PC 侧缺陷（均已逆向坐实）：

1. `NHmdServerDriver::ReportData` 把 `vecAngularVelocity` 显式清零、`vecVelocity`/
   `vecAcceleration` 不写、`vecPosition` 写死 `{0.0, 0.6, -0.05}`，同时 `poseTimeOffset`
   硬编码 `-1.5/refreshRate`（72Hz → −20.8ms）。等于告诉 OpenVR「位姿过期 20.8ms」
   却又让它没法外推，**SteamVR 自带的位姿预测被废掉**。
2. 未重载 `IVRDriverDirectModeComponent::PostPresent` / `GetFrameTiming`（`nm` 显示是
   `openvr_driver.h` 的 weak 空实现），compositor 不受节流自由跑——实测 `FrameEncoder`
   以 104–109fps 编码发送内容重复的帧，而面板只有 72Hz。

另注：`bigroomconfig.json` 里 `secondsFromVsyncToPhotons` / `steamVRDisplayFPS` /
`frameQueueSize` / `useKeyedMutex` 等一大批键是**死键**，driver 实际只读 15 个键，
其中没有任何预测/重投影/延迟补偿开关——**别再翻配置找隐藏旋钮**。

## 8. 链路全图：从本工程到 NOLO 头显

排查时先认清"哪一段是我们的代码"——五段里只有第 1 段和第 5 段能改：

| 段 | 组件 | 位置 | 可改性 |
|---|---|---|---|
| 1 | 本工程：开 XR、选 runtime | `sim_main.py` | ✅ 全部 |
| 2 | kit XR 扩展：持 OpenXR session，提交 **projection layer** | `isaacsim/extscache/omni.kit.xr.system.openxr-107.3.109.../` | 只能配设置 |
| 3 | SteamVR OpenXR runtime → `vrcompositor` 合成 | 闭源 | ❌ |
| 4 | NOLO direct-mode 驱动：拷贝 → NVENC → FEC/UDP | `~/nolo_driver_deploy/bin/linux64/` | ❌ 闭源二进制 |
| 5 | 推流参数 | `~/nolo_driver_deploy/{XrLinkConfig,bigroomconfig}.json` | ✅ 但多为死键 |

**第 1 段实际只做三件事**（本工程里没有任何推流/编码代码）：

- `sim_main.py:463-471`：`--teleop_device motion_controllers` 强制 `args_cli.xr = True`
- `sim_main.py:624-721`：`--xr_runtime` 决定接哪个 runtime（写 `XR_RUNTIME_JSON` +
  kit 的 `/persistent/xr/system/openxr/{runtime,activeRuntimeJSON}`）
- `sim_main.py:605-620`：`--disable_xr_frame_cap` 注入 kit quirk

**第 4 段内部结构**（逆向所得）：`driver_nolo.so` 注册为 `IVRDriverDirectModeComponent_008`；
`FrameRender::RenderFrame` 只有 `vkCmdPipelineBarrier` + `vkCmdCopyImageToBuffer`；
`SubmitLayer` → `HmdMatrix_MatToQuat` → `FrameEncoder::CopyToStaging(HmdQuaternionf_t)` →
`libnolo-link-encoder.so`（NVENC，11.3–11.9ms/帧）→ `Listener::SendVideo` → `FECSend` → UDP 9936。
注册入口是 `~/.config/openvr/openvrpaths.vrpath` 的 `external_drivers`。
反向链路：头显 → 9236 端口高频 tracking → `NHmdServerDriver::ReportData`。

**第 5 段现状**：`XrLinkConfig.json` 的 `FPS=72`、`renderWidth/Height=1920`、`Fov*=52`
（厂商原值 46，待 NOLO 确认）；`bigroomconfig.json` 的 `encodeFPS=72`、`encodeBitrateInMBits=120`、
`codec=1`。⚠️ 同文件里 `steamVRDisplayFPS` 至今写着 90 却不生效——它是死键（见 §7 末）。

## 5. 排查时容易混入的无关故障

2026-07-29 同期发生过两起 NVIDIA 驱动故障，症状（头显画面定格、帧率异常）与本文的卡顿
容易混淆，但**成因完全无关**：

1. 用户态库升级后未重启，内核模块版本不匹配 → Vulkan 掉 llvmpipe → `Hmd Not Found (108)`
2. 580.173.02 上 vrcompositor 启动约 5 秒即触发内核 `Xid 32` → `VK_ERROR_DEVICE_LOST`
   → 持续刷 `vkerror=-4` 直至崩溃

判别方法：`journalctl -k | grep -i xid` 有无 Xid；`grep -c vkerror ~/.steam/steam/logs/vrcompositor.txt`
是否非零。这两项干净才是在测真正的卡顿。

## 相关文件

- `sim_main.py`：`--late_render_interval` / `--late_render_repeat` / `--disable_xr_frame_cap` / `--xr_runtime`
- `tools/xr_overlay_probe.py`：§4.1 的 overlay 探针（`tests/test_xr_overlay_probe.py` 守其判读逻辑）
- `layeredcontrol/robot_control_system.py`：晚渲染与渲染计数实现
- `doc/sonic_g1_29dof_phase1_handoff_zh.md`：SONIC 锁步时序（改渲染节奏前必读）
