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
4. **⭐ 借鉴 dashboard：逼 compositor 转入合成模式** —— 见 §4.1。基于 §1 的实证，
   这是目前唯一有希望"留在 SteamVR + NOLO 链上还不抖"的方向，且**若成立代价极小**。

### 4.1 ⭐ 待实测：逼 compositor 退出直通（探针已就绪）

**假设**：`Compositor Time GPU: 0.007ms`（证据 6）说明正常模式下 compositor 在**直通**——
把应用的 swapchain 图像原样转手给 direct-mode 驱动，连畸变都不做。而直通的前提是只有单一
layer。**只要存在任何一个可见 overlay，compositor 就必须自己合成，可能顺带就把应用画面
也按最新头姿每帧重新变换了**（正是 §1 里 dashboard 表现出来的行为）。

若假设成立，**代价近乎为零**：不牺牲立体、不损失视野、不改 Isaac 一行代码，
只需常驻一个几乎不可见的小 overlay。

**探针**：`tools/xr_overlay_probe.py`（依赖 `pip install openvr`，纯 ctypes 绑定）。
它以 `VRApplication_Overlay` 身份接入，**不会**抢占 Isaac 的 scene app 身份，
四种模式：

| mode | 作用 |
|---|---|
| `none` | 只做遥测不建 overlay —— 拿基线 |
| `tiny` | 3cm 的世界锚定小 overlay —— 测假设的零代价解 |
| `panel` | 1.2m 网格面板 —— 若主画面仍抖，看 overlay 自身稳不稳（决定要不要走"画面搬进 overlay"） |
| `blink` | 定时交替显示/隐藏 —— 戴着头显不动就能做 A/B |

**客观判据（不需要戴头显）**：探针每秒打印 `IVRCompositor::GetFrameTiming` 的
`m_flCompositorRenderGpuMs`。基线应贴近 0（对应日志里的 0.007ms）；
`tiny` 下若它**显著上升**（阈值 0.10ms），就客观证明 compositor 退出了直通。
退出时会打印结论行与均值/峰值。

**实验步骤**（需要 SteamVR 已由 NOLO Link / ALVR 拉起，Isaac 已 `--xr` 接入）：

```bash
python tools/xr_overlay_probe.py --mode none    # 1. 基线，看 comp_gpu ≈ 0.0x
python tools/xr_overlay_probe.py --mode tiny    # 2. comp_gpu 是否跳起来
# 3. 若 comp_gpu 跳起来了 → 戴头显转头，看 Isaac 画面还抖不抖
python tools/xr_overlay_probe.py --mode panel   # 4. 若主画面仍抖，看 overlay 自身稳不稳
```

**三种结果分别指向**：

| 结果 | 含义 | 下一步 |
|---|---|---|
| `tiny` 下 comp_gpu 跳起来 **且**主画面不抖 | ✅ 零代价解 | 做成常驻 overlay，接进 `sim_main.py` |
| comp_gpu 跳起来但主画面仍抖 | compositor 合成了但没重新变换 scene layer | 看 `panel`：overlay 稳 → 走下面的"画面搬进 overlay" |
| comp_gpu 全程贴近 0 | 仅存在 overlay 不足以退出直通 | 这条路死，收口到 CloudXR |

**退路：把画面搬进 overlay**（代价大，仅当上面第二种结果时才考虑）——
不再提交立体 projection layer，改由 `IVROverlay` 持有画面纹理、scene layer 提交纯黑。
代价：画面退化为平面/曲面（`VROverlayFlags_SideBySide` 能做立体，但只有平面近似、
无真视差与深度）；且同进程内 OpenXR 与 OpenVR 会话共存的可行性未验证。

**已否决的近亲写法**：通过 OpenXR 提交 `XrCompositionLayerQuad` —— 它仍然走应用 submit 路径，
而证据 3 表明该路径在没有新 submit 时只做原样重发；kit 也没暴露这个开关。

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
