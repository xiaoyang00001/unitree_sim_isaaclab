# AR 画面卡顿（judder）根因与处置

> 2026-07-29 调查记录。**结论先行：卡顿的根因是 SteamVR + NOLO XrLink 这条链路上
> 全程没有任何重投影（reprojection / ATW）。这不是帧率问题，也不是本工程的渲染优化
> 失效。在无重投影的前提下，应用帧率只要低于头显面板刷新率就必然卡顿，帧率与帧节奏
> 优化只能减轻，不能消除。**

## 1. 症状

`--xr --teleop_device motion_controllers` 下，头显内画面"粘一下、跳一下"，头转动时尤其明显。
表现与应用帧率高低不完全相关：30fps 抖，36fps 也抖。

**极具误导性的现象：打开 SteamVR dashboard（菜单）时不抖。**
这一度让人误判"某个配置能修好它"。真相是 dashboard 由 vrcompositor 自己以面板刷新率
逐帧、用当前最新头姿渲染，它永远不抖；它盖在应用画面之上时掩盖了应用层的 judder。

> ⚠️ 历史教训：2026-07-28 记录的"72Hz 全链对齐 + 应用匀速 36fps = 流畅"结论，事后
> 复盘确认当时多半开着 dashboard 观察，**该结论已被推翻**。做 XR 主观评测时必须确认
> dashboard 已关闭，否则测的是 compositor 而不是你的应用。

## 2. 根因证据链

| # | 证据 | 出处 |
|---|---|---|
| 1 | `Async support disabled by user setting or a direct mode driver` | `~/.steam/steam/logs/vrcompositor.txt`，每次启动必打印 |
| 2 | `Total.. 228920 presents. 0 dropped. **0 reprojected**` | 同上，会话结束统计。跑了 22 万帧，重投影次数为 0 |
| 3 | `Timed out.641 total.... 2433 presents. 0 reprojected` | 同上。应用没赶上刷新时 compositor **原样重发旧帧**，不做任何姿态校正 |
| 4 | 设 `steamvr.vrsettings` 的 `"enableLinuxVulkanAsync": true` 后重启 SteamVR，日志**仍**打印证据 1 | 2026-07-29 15:30 实测 |
| 5 | NOLO 驱动日志中 ATW / timewarp / reprojection 零命中 | `~/nolo_driver_deploy/log/*.log` |

证据 4 是关键的排除法：`user setting` 这一半已被排除，那么打印这行的原因只剩
`a direct mode driver` —— **NOLO 是 direct-mode 串流驱动，SteamVR 不对它启用 PC 侧异步重投影**。

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

## 5. 排查时容易混入的无关故障

2026-07-29 同期发生过两起 NVIDIA 驱动故障，症状（头显画面定格、帧率异常）与本文的卡顿
容易混淆，但**成因完全无关**：

1. 用户态库升级后未重启，内核模块版本不匹配 → Vulkan 掉 llvmpipe → `Hmd Not Found (108)`
2. 580.173.02 上 vrcompositor 启动约 5 秒即触发内核 `Xid 32` → `VK_ERROR_DEVICE_LOST`
   → 持续刷 `vkerror=-4` 直至崩溃

判别方法：`journalctl -k | grep -i xid` 有无 Xid；`grep -c vkerror ~/.steam/steam/logs/vrcompositor.txt`
是否非零。这两项干净才是在测真正的卡顿。

## 相关文件

- `sim_main.py`：`--late_render_interval` / `--late_render_repeat` / `--disable_xr_frame_cap`
- `layeredcontrol/robot_control_system.py`：晚渲染与渲染计数实现
- `doc/sonic_g1_29dof_phase1_handoff_zh.md`：SONIC 锁步时序（改渲染节奏前必读）
