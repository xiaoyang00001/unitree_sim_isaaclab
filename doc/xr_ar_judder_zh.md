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

1. **换用带客户端重投影的串流通路（CloudXR）** —— 已知 CloudXR 无此问题，其客户端自带 ATW。
   这是唯一能**根治**的方向。
2. **头显端 NOLO 客户端实现 ATW** —— 串流方案的重投影正确位置就在头显端
   （ALVR、Quest Link 皆如此）。PC 侧无源码不可为，需向 NOLO 提需求。
3. **提高并稳住应用帧率** —— 治标。无重投影时 36fps（均匀重复 2 次）是能达到的最舒服状态，
   优于 30fps 的交替抖动。conveyor 场景物理比主场景重约 4ms，是它掉出 36fps 的原因。

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
