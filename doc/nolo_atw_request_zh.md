# 致 NOLO：XrLink 串流画面卡顿问题——需求与缺陷报告

> 本文档可直接发给 NOLO 技术团队。所有结论均来自 2026-07-29 在 Linux + SteamVR + NOLO XrLink
> 环境下的实测与二进制逆向，证据随附。
>
> 一句话诉求：**请在头显端客户端实现 ATW（异步时间扭曲）。协议层面所需的数据你们已经全部传过去了，
> 不需要改协议，也不需要改 PC 侧任何代码。**

## 1. 问题现象

应用（NVIDIA Isaac Sim）通过 SteamVR + NOLO XrLink 串流到 NOLO 头显时，画面在头部转动时
"粘一下、跳一下"，长时间使用引起不适。

现象与应用帧率高低不完全相关：应用 30fps 时抖，稳定在 36fps 时同样抖。

一个关键旁证：**打开 SteamVR dashboard（系统菜单）时完全不抖。** 因为 dashboard 由
vrcompositor 自己以面板刷新率、逐帧使用最新头姿渲染，它不经过"应用帧被重复展示"这条路径。

## 2. 根因：整条链路没有任何重投影

| # | 证据 | 出处 |
|---|---|---|
| 1 | `Async support disabled by user setting or a direct mode driver` | `~/.steam/steam/logs/vrcompositor.txt`，每次启动必打印 |
| 2 | `Total.. 228920 presents. 0 dropped. **0 reprojected**` | 同上，会话结束统计。22 万帧，重投影 0 次 |
| 3 | `Timed out.641 total.... 2433 presents. 0 reprojected` | 应用没赶上刷新时，compositor **原样重发旧帧** |
| 4 | 设 `steamvr.vrsettings` 的 `"enableLinuxVulkanAsync": true` 后重启，日志**仍**打印证据 1 | 实测 |

证据 4 是关键的排除法：`user setting` 这一半已被排除，那么剩下的原因只有
`a direct mode driver`——**SteamVR 不对 direct-mode 驱动启用 PC 侧异步重投影**。

### 机制

面板 72Hz，每 13.9ms 必须出一帧。应用 36fps 时每个应用帧要展示 2 次。
**没有重投影 = 第 2 次展示时用的还是那一帧渲染时刻的头部姿态。**
用户的头在这 13.9ms 里一直在动，画面相对头部"粘住"，等新帧到来再"跳"回正确位置。

这也解释了为何 30fps 比 36fps 更难受：36fps 是均匀的"每帧重复 2 次"，
30fps 在 2 次与 3 次之间交替，抖动本身还带节奏变化。

## 3. 为什么必须由头显端实现（PC 侧做不到）

我们认真评估过在 PC 侧补 ATW 的可能，结论是**不可行**，三条独立理由：

1. **驱动没有图形管线。** 逆向 `driver_nolo.so` / `libnolo-link-encoder.so` / `libTouPing.so`：
   整包 SPIR-V magic 出现 **0 次**；`FrameRender::RenderFrame` 只有 `vkCmdPipelineBarrier` +
   `vkCmdCopyImageToBuffer`。驱动只做拷贝与编码，不具备任何渲染/warp 能力。
2. **架构上没有插入点。** NOLO 注册为 direct-mode 驱动
   （`IVRDriverDirectModeComponent_008`），在 vrcompositor 之后 PC 侧已无 warp 阶段。
3. **即使能做也补不了延迟。** ATW 必须使用「扫描输出时刻」的最新头姿。PC 侧在编码前做 warp，
   只能补偿 PC 内部的几毫秒，补不了 **编码（实测 11.3–11.9ms）+ 网络传输 + 解码 + 显示**
   这几十毫秒。

这也是业界通行做法：ALVR、Quest Link 等串流方案的重投影都在头显客户端完成。

## 4. 好消息：协议已经够了，只差客户端实现

ATW 需要三样东西，你们**已经全部具备**：

| ATW 所需 | 现状 | 证据 |
|---|---|---|
| 每帧画面对应的渲染头姿 | ✅ 已随视频帧传输 | `SubmitLayer` → `HmdMatrix_MatToQuat` → `FrameEncoder::CopyToStaging(vr::HmdQuaternionf_t)` → `Listener::SendVideo` → `FECSend` |
| PC 与头显的时钟同步 | ✅ 已有 | `NOLO_PACKET_TYPE_TIME_SYNC` |
| 头显端的高频最新头姿 | ✅ 本来就在头显端 | 9236 端口高频 tracking 上报链路 |

**因此：客户端在解码后、提交给 PICO 系统 compositor 之前，用「当前最新头姿」与「帧携带的渲染
头姿」之差做一次 reprojection 即可。不需要改协议，不需要改 PC 侧代码。**
（当前客户端 `Version=18`。）

## 5. 顺带报告两个 PC 侧驱动缺陷

这两条独立于 ATW，修起来都很便宜，但都只能你们改：

### 缺陷 A：位姿上报自相矛盾，废掉了 SteamVR 自带的预测

`NHmdServerDriver::ReportData` 中：

- `vecAngularVelocity` 被**显式清零**；
- `vecVelocity` / `vecAcceleration` **不写**；
- `vecPosition` **写死** `{0.0, 0.6, -0.05}`；
- 同时 `poseTimeOffset` 硬编码为 `-1.5 / refreshRate`（72Hz → **−20.8ms**）。

后果：等于告诉 OpenVR「这个位姿已经过期 20.8ms，请你外推」，却又把外推所需的速度/角速度
全部清零或不填——**SteamVR 内建的位姿预测被完全废掉**。

> 建议同时复核：`vecPosition` 写死意味着 HMD 位置不随用户走动变化（我们未穷举所有写
> `0x98(%rdi)` 的位置，请以你们源码为准）。

### 缺陷 B：未实现帧节流接口，compositor 自由跑

`IVRDriverDirectModeComponent::PostPresent` 与 `GetFrameTiming` 未被重载
（`nm` 显示仍是 `openvr_driver.h` 里的 weak 空实现）。

后果：编码侧不受 compositor 节流约束，可以跑到远超面板刷新率的速率去发送**内容重复**的帧。

实测（`~/nolo_driver_deploy/log/nolo-link-driver.log` 全量 11024 个 `FrameEncoder FPS` 采样）：

| 统计量 | 值 |
|---|---|
| 峰值 | **112.39 fps**（面板 72Hz，超出 56%） |
| 超过 100fps 的采样占比 | 1.32%（145 / 11024），集中在应用未提交/静止内容的窗口 |
| p90 / 中位 | 49.6 / 15.5 fps |

> 说明：**这不是稳态现象**，中位数很低（日志累积了大量空转时段）。我们据以判断的是
> **结构**而非平均值——`PostPresent` / `GetFrameTiming` 是 weak 空实现，意味着编码速率
> 没有任何来自 compositor 的节流约束，因此在内容静止时会自由跑到 112fps 发重复帧。
> 这是确定的 CPU/GPU/带宽浪费，也让帧节奏无从对齐。请以 `nm` 的符号证据为准，
> 上表仅为其外在表现的量化。

## 6. 一个待确认项：FOV 配置

`bigroomconfig.json` 中 `FovUp` / `FovDown` / `FovLeft` / `FovRight` 当前为 **52**，
而厂商原始配置为 **46**。请确认 Pico_A9410 面板的真实半角 FOV。

投影几何失配会产生类似"世界随头动而轻微缩放/畸变"的观感，**容易被误判为卡顿**，
建议一并核对。

## 7. 附：配置项现状说明（供你们文档参考）

我们曾尝试从配置中寻找预测/重投影开关，结论是没有：`driver_nolo.so` 实际只读取
**15 个键**，`bigroomconfig.json` 里的 `secondsFromVsyncToPhotons`、`steamVRDisplayFPS`、
`frameQueueSize`、`useKeyedMutex` 等一大批键是**死键**——它们只出现在「配置文件缺失时写出的
默认模板字符串」里，运行时并不读取。

建议在文档中明确哪些键真正生效，可以省下用户大量试错时间。

---

## 附录：复现环境

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 24.04，内核 7.0.0-28-generic |
| GPU / 驱动 | RTX 4070 Ti / NVIDIA 580.173.02 |
| SteamVR | 2.16.7 |
| 头显 | Pico 4 Ultra（面板 72Hz） |
| XrLinkConfig | `FPS=72`，`renderWidth/Height=1920` |
| 应用 | NVIDIA Isaac Sim 5.1（OpenXR，仅使用视角锚定） |
| 应用帧率 | 30–36fps（低于面板刷新率，正是需要 ATW 的工况） |
