# Pico VR 控制链部署任务书（pipeline 双机器人）

> 本文是**从零把 Pico VR 控制链部署到位**的施工顺序 + 操作手册 + 判读手册。
> 架构盘点结论与设计取舍见权威文档 `doc/pipeline_dual_robot_host_viewer_zh.md` §8，
> 两份互补不重复。状态（2026-08-03，tag `pipeline-pico-pose-v1`）：头显实测
> **发车 + POSE 全身跟随已通**（操作者动、机器人跟动）；摇杆行走/急停⏳待补测。

## 0. 链路一图流

```
┌─ Pico 4 Ultra（192.168.50.178，无线 adb 常开）
│    GameLink app（包名 com.Nolo.CloudVR）
│    └─ 全身追踪 + 手柄按键/摇杆 JSON ──UDP──> Ubuntu:63901
│
└─ Ubuntu host（192.168.50.68，本机全链）
     pico_manager_thread_server.py --manager（GR00T 仓库 .venv_teleop）
       └─ ZMQ PUB :5556（pose/command/planner 三话题同端口，前缀区分）
     deploy#1 --input-type zmq_manager（其余同 keyboard 版 isaac profile）
       └─ DDS rt/* 锁步 ↔ host sim（Isaac 段零改动）
     deploy#2 --input-type keyboard（robot_2 调试通道，不变）
     win 侧 AR viewer（观看链，与控制链完全并行独立）
```

要点：**控制链是头显→Ubuntu 直连 UDP，不经过 Windows**；win 上的 AR viewer
只是观看链。manager 走 `XROBO_TRANSPORT=udp`，**不需要安装 XRoboToolkit
PC Service**（.deb 留在 GR00T 仓库根，sdk 模式才用）。

## 1. 前置条件（新机器逐项核对）

| 项 | 要求 | 本机现状 |
|---|---|---|
| GR00T 仓库 | `/home/nolo/GR00T-WholeBodyControl`，deploy isaac profile 可用 | ✅ |
| GR00T 版本 | `feat/isaac-state-sync` **≥ `6783bb8`**（14f8bf1 误删的 666 个 gear_sonic 文件已全量恢复；旧检出 manager 收数据/进 POSE 必崩） | ✅ |
| `.venv_teleop` | Python 3.10，由 `install_scripts/install_pico.sh` 创建（uv）；`import xrobotoolkit_sdk, zmq, msgpack` 通过 | ✅ 实测通过 |
| 防火墙 | 入站 UDP 63901 放行（头显→本机） | ✅ ufw 不活动 |
| Pico 侧 app | GameLink（`com.Nolo.CloudVR`）已装 | ✅ |
| Pico 侧配置 | `XrLinkConfig.json` 的 `wholeBodyTracking`：`serverHost`=Ubuntu IP、`serverPort`=63901、`sendControllers`=true | ✅ 已指向 192.168.50.68 |
| sim 侧 | 本工程 host 模式可跑（见权威文档 §2） | ✅ |

- ⚠️ `.venv_teleop` **没装** sim 依赖（tyro/mujoco/onnxruntime），跑不了
  `run_sim_loop`——只跑 manager 足够，别顺手往里装东西。
- 头显配置改法（**免重打包**，app 读的是 files/ 下副本，改完重启 app 即生效）：

```bash
adb connect 192.168.50.178:5555
adb shell cat /sdcard/Android/data/com.Nolo.CloudVR/files/XrLinkConfig.json
# 编辑 wholeBodyTracking.serverHost / serverPort 后推回，重启 GameLink app
```

## 2. 启动（Ubuntu 侧）

一键（推荐，日志落 `/tmp/pipeline_pico/`，`PIPELINE_LOG_DIR` 可覆盖）：

```bash
bash tools/pipeline_pico_bringup.sh
# 换机器时路径变量与命令同一行传入（分行裸赋值传不进去）：
#   PIPELINE_SIM_DIR=<仿真工程> GR00T_WBC_ROOT=<GR00T> PIPELINE_SIM_PY=<python> bash tools/...
# 纯 ssh/无桌面会话加 PIPELINE_HEADLESS=1（否则 kit 拿不到 X 会在 RTX 插件初始化段错误，
# 脚本有预检直接报错）；GUI 形态 DISPLAY 透传，PIPELINE_DISPLAY 可强制。
```

脚本做的事（手动分步照此复刻）：硬清残余 → host sim（HOST_MODE，CRC 两刀）→
**双 deploy 并行**（#1 `--input-type zmq_manager`、#2 keyboard，错峰 20s）→
manager（`XROBO_TRANSPORT=udp PYTHONUNBUFFERED=1 .venv_teleop/bin/python
gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose --port 5556`）→
channel#2 发 `]` 发车+预激活 planner。

启动顺序不敏感的原因（改脚本前要懂）：manager 拿到**第一帧头显数据才 bind 5556**；
deploy 的 SUB 是 connect 语义会一直重试；command 话题有 **1Hz keepalive** 兜
slow-joiner——所以 manager 晚起、deploy 重启都能自动接上。

两条必须遵守：

- **`--no_auto_pose` 必带**：默认 auto_pose 会在头显数据一到就直接进 POSE 全身
  跟随（机器人立刻开始模仿身体），显式按键发车才可控。
- **manager 必须 `PYTHONUNBUFFERED=1`**：否则 stdout 全在缓冲里，日志文件恒空，
  判读全瞎（踩过）。

## 3. 操作手册（头显侧）

1. 戴上头显、手柄唤醒，启动 GameLink；
2. 看 Ubuntu 侧 `pico_manager.log`：不再刷 `waiting for body data...` = 数据到；
3. **四键同按 A+B+X+Y** = 发车，进 **PLANNER**（摇杆行走）——
   同按瞬间的身体姿态被用作 VR 追踪校准零位，**站直了再按**；
4. 摇杆：**左摇杆=行走方向，右摇杆=转向**；**A+B 升档**（SLOW_WALK→WALK→RUN），
   **X+Y 降档**；左摇杆推杆量映射速度（WALK 档速度固定）；
5. **A+X** 切 **POSE 全身跟随**；**左摇杆按下**切 VR_3PT（上身 VR 三点+行走保持）；
6. 再按 **A+B+X+Y = 急停**：manager 发 stop 后**自行退出**，重新发车要在
   Ubuntu 重启 manager（bringup 脚本或单起 manager 命令）。

manager 侧对应日志：`[Manager] Buttons: A=1 B=1 ...`（每次按键变化都打，
组合"没反应"时先看哪个键没到 1——手柄休眠是常见原因）→
`StreamMode switch: OFF -> PLANNER`；sim 侧 channel#1 出现
`[sonic_dds] CONTROL marker received` = 发车成功。

## 4. 判读与故障排查

| 现象 | 判读 |
|---|---|
| manager 刷 `waiting for body data...` | 头显数据没到：GameLink 没起/没戴（追踪不跑）/serverHost 不对/不同网段 |
| manager 日志文件恒空 | 忘了 `PYTHONUNBUFFERED=1`（进程其实活着） |
| 按 A+B+X+Y 无反应 | 看 `[Manager] Buttons:` 哪个键恒 0——手柄休眠先动一下摇杆唤醒；四键要真同帧按下 |
| sim 一直 `STARTUP HOLD`（channel#1） | 正常态=还没发车；发车后仍 HOLD 才是问题（查 manager 是否进了模式、deploy#1 是否连上 5556） |
| 未发车时担心锁步 | 不用：deploy Init 后 ack 照常回，`physics_steps` 照涨、`sync_waits` 不涨（已实测） |
| manager 收到首帧数据即崩 `ModuleNotFoundError`（robot_model/teleop） | GR00T 检出太旧：14f8bf1 同步删了源文件——`git pull` 到 ≥`6783bb8` |
| 进 POSE 崩 `FileNotFoundError: .../human_joints_info.pkl` | 同上，数据文件也在被删清单里——`git pull` 到 ≥`6783bb8` |
| PLANNER 里行走退化成原地踉跄 | §5.6 同款 planner 退化态——**PLANNER 子模式 bug 在环**（见下），重启 deploy 全链恢复。⚠️ manager 中途崩掉/退出也会经 planner 1s 超时触发这条路径 |
| FROZEN/VR_3PT 进入时 WARNING 回退零位 | 已知：feedback 通道现状为断（见下），step① 不依赖 |

已知限制（详见权威文档 §8）：

- **PLANNER 子模式共用 keyboard 的 kplanner 线程**——退化 bug 在环，且 planner
  话题 1s 超时自动回 IDLE（manager 20Hz 发送正常不会触发；manager 停发/卡顿会）。
  POSE 全身流子模式绕过 planner，无此问题。
- **feedback 通道断**：manager 订 ZMQ 5557/`g1_debug`，deploy#1 实际输出 UDP 且
  话题 `g1_1_debug`——只影响 PLANNER_FROZEN 上身目标与 VR_3PT 重校准（回退零位）。
  接通待验方案：deploy#1 加 `--output-type zmq --zmq-out-topic g1_debug`。
- **Isaac 整场景 reset 不通知输入链**：不清 manager/planner 缓冲（step③ 单独验）。

## 5. 双 Pico 扩展预案（step②，未实施）

隔离点两处，都在 udp 模式下才成立（**sdk/PC Service 模式无设备隔离**——pybind
回调写同一组全局变量，同机双 Pico 数据互相覆盖，不可用）：

| 层 | 实例 #1（robot_1） | 实例 #2（robot_2） |
|---|---|---|
| 头显 JSON `serverPort` | 63901 | **63902** |
| manager | `--port 5556`（默认 XROBO_UDP_PORT=63901） | `--port 5566` + `XROBO_UDP_PORT=63902` |
| deploy | 默认（SUB localhost:5556） | `G1_LOCAL_ROBOT_ID=2` + **`--zmq-port 5566`** |

⚠️ ZMQ 输入端口**不随 `G1_LOCAL_ROBOT_ID` 自动分流**（ID=2 只改输出侧 5567/rt/r2），
`--zmq-port` 必须显式给；command/planner 话题名硬编码，靠话题区分双实例走不通。

## 6. 与 AR 观看链的关系（step④，开放问题）

控制链（头显→Ubuntu:63901）与观看链（Ubuntu→win viewer→SteamVR AR→头显）
互相独立。值得注意：GameLink 这个 app 的**视频串流目标与全身追踪目标本来就是
两条独立配置**（前者在 dex 内硬编码、后者在 XrLinkConfig.json）——"同一台头显
边看 AR 边发追踪控制"在配置层面可能天然成立，⏳未实测，属 step④ 验证范围。
