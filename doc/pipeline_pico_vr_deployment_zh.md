# Pico VR 控制链部署任务书（pipeline 多机器人）

> 本文是**从零把 Pico VR 控制链部署到位**的施工顺序 + 操作手册 + 判读手册。
> 架构盘点结论与设计取舍见权威文档 `doc/pipeline_dual_robot_host_viewer_zh.md` §8，
> 两份互补不重复。状态（2026-08-04）：单 Pico 发车 + POSE 全身跟随已通
>（2026-08-03，tag `pipeline-pico-pose-v1`）；**双 Pico 双操作者各控一台已实机
> 验证通过**（2026-08-04，用户实测确认，`PIPELINE_DUAL_PICO=1` 编排）。
> §5.2 四道验收中急停/端口隔离判据未逐项留痕，复验时可补记录。
> 自动倒地、Ubuntu F12 和 Pico 左 X 的统一 reset 说明见
> [Isaac/SONIC 场景复位说明](scene_reset_zh.md)。
>
> 2026-08-20 起生产 bringup 收敛为三台 SONIC 动力学机器人：本手册的 Pico 主链仍聚焦
> robot_1/2，robot_3 默认走独立 keyboard deploy；robot_4/5 恢复原始 visual-only
> standby。四/五路仍可显式启用，完整 DDS/端口和历史性能说明见
> [流水线多机器人 SONIC 控制](pipeline_five_robot_sonic_zh.md)。

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
     deploy#2 --input-type keyboard（默认）
       └─ 双 Pico模式改为 zmq_manager :5566 ← Pico#2 UDP :63902
     deploy#3 --input-type keyboard（生产默认）
       └─ DDS rt/r3/*（独立输入/调试端口；三路 ack AND）
     robot_4/5 visual-only standby
       └─ 显式 PIPELINE_SONIC_ROBOT_COUNT=4|5 时才增加 deploy 与 rt/r4..r5/*
     win 侧 AR viewer（观看链，与控制链完全并行独立）
```

要点：**控制链是头显→Ubuntu 直连 UDP，不经过 Windows**；win 上的 AR viewer
只是观看链。manager 走 `XROBO_TRANSPORT=udp`，**不需要安装 XRoboToolkit
PC Service**（.deb 留在 GR00T 仓库根，sdk 模式才用）。

## 1. 前置条件（新机器逐项核对）

| 项 | 要求 | 本机现状 |
|---|---|---|
| GR00T 仓库 | `<GR00T仓库>`，deploy isaac profile 可用 | ✅ |
| GR00T 版本 | 至少含恢复提交 `6783bb8`、HandCmd 参数提交 `0f4e0b4` 和定时校准修复 `122c947`；只含 `0f4e0b4` 时配置 100 Hz 实际约 84–85 Hz | ✅ |
| `.venv_teleop` | Python 3.10，由 `install_scripts/install_pico.sh` 创建（uv）；`import xrobotoolkit_sdk, zmq, msgpack` 通过 | ✅ 实测通过 |
| 防火墙 | 入站 UDP 63901 放行；双 Pico 还要放行 63902（头显→本机） | ✅ ufw 不活动 |
| Pico 侧 app | GameLink（`com.Nolo.CloudVR`）；使用内置配置强刷包时，#1/#2 必须安装各自的机器人专用 APK | #1 ✅；#2 ⏳待设备 |
| Pico 侧配置 | `XrLinkConfig.json` 的 `wholeBodyTracking`：`serverHost`=Ubuntu IP、#1 `serverPort`=63901、#2=63902、`sendControllers`=true | #1 ✅ 已指向 192.168.50.68；#2 ⏳待回读 |
| sim 侧 | 本工程 host 模式可跑（见权威文档 §2） | ✅ |

- ⚠️ `.venv_teleop` **没装** sim 依赖（tyro/mujoco/onnxruntime），跑不了
  `run_sim_loop`——只跑 manager 足够，别顺手往里装东西。

### 1.1 GameLink APK 安装与配置刷新语义

⚠️ 必须先判断 APK 类型，不能把“包内 asset”和“设备 files 副本”视为同一份配置：

- 原版及旧重打包版仅在目标文件不存在时，把 APK 内的
  `assets/XrLinkConfig.json` 复制到
  `/sdcard/Android/data/com.Nolo.CloudVR/files/XrLinkConfig.json`。`adb install -r`
  会保留 files 副本，所以换 APK 后仍可能继续使用旧 IP/端口；此类包要么先卸载再安装，
  要么安装后手工覆盖 files 副本并回读。
- 2026-08-04 起生成的机器人专用**强制刷新版**会在每次 native 启动前只删除旧
  `XrLinkConfig.json`，随后由 GameLink 原生流程从当前 APK asset 重新生成。因此同一签名
  的强刷版可用 `adb install -r` 在 robot#1/#2 之间切换，启动后会自动切到包内的
  63901/63902；手工 `adb push` 的修改会在下次启动时被覆盖。
- 原厂 APK 与本项目重签 APK 的证书不同，二者之间不能覆盖安装。首次换到重签包时，
  先备份现场 JSON，再 `adb uninstall com.Nolo.CloudVR`；卸载会清除应用数据和授权。

原厂/未知签名包换成机器人专用包的完整流程：

```bash
PICO_ADB=<Pico-IP>:5555
PACKAGE=com.Nolo.CloudVR
PICO_CONFIG=/sdcard/Android/data/$PACKAGE/files/XrLinkConfig.json
adb connect "$PICO_ADB"
adb -s "$PICO_ADB" pull "$PICO_CONFIG" XrLinkConfig.before.json || true
adb -s "$PICO_ADB" uninstall "$PACKAGE"
adb -s "$PICO_ADB" install <robot-specific-GameLink.apk>
adb -s "$PICO_ADB" shell pm grant "$PACKAGE" android.permission.RECORD_AUDIO
adb -s "$PICO_ADB" shell monkey -p "$PACKAGE" \
  -c android.intent.category.LAUNCHER 1
adb -s "$PICO_ADB" shell cat "$PICO_CONFIG"  # 必须核对 host 与 63901/63902
```

已安装同一套重签强刷版时，换机器人专用包可把 `uninstall` + `install` 两行替换为
`adb -s "$PICO_ADB" install -r <robot-specific-GameLink.apk>`；仍须启动并回读，不能只凭
APK 文件名判断生效。

- 旧版/通用 GameLink 临时配置改法（**仅适用于不会在启动时强制刷新 JSON 的包**；
  app 运行时读的是 files/ 下副本）：

```bash
PICO_ADB=192.168.50.178:5555
PICO_CONFIG=/sdcard/Android/data/com.Nolo.CloudVR/files/XrLinkConfig.json
adb connect "$PICO_ADB"
adb -s "$PICO_ADB" pull "$PICO_CONFIG" XrLinkConfig.before.json
cp XrLinkConfig.before.json XrLinkConfig.edit.json
# 用编辑器修改 XrLinkConfig.edit.json 的 wholeBodyTracking 后：
adb -s "$PICO_ADB" push XrLinkConfig.edit.json "$PICO_CONFIG"
adb -s "$PICO_ADB" shell am force-stop com.Nolo.CloudVR
adb -s "$PICO_ADB" shell monkey -p com.Nolo.CloudVR \
  -c android.intent.category.LAUNCHER 1
adb -s "$PICO_ADB" shell cat "$PICO_CONFIG"  # 必须回读核对后才继续
```

强制刷新版若要永久换 IP/端口，必须修改 APK 内置 asset 并重新打包；直接 push 到设备的
JSON 只会保留到下一次启动。

## 2. 启动（Ubuntu 侧）

一键（推荐，日志落 `/tmp/pipeline_pico/`，`PIPELINE_LOG_DIR` 可覆盖）：

```bash
# 生产默认三机：单 Pico 控 robot_1 + keyboard 控 robot_2/3
bash tools/pipeline_pico_bringup.sh

# 双 Pico：两套 manager 分别控制 robot_1/robot_2
PIPELINE_DUAL_PICO=1 bash tools/pipeline_pico_bringup.sh

# 回退历史双机：
PIPELINE_SONIC_ROBOT_COUNT=2 bash tools/pipeline_pico_bringup.sh

# 显式四/五机联调（robot_4/5 才从 visual-only standby 升级为 SONIC 真身）
PIPELINE_SONIC_ROBOT_COUNT=5 bash tools/pipeline_pico_bringup.sh

# HandCmd 兼容性回滚：必须重启完整 bringup，不是热更新
PIPELINE_ISAAC_HANDCMD_HZ=500 bash tools/pipeline_pico_bringup.sh

# 双 Pico 回滚时保留原来的双 Pico 变量
PIPELINE_DUAL_PICO=1 PIPELINE_ISAAC_HANDCMD_HZ=500 \
bash tools/pipeline_pico_bringup.sh

# 新 checkout / GPU / Isaac Lab 升级后的一次性执行器 tensor 契约验收
PIPELINE_SONIC_VALIDATE_ACTUATORS=1 bash tools/pipeline_pico_bringup.sh

# 单组执行器兼容性回滚：必须完整重启，恢复原始 6 组
PIPELINE_SONIC_MERGE_ACTUATORS=0 PIPELINE_SONIC_VALIDATE_ACTUATORS=0 \
bash tools/pipeline_pico_bringup.sh

# 换机器时路径变量与命令同一行传入（分行裸赋值传不进去）：
#   PIPELINE_SIM_DIR=<仿真工程> GR00T_WBC_ROOT=<GR00T> PIPELINE_SIM_PY=<python> bash tools/...
# 纯 ssh/无桌面会话加 PIPELINE_HEADLESS=1（否则 kit 拿不到 X 会在 RTX 插件初始化段错误，
# 脚本有预检直接报错）；GUI 形态 DISPLAY 透传，PIPELINE_DISPLAY 可强制。
```

一键脚本默认对所有 Isaac deploy 显式传 `--isaac-handcmd-hz 100`。该参数只降低左右手
Dex3 HandCmd；LowCmd、锁步 ACK、Control 和 Planner 仍保持 500 Hz，外仓 standalone
Isaac 默认及非 Isaac/实机路径也仍为 500 Hz。`PIPELINE_ISAAC_HANDCMD_HZ` 只在启动时
读取；回滚到 500 Hz 必须重启完整 bringup。

真身执行器的生产一键默认是 `PIPELINE_SONIC_MERGE_ACTUATORS=1`、
`PIPELINE_SONIC_VALIDATE_ACTUATORS=0`；核心直接启动的两个 `ISAACLAB_*` 开关仍默认关闭。
`validate=1` 只用于首次/升级后单次启动的 GPU→CPU tensor 契约验收，核对 hash 后下一次
完整 bringup 恢复 `0`。`merge` 不是热开关；回滚必须按上面的 `merge=0 validate=0` 命令
完整重启，不能只给正在运行的 shell 重新赋值。两个变量只接受字面 `0/1`，非法值会在
脚本停止旧进程前失败。

启动后可核对全部 deploy 日志，确认没有某一路静默落回 500 Hz：

```bash
rg -n "Dex3 HandCmd Rate: 100 Hz|Isaac Dex3 HandCmd publish rate: 100 Hz" \
  /tmp/pipeline_pico/deploy_r*.log
```

脚本做的事（手动分步照此复刻）：硬清残余 → host sim（HOST_MODE，CRC 两刀）→
**全部 deploy 并行存活**（错峰启动）→ manager UDP receiver 就绪 → 全部 deploy Init →
STARTUP HOLD。生产默认模式的 deploy#2/3 是 keyboard，脚本继续给对应 channel 发 `]` 并预激活
planner；双 Pico 模式的 deploy#2 接 `zmq_manager:5566`，另起
`pico_manager_r2.log`，两路都只等各自操作者 A+B+X+Y，脚本不代按。

默认数量的选择来自 `PIPELINE_SONIC_ROBOT_COUNT=3`；显式设为 `4` 或 `5` 时，脚本才会
为 robot_4/5 创建真身、DDS 与额外 keyboard deploy。数量切换不是热更新，必须完整重启
bringup。空闲纸箱 velocity 写入候选仅提升 1.057%，未达到 5% 门槛，生产未启用；核心
`ISAACLAB_CONVEYOR_SKIP_IDLE_VELOCITY_WRITES` 默认仍为 `0`；
`ISAACLAB_CONVEYOR_CONTACT_HISTORY_LENGTH` 默认仍为 `4`。

启动顺序不敏感的原因（改脚本前要懂）：manager 拿到**第一帧头显数据才 bind 对应
ZMQ PUB 5556/5566**；deploy 的 SUB 是 connect 语义会一直重试；command 话题有 **1Hz keepalive** 兜
slow-joiner——所以 manager 晚起、deploy 重启都能自动接上。

两条必须遵守：

- **`--no_auto_pose` 必带**：默认 auto_pose 会在头显数据一到就直接进 POSE 全身
  跟随（机器人立刻开始模仿身体），显式按键发车才可控。
- **manager 必须 `PYTHONUNBUFFERED=1`**：否则 stdout 全在缓冲里，日志文件恒空，
  判读全瞎（踩过）。

## 3. 操作手册（头显侧）

1. 戴上头显、手柄唤醒，启动 GameLink；
2. 看 Ubuntu 侧对应日志（#1=`pico_manager.log`、#2=`pico_manager_r2.log`）：
   不再刷 `waiting for body data...` = 该路数据到；
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
- **Isaac 整场景 reset 不通知输入链**：不清 manager/planner 缓冲。
  **step③ 首个实测数据点（2026-08-04，另机，用户实测）：倒地自动复位后可继续
  操作**——机制假说：VR 链 manager 以 20Hz 连续发 planner 流，可能天然避开
  keyboard 链退化的 WALK→IDLE→WALK 续接路径（待证）。要把它变成实锤还差两个
  自查：①reset 后看 deploy 终端 `Planner Model` 耗时应 >90ms（36-79ms=退化态）；
  ②POSE 子模式下的 reset 表现与手动 DDS reset（`rt/reset_pose/cmd`）未测。

## 5. 双 Pico 施工任务书（step②，✅2026-08-04 双操作者双控实机验证通过）

> 🗺️ 可视化拓扑/施工图：`doc/pipeline_dual_pico_plan_zh.html`（自包含单文件，
> 浏览器直接打开；与本节内容同源，双泳道端口一图流）。

> 结论先行：**Isaac 与 deploy 侧零代码改动**，全部工作 = 头显#2 配置 + 第二套
> manager 启动参数 + bringup 编排。三个承重旋钮已对代码核验（非猜测）：
> `XROBO_UDP_PORT` 在 `decoupled_wbc/control/teleop/device/pico/xr_client.py`
> 读取（⚠️ 不在 gear_sonic 里，grep 时别漏目录）；deploy.sh `--zmq-port` 是
> 一等参数（:588）；manager `--port` 即 ZMQ PUB bind 口。

隔离矩阵（udp 模式专属——**sdk/PC Service 模式无设备隔离**，pybind 回调写同一组
全局变量，双 Pico 数据互相覆盖，不可用）：

| 层 | 实例 #1（robot_1） | 实例 #2（robot_2） |
|---|---|---|
| 头显 JSON `wholeBodyTracking.serverPort` | 63901 | **63902** |
| manager | `XROBO_UDP_PORT=63901` + `--port 5556` | `XROBO_UDP_PORT=63902` + `--port 5566` |
| deploy | `--zmq-port 5556` | `G1_LOCAL_ROBOT_ID=2` + **`--zmq-port 5566`** |

⚠️ ZMQ 输入端口**不随 `G1_LOCAL_ROBOT_ID` 自动分流**（ID=2 只改输出侧 5567/rt/r2），
`--zmq-port` 必须显式给；command/planner 话题名硬编码，靠话题区分双实例走不通。

### 5.0 已批准实施计划与交付口径

现场配置与实机验收严格按以下顺序，前一项不过不进入后一项；脚本可提前实现和静态
验证，但不能据此跳过头显/链路 gate：

1. 配置并回读头显#2 的 GameLink `wholeBodyTracking`，确认 host IP、UDP 63902、
   `sendControllers=true`；改前保留原 JSON 备份；
2. 手动启动 manager#2（UDP 63902 → ZMQ PUB 5566），只开头显#2 验证它与
   manager#1（UDP 63901 → ZMQ PUB 5556）确实隔离；
3. 手动将 deploy#2 切到 `G1_LOCAL_ROBOT_ID=2 + zmq_manager:5566`，从启动日志
   核对输入端口及 `rt/r2/*` DDS 身份；
4. 把已验证参数固化进 `tools/pipeline_pico_bringup.sh` 的
   `PIPELINE_DUAL_PICO=1` 分支，默认未设置时完整保留单 Pico + channel#2 keyboard；
5. 先做 `bash -n`、`git diff --check` 与双分支静态审查，再按 §5.2 四道门实机验收；
6. 只按真实结果更新本节状态：脚本完成不等于实机验收完成，不提前标 ✅。

当前进度：

| 项 | 状态 | 证据/剩余工作 |
|---|---|---|
| bringup 双分支实现 | ✅ 静态完成 | `bash -n`、`git diff --check`、默认/双 Pico 隔离桩测通过 |
| manager#2 无头显冒烟 | ✅ 完成 | 实际监听 UDP 63902 并进入 `waiting for body data...`；因无头显，尚未 bind ZMQ 5566 |
| 头显#2 配置与回读 | ⏳ 待设备 | 当前 ADB 无在线 Pico，未改设备配置 |
| manager/deploy 手动隔离 | ⏳ 待实机 | 不打断当前正在运行的单 Pico + keyboard 链 |
| §5.2 四道验收 | ⏳ 待双人窗口 | 未发车、未做双人联动和急停 |

代码改动边界：只改本工程 bringup 编排和配套文档；Isaac、manager、deploy 源码
均不改。双 Pico 分支必须显式钉死 manager#1/#2 的 UDP 端口为 63901/63902，避免
调用环境残留的 `XROBO_UDP_PORT` 污染；deploy#2 去掉长寿命键管道后，仍要给
`deploy.sh` 的 `Proceed with deployment? [Y/n]` 一次性送入确认换行，否则后台启动
会在 `read` 处退出。脚本仍保留双 deploy 并行 Init、两套 manager UDP receiver、
STARTUP HOLD 就绪检查和 renice，但不等待头显首帧、不代替任何操作者发车。

回滚口径：不设置 `PIPELINE_DUAL_PICO=1` 即回到现有单 Pico + keyboard#2；头显#2
可用施工前备份恢复。完整 bringup 会清理既有 sim/deploy/manager 进程，实机联调只在
两台头显与两位操作者到场的测试窗口执行。

### 5.1 施工步骤

**A. 头显 #2 配置**（一次性）：
1. 第二台 Pico 4 Ultra 安装 robot#2 专用强制刷新版（内置
   `wholeBodyTracking.serverPort=63902`），不能拿 robot#1 的 63901 包代替；原厂/未知
   签名包按 §1.1 先卸载再安装，同一套重签强刷版之间才可 `install -r`；
2. 启动 GameLink，让包内 `assets/XrLinkConfig.json` 强制刷新到 files 目录；若只能使用
   旧版/通用包，则按 §1 命令先 `pull` 备份，再手工设置
   `wholeBodyTracking` = `{serverHost: <host IP>, serverPort: 63902,
   sendControllers: true}` 后 force-stop/重启；
3. 无论采用哪种包，都要用 `adb shell cat` 回读并逐项核对 host、63902、
   `sendControllers=true`；
4. ⚠️ **端口配错的故障模式是静默混流不是报错**：两台头显都发 63901 时，
   `xr_client` 只留"最新一帧"，两人身体数据交替覆盖、机器人抽搐。防呆判据见 5.3-①。

**B. 两套 manager 隔离启动**（#2 是新增的第五终端）：

```bash
cd <GR00T路径>
# manager#1：双 Pico 联调时也显式钉死 63901
XROBO_TRANSPORT=udp XROBO_UDP_PORT=63901 PYTHONUNBUFFERED=1 .venv_teleop/bin/python \
  gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose --port 5556

# manager#2：另一个终端
XROBO_TRANSPORT=udp XROBO_UDP_PORT=63902 PYTHONUNBUFFERED=1 .venv_teleop/bin/python \
  gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose --port 5566
```

**C. deploy#2 从 keyboard 切 zmq_manager**（§4.2 终端③ 改为）：

```bash
cd <GR00T路径>/gear_sonic_deploy
G1_LOCAL_ROBOT_ID=2 bash deploy.sh --disable-crc-check \
  --input-type zmq_manager --zmq-port 5566 \
  --isaac-handcmd-hz 100 isaac
```

**D. bringup 脚本改造**（`tools/pipeline_pico_bringup.sh` 加 `PIPELINE_DUAL_PICO=1`）：
- 只接受 `PIPELINE_DUAL_PICO=0/1`，默认 0 完整保留现有行为；
- deploy#2 参数换成上面 C 的形式（去掉长寿命键管道 tail dk_r2，但给 deploy.sh
  一次性确认换行）；
- 追加 manager#2（B 的参数，日志 `pico_manager_r2.log`）；
- 双 Pico 模式显式设置 manager#1 `XROBO_UDP_PORT=63901`、manager#2
  `XROBO_UDP_PORT=63902`，不继承外部同名变量；
- **去掉 channel#2 的 `]` 发车与 planner 预激活**（zmq_manager 下发车来自操作者
  A+B+X+Y，脚本不能替按）；`wait_for CONTROL marker` 相应不再作为脚本内验收步；
- manager UDP receiver、renice 和双 deploy `Init Done` 检查两种模式都保留，结束提示
  列出两份 manager 日志；UDP 就绪不等于头显数据/ZMQ PUB/实机控制已验收。

实现后的双 Pico 一键命令：

```bash
PIPELINE_DUAL_PICO=1 bash tools/pipeline_pico_bringup.sh
```

### 5.2 验收序列（先隔离后联动，别一步到位）

联调时至少同时盯以下日志（`PIPELINE_LOG_DIR` 覆盖过时替换目录）：

```bash
tail -F /tmp/pipeline_pico/{pico_manager,pico_manager_r2,deploy_r1,deploy_r2,host_dual}.log
```

1. **单头显打 63902 通路**：只戴头显#2 → manager#2 日志停止刷 waiting、
   manager#1 仍在 waiting（= 端口隔离成立，无串流）；
2. **头显#2 单独发车**：A+B+X+Y → robot_2 进控制（sim 日志 channel#2
   精确指纹 `[sonic_dds:r2] CONTROL marker received`）、robot_1 不动；
3. **双头显同时**：两人各自发车、各控各机器人；重点盯 `sync_waits` 不涨、
   帧率账本 S 余量（多一个 manager 的 CPU 负载）；
4. **急停语义**：任一人 A+B+X+Y 只停自己的 manager，另一路不受影响。

### 5.3 已知边界与风险（做前读）

- ① **同端口双发送者静默混流**：`xr_client` 记录 `_latest_sender` 但不过滤。
  现有 manager 日志不打印 `_latest_sender`，标准 gate 以 §5.2-① 的 waiting 隔离为准；
  需要核对包源时另开 `sudo tcpdump -ni <iface> 'udp dst port 63901 or udp dst port 63902'`，
  确认两个头显源 IP 分别只打对应目标端口。出现机器人抽搐先查 serverPort；
- ② **AR 观看第二路是独立工作包**：win2 一台机只能跑一个 XR 实例（12GB 显存上限），
  操作者#2 要 AR 得部署 win1 + `run_pipeline_viewer_ar_robot2.bat`；且 GameLink 的
  **视频串流目标 IP 在 dex 里硬编码**（与追踪目标独立），头显#2 要看 win1 画面需
  按 apk 重打包流程改 dex——step② 可先做"双控制 + 单 AR/无 AR"，视频链后补；
- ③ **feedback 通道对 #2 同样是断的**（manager 订 5557/`g1_debug`，deploy#2 实际
  UDP 5567/`g1_2_debug`）——FROZEN/VR_3PT 重校准回退零位，PLANNER/POSE 不受影响；
  将来接通时注意 manager#2 要 `--zmq_feedback_port 5567`；
- ④ **planner 退化 bug 双路都在环**（PLANNER 子模式共用 kplanner 线程逻辑，
  每个 deploy 各自一份状态）：任一路行走退化 = 重启对应 deploy；
- ⑤ **整场景 reset 不通知输入链**双路同样成立——step③ 首例实测（倒地复位后可
  继续操作，另机）见 §4 已知限制条目，退化指纹自查与 POSE/手动 reset 细项未收口。

## 6. 与 AR 观看链的关系（step④，开放问题）

控制链（头显→Ubuntu:63901/63902）与观看链（Ubuntu→win viewer→SteamVR AR→头显）
互相独立。值得注意：GameLink 这个 app 的**视频串流目标与全身追踪目标本来就是
两条独立配置**（前者在 dex 内硬编码、后者在 XrLinkConfig.json）——"同一台头显
边看 AR 边发追踪控制"在配置层面可能天然成立，⏳未实测，属 step④ 验证范围。
