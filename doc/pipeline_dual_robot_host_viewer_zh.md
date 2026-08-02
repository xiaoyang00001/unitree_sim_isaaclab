# 流水线双机器人 host + viewer 架构（工作包 A/B 实录）

> 里程碑（2026-08-02）：**一台 Ubuntu 同进程跑 robot_1+robot_2 双全动力学 SONIC 机器人，
> 各由一套 GR00T deploy 独立控制（键盘可分别驱动行走），win2 上的纯镜像 viewer 实时同步
> 全场景**——"1 host + N viewer"目标架构首次实景运行。
> 配套 GR00T 仓库提交：话题前缀参数化 + deploy.sh 环境优先修复（分支 feat/isaac-state-sync）。

## 1. 三种身份模式

conveyor 任务（`Isaac-G1-29DoF-Sonic-Conveyor`）现在有三种身份，全部由
`sync_identity.py` **单一真源**解析（sim_main 与 conveyor_env_cfg 共用同一份实现，
进程环境变量永远优先于 `configs/scene_sync.env`）：

| 身份 | 环境变量 | 场景 | ZMQ 方向 |
|---|---|---|---|
| 对等端 | `ISAACLAB_LOCAL_ROBOT_ID=1/2` | 本机全动力学 robot + 对端镜像体 | 双向（机器人互发；物体/复位 ID=1 权威） |
| viewer | `ISAACLAB_LOCAL_ROBOT_ID=0` | 双镜像体 + 场外 ghost + kinematic 物体 | 只收不发（SUB host:15555） |
| **host** | `ID=1` + `ISAACLAB_HOST_BOTH_ROBOTS=1` | **robot_1+robot_2 双全动力学**，无镜像体 | 只发不收（robot_1+robot_2+物体） |

host 的第二机器人通过**第二套 DDS 通道**驱动：话题 `rt/r2/lowcmd|lowstate|secondary_imu`
与 `rt/r2/dex3/*`、DDS 注册名 `g129_r2`/`dex3_r2`、共享内存 `isaac_robot_state_r2` 等。
⚠️ **shm 名必须隔离**——SharedMemoryManager 对同名段是静默 attach 共享，漏改后缀的
症状是两台机器人互相执行对方命令且无任何报错。

## 2. 启动手册（Ubuntu host + 双 deploy + win 侧 viewer）

```bash
# ① host sim（GUI 验证形态；无画面跑法去掉 --hide_ui 换 --no_render，省 ~10ms/帧）
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl \
UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
ISAACLAB_LOCAL_ROBOT_ID=1 ISAACLAB_HOST_BOTH_ROBOTS=1 \
UNITREE_SKIP_LOWSTATE_CRC=1 UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50 \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
  --action_source sonic_dds --device cpu --hide_ui --stats_interval 10

# ② 两套 deploy —— ⚠️ 必须并行启动（错峰 ~20s 即可），不能串行等 Init Done：
#    双 ack AND 门下 deploy#1 的 Init 要看到新鲜物理样本，而物理推进又在等
#    deploy#2 的 ack——串行启动会自锁在 4Hz。
cd $GR00T_WBC_ROOT/gear_sonic_deploy
bash deploy.sh --disable-crc-check --input-type keyboard isaac                      # 终端A
G1_LOCAL_ROBOT_ID=2 bash deploy.sh --disable-crc-check --input-type keyboard isaac  # 终端B

# ③ win 侧 viewer（win2 桌面双击，参数见 run_pipeline_viewer.bat）：
#    ISAACLAB_LOCAL_ROBOT_ID=0 + ISAACLAB_SCENE_SYNC_PEER_IP=<Ubuntu IP>
```

- 测前清残余：`pgrep -fa g1_deploy_onnx_ref` 必须为 0——残留实例会以 kHz 级频率
  轰 ack，链路数据完全不可信（本轮实测 12 个残留 = 2kHz lowcmd）。
- 首次双实例并发冷启动会争写 planner 的 `.trt` 缓存，先单跑一次预热。

## 3. 键盘控制（两台各自独立）

每台机器人的 deploy 都是完整键盘协议，激活序列：**`]`（开始控制）→ 回车（开 planner）→
`2`（选行走模式）→ 移动键**。移动是脉冲语义（一发走一段自动停回 IDLE）。

| 键 | 动作 | 键 | 动作 |
|---|---|---|---|
| `w`/`s` | 前进/后退 | `a`/`d` | 左移/右移 |
| `j`/`l` | 左转/右转 | `9`/`0` | 减速/加速 |
| `1`-`8` | 动作模式集 | `` ` `` | 急停 |

实测（2026-08-02，多轮）：robot_1/robot_2 单发脉冲行程可达 10-16m，SONIC 自平衡
全程稳定，win2 viewer/AR 同步无丢帧；robot_1 的 180° yaw 出生朝向下行走正常（此前
"需 Phase 1 实测"的悬案就此关闭）。

操作要领（实测标定）：
- **起步有 ~10s 的慢加速斜坡**（前几秒 <0.01m/s 的挪动是动量爬升，不是没响应——
  别在这个窗口重复排障或猛按键）；随后进入 ~0.25m/s 巡航直到脉冲行程走完自动停；
- planner 激活（回车→`2`）在 deploy 进程存续期内保持，不需要每次移动前重按；
  `host_bringup` 类编排脚本可在发车后直接预激活两台（发 `\n` 与 `2` 进键管道）；
- 键盘走管道文件时：`printf 's' >> <keyfile>`，单字符即时生效无需换行。

## 4. 帧率账本（host 双机器人，i9/20 核 Linux）

| 阶段 | 主循环 | 关键数字 |
|---|---|---|
| 贯通初始（全 CRC） | 25-26 Hz | E 20-42ms 波动，A 有 20ms 尖峰 |
| + LowState 发布免 CRC | 26-29 Hz | A 尖峰消失 |
| + lowcmd 接收 1/50 抽样 | 31-33 Hz | E 收敛 ~19ms |
| + renice -10 | **34-36 Hz** | headless 形态 |
| GUI（--hide_ui） | 24-26 Hz | R≈10ms 渲染成本 |

归因（py-spy 150Hz）：瓶颈是 **GIL 单线程天花板**（20 核只用 1.2 核，sim 101.5%）。
- ⚠️ **Linux 上 unitree CRC 也是纯 Python**（旧账本"C 库 ~0.01ms"不成立）——双通道
  1000 包/秒下发布 CRC 20.5% + 接收校验 14.5%，两刀 CRC 是最大收益；
- ⭐ **执行器逐子步记账仅 5.6%**——win2 上 39% 的头号嫌疑在 Linux 不成立，
  主机器人执行器合并这条红线刀**不需要动**；
- 剩余：native PhysX ~40%（双 43DoF 固有）、cyclonedds 反序列化 ~10%
  （500Hz lowcmd 九成是重发包，深水区）、观测/metrics 小项 ~5%。

锁步语义下 35Hz = 0.7× 慢放（不丢帧不失真）；viewer 侧渲染恒 50fps。

## 5. 已知坑（每条都实测踩过）

1. **身份双源裂脑**：sim_main 与 env cfg 曾各自解析身份——已收敛进 `sync_identity.py`，
   新增身份判定一律走它，别在任何一侧重新实现。
2. **deploy.sh 的 env 文件强覆盖**（GR00T 侧已修）：`config/g1_udp_network.env` 里的
   `G1_LOCAL_ROBOT_ID=1` 曾无条件覆盖命令行身份，第二实例静默变第一实例。
3. **双 deploy 串行启动自锁**、**残留 deploy 污染**（见 §2）。
4. deploy C++ 的话题常量是 namespace 级 static、**先于 main() 初始化**——
   `SONIC_DDS_TOPIC_PREFIX` 只能走环境变量，改成 CLI 参数会静默失效。
5. 观测函数缓存必须按 asset 键控——共用 buffer 是 aliasing 覆写、共用 sample_seq
   会让双锁步 tick 串台（ack 永不匹配），都是排查代价极高的静默错误。
6. ⛔ **deploy planner 的"退化态"**（2026-08-02 实锤 + 双 agent 日志/源码对照定因）：
   WALK→IDLE 之后（整场景 reset 是最强诱因，但**不限于 reset**）再次下发 WALK 时，
   planner 从残留内部状态 replanning——判别指纹是 **Planner Model 推理耗时从健康的
   93-182ms 掉到 36-79ms（约 1/3）**，生成的轨迹退化为站立或原地踉跄（tilt 冲 7.7°
   无位移）。`t`/`r`/回车重开 planner 均救不回（重开会新增"Reset init reference"
   绑定行，但 Model 耗时不回升——残留在更深的轨迹续接状态里）。
   **唯一已验证恢复路径 = 重启 deploy（按测量纪律 = 全链路重启）**。
   GR00T 侧修法方向：IDLE→WALK 转换时重置 planner 的 motion buffer/heading 续接
   （gen_frame_/current_frame_ 逻辑，见 localmotion_kplanner 与 CurrentFrameAdvancement）。
   运维约束：行走演示期间不要发整场景 reset；归位优先用 `w` 走回。
   ⚠️ **判据勘误**：`cmd_max_abs_dq≡0` 不能当"没发运动目标"的证据——本 deploy 的
   lowcmd 无条件 tau_ff=0/dq_target=0（纯位置目标+kp/kd 是设计）。正确判据：
   host 侧 `root_speed_max`（行走 ≈1.2m/s vs 退化 ≤0.008）与 `pd_target_max`
   （行走 1.33-1.50rad vs 站立 ≈0.39），deploy 侧 Planner Model 耗时（>90ms 健康）。

## 6. 新机器部署 viewer（如 win1）——相对通用 Windows 部署的增量

基础环境先按 `doc/windows_deployment_zh.md`（及知识库《新Windows机器从零部署任务书》）
走通用部署：conda `env_isaaclab`(py3.11) + isaacsim 5.1.0 + IsaacLab fork(`0703` 分支) +
**cyclonedds 自编 0.10.x**（千万别 11.x）+ Windows 长路径。在此之上：

**viewer 可以豁免的**（比对等/host 端轻很多）：
- ❌ GR00T deploy 二进制与模型（viewer 无锁步、不跑推理）——但 ⚠️ `groot_assets` 子集
  （win2 上 137M）**仍然必需**：`import tasks` 在模块级合成 SONIC URDF，与跑哪个任务无关；
- ❌ 防火墙入站规则（viewer 只有**出站** SUB 连 host:15555，无入站需求）；
- ❌ deploy 侧的一切（`--disable-crc-check` 等都是 host 侧的事）。

**viewer 必需的**：
- 工程 checkout `feat/pipeline-host-viewer`（或 tag `pipeline-dual-robot-walking-v1`），
  镜像机器人 USD 已随 git 入库（`scene_assets/peer_robot/`，约 85MB，无需构建）；
- `pyzmq`（env_isaaclab 内 pip 装）；
- 环境变量：`PIPELINE_HOST_IP=<Ubuntu IP>`（bat 里可覆盖，win2 默认已探测）；
- **AR 额外**：`isaacsim-extscache-{kit,kit-sdk,physics}` 三包（装前先开长路径，
  半装残渣会让 h5py DLL 崩掉普通仿真）+ SteamVR（NOLO Link 或 ALVR 拉起）。

**启动与判据**：桌面双击 `run_pipeline_viewer.bat`（AR 用 `run_pipeline_viewer_ar.bat`，
不能 ssh 起）。判通看日志四行：`Pure-mirror viewer mode`、`本机身份: viewer`、
（AR）`viewer XR 锚定挂镜像体 PeerRobot`、启动后无持续 `Stream stale`。
多台 viewer 无需 host 感知——PUB 天然扇出，第 N 台只管 SUB 上来。

## 7. 遗留与下一步

- 🎯 **Pico VR 控制接入**（正路，keyboard 只是调试工具）：目标形态=两操作者各戴
  Pico、各控一台机器人。走 SONIC 既有 VR 链（Pico manager → ZMQ:5556 →
  deploy `--input-type zmq_manager`，全身控制流），双机器人=两套 manager 分别喂
  deploy#1/#2（`G1_LOCAL_ROBOT_ID=2` 已默认 5567 独立端口）。VR 链不经过键盘
  planner ⇒ §5.6 的退化 bug 不在此路径上（但 VR 链的 reset 行为需单独验证）；
- ⛔ **keyboard planner 退化态**（§5.6，降级为调试工具限制）：修复方向已定位
  （IDLE→WALK 轨迹续接重置），因正路是 VR 控制、不优先修；调试期按运维约束绕行；
- AR 视角锚定的头显实测（代码已落地挂 PeerRobot/PeerRobot2，pxr 验证过 USD 层级；
  头显侧确认视角位置/pelvis yaw 跟随/B 键 recenter 待做）；
- win1 部署（照 §6 增量指引）与**多 viewer 并发**实测（传输层天然扇出，未实测）；
- host 50Hz：cyclonedds 反序列化去重（重发包先比 raw bytes 再解？需下探 SDK 层）、
  观测/metrics 小项打包、native 部分无大油水；当前 headless 34-36Hz / GUI 24-26Hz；
- AR viewer 实测 50Hz 满帧（A 0.0/E 6.9/R 10.6）——物理留 host、画面全推 viewer 的
  架构红利已被数字证实。
