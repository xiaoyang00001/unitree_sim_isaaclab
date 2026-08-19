# 流水线五机器人 SONIC 控制

## 1. 结果与边界

`Isaac-G1-29DoF-Sonic-Conveyor` 的五个站位现在都可以在同一个 host 进程中成为
43-DoF SONIC 动力学机器人。`robot_3`、`robot_4`、`robot_5` 复用原三台纯显示站位的
位置和朝向，但已经具备独立 articulation、执行器、足底接触传感器、动作、观测及 DDS
通道。五台机器人共使用 15 个动作 term，动作张量为 `5 × 43 × 3 = 645` 维。

这个数量只扩展同一 host 内的 SONIC 通道，**不扩展场景同步身份**：

- Ubuntu host 仍是 `ISAACLAB_LOCAL_ROBOT_ID=1`；
- Windows/Linux viewer 仍是 `ISAACLAB_LOCAL_ROBOT_ID=0`；
- 原 ID=1/2 对等模式保持双机语义；不要设置场景 ID=3/4/5。

`ISAACLAB_SONIC_ROBOT_COUNT` 支持 `2..5`，便于逐台联调。默认配置为 `5`；设置为
`2` 即回退原双 SONIC 真身，剩余三个站位继续使用无物理纯显示资产。额外真身只在
`ISAACLAB_TOTES_ON_CONVEYOR=1` 的流水线布局中启用；推车布局只有两个站位，即使共享
配置请求 `5`，EnvCfg、DDS 和 provider 也会一起自动回退到有效数量 `2`。

## 2. 通道映射

| 机器人 | 场景 asset / prim | 身体 DDS 对象 | Dex3 DDS 对象 | DDS topic prefix | Python shm 后缀 | 调试输入 / 输出端口 |
|---|---|---|---|---|---|---|
| 1 | `robot` / `Robot` | `g129` | `dex3` | `rt` | 无 | `5556` / `5557` |
| 2 | `robot_2` / `Robot2` | `g129_r2` | `dex3_r2` | `rt/r2` | `_r2` | `5566` / `5567` |
| 3 | `robot_3` / `Robot3` | `g129_r3` | `dex3_r3` | `rt/r3` | `_r3` | `5576` / `5577` |
| 4 | `robot_4` / `Robot4` | `g129_r4` | `dex3_r4` | `rt/r4` | `_r4` | `5586` / `5587` |
| 5 | `robot_5` / `Robot5` | `g129_r5` | `dex3_r5` | `rt/r5` | `_r5` | `5596` / `5597` |

五路 LowCmd ack 是 AND 门：任意一个已配置 deploy 没有运行或没有回当前 tick 的 ack，
整个环境都会保持在当前物理步。因此启动时应让五个 deploy 并行存活，不能等待前一个
完全握手后再启动下一个。

## 3. 推荐启动

一键脚本默认启动五台：robot_1 走 Pico，robot_2..5 走互相隔离的 keyboard 调试通道。

```bash
cd <仿真工程>

# 无桌面会话时加 PIPELINE_HEADLESS=1
PIPELINE_SONIC_ROBOT_COUNT=5 \
bash tools/pipeline_pico_bringup.sh
```

双 Pico 仍只接管前两台；后三台保持 keyboard：

```bash
PIPELINE_SONIC_ROBOT_COUNT=5 PIPELINE_DUAL_PICO=1 \
bash tools/pipeline_pico_bringup.sh
```

脚本的键管道为 `/tmp/pipeline_pico/dk_rN`。例如：

```bash
printf 'w' >> /tmp/pipeline_pico/dk_r3
printf 'a' >> /tmp/pipeline_pico/dk_r4
printf 's' >> /tmp/pipeline_pico/dk_r5
```

只启动仿真 host 的等价命令：

```bash
# 先进入包含 Isaac Lab 依赖的 Python 环境，例如 conda activate env_isaaclab
UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
ISAACLAB_LOCAL_ROBOT_ID=1 \
ISAACLAB_HOST_BOTH_ROBOTS=1 \
ISAACLAB_SONIC_ROBOT_COUNT=5 \
ISAACLAB_TOTES_ON_CONVEYOR=1 \
python sim_main.py \
  --task Isaac-G1-29DoF-Sonic-Conveyor \
  --robot_type g129 --action_source sonic_dds --device cpu --no_render \
  --stats_interval 10 --sim-state-export-hz 0 --lowstate-pub-hz 55 \
  --handstate-pub-hz 10
```

Pico bringup 和上面的性能命令关闭 `rt/sim_state` 导出：五机完整状态超过当前该通道的
共享内存容量，而且 host/viewer 场景同步走独立 ZMQ 链路。若确有外部消费者依赖
`rt/sim_state`，需先扩容并重新评估开销，再移除这个参数。

`--handstate-pub-hz 10` 只把 Dex3 HandState 的空闲保活从通用默认 100 Hz 调到
10 Hz；每个新 PhysX 手部样本仍通过事件唤醒立即发布，因此它不是 10 Hz 硬限流。
未显式传入该参数时，通用 CLI 仍保持 100 Hz 默认值。

## 4. robot_3..5 deploy 的必要参数

当前 `<GR00T 仓库>/gear_sonic_deploy/deploy.sh` 只对
`G1_LOCAL_ROBOT_ID=2` 自动选择 `rt/r2`。ID=3/4/5 会落回 robot_1 的 `rt` 默认值，导致
静默串台。因此在外部 GR00T 仓库完成通用映射前，robot_3..5 必须同时显式传入
`G1_LOCAL_ROBOT_ID` 和 `SONIC_DDS_TOPIC_PREFIX`，并隔离调试端口。例如 robot_3：

```bash
cd <GR00T 仓库>/gear_sonic_deploy
G1_LOCAL_ROBOT_ID=3 SONIC_DDS_TOPIC_PREFIX=rt/r3 \
bash deploy.sh --disable-crc-check --input-type keyboard \
  --zmq-port 5576 \
  --zmq-out-port 5577 --zmq-out-topic g1_3_debug \
  --udp-out-port 5577 --udp-out-topic g1_3_debug \
  isaac
```

robot_4/5 分别替换成 `rt/r4`、`rt/r5` 和表中的端口。只设置
`G1_LOCAL_ROBOT_ID=3/4/5` 不足以隔离 DDS。

## 5. Viewer

host 的一帧 `scene_state` 同时发布 `robot_1..5`。viewer 使用相同数量即可生成五个镜像：

```bash
ISAACLAB_LOCAL_ROBOT_ID=0 \
ISAACLAB_SONIC_ROBOT_COUNT=5 \
ISAACLAB_SCENE_SYNC_PEER_IP=<host-ip> \
python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor \
  --robot_type g129 --device cpu --hide_ui
```

viewer 不启动五套 DDS，也不参与 LowCmd ack。需要降低镜像端 PhysX 成本时，可增加
`ISAACLAB_PEER_ROBOT_MODE=visual_lod`，五个镜像均按 scene_state 的 base pose 与关节角
更新纯显示 USD。

## 6. 验证与性能

不启动 DDS/deploy，也可以先做五台场景、动作布局和短步物理门检查：

```bash
python tools/smoke_conveyor_scene.py \
  --sonic-robot-count 5 --scene-only --steps 10 --device cpu
```

判定应包含：

```text
Active Action Terms (shape: 645)
[smoke] SONIC robots=5, action_dim=645
[smoke] robot: root_drift=...m, rotation_delta=...deg
[smoke] RESULT: PASS (scene-only short-step gate, robots=5, viewer_mirrors=0, action_dim=645)
```

该命令还会拒绝 NaN/Inf、根节点位移超过 0.10 m 或相对姿态变化超过 20°。它使用默认
关节目标且没有真实 provider 的启动 Root pin，只适合推荐的短窗口创建检查；不能用其
替代五套 SONIC deploy 的站立、行走和倒地恢复闭环验收。步数拉长后机器人失稳应报告
`FAIL`，而不是把“仍能 step”误记为通过。

真实 DDS 联调还要逐台确认 `DDS topics`、`Init Done` 和
`[sonic_dds:rN] CONTROL marker received`，并观察 host 的
`[Performance] A/E/R/S/T`、`physics_steps`、`sync_waits` 是否持续前进。

2026-08-19 在同一台机器、CPU PhysX、本地渲染、脚踝接触传感器开启且
`--sim-state-export-hz 0` 下得到以下现场数据：

| 场景 | 实测频率 | 平均循环时间 | 说明 |
|---|---:|---:|---|
| 四机、LowCmd 重复包快路前 | 总体 3.65 Hz；最近 100 步 4.09 Hz | 273.68 ms | 四路 CONTROL，timeout=0 |
| 四机、LowCmd 重复包快路后 | 总体约 6.9–7.1 Hz；最近约 6.3–7.6 Hz | 约 142–145 ms | 四路 CONTROL，timeout=0、sync_waits=0 |
| 五机、优化前 | 总体 0.85 Hz；最近 0.86–0.87 Hz | 约 1170 ms | 五路已通，但出现明显非线性性能断点 |

SONIC 每路约 500 Hz 发布 LowCmd，而四机仿真实际只产生约 7 Hz 新状态。重复包快路仍
逐包刷新存活时间和 ACK，但只对实际变化的约 7 Hz 命令做完整字段展开、JSON 和共享内存
写入；四机现场每路约 492–493 Hz 命中快路。该优化没有降低真实控制指令频率，也没有关闭
足底传感器。五机尚未用这版快路重新测量，因此不能把四机增益直接外推到五机。

在 LowCmd 重复包快路阶段，四机剩余主要耗时在 `env.step`（动作应用、PhysX、事件与十路 host 观测/DDS 状态），
不是 N 路 ACK 等待；五机的非线性断点还需独立做无 DDS 的 2..5 台对照。配置目标仍为
50 Hz，不能把配置值当作实测。若业务硬性要求稳定实时 50 Hz，需要进一步做物理/观测
降载、多进程拆分或硬件预算评估。

同一版代码重新启动后做 LowState 发布频率 A/B：100 Hz 时四机总体 8.18 Hz、最近
8.01 Hz、平均循环 122.2 ms；55 Hz 时总体 9.24 Hz、最近 9.29 Hz、平均循环 108.2 ms，
约提升 13%，且四路均保持 `timeouts=0`。55 Hz 仍高于 SONIC 的 50 Hz 控制节拍，因此
一键脚本采用 55 Hz 作为多机 keepalive；通用 CLI 默认值不变，需要排查兼容性时可显式
回到 `--lowstate-pub-hz 100`。

### HandState 发布组合优化 A/B

在 LowCmd 重复包快路、LowState 55 Hz 和 Dex3 HandCmd 快路均已生效后，继续对四机
Dex3 HandState 做同口径 A/B。下表的 HandState 频率均为八个左右手 state topic 中
单个 topic 的实测频率：

| 四机配置 | 闭环频率 | 单 HandState topic | 健康门禁 |
|---|---:|---:|---|
| HandState 原路径（通用 100 Hz 调度） | 12.574 Hz | 98.2 Hz | 四路 CONTROL；`physics_steps` 持续增长；`sync_waits=0`；四路 `timeouts=0` |
| 状态 generation cache + 10 Hz 空闲保活 + 新样本即发 | 14.55 Hz | 14.9–15.1 Hz | 四路 CONTROL；`physics_steps` 持续增长；`sync_waits=0`；四路 `timeouts=0` |

组合优化使四机闭环频率提升约 15.7%。八个 HandState topic 的合计发布量由约
785.6 条/秒降到约 119–121 条/秒，减少约 85%；新鲜样本仍随实际物理步立即发出。
generation cache 同时避免保活周期反复读取共享内存 JSON、重复展开 42 个状态字段；
现场日志中 `shm_reads=0.0Hz`。最终 60 个性能采样的循环耗时中位数为 64.5 ms、
p90 为 82.9 ms，全部 `stepped=1` 且 `sync_waits=0`。本次 A/B 验证的是上述组合方案，
不能把收益单独归因于 10 Hz 参数，也不能外推为五机性能结果。
