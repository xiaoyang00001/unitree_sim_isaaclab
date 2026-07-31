# win2 conveyor 跨机闭环帧率账本（2026-07-31）

目标：`Isaac-G1-29DoF-Sonic-Conveyor` 跨机闭环（win2 仿真 + Ubuntu deploy，WiFi）达 50Hz。
口径：`[Performance] A/E/R/S/T` 中位数，两侧同重启、暖机后丢前 8 样本、`sync_waits` 零增长才算数。
测量环境：i5-14600KF + RTX 4070 Ti，XR 常驻服务已杀，`UNITREE_SKIP_LOWSTATE_CRC=1` + deploy `--disable-crc-check`。

## 结果总表（满场景=同步+镜像+道具全开）

| 配置 | A | E | R(摊薄) | T | 频率 |
|---|---|---|---|---|---|
| headless 基线 | 6.0 | 31.0 | — | 37.2 | 25.0Hz |
| headless + 镜像执行器合并 | 6.8 | 24.6 | — | 31.3 | 30.1Hz |
| GUI 精简kit + hide_ui + 隔圈渲染 | 6.1 | 14.3 | ~6.2 | — | 39.0Hz |
| 上行 + 镜像执行器合并 | 6.3 | 12.0 | ~6.2 | — | 43.0Hz |
| **+ 渲染1/3圈 + lowstate 55Hz（combo）** | **5.0** | **12.5** | **4.3** | **18.2** | **45.7Hz** ✅当前最优 |

启动（combo 形态，在 `perf/conveyor-peer-actuator` 分支上）：

```
run_win.bat --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 ^
    --action_source sonic_dds --device cpu --hide_ui --late_render_interval 3 ^
    --lowstate_pub_hz 55 --dds-interface <本机IP> --stats_interval 5 --profile_interval 250
```

## E=31ms 的归因（py-spy 45s 采样 + 消融，全部实测）

1. **执行器逐组每子步 Python 记账**：`_apply_actuator_model` 占主线程 39%（≈7.6ms/圈）。
   执行器本就是 Implicit——贵的不是 PD，是**组数**（6 组×2 台×每子步）。镜像体组划分无语义
   ⇒ 合并单组（a6b9437），headless 实测 −6.5ms。主机器人组划分是 SONIC 动力学红线，未动。
2. **后台线程 GIL ≈31% 墙钟**：lowstate 发布 13.6%（cyclonedds 纯 Python 序列化@100Hz，86Hz
   是重复样本）；lowcmd 接收 12.4%（deploy 284Hz 发包、真变化 13.6Hz，逐包纯 Python 反序列化）；
   shm 的 json.dumps + 抽样 CRC 5.3%。
3. **GUI 反而比 headless 快的机理**：每圈 ~12ms 渲染在 C++ 里释放 GIL，后台线程得到并行窗口，
   不再打断 E。headless 满场景 E=31 > GUI 摘光场景 18.5，就是这块差价。
4. 消融：ZMQ 场景同步 ≈0；镜像机器人 5.6ms；道具 6.7ms；warehouse 背景 ≈0。

## 已否决（别重试）

| 方向 | 实测 |
|---|---|
| `--no_render`（headless） | E 零变化：headless 无相机时 env.step 本就不渲染 |
| `--lowstate_pub_hz 55` 单独用 | 仅 −1.3ms（但确认 55Hz 不破坏锁步） |
| 换平地面（plain_ground） | E 反而 +2.3ms，背景不是负担 |

## 到 50Hz 的剩余路径（按优先级）

1. **A（≈5ms）**：WiFi RTT ~3.3ms 是硬地板，win2 插有线直连可砍到 1-2ms —— 纯基建，最省事。
2. **主机器人执行器同款合并**（预计 −3~4ms）：数学上逐关节参数不变，但须过 SONIC 跟踪
   验证（joint_metrics 对比 + 行走回归）才能收，单独分支做。
3. **shm json→二进制 + lowstate 重复样本序列化缓存**（预计 −2~3ms）：dds 层改造。
4. deploy 侧降 lowcmd 发包率（284Hz→~60Hz）：需 GR00T 仓库配合。

## 测量纪律补充（本轮新踩的坑）

- PowerShell `Select-String` 默认大小写不敏感："HOLD" 会匹配到 ZMQ 告警里的 "holding"，
  且 stderr 无缓冲抢先落盘，轮询判据必须 `-CaseSensitive -Pattern 'HOLD:'`。
- `pkill -f`/`pgrep -f` 的模式出现在自己命令行里会自匹配（把自己杀掉/计数虚高），用 `[x]` 技巧。
- win2 上 `del` 被运行中进程占用的日志会静默失败，新 run 追加进旧文件——判 run 边界看时间戳。
