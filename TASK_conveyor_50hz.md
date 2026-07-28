# 任务:把 Isaac-G1-29DoF-Sonic-Conveyor 闭环主循环调到 ~50Hz

> ✅ **已完成(2026-07-28 下午)**:闭环(deploy+锁步+GUI)300 秒稳态 **50.00Hz**
> (6 窗最低 49.96),timeouts=0、sync_waits 不增长、无倒地复位;单机+双进程同步
> 冒烟 PASS;单测 23 例全过。
> 修复四件套(均已进默认):①晚渲染(渲染挪到 lowstate 发布后,与 C++ 推理并行,
> ack 等待 3-5ms→0.1ms);②GUI 隔圈渲染 `--late_render_interval 2`(GUI 25Hz);
> ③镜像机器人 solver 8/4→1/1(E 14→11.5ms);④启动处方 `taskset -c 0-11` +
> `sudo renice -n -10`(14600KF 混合核调度抖动偷走 ~2.5ms/圈,这步决定性,
> 44.9→50.00Hz)。
> 注意:任务文件原基线"E 18-30ms"含渲染且受冷缓存污染——首轮 GUI 的 41Hz/62ms
> 尖峰是 shader 编译假象,暖机后即消失;"第 1 步"的结论实为渲染+调度而非物理。
> 完整实录(含负结论与探针/编排坑)见知识库调优专题《SONIC-50Hz帧率调优实录》
> 下半场章节。
>
> 原任务书如下(历史记录,基线数字已过时):

> 交接自 2026-07-28 会话。普通 Sonic 任务已达标(47-50Hz),本任务是把同样的目标
> 在流水线场景上兑现。**做之前先读完"已完成/已否决"两节,别重跑已判死的实验。**

## 目标与验收

- 闭环(GR00T deploy + 锁步开 + GUI 渲染)下 `moving average frequency` 稳定 ≥48Hz;
- 锁步保持健康:`[sonic_dds][sync] timeouts=0`、`sync_waits` 不随时间增长;
- 不破坏 conveyor 任务语义:双机 ZMQ 场景同步、PeerRobot 镜像、传送带物体行为不变
  (改完至少跑一次 conveyor 页记载的无头双进程验证);
- `python -m unittest discover -s tests` 全过。

## 现状基线(2026-07-28 实测,本机 i5-14600KF + RTX 4070 Ti)

| 任务 | 闭环主循环 | E(env.step) | A(等ack) |
|---|---|---|---|
| Isaac-G1-29DoF-Sonic | 47-50Hz | 11-18ms | 3-5ms |
| **Isaac-G1-29DoF-Sonic-Conveyor** | **33-37Hz** | **18-30ms(波动大)** | 3-5ms |

**全部差距在 E 多出的 ~8-12ms**:warehouse 场景 + 传送带及带上物体物理 + PeerRobot
镜像体的每帧写入/渲染。锁步与 DDS 路径无回归。

## 已完成的修复(在 perf/sonic-50hz-main-loop 分支,勿重做)

1. kit loop 20ms 节拍器解除(sim_main.py,启动日志应有
   `[sim] kit loop pacing disabled`);
2. 锁步 lowstate"新样本即发"(dds_master.py + g1_robot_dds.py,日志应有
   `immediate publish on fresh sample enabled for 'g129'`);
3. 系统电源计划 `powerprofilesctl set performance`(每次重启后确认);
4. GR00T 网格符号链接(机器人渲染可见,`model_data/g1/meshes` → decoupled_wbc 拷贝)。

## 已否决的手段(实测无效或更糟,别再试)

降渲染分辨率 / `--/app/vsync=false` / `--rendering_mode performance` /
asyncRendering / 删场景道具(对普通任务) / `--render_interval 8`(更糟,单帧成本
随间隔翻倍) / `--device cuda:0`(单机器人更慢) / lowstate 蛮力提频 500Hz(GIL 税)。
详见知识库 `NVIDIA/IsaacLab/unitree/调优专题/SONIC-50Hz帧率调优实录`。

## 建议路线(按顺序)

**第 1 步:拆分 E 的构成(必做,~20 分钟)**
conveyor + `--no_render --no_sonic_sync_with_lowstate --no_auto_reset_on_fall`
(DDS 用 `UNITREE_DDS_DOMAIN=91 UNITREE_DDS_INTERFACE=lo` 隔离,不需要 deploy):
- no_render 能到 50Hz → 差距全在渲染 → 走第 2 步;
- no_render 也不到 → 传送带物理是主因 → 走第 3 步;
- 也可以用探针把 E 里的 app.update() 单独计时(方法见下"工具箱")。

**第 2 步:渲染侧候选(若第 1 步指向渲染)**
- PeerRobot 镜像体:它是无碰撞的帧直写投影,试给它换简化视觉(或临时 visibility
  invisible 做 A/B 定量),确认它占多少毫秒;
- warehouse 资产:找出三角形/材质大户(Stage 里逐个隐藏 A/B),考虑简化或替换;
- 传送带上物体数量:减半 A/B。

**第 3 步:物理侧候选(若第 1 步指向物理)**
- 传送带实现方式:kinematic 驱动 vs 表面速度(surface velocity)成本差异;
- 带上物体的碰撞对数量、contact offset、solver 迭代数(sim_main 有
  `--solver_iterations` 现成参数);
- 注意 conveyor 页记载的历史坑:UrdfConverter instanceable 碰撞关不掉、
  GPU-Pipeline 传送带适配问题(知识库有专页)。

**第 4 步:回归验证**
- 闭环复测(deploy + 锁步 + GUI)≥3 分钟稳态;
- conveyor 双机同步语义验证(无头双进程,见知识库 conveyor 移植页);
- 单测全过;把结论(含负结论)补进知识库调优专题页。

## 工具箱

- **A/E/S 判读**:`[Performance] A:(等ack) E:(物理+渲染) S:(睡眠余量)`;
  `--stats_interval 5 --profile_interval 250`;
- **渲染探针**(不改源码拆 E):PYTHONPATH 注入 sitecustomize 猴补丁
  `SimulationContext.render()`,模板在上一会话 scratchpad,思路:复刻
  FULL_RENDERING 分支,分别计时 forward() 与 self._app.update();
- **deploy 无人值守**:`tail -f <keyfile> | bash deploy.sh --input-type keyboard isaac`,
  先 `echo "" > keyfile` 应答确认,Init Done + 仿真出现 STARTUP HOLD 后
  `printf ']' >> keyfile` 启动控制;
- **闭环启动命令**:
  ```bash
  # 终端1
  cd /home/nolo/GR00T-WholeBodyControl/gear_sonic_deploy && bash deploy.sh --input-type keyboard isaac
  # 终端2(分支 perf/sonic-50hz-main-loop)
  GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl \
  UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
  python sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
    --action_source sonic_dds --device cpu --stats_interval 5 --profile_interval 250
  ```
- 单机跑 conveyor 时 `[ZMQ Scene Sync] Stream stale` 警告是预期(无对端),
  非阻塞不拖帧率,但可顺手评估要不要降噪(2 秒一条略吵)。

## 关键文件

- `sim_main.py`(kit loop 解除段、`--keep_kit_loop_pacing`/`--lowstate_pub_hz`)
- `tasks/g1_tasks/g1_29dof_sonic_conveyor/conveyor_env_cfg.py`(场景/同步 term)
- `tasks/g1_tasks/g1_29dof_sonic_conveyor/zmq_scene_sync.py`(NOBLOCK 收发)
- `layeredcontrol/robot_control_system.py`(A/E/S 计时与 deadline 定频)
- 知识库:`~/文档/robotics-knowledge-base/NVIDIA/IsaacLab/unitree/`
  (调优专题 + conveyor 移植页 + 抖动专题证伪清单)
