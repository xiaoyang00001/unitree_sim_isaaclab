# 131 一键启动

PICO 串流端准备好后执行：

```bash
cd /home/nolovr/Documents/unitree_sim_isaaclab
./start_all.sh
```

脚本会依次托管 PICO Manager 和 SteamVR/NOLO，等待 GR00T deploy 正常运行后启动 Isaac Lab。默认命令为：

```bash
ISAACLAB_LOCAL_ROBOT_ID=1 ISAACLAB_SCENE_SYNC=0 \
GR00T_WBC_ROOT=/home/nolovr/GR00T-WholeBodyControl \
UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
python sim_main.py \
  --task Isaac-G1-29DoF-Sonic-Conveyor \
  --robot_type g129 \
  --action_source sonic_dds \
  --device cpu \
  --no_sonic_sync_with_lowstate
```

自定义 Isaac 参数：

```bash
./start_all.sh start --task <任务名> [其他 sim_main.py 参数...]
```

`start` 后的参数会原样传给 `sim_main.py`；不传参数时使用上述默认值。也可在命令前覆盖环境变量。

常用管理命令：

```bash
./start_all.sh status   # 查看状态
./start_all.sh logs     # 跟踪日志，Ctrl+C 退出日志
./start_all.sh stop     # 停止全部，保留 Steam 客户端
```

单独启动 Isaac 时可使用 `./start_isaac_xr.sh`，它会检查 SteamVR 和 NVENC 状态。
