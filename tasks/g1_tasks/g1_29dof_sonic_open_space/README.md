# Isaac-G1-29DoF-Sonic-OpenSpace

单台 G1 SONIC 在真实环境资产中行走。场景复用已验证的 43-DoF G1 + Dex3
SONIC 机器人资产和 DDS 控制链；每个任务只加载一台机器人。

其中 `Warehouse` 是完整场景（地面、墙体、货架、灯光和导航结构）；前五个
Grass/Gravel/Mud/Slate/Snow 是独立的真实地形片段，保留用于材质/接触对比。

可选任务：

- `Isaac-G1-29DoF-Sonic-Grass`
- `Isaac-G1-29DoF-Sonic-Gravel`（默认推荐）
- `Isaac-G1-29DoF-Sonic-Mud`
- `Isaac-G1-29DoF-Sonic-Slate`
- `Isaac-G1-29DoF-Sonic-Snow`
- `Isaac-G1-29DoF-Sonic-OpenSpace`（Gravel 的兼容别名）
- `Isaac-G1-29DoF-Sonic-Warehouse`（Isaac Sim Simple Warehouse 完整场景，推荐先看这个）
- `Isaac-G1-29DoF-Sonic-Apartment`（Lightwheel Apartment 完整建筑/室内）
- `Isaac-G1-29DoF-Sonic-Staircase`（Lightwheel 两层楼梯/阁楼完整场景）
- `Isaac-G1-29DoF-Sonic-Office`（Isaac Sim 官方 Office 完整场景）

启动：

```bash
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl python sim_main.py \
  --task Isaac-G1-29DoF-Sonic-Gravel \
  --robot_type g129 \
  --action_source sonic_dds
```

完整仓库场景：

```bash
GR00T_WBC_ROOT=/home/nolo/GR00T-WholeBodyControl python sim_main.py \
  --task Isaac-G1-29DoF-Sonic-Warehouse \
  --robot_type g129 \
  --action_source sonic_dds
```

仓库 USD 默认来自 Isaac Sim 官方资源库：
`{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd`；可用
`ISAAC_REAL_WAREHOUSE_USD` 指向可访问的镜像或本地缓存。

资产路径由 `LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR` 控制，默认根目录为
`/home/nolo/Lightwheel_OpenSource/Locomotion/`。GravelGround 约 126 m × 13 m；Grass
约 8 m × 6 m；Mud、Slate、Snow 分别是泥地、石板路和雪路。
