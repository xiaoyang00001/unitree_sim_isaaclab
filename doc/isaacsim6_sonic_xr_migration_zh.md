# Isaac Sim 6 / Isaac Lab 6 SONIC XR 迁移记录

> 日期：2026-07-30
>
> 任务：`Isaac-G1-29DoF-Sonic`
>
> 运行环境：Isaac Sim 6.0、Python 3.12、CPU PhysX、SteamVR/NOLO OpenXR

本文记录 SONIC G1 场景从 Isaac Sim 5.x 迁移到 Isaac Sim 6 后遇到的启动、USD
层级、材质和姿态问题。这里的修复分布在两个仓库：

- `unitree_sim_isaaclab`：SONIC 场景、机器人显示材质和 XR 配置；
- `IsaacLab` fork：Isaac Sim 6 URDF importer 3.0 适配、嵌套刚体接触传感器和 OpenXR
  锚点兼容。

两个仓库使用同名分支 `fix/isaac6-xr-orientation`。只提交其中一个仓库不能完整复现
当前环境。

IsaacLab fork 的兼容基线是 `origin/feature/isaacsim-6-0`，对应修复提交为
`36dc6ebfb`（`fix(isaac6): port URDF conversion and XR anchors`）。

## 1. 启动命令

```bash
cd /home/nolovr/Documents/unitree_sim_isaaclab

UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
/home/nolovr/IsaacLab-6/isaaclab.sh -p sim_main.py \
  --task Isaac-G1-29DoF-Sonic \
  --robot_type g129 \
  --action_source sonic_dds \
  --device cpu \
  --teleop_device motion_controllers \
  --xr \
  --stats_interval 5 \
  --profile_interval 250
```

修改 Python 源码后必须结束旧进程再重新执行命令。已经创建的环境不会热加载
`EnvCfg`、URDF 或 XR anchor 配置；这里的“重启”不是重建 Conda 环境。

## 2. 故障与根因

### 2.1 URDF importer 接口已删除

Isaac Sim 6 的 URDF importer 3.0 不再导出：

```text
isaacsim.asset.importer.urdf._urdf.acquire_urdf_interface
```

继续使用 5.x 的 `UrdfConverter` 会在创建 `Articulation` 时直接失败。IsaacLab fork
中的转换器已经改为调用 `urdf-usd-converter` 和 asset transformer，并在转换前处理
fixed-joint merge。不要通过重新建环境或回装旧 `_urdf` 二进制来绕过这个问题。

### 2.2 Isaac Lab 6 配置四元数统一为 `xyzw`

这是机器人头朝下、桌子倒扣并进入地面、XR 世界翻转的共同根因。

Isaac Lab 6 的 `AssetBaseCfg.InitialStateCfg.rot` 与 `XrCfg.anchor_rot` 都使用：

```text
(x, y, z, w)
```

常用值如下：

| 含义 | Isaac 5 旧 `wxyz` | Isaac 6 `xyzw` |
|---|---:|---:|
| identity | `(1, 0, 0, 0)` | `(0, 0, 0, 1)` |
| 绕 Z 轴 -90° | `(0.7071, 0, 0, -0.7071)` | `(0, 0, -0.7071, 0.7071)` |

旧 identity `(1, 0, 0, 0)` 被 Isaac Lab 6 解释为绕 X 轴 180°，会让机器人和 XR
世界上下翻转。旧的桌子 yaw 四元数则会变成绕 X 轴约 90°，所以桌子会侧倒，且其
原点仍保持 `z=-0.3`，大部分几何因而进入地面以下。

本任务已按 `xyzw` 修正以下四处：

- SONIC 机器人初始姿态；
- 三个方块初始姿态；
- PackingTable 的 -90° yaw；
- XR anchor identity。

桌子 `z=-0.3` 是上游 locomanipulation 场景的正常布局值，不应为了补偿错误旋转而
抬高。正确姿态下桌面世界高度为 `SONIC_TABLE_TOP_Z=0.6996`。

### 2.3 OpenXR 配置与 USD 构造器的四元数边界不同

`XrCfg.anchor_rot` 是 `xyzw`，但 Isaac Sim 的 legacy `SingleXFormPrim` 构造器仍接收
标量在前的 `wxyz`。OpenXR 设备创建 `/XRAnchor` 前必须显式转换：

```python
x, y, z, w = xr_cfg.anchor_rot
anchor_rot_wxyz = (w, x, y, z)
```

动态同步路径直接构造 `pxr.Gf.Quatd(real=w, imaginary=(x, y, z))`，不要在该处重复
转换。

### 2.4 Isaac Sim 6 的机器人 prim 层级改变

URDF importer 3.0 生成的活动层级位于：

```text
/World/envs/env_0/Robot/Geometry/pelvis/...
```

旧路径 `/World/envs/env_0/Robot/torso_link/head_link` 不存在，因此 XR 同步回调会提前
返回，配置看起来“改了但没有生效”。当前配置使用：

```text
位置锚：/World/envs/env_0/Robot/Geometry/pelvis/waist_yaw_link/waist_roll_link/torso_link
旋转锚：/World/envs/env_0/Robot/Geometry/pelvis
```

位置跟随 torso，旋转只跟随 pelvis yaw，避免 torso 的 roll/pitch 把整个 XR 世界带歪。
这需要 IsaacLab fork 中新增的 `XrCfg.anchor_rotation_prim_path`。

### 2.5 材质绑定不能触碰碰撞实例

Isaac Sim 6 不再生成旧版每个 link 下的 `visuals` prim，而是在 `Geometry` 层级下生成
instance。部分 link 还会生成带 `_1` 后缀的碰撞 instance。若在 PhysX tensor view
建立后给碰撞 instance 写材质，会触发 stage 重组，随后出现：

```text
Simulation view object is invalidated
prim deleted
```

当前材质恢复逻辑会检查 instance prototype，只给包含
`UsdShade.MaterialBindingAPI`/`material:binding` 的 render instance 绑定白色、深色和
logo 材质。碰撞 instance、刚体、惯量和 joint drive 均不修改。

### 2.6 嵌套刚体必须完整遍历

Isaac Sim 6 的 articulation 是嵌套 link 层级。旧接触传感器代码遇到第一个
`RigidBodyAPI` 后停止向下遍历，会漏掉 ankle/sole 等后代刚体。IsaacLab fork 的
`activate_contact_sensors()` 现在会遍历完整子树，并给每个刚体 author contact report
schema 和零阈值。

### 2.7 运行时 IMU 四元数也改为 `xyzw`

初始配置修正后机器人能够正常站立，但 SONIC 仍可能出现“手臂可动、下半身不跟随”。
原因是 Isaac Lab 6 的 `body_link_pose_w` 和 `root_quat_w` 运行时张量同样改成了
`xyzw`，而 Unitree `LowState.imu_state.quaternion` 协议仍要求 `wxyz`。

旧代码把 Isaac 6 原始值直接写入 DDS。例如接近单位姿态的
`(x, y, z, w) = (-0.0008, -0.0043, 0.0071, 0.9999)` 被 SONIC 当作
`(w, x, y, z)` 后接近错误的 180° 旋转。关节状态和手臂命令本身仍是正确的，因此手臂
可以响应；依赖 pelvis IMU 姿态和机体系加速度的 locomotion policy 则无法正常控制腿部。

`isaaclab_compat.py` 现在读取已安装 Isaac Sim 包的主版本来处理边界：Isaac 5 保持
`wxyz`，Isaac 6 把运行时 `xyzw` 转为 `wxyz`，之后再进行世界系到 IMU 机体系的
加速度、角速度变换并写入 DDS。同一个兼容函数也用于跌倒检测和 SONIC 姿态统计，
避免诊断值再次按错顺序。版本检查只读取包元数据，不会在纯 Python 测试中启动 Kit。

## 3. 修改范围

### `unitree_sim_isaaclab`

- `tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py`
  - 修正 Isaac Lab 6 `xyzw` 初始姿态；
  - 使用 Isaac Sim 6 的实际 XR prim 路径；
  - 位置锚和旋转锚分离。
- `robots/g1_sonic_visuals.py`
  - 支持 Isaac Sim 6 的 nested `Geometry` render instance；
  - 排除碰撞 instance，防止 PhysX articulation 失效。
- `isaaclab_compat.py`、`tasks/common_observations/g1_29dof_state.py`
  - 将 Isaac 6 运行时姿态从 `xyzw` 转为 Unitree DDS 所需的 `wxyz`；
  - 使用转换后的姿态计算 IMU 机体系加速度和角速度。
- `sim_main.py`、`action_provider/action_provider_sonic_dds.py`
  - 修正 Isaac 6 下的跌倒检测和姿态统计。

### `IsaacLab` fork

- `sim/converters/*`：适配 URDF importer 3.0；
- `sim/schemas/schemas.py`：支持嵌套刚体的 contact report；
- `devices/openxr/xr_cfg.py`、`xr_anchor_utils.py`：支持独立旋转锚；
- `devices/openxr/openxr_device.py`、`manus_vive.py`：在 XrCfg 与
  `SingleXFormPrim` 边界转换 `xyzw -> wxyz`。

## 4. 验证清单

### 静态检查

```bash
/home/nolovr/miniconda3/envs/isaaclab6/bin/python -m py_compile \
  tasks/g1_tasks/g1_29dof_dex3_sonic/g1_29dof_dex3_sonic_env_cfg.py \
  robots/g1_sonic_visuals.py

git diff --check
```

四元数矩阵应满足：

- identity 为单位矩阵；
- 桌子的 -90° yaw 只旋转 XY 平面，Z 轴保持 `(0, 0, 1)`；
- 不再有绕 X 轴 90°/180° 的场景补偿。

### 运行检查

重新启动后逐项确认：

1. 不再出现 `cannot import name 'acquire_urdf_interface'`；
2. G1 脚在下、头在上，初始 root 高度约 `0.76 m`；
3. PackingTable 桌面朝上，桌面约位于 `z=0.6996 m`；
4. 三个方块落在桌面上，而不是穿过桌面或飞离场景；
5. XR 中地面位于脚下，桌子与桌面窗口中方向一致；
6. 日志不出现 `Simulation view object is invalidated` 或 `prim deleted`。
7. `rt/lowstate` 中直立 pelvis 的四元数接近 `(w, x, y, z) = (1, 0, 0, 0)`，
   SONIC 启动后腿部 `rt/lowcmd` 持续更新且下半身能够跟随。

截至本文提交，URDF 导入、环境创建和静态旋转矩阵已经验证；最终四元数修复需要在
结束旧 Isaac 进程并重新启动后完成一次头显运行验收。

## 5. 排查原则

- 地面、机器人和桌子一起翻转：先检查 XR anchor，不要改物理场景；
- 只有机器人头朝下：检查机器人 `InitialStateCfg.rot` 的顺序；
- 桌子侧倒并进入地面：检查桌子的 yaw 是否误按 `wxyz` 填写；
- 配置修改完全没反应：确认旧进程已经退出，并确认 anchor prim 路径在当前 stage 存在；
- 材质应用后机器人消失或 tensor invalidated：检查是否误写了碰撞 instance。

不要用同时旋转机器人、桌子和 XR 的多重补偿来“看起来摆正”。物理世界始终保持
Z-up；模型布局和 XR 显示应分别在自己的坐标边界上修正。
