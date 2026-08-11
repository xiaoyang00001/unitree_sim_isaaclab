# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Runtime events for the warehouse conveyor scene.

legacy 驱动移植自 IsaacLab 分叉 feat/conveyor-loop-totes-wip 的
``pick_place/mdp/events.py``（17e71c0a0），保留其无 GPU→CPU 同步的张量写入逻辑。

流水线驱动：Surface 模式由场景配置选用纯视觉 ConveyorBelt
adapter，在 USD 组合阶段去掉原生物理，改由任务的不可见
kinematic proxy 带面托住并驱动物体。A/B 后端为：

* ``legacy``：按固定周期覆写筐的 root 线速度（保留历史行为）；
* ``surface_velocity``：在入料段碰撞面启用 Isaac Sim 5.1 的
  ``PhysxSurfaceVelocityAPI``，下游停止/抓取段保持静态摩擦面。

另含背景刚体的 kinematic 锁定，见 ``lock_background_rigid_bodies``。
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab.managers import ManagerTermBase
from isaaclab.sim.utils import clone
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, PhysxSchema, Usd, UsdPhysics

from . import conveyor_queue

# 带面判据的默认参数直接取自 conveyor_drive 的常量，避免第二处真源。
# ⚠️ ConveyorEventsCfg 只显式传 y_stop / y_recycle / y_respawn，**不传 y_range**，
#    所以下面的默认值就是运行时真正生效的判据；改 conveyor_drive 常量会自动跟随，
#    别在这里另写字面量造第二真源。
from .conveyor_drive import (
    BELT_TOP_Z,
    BELT_Y_MAX,
    BELT_Y_MIN,
    DEFAULT_Y_RECYCLE,
    DEFAULT_Y_RESPAWN,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


@clone
def spawn_surface_velocity_cuboid(
    prim_path: str,
    cfg: sim_utils.CuboidCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn a cuboid with a disabled PhysX surface-velocity API pre-authored.

    Isaac Sim 5.1's own ``CreateConveyorBelt`` command applies
    ``UsdPhysics.RigidBodyAPI``, ``UsdPhysics.CollisionAPI`` and
    ``PhysxSchema.PhysxSurfaceVelocityAPI`` before playback.  The ordinary
    Isaac Lab cuboid already supplies the rigid body and child collision; this
    wrapper adds the same surface API to the rigid-body root before the outer
    clone operation.  The startup event only changes existing attributes, so
    PhysX never needs to discover a newly applied schema after simulation has
    started.

    The API starts disabled and at zero velocity on every backend/robot ID.
    ``configure_conveyor_surface_velocity`` is the sole activation point.
    """

    # The outer @clone has already resolved prim_path to env_0.  Calling the
    # standard decorated spawner with that concrete path creates one source;
    # the outer decorator then copies the fully authored API to other envs.
    prim = sim_utils.spawn_cuboid(
        prim_path,
        cfg,
        translation=translation,
        orientation=orientation,
        **kwargs,
    )
    surface_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
    surface_api.CreateSurfaceVelocityEnabledAttr().Set(False)
    surface_api.CreateSurfaceVelocityLocalSpaceAttr().Set(False)
    surface_api.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    surface_api.CreateSurfaceAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    return prim


def configure_conveyor_surface_velocity(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    prim_name: str = "ConveyorCollider",
    velocity_y: float = -0.3,
    enabled: bool = False,
    local_robot_id: int = 0,
):
    """Enable the moving collision surface on the fixed ID=1 authority only.

    The velocity is expressed in world coordinates because the configured
    collider has identity rotation and the task's conveyor direction is fixed
    at ``-Y``.  All non-authority/mirror/legacy cases explicitly leave the
    pre-authored API disabled at zero, preventing contact drive and legacy root
    velocity writes from running together.
    """

    stage = get_current_stage()
    if stage is None:
        raise RuntimeError("无法配置流水线 Surface Velocity：当前 USD Stage 不存在")

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device, dtype=torch.long)

    authority_enabled = enabled and local_robot_id == 1 and abs(velocity_y) >= 1e-8
    for env_id in env_ids.tolist():
        prim_path = f"/World/envs/env_{env_id}/{prim_name}"
        prim = stage.GetPrimAtPath(prim_path)
        if not (prim and prim.IsValid()):
            raise RuntimeError(f"流水线驱动碰撞面不存在：{prim_path}")

        rigid_api = UsdPhysics.RigidBodyAPI(prim)
        if not rigid_api:
            raise RuntimeError(f"Surface Velocity 目标不是刚体：{prim_path}")
        if authority_enabled and not bool(rigid_api.GetKinematicEnabledAttr().Get()):
            raise RuntimeError(f"Surface Velocity 目标必须是 kinematic 刚体：{prim_path}")

        surface_api = PhysxSchema.PhysxSurfaceVelocityAPI(prim)
        if not surface_api:
            # 防守性回退：正常路径已在 spawn 时预先 Apply，不应走到这里。
            surface_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
        surface_api.GetSurfaceVelocityLocalSpaceAttr().Set(False)
        surface_api.GetSurfaceVelocityAttr().Set(
            Gf.Vec3f(0.0, velocity_y if authority_enabled else 0.0, 0.0)
        )
        surface_api.GetSurfaceAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        surface_api.GetSurfaceVelocityEnabledAttr().Set(authority_enabled)

        state = f"{velocity_y:.3f} m/s 沿世界 -Y" if authority_enabled else "关闭"
        print(f"[conveyor_event] surface_velocity: {prim_path} -> {state}")


def lock_background_rigid_bodies(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    prim_names: tuple[str, ...] = (),
    parent_path: str = "Background/ConveyorBelt",
    kinematic: bool = True,
):
    """把背景 USD 里的刚体切成 kinematic，钉在原始摆位上。

    背景里的分拣料箱有两种状态：``blue_sorting_bin_01`` 在 USD 里已是 kinematic
    （实测 z 恒 0.4355 不动），而 ``blue_sorting_bin_02`` 是**动态**刚体——开局
    悬空约 1.5 cm，仿真一起步就下沉、回弹后落在 packing table 面上（z 0.3918 →
    0.3737），y 也漂几毫米，而且之后会被机器人撞飞。这里统一锁成 kinematic。

    只改 ``kinematicEnabled``，不动 collision/visual：料箱仍是可碰撞的实体，只是
    不再受重力与外力驱动。
    """

    if not prim_names:
        return

    stage = get_current_stage()
    if stage is None:
        return

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device, dtype=torch.long)

    for env_id in env_ids.tolist():
        for name in prim_names:
            root = stage.GetPrimAtPath(f"/World/envs/env_{env_id}/{parent_path}/{name}")
            if not (root and root.IsValid()):
                print(f"[conveyor_event] lock_background: prim 不存在 {name}")
                continue

            locked = 0
            for prim in Usd.PrimRange(root):
                if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    continue
                attr = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr()
                if not attr:
                    attr = UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(kinematic)
                attr.Set(kinematic)
                locked += 1

            if locked == 0:
                print(f"[conveyor_event] lock_background: {name} 下没有刚体")
            else:
                print(f"[conveyor_event] lock_background: {name} 锁定 {locked} 个刚体 kinematic={kinematic}")


def reset_scene_mirror_safe(env):
    """镜像端（ID=2）安全版整场景复位：kinematic 镜像物体只写位姿、不写速度。

    基座 SimpleEventManager 注册的 mdp.reset_scene_to_default 会向所有刚体写完整
    root state（含速度）。ID=2 侧的同步物体是 kinematic，CPU pipeline 下速度写入
    落到逐 body PhysX 调用被拒（Body must be non-kinematic!），每次复位刷 ~14 条
    错误、累计撞满 1000 条上限后整个仿真被 PhysX 掐停。此函数与
    reset_scene_to_default(reset_joint_targets=True) 行为对齐，仅跳过 kinematic
    刚体的速度写入（其位姿反正下一帧就被权威 scene_state 帧接管）。
    """

    from .zmq_scene_sync import _spawn_is_kinematic

    env_ids = torch.arange(env.scene.num_envs, device=env.device, dtype=torch.long)
    origins = env.scene.env_origins[env_ids]

    for articulation in env.scene.articulations.values():
        default_root = articulation.data.default_root_state[env_ids].clone()
        default_root[:, :3] += origins
        articulation.write_root_pose_to_sim(default_root[:, :7], env_ids=env_ids)
        articulation.write_root_velocity_to_sim(default_root[:, 7:], env_ids=env_ids)
        joint_pos = articulation.data.default_joint_pos[env_ids].clone()
        joint_vel = articulation.data.default_joint_vel[env_ids].clone()
        articulation.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
        articulation.set_joint_position_target(joint_pos, env_ids=env_ids)
        articulation.set_joint_velocity_target(joint_vel, env_ids=env_ids)

    for rigid_object in env.scene.rigid_objects.values():
        default_root = rigid_object.data.default_root_state[env_ids].clone()
        default_root[:, :3] += origins
        rigid_object.write_root_pose_to_sim(default_root[:, :7], env_ids=env_ids)
        if not _spawn_is_kinematic(rigid_object):
            rigid_object.write_root_velocity_to_sim(default_root[:, 7:], env_ids=env_ids)


def drive_belt_boxes_on_conveyor(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    object_names: tuple[str, ...] = (),
    velocity_y: float = -0.3,
    enabled: bool = True,
    belt_top_z: float = BELT_TOP_Z,
    z_tolerance: float = 0.15,
    x_range: tuple[float, float] = (-6.17, -5.07),
    # 带面判据跟 conveyor_drive 常量走，避免第二真源。
    y_range: tuple[float, float] = (BELT_Y_MIN, BELT_Y_MAX),
    y_stop: float | None = None,
    queue_gap: float = 0.07,
    half_lengths: tuple[float, ...] = (),
    # —— 入口弯道（endless intake）两段式驱动 ——
    # path_enabled=True 时按 L 形路径驱动（西拐）：支线 +X → 圆角弧 → 主线 -Y；
    # 排队坐标从裸 y 换成沿路径距离 s，on_belt 判据换成主线矩形 ∪ extra_rects 并集。
    # 参数由 env cfg 从 endless_intake 常量取值传入；默认值维持纯 -Y 直线行为。
    path_enabled: bool = False,
    path_corner_center: tuple[float, float] = (0.0, 0.0),
    path_radius: float = 1.40,
    path_s_origin_x: float = 0.0,
    extra_rects: tuple[tuple[float, float, float, float], ...] = (),
    # —— 工位箱平面偏离门控（仅停止式 legacy 后端使用） ——
    departure_completed: torch.Tensor | None = None,
    arrived: torch.Tensor | None = None,
    lifted: torch.Tensor | None = None,
    belt_dwell: torch.Tensor | None = None,
    release_settle_steps: int = 25,
):
    """把纸箱队列沿带面送到工位，工位箱偏出流水线通道后放行下一格。

    与 ``drive_totes_on_conveyor``（每个筐各自独立判断 ``y > y_stop``）的差别只在
    停止线怎么算：队首停在工位，后车原地保持队形；工位箱只被抬高、根位置 XY 仍在
    流水线通道上方时继续停线，横向偏出通道后下一格立即补位。判据本身在
    ``conveyor_queue``（无 Isaac 依赖、有单测）：直线形态用
    ``queue_drive_mask``（裸 y），弯道形态用 ``path_progress`` +
    ``queue_drive_mask_along_path``（沿路径距离 s），完整推导见各自 docstring。
    本函数只负责读位置、写速度。

    “还躺在带上”与“仍在流水线上方”使用两张 mask：前者包含高度窗口，只决定是否给
    当前箱子写输送速度；后者只看根位置的 XY 投影，决定是否继续按住整带。偏出完成位
    按箱锁存，避免抓取轨迹回摆或边界抖动让已经启动的流水线中途反悔。
    ``DriveBeltBoxesOnConveyor.reset`` 在整场景/F12/DDS 复位时按 env 清零锁存；状态
    只存在物体权威端，不需要增加双机同步字段。

    ``arrived`` 是与 ``departure_completed`` 同形的到位锁存：箱子首次越过工位停止
    线即置位。抱取时机器人把工位箱推挤回停止线上游 1-2 cm 是常态，裸
    ``lead > stop`` 比较会当场误开整带并把手里的箱子往回拖；锁存后"工位被占用"
    直到该箱平面偏出流水线（completed）才解除——启停节拍在一次抱取内只翻转一次。
    已放行（completed）的箱子同时从队首判定与驱动输出中剔除，手臂回摆把它带回
    带面上方时不会再按停整带或被重新拖走。

    ``lifted`` / ``belt_dwell`` 是悬空压停锁存及其驻留计数（推导见
    ``update_lift_hold_latch``）：上游截抓的箱子既没有 arrived 也没有 completed，
    携行高度又恰好横跨 z 窗阈值，瞬时判会让整带跟着手臂颠簸启停；锁存化后抬离
    带面即稳定压停，回带连续驻留 ``release_settle_steps`` 步（默认 25 步 = 0.5 s
    @50Hz tick）才解除。同一驻留判据同时给 completed 提供场内解除
    （``release_resettled_boxes``）：脱手掉回带上的已放行箱重新入队，不再变成
    堵死队列的呆滞障碍。已知限制：箱子被抱住但仍贴在带面 z 窗内的瞬间（尚未
    抬起），几何判据无法区分"被抱住"与"自由躺放"，若恰逢整带此刻放行，该箱在
    抬离前仍会被短暂写输送速度——这是纯几何判据的固有盲区，靠缩短贴带抱取
    时间缓解。

    弯道形态下箱子**不旋转**（保持世界朝向）：速度矢量沿路径航向逐步更新，
    姿态不动——双机同步/镜像端零改动，观感是箱子"平移着拐弯"。

    ``y_stop=None``（``ISAACLAB_CONVEYOR_Y_STOP<=0`` 的循环模式）下退化为只受前车
    约束、一路流到带尾。本函数**不做回收**——与塑料筐的 ``drive_totes_on_conveyor``
    不同，那边自带 loop 分支会把筐传回入料端。纸箱的回收由 env cfg 在循环模式下
    额外挂上的 ``recycle_totes_on_surface_conveyor`` 负责
    （见 ``CONVEYOR_BELT_BOX_RECYCLE_ENABLED``）；两者拆开是因为回收要写位姿，
    而写位姿和逐步写速度混在一个函数里会让排队判据难以推理。

    性能注意：与 ``drive_totes_on_conveyor`` 同一条铁律——本函数每个物理步都跑，
    **不允许任何 GPU→CPU 同步**（``.item()`` / ``.any()`` / ``if tensor``）。
    ``path_enabled`` 是 Python 布尔，分支不引入同步。
    """

    if not enabled or abs(velocity_y) < 1e-8 or not object_names:
        return
    if len(half_lengths) != len(object_names):
        raise ValueError(
            "half_lengths 必须与 object_names 一一对应："
            f"{len(half_lengths)} vs {len(object_names)}"
        )

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device, dtype=torch.long)
    if len(env_ids) == 0:
        return

    origins = env.scene.env_origins[env_ids]
    objects = [env.scene[name] for name in object_names]

    # (N, E, 3)：N 个箱子 × E 个 env 的 env-local 位置。
    pos_local = torch.stack(
        [obj.data.root_pos_w[env_ids] - origins for obj in objects], dim=0
    )
    corridor_rects = extra_rects if path_enabled else ()
    in_conveyor_corridor = conveyor_queue.in_conveyor_corridor_mask(
        pos_local,
        x_range=x_range,
        y_range=y_range,
        extra_rects=corridor_rects,
    )  # (N, E)，纯 XY；抬高后仍为真
    on_belt = conveyor_queue.on_belt_mask(
        pos_local,
        belt_top_z=belt_top_z,
        z_tolerance=z_tolerance,
        x_range=x_range,
        y_range=y_range,
        extra_rects=corridor_rects,
    )  # (N, E)
    if path_enabled:
        ss, hx, hy = conveyor_queue.path_progress(
            pos_local,
            corner_center=path_corner_center,
            radius=path_radius,
            s_origin_x=path_s_origin_x,
        )
        # 工位在主线段上：y_stop → s_stop 是标量换算，不进张量。
        # ⚠️ 西拐口径：s_arc_start = cx − s_origin_x（东拐是 s_origin_x − cx）。
        if y_stop is None:
            s_stop = None
        else:
            s_arc_end = (path_corner_center[0] - path_s_origin_x) + path_radius * 1.5707963267948966
            s_stop = s_arc_end + (path_corner_center[1] - y_stop)
    else:
        ss = hx = hy = None
        s_stop = None

    transfer_complete = None
    completed = None
    if y_stop is not None:
        if departure_completed is None or arrived is None or lifted is None or belt_dwell is None:
            raise ValueError("停止式纸箱队列必须由有状态事件 term 提供平面偏离/到位/悬空锁存")
        completed = conveyor_queue.update_departure_completion_latch(
            departure_completed[:, env_ids],
            in_conveyor_corridor,
        )
        if path_enabled:
            arrived_now = conveyor_queue.update_arrival_latch_along_path(
                arrived[:, env_ids], ss, s_stop=s_stop
            )
        else:
            arrived_now = conveyor_queue.update_arrival_latch(
                arrived[:, env_ids], pos_local[..., 1], y_stop=y_stop
            )
        lifted_now, dwell_now, settled = conveyor_queue.update_lift_hold_latch(
            lifted[:, env_ids],
            belt_dwell[:, env_ids],
            on_belt,
            in_conveyor_corridor,
            completed,
            release_steps=release_settle_steps,
        )
        completed, arrived_now = conveyor_queue.release_resettled_boxes(
            completed, arrived_now, settled
        )
        departure_completed[:, env_ids] = completed
        arrived[:, env_ids] = arrived_now
        lifted[:, env_ids] = lifted_now
        belt_dwell[:, env_ids] = dwell_now
        # 两道门相与：既没有"悬空未离线"的箱子，也没有"到位未离线"的箱子。
        transfer_complete = conveyor_queue.lift_hold_gate(
            lifted_now
        ) & conveyor_queue.belt_release_gate(arrived_now, completed)  # (E,)
    # (N, 1)：广播到 (N, E)。箱型通常只有十余个，每周期重建此小张量成本可忽略。
    half_len = pos_local.new_tensor(half_lengths).unsqueeze(-1)
    speed = -velocity_y  # velocity_y 约定为负（-Y 方向）；沿路径恒速取其模。

    if path_enabled:
        drive = conveyor_queue.queue_drive_mask_along_path(
            ss,
            on_belt,
            half_len,
            s_stop=s_stop,
            queue_gap=queue_gap,
            transfer_complete=transfer_complete,
            completed=completed,
        )  # (N, E)
    else:
        drive = conveyor_queue.queue_drive_mask(
            pos_local[..., 1],
            on_belt,
            half_len,
            y_stop=y_stop,
            queue_gap=queue_gap,
            transfer_complete=transfer_complete,
            completed=completed,
        )  # (N, E)

    for index, obj in enumerate(objects):
        # 只覆写水平速度，Z 留给重力/接触——与 drive_totes_on_conveyor 一致，
        # 避免把箱子按进碰撞板里。
        vel = obj.data.root_vel_w[env_ids].clone()
        drive_i = drive[index]
        if path_enabled:
            vel[:, 0] = torch.where(drive_i, speed * hx[index], vel[:, 0])
            vel[:, 1] = torch.where(drive_i, speed * hy[index], vel[:, 1])
        else:
            vel[:, 0] = torch.where(drive_i, torch.zeros_like(vel[:, 0]), vel[:, 0])
            vel[:, 1] = torch.where(drive_i, torch.full_like(vel[:, 1], velocity_y), vel[:, 1])
        obj.write_root_velocity_to_sim(vel, env_ids=env_ids)


class DriveBeltBoxesOnConveyor(ManagerTermBase):
    """带 per-box 平面偏离/到位/悬空三锁存的纸箱驱动事件 term。"""

    def __init__(self, cfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        num_objects = len(cfg.params.get("object_names", ()))
        self._departure_completed = torch.zeros(
            (num_objects, env.num_envs),
            device=env.device,
            dtype=torch.bool,
        )
        self._arrived = torch.zeros_like(self._departure_completed)
        self._lifted = torch.zeros_like(self._departure_completed)
        self._belt_dwell = torch.zeros(
            (num_objects, env.num_envs),
            device=env.device,
            dtype=torch.int32,
        )

    def reset(self, env_ids=None) -> None:
        ids = slice(None) if env_ids is None else env_ids
        self._departure_completed[:, ids] = False
        self._arrived[:, ids] = False
        self._lifted[:, ids] = False
        self._belt_dwell[:, ids] = 0

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        object_names: tuple[str, ...] = (),
        velocity_y: float = -0.3,
        enabled: bool = True,
        belt_top_z: float = BELT_TOP_Z,
        z_tolerance: float = 0.15,
        x_range: tuple[float, float] = (-6.17, -5.07),
        y_range: tuple[float, float] = (BELT_Y_MIN, BELT_Y_MAX),
        y_stop: float | None = None,
        queue_gap: float = 0.07,
        half_lengths: tuple[float, ...] = (),
        path_enabled: bool = False,
        path_corner_center: tuple[float, float] = (0.0, 0.0),
        path_radius: float = 1.40,
        path_s_origin_x: float = 0.0,
        extra_rects: tuple[tuple[float, float, float, float], ...] = (),
        release_settle_steps: int = 25,
    ) -> None:
        drive_belt_boxes_on_conveyor(
            env,
            env_ids,
            object_names=object_names,
            velocity_y=velocity_y,
            enabled=enabled,
            belt_top_z=belt_top_z,
            z_tolerance=z_tolerance,
            x_range=x_range,
            y_range=y_range,
            y_stop=y_stop,
            queue_gap=queue_gap,
            half_lengths=half_lengths,
            path_enabled=path_enabled,
            path_corner_center=path_corner_center,
            path_radius=path_radius,
            path_s_origin_x=path_s_origin_x,
            extra_rects=extra_rects,
            departure_completed=self._departure_completed,
            arrived=self._arrived,
            lifted=self._lifted,
            belt_dwell=self._belt_dwell,
            release_settle_steps=release_settle_steps,
        )


def drive_totes_on_conveyor(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    object_names: tuple[str, ...] = ("cart2_tote1", "cart2_tote2"),
    velocity_y: float = -0.3,
    enabled: bool = True,
    belt_top_z: float = BELT_TOP_Z,
    z_tolerance: float = 0.15,
    x_range: tuple[float, float] = (-6.17, -5.07),
    y_range: tuple[float, float] = (BELT_Y_MIN, BELT_Y_MAX),
    y_stop: float | None = None,
    y_recycle: float = DEFAULT_Y_RECYCLE,
    y_respawn: float = DEFAULT_Y_RESPAWN,
    respawn_z: float = 0.775,
):
    """把塑料筐沿流水线 -Y 方向匀速送走，到工位停住（或到出料端传回入料端）。

    每个 interval tick 只对"确实还躺在滚轮面上"的筐覆写水平速度（Z 速度保留给
    重力/接触，避免把筐按在碰撞板里）。判定用带面几何：

    * ``belt_top_z`` ± ``z_tolerance``：筐原点在底面，静止时 z≈0.775。被机器人拎起
      或掉到地上就超出窗口 → 立即停止驱动，不会把抓在手里的筐硬拖走。
    * ``x_range`` / ``y_range``：滚轮可用带面（略放宽于碰撞板 x[-6.07,-5.17]）。

    ``y_stop`` 是机器人工位：筐一旦流到该 y 就不再驱动，靠 μd=0.6 的动摩擦自然
    停住（约 5 mm 滑行）。这里刻意**不写零速度**——每步硬写零会和机器人的抓取动作
    对抗，而摩擦本身足够大，停得又快又稳。设为 None 则不停、一路流到出料端。

    到达 ``y_recycle`` 后瞬移回 ``y_respawn``（保持各自 X 车道、清零速度）。设了
    ``y_stop`` 时筐停在工位、永远到不了出料端，回收逻辑整段跳过。坐标全部是相对
    env origin 的局部系，与场景配置里的数值同一套。

    性能注意：本函数每个物理步都跑，**必须避免任何 GPU→CPU 同步**。早期版本用
    ``if not on_belt.any(): continue`` 之类做提前返回，每步两个筐最多 6 次同步，
    实测让整个 env.step 从 ~120 ms 涨到 ~160 ms（+30%）。现在一律用 ``torch.where``
    做无分支组装、每个筐只发一次写入。判据里只有 Python 标量（``y_stop is None``）
    才允许走分支。
    """

    if not enabled or abs(velocity_y) < 1e-8 or not object_names:
        return

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device, dtype=torch.long)

    if len(env_ids) == 0:
        return

    origins = env.scene.env_origins[env_ids]

    for object_name in object_names:
        obj = env.scene[object_name]

        pos_local = obj.data.root_pos_w[env_ids] - origins

        on_belt = conveyor_queue.on_belt_mask(
            pos_local,
            belt_top_z=belt_top_z,
            z_tolerance=z_tolerance,
            x_range=x_range,
            y_range=y_range,
        )

        if y_stop is None:
            # 纯循环模式：到出料端就传回入料端。y_stop 是 Python 标量，走分支不引入同步。
            recycle = on_belt & (pos_local[:, 1] <= y_recycle)
            pose = obj.data.root_state_w[env_ids, :7].clone()
            pose[:, 1] = torch.where(recycle, origins[:, 1] + y_respawn, pose[:, 1])
            pose[:, 2] = torch.where(recycle, origins[:, 2] + respawn_z, pose[:, 2])
            obj.write_root_pose_to_sim(pose, env_ids=env_ids)
            drive = on_belt & ~recycle
        else:
            # 已到工位的筐不再驱动，交给摩擦停住。
            recycle = None
            drive = on_belt & (pos_local[:, 1] > y_stop)

        vel = obj.data.root_vel_w[env_ids].clone()
        if recycle is not None:
            # 回收的筐清零全部 6 个分量，避免带着旧速度回到入料端。
            keep = (~recycle).unsqueeze(-1)
            vel = vel * keep
        vel[:, 0] = torch.where(drive, torch.zeros_like(vel[:, 0]), vel[:, 0])
        vel[:, 1] = torch.where(drive, torch.full_like(vel[:, 1], velocity_y), vel[:, 1])
        obj.write_root_velocity_to_sim(vel, env_ids=env_ids)


def recycle_totes_on_surface_conveyor(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    object_names: tuple[str, ...] = ("cart2_tote1", "cart2_tote2"),
    enabled: bool = False,
    belt_top_z: float = BELT_TOP_Z,
    z_tolerance: float = 0.15,
    x_range: tuple[float, float] = (-6.17, -5.07),
    y_range: tuple[float, float] = (BELT_Y_MIN, BELT_Y_MAX),
    y_recycle: float = DEFAULT_Y_RECYCLE,
    y_respawn: float = DEFAULT_Y_RESPAWN,
    respawn_z: float = 0.775,
    respawn_x: float | None = None,
):
    """Recycle totes in full-length surface-velocity loop mode without driving them.

    The stopped workflow has a static downstream segment and never enables this
    event.  When ``ISAACLAB_CONVEYOR_Y_STOP<=0`` selects loop mode, PhysX contact
    motion remains the only drive; this event merely teleports a tote that
    reaches the outfeed back to the infeed and clears its velocity.

    ``respawn_x``: 默认 None 保持各自 X 车道（历史行为）。入口弯道（endless
    intake）方案把回生点搬到 X 支线最深处（世界 (-16.66, 20.0534)，藏在货架
    排 B 后面），x 必须跟着写，否则箱子会回生在主车道北端的弯道体内部。

    Only environments selected by the recycle mask are written.  Objects still
    travelling on the belt are left entirely to PhysX contact motion instead of
    being needlessly woken by a periodic pose/velocity rewrite.
    """

    if not enabled or not object_names:
        return

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device, dtype=torch.long)
    if len(env_ids) == 0:
        return

    origins = env.scene.env_origins[env_ids]
    for object_name in object_names:
        obj = env.scene[object_name]
        pos_local = obj.data.root_pos_w[env_ids] - origins
        on_belt = conveyor_queue.on_belt_mask(
            pos_local,
            belt_top_z=belt_top_z,
            z_tolerance=z_tolerance,
            x_range=x_range,
            y_range=y_range,
        )
        recycle = on_belt & (pos_local[:, 1] <= y_recycle)

        recycle_env_ids = env_ids[recycle]
        if len(recycle_env_ids) == 0:
            continue

        pose = obj.data.root_state_w[recycle_env_ids, :7].clone()
        if respawn_x is not None:
            pose[:, 0] = origins[recycle, 0] + respawn_x
        pose[:, 1] = origins[recycle, 1] + y_respawn
        pose[:, 2] = origins[recycle, 2] + respawn_z
        obj.write_root_pose_to_sim(pose, env_ids=recycle_env_ids)

        velocity = torch.zeros_like(obj.data.root_vel_w[recycle_env_ids])
        obj.write_root_velocity_to_sim(velocity, env_ids=recycle_env_ids)
