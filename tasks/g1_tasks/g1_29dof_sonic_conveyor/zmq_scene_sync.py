# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""双机 ZMQ 场景同步（对等/混合权威模式）。

移植自 IsaacLab 分叉 feat/conveyor-loop-totes-wip 的 ``pick_place/zmq_object_sync.py``
（tip 17e71c0a0，含 14f087ce8 kinematic 只写位姿、17e71c0a0 机器人速度清零两个修复），
并按 unitree_sim_isaaclab 双机各控一台 G1 的形态改成**对等模式**：

* 每台机器绑定自己的 PUB socket、SUB 连到对端，各自发布**本机权威**的状态：
  两侧都发布自己的机器人；场景物体只有物体权威端（ID=1）发布。
* 订阅侧只应用**非本机权威**的状态：对端机器人写进本地 ``peer_robot`` 镜像体，
  物体状态只在 ID=2（镜像侧）应用。
* 原版每物理步（200 Hz）发一帧；这里加了 ``publish_decimation``（默认 4 → 50 Hz），
  与 env step 同频，镜像效果无损、省 3/4 的 JSON 序列化开销。

复位事件通道 ``ZmqEnvResetSyncAction`` 保持源版单向语义：ID=1 是复位权威
（publisher），ID=2 跟随。两个 topic 共用 ID=1 的 PUB socket。

硬约束：num_envs=1；articulation 两端关节序逐字一致（joint_order_hash 校验）；
纯显示接收端用帧内 joint_names 校验 43 个名称并按名映射；两端物体清单与命名一致。
"""

from __future__ import annotations

import hashlib
import json
import logging
import time
import uuid
from typing import TYPE_CHECKING, Any

import torch

try:
    import zmq
except ModuleNotFoundError:
    zmq = None

from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs.manager_based_env import ManagerBasedEnv

logger = logging.getLogger(__name__)


def _spawn_is_kinematic(rigid_object) -> bool:
    """Whether this scene object was spawned as a kinematic body.

    镜像侧（ID=2）的同步物体在 spawn cfg 里被翻成 kinematic，此时不能再写速度。
    权威端同名物体是普通动态刚体，仍然要写完整 root state。
    """

    rigid_props = getattr(getattr(rigid_object.cfg, "spawn", None), "rigid_props", None)
    return bool(getattr(rigid_props, "kinematic_enabled", False))


class ZmqPubSocketManager:
    """Share one PUB socket across the scene-state and reset topics."""

    _context = None
    _sockets = {}

    @classmethod
    def get_pub_socket(cls, endpoint: str, send_hwm: int):
        if zmq is None:
            return None
        if endpoint not in cls._sockets:
            if cls._context is None:
                cls._context = zmq.Context()
            sock = cls._context.socket(zmq.PUB)
            sock.setsockopt(zmq.SNDHWM, max(1, int(send_hwm)))
            sock.setsockopt(zmq.LINGER, 0)
            sock.bind(endpoint)
            cls._sockets[endpoint] = sock
            logger.info(
                "[ZMQ Shared PUB] Publisher bound to endpoint %s (SNDHWM=%d, LINGER=0)",
                endpoint,
                max(1, int(send_hwm)),
            )
        return cls._sockets[endpoint]


@configclass
class ZmqSceneStateSyncActionCfg(ActionTermCfg):
    """Configuration for the peer-to-peer scene-state synchronization term."""

    class_type: type = None  # set in __post_init__

    bind_endpoint: str = ""
    """本机 PUB socket 绑定端点（空 = 不发布）。"""

    connect_endpoint: str = ""
    """对端 PUB socket 的连接端点（空 = 不订阅）。"""

    topic: str = "scene_state"
    """PUB/SUB topic for the unified scene frame."""

    local_sender_name: str = "robot_1"
    """本机在协议里的全局身份（robot_1 / robot_2），写进每帧的 sender 字段。"""

    publish_robots: dict[str, str] = {}
    """本机发布的机器人：{全局名: 本地场景实体名}，例如 {"robot_1": "robot"}。"""

    apply_robots: dict[str, str] = {}
    """从对端帧应用的机器人：{全局名: 本地场景实体名}，例如 {"robot_2": "peer_robot"}。"""

    apply_visual_robots: dict[str, str] = {}
    """纯显示镜像：{全局名: USD 根 Prim 路径}。与 ``apply_robots`` 互斥。

    该路径直接由无 PhysX 的 FK/Xform 后端更新，不会向 Isaac Lab 场景注册
    Articulation、RigidObject、collider、actuator 或 contact reporter。
    """

    visual_robot_joint_names: tuple[str, ...] = ()
    """纯显示后端接受的完整关节名称集合；wire order 可由每帧 ``joint_names`` 决定。"""

    publish_object_names: tuple[str, ...] = ()
    """本机发布的刚体清单（仅物体权威端非空；全局名 = 场景实体名）。"""

    apply_object_names: tuple[str, ...] = ()
    """从对端帧应用的刚体清单（仅镜像侧非空）。"""

    publish_decimation: int = 4
    """每 N 次 apply_actions 发一帧。物理 200 Hz、decimation=4 → 50 Hz 发布。"""

    send_hwm: int = 3
    """Publisher high-water mark."""

    receive_hwm: int = 3
    """Subscriber high-water mark before the latest-frame drain loop."""

    stale_timeout_s: float = 0.5
    """Seconds without a frame before the mirror reports the stream as stale."""

    stale_log_interval_s: float = 2.0
    """Minimum interval between stale warnings."""

    reset_gate_timeout_s: float = 2.0
    """reset_id 门控的最长等待秒数。超时放行并告警——否则镜像端错过一次复位事件
    （如两次复位间隔太近、事件包全部丢失）就会无限期拒收携带新 reset_id 的帧，
    表现为镜像整体冻结 + 误报 Stream stale。"""

    external_pump: bool = False
    """True = 收发不挂 env.step（apply_actions 变 no-op），由宿主主循环每迭代调一次
    pump()。SONIC 锁步下 deploy 停发 lowcmd 时 env.step 停摆，ActionTerm 挂载的
    同步会随之冻结；主循环挂载不受影响（方案 b）。"""

    def __post_init__(self):
        self.class_type = ZmqSceneStateSyncAction


class ZmqSceneStateSyncAction(ActionTerm):
    """对等场景同步：发布本机权威状态帧，同时应用对端权威状态帧。"""

    cfg: ZmqSceneStateSyncActionCfg

    def __init__(self, cfg: ZmqSceneStateSyncActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._action_dim = 0
        self._raw_actions = torch.zeros((self.num_envs, 0), device=self.device)
        self._processed_actions = torch.zeros_like(self._raw_actions)
        self._export_IO_descriptor = False

        self._publish_robots = {
            global_name: self._env.scene[entity] for global_name, entity in dict(cfg.publish_robots).items()
        }
        self._apply_robots = {
            global_name: self._env.scene[entity] for global_name, entity in dict(cfg.apply_robots).items()
        }
        duplicate_apply_names = set(cfg.apply_robots).intersection(cfg.apply_visual_robots)
        if duplicate_apply_names:
            raise ValueError(
                "scene sync robot cannot use articulation and visual_lod backends together: "
                f"{sorted(duplicate_apply_names)}"
            )
        self._apply_visual_robots = {}
        if cfg.apply_visual_robots:
            from .peer_visual_lod import UsdVisualLodMirror, validate_joint_order

            validate_joint_order(cfg.visual_robot_joint_names)
            self._apply_visual_robots = {
                global_name: UsdVisualLodMirror(str(root_prim_path))
                for global_name, root_prim_path in dict(cfg.apply_visual_robots).items()
            }
        self._publish_objects = {name: self._env.scene[name] for name in cfg.publish_object_names}
        self._apply_objects = {name: self._env.scene[name] for name in cfg.apply_object_names}
        self._object_is_kinematic = {
            name: _spawn_is_kinematic(rigid_object) for name, rigid_object in self._apply_objects.items()
        }

        self._publish_enabled = bool(cfg.bind_endpoint) and bool(self._publish_robots or self._publish_objects)
        self._apply_enabled = bool(cfg.connect_endpoint) and bool(
            self._apply_robots or self._apply_visual_robots or self._apply_objects
        )

        self._publisher_session = uuid.uuid4().hex
        self._publisher_frame_id = 0
        self._publisher_reset_id = f"{self._publisher_session}:initial"
        self._publish_decimation = max(1, int(cfg.publish_decimation))
        self._publish_tick = 0
        self._last_session: str | None = None
        self._last_frame_id = -1
        self._active_reset_id: str | None = None
        self._expected_reset_id: str | None = None
        self._expected_reset_set_time = 0.0
        self._received_first_frame = False
        self._missing_robot_warned: set[str] = set()
        self._subscriber_start_time = time.monotonic()
        self._last_receive_time: float | None = None
        self._last_stale_warning_time = 0.0
        self._stale_reported = False

        # articulation 两端（乃至本机 robot 与 peer_robot 镜像）必须共用同一
        # 关节序，哈希不符整帧拒收。纯显示 viewer 没有镜像 articulation，改用
        # 帧内 joint_names 恢复 wire order，并校验完整名称集合与哈希。
        all_robots = {**self._publish_robots, **self._apply_robots}
        joint_orders = [tuple(robot.joint_names) for robot in all_robots.values()]
        if joint_orders:
            if any(order != joint_orders[0] for order in joint_orders[1:]):
                raise RuntimeError(
                    "scene sync requires the local robot and the peer mirror to share one fixed joint order"
                )
            self._joint_count = len(joint_orders[0])
            self._joint_names = joint_orders[0]
            self._joint_order_hash = hashlib.sha256("\0".join(joint_orders[0]).encode("utf-8")).hexdigest()[:16]
            if self._apply_visual_robots:
                from .peer_visual_lod import validate_joint_order

                validate_joint_order(self._joint_names)
        elif self._apply_visual_robots:
            # Pure viewer: no local articulation exists from which to discover
            # wire order.  The publisher includes joint_names; here we only pin
            # the expected 43-name set and validate each received order/hash.
            from .peer_visual_lod import validate_joint_order

            self._joint_names = None
            self._joint_count = len(validate_joint_order(cfg.visual_robot_joint_names))
            self._joint_order_hash = ""
        else:
            self._joint_count = 0
            self._joint_names = ()
            self._joint_order_hash = ""

        self._pub_socket = None
        self._sub_context = None
        self._sub_socket = None
        self.topic = str(cfg.topic).encode("utf-8")

        if self.num_envs != 1:
            logger.error("[ZMQ Scene Sync] Only num_envs=1 is supported; disabling scene synchronization")
            self._publish_enabled = False
            self._apply_enabled = False
            return
        if not (self._publish_enabled or self._apply_enabled):
            logger.info("[ZMQ Scene Sync] Both directions empty; scene synchronization disabled")
            return
        if zmq is None:
            logger.error("[ZMQ Scene Sync] pyzmq is not installed; disabling scene synchronization")
            self._publish_enabled = False
            self._apply_enabled = False
            return

        try:
            if self._publish_enabled:
                self._pub_socket = ZmqPubSocketManager.get_pub_socket(str(cfg.bind_endpoint), cfg.send_hwm)
            if self._apply_enabled:
                self._sub_context = zmq.Context()
                self._sub_socket = self._sub_context.socket(zmq.SUB)
                self._sub_socket.setsockopt(zmq.RCVHWM, max(1, int(cfg.receive_hwm)))
                self._sub_socket.setsockopt(zmq.LINGER, 0)
                self._sub_socket.setsockopt(zmq.SUBSCRIBE, self.topic)
                self._sub_socket.connect(str(cfg.connect_endpoint))
            logger.info(
                "[ZMQ Scene Sync] Peer ready sender=%s publish=%s(bind=%s, every %d ticks) "
                "apply=%s(connect=%s) joint_count=%d joint_hash=%s",
                cfg.local_sender_name,
                sorted(self._publish_robots) + sorted(self._publish_objects),
                cfg.bind_endpoint or "-",
                self._publish_decimation,
                sorted(self._apply_robots) + sorted(self._apply_visual_robots) + sorted(self._apply_objects),
                cfg.connect_endpoint or "-",
                self._joint_count,
                self._joint_order_hash,
            )
        except Exception as exc:
            logger.error(
                "[ZMQ Scene Sync] Failed to initialize bind=%s connect=%s: %s",
                cfg.bind_endpoint,
                cfg.connect_endpoint,
                exc,
            )
            self._close_subscriber_resources()
            self._pub_socket = None
            self._sub_socket = None
            self._publish_enabled = False
            self._apply_enabled = False

    def __del__(self):
        self._close_subscriber_resources()
        try:
            super().__del__()
        except Exception:
            pass

    def _close_subscriber_resources(self) -> None:
        socket = getattr(self, "_sub_socket", None)
        if socket is not None:
            try:
                socket.close(0)
            except Exception:
                pass
        context = getattr(self, "_sub_context", None)
        if context is not None:
            try:
                context.term()
            except Exception:
                pass

    @property
    def action_dim(self) -> int:
        return self._action_dim

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @property
    def current_reset_id(self) -> str | None:
        """Current publisher reset ID or the last accepted mirror reset ID."""

        return self._publisher_reset_id if self._publish_enabled else self._active_reset_id

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions = actions
        self._processed_actions = actions

    def reset(self, env_ids=None) -> None:
        """Preserve network sequence and reset gating across ``env.reset()``."""

        self._raw_actions.zero_()
        self._processed_actions.zero_()

    def set_publisher_reset_id(self, reset_id: str | None) -> None:
        """Start publishing post-reset frames under ``reset_id``（复位权威端 ID=1）。"""

        if self._publish_enabled and reset_id:
            self._publisher_reset_id = str(reset_id)
            logger.info("[ZMQ Scene Sync] Publisher entered reset_id=%s", self._publisher_reset_id)

    def expect_reset_id(self, reset_id: str | None) -> None:
        """Reject pre-reset frames on the mirror until ``reset_id`` arrives."""

        if self._apply_enabled and reset_id:
            self._expected_reset_id = str(reset_id)
            self._expected_reset_set_time = time.monotonic()
            logger.info("[ZMQ Scene Sync] Mirror waiting for reset_id=%s", self._expected_reset_id)

    def apply_actions(self):
        # 主循环挂载模式：收发由宿主每迭代调 pump()，不跟随 env.step——SONIC 锁步下
        # deploy 停发 lowcmd 时 env.step 停摆，ActionTerm 挂载的同步会随之冻结。
        if self.cfg.external_pump:
            return
        self.pump()

    def pump(self):
        """执行一轮收发。ActionTerm 模式下由 apply_actions 每物理步调用；
        主循环模式（external_pump=True）下由 sim_main 每迭代（step_hz）调用。"""

        if self._publish_enabled and self._pub_socket is not None:
            # 计数节流：ActionTerm 模式 = 物理 200 Hz、decimation=4 → 50 Hz 发布；
            # 主循环模式 = pump 本身就是 step_hz（50 Hz），decimation 默认 1。
            self._publish_tick += 1
            if self._publish_tick >= self._publish_decimation:
                self._publish_tick = 0
                self._publish_scene_frame()
        if self._apply_enabled and self._sub_socket is not None:
            # 门控超时放行：等不到期望 reset_id（事件错过/连环复位）时不能永久冻结镜像。
            if self._expected_reset_id is not None:
                gate_timeout = max(0.1, float(self.cfg.reset_gate_timeout_s))
                if time.monotonic() - self._expected_reset_set_time > gate_timeout:
                    logger.warning(
                        "[ZMQ Scene Sync] reset_id=%s 等待超过 %.1fs，放行门控（可能错过了复位事件）",
                        self._expected_reset_id,
                        gate_timeout,
                    )
                    self._expected_reset_id = None
            received = self._receive_latest_scene_frame()
            now = time.monotonic()
            if received:
                if self._stale_reported:
                    logger.info("[ZMQ Scene Sync] Stream recovered endpoint=%s", self.cfg.connect_endpoint)
                self._last_receive_time = now
                self._stale_reported = False
            else:
                self._warn_if_stale(now)

    def _publish_scene_frame(self) -> None:
        payload = {
            "schema": "g1_peer_scene_state.v1",
            "sender": self.cfg.local_sender_name,
            "session": self._publisher_session,
            "frame_id": self._publisher_frame_id,
            "timestamp_s": time.time(),
            "reset_id": self._publisher_reset_id,
            "joint_count": self._joint_count,
            "joint_order_hash": self._joint_order_hash,
            # Backward-compatible v1 extension: articulation receivers still
            # validate the hash as before and ignore this field.  visual_lod
            # receivers need names because a viewer has no local articulation
            # from which it could recover Isaac's actual wire order.
            "joint_names": list(self._joint_names or ()),
            "robots": {
                name: {
                    "root_state": robot.data.root_state_w[0].tolist(),
                    "joint_pos": robot.data.joint_pos[0].tolist(),
                    "joint_vel": robot.data.joint_vel[0].tolist(),
                }
                for name, robot in self._publish_robots.items()
            },
            "objects": {
                name: {"root_state": rigid_object.data.root_state_w[0].tolist()}
                for name, rigid_object in self._publish_objects.items()
            },
        }
        self._publisher_frame_id += 1

        try:
            message = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self._pub_socket.send_multipart([self.topic, message], flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except zmq.ZMQError as exc:
            logger.warning("[ZMQ Scene Sync] Publish failed endpoint=%s: %s", self.cfg.bind_endpoint, exc)

    def _receive_latest_scene_frame(self) -> bool:
        latest_message = None
        while True:
            try:
                parts = self._sub_socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except zmq.ZMQError as exc:
                logger.warning("[ZMQ Scene Sync] Receive failed endpoint=%s: %s", self.cfg.connect_endpoint, exc)
                return False
            if len(parts) == 2 and parts[0] == self.topic:
                latest_message = parts[1]

        if latest_message is None:
            return False

        try:
            payload: dict[str, Any] = json.loads(latest_message.decode("utf-8"))
            if payload.get("schema") != "g1_peer_scene_state.v1":
                raise ValueError(f"unsupported schema {payload.get('schema')!r}")
            if str(payload.get("sender", "")) == self.cfg.local_sender_name:
                # 自己的帧被环回（同机双进程测试时 connect 配错），不应用。
                return False

            session = str(payload["session"])
            frame_id = int(payload["frame_id"])
            reset_id = str(payload["reset_id"])
            if self._last_session is not None and session != self._last_session:
                logger.info(
                    "[ZMQ Scene Sync] Peer session changed %s -> %s; accepting the new scene stream",
                    self._last_session,
                    session,
                )
                self._last_frame_id = -1
                self._expected_reset_id = None
            if session == self._last_session and frame_id <= self._last_frame_id:
                return False
            if int(payload["joint_count"]) != self._joint_count:
                raise ValueError(f"joint_count mismatch: remote={payload['joint_count']} local={self._joint_count}")
            frame_joint_names = self._validate_remote_joint_order(payload)
            if self._expected_reset_id is not None and reset_id != self._expected_reset_id:
                return False

            robot_states = self._parse_robot_states(payload["robots"])
            object_states = self._parse_object_states(payload["objects"])
            self._apply_scene_states(robot_states, object_states, frame_joint_names)

            self._last_session = session
            self._last_frame_id = frame_id
            self._active_reset_id = reset_id
            if self._expected_reset_id == reset_id:
                self._expected_reset_id = None
                logger.info("[ZMQ Scene Sync] Mirror accepted post-reset frame reset_id=%s", reset_id)
            if not self._received_first_frame:
                logger.info(
                    "[ZMQ Scene Sync] Received first frame endpoint=%s frame_id=%d reset_id=%s",
                    self.cfg.connect_endpoint,
                    frame_id,
                    reset_id,
                )
                self._received_first_frame = True
            return True
        except Exception as exc:
            logger.warning("[ZMQ Scene Sync] Ignored invalid scene frame: %s", exc)
            return False

    def _parse_robot_states(
        self, payload: dict[str, Any]
    ) -> dict[str, tuple[torch.Tensor, torch.Tensor, torch.Tensor]]:
        parsed = {}
        for name in {**self._apply_robots, **self._apply_visual_robots}:
            # 容缺：apply 清单允许超集（viewer 声明 robot_1+robot_2，而权威端在
            # 双机器人落地前只发 robot_1）。缺席的机器人跳过、保持当前姿态；
            # 若整帧拒收，会把帧里已有的其他实体一起拖死。
            state = payload.get(name)
            if state is None:
                if name not in self._missing_robot_warned:
                    self._missing_robot_warned.add(name)
                    logger.warning(
                        "[ZMQ Scene Sync] Peer frames carry no state for %r yet; "
                        "its mirror holds the spawn pose until the peer publishes it",
                        name,
                    )
                continue
            root_state = self._payload_tensor(state, "root_state", 13)
            joint_pos = self._payload_tensor(state, "joint_pos", self._joint_count)
            joint_vel = self._payload_tensor(state, "joint_vel", self._joint_count)
            parsed[name] = (root_state, joint_pos, joint_vel)
        return parsed

    def _validate_remote_joint_order(self, payload: dict[str, Any]) -> tuple[str, ...]:
        """Validate old articulation frames and name-carrying visual frames."""

        remote_hash = str(payload["joint_order_hash"])
        raw_names = payload.get("joint_names")
        if self._apply_visual_robots:
            if not isinstance(raw_names, list):
                raise ValueError(
                    "visual_lod requires scene_state frames with joint_names; update the publisher"
                )
            from .peer_visual_lod import validate_wire_joint_order

            frame_joint_names = validate_wire_joint_order(
                tuple(str(name) for name in raw_names), remote_hash
            )
            if self._joint_order_hash and remote_hash != self._joint_order_hash:
                raise ValueError(
                    f"joint order mismatch: remote={remote_hash} local={self._joint_order_hash}"
                )
            return frame_joint_names

        if remote_hash != self._joint_order_hash:
            raise ValueError(
                f"joint order mismatch: remote={remote_hash} local={self._joint_order_hash}"
            )
        return tuple(self._joint_names or ())

    def _parse_object_states(self, payload: dict[str, Any]) -> dict[str, torch.Tensor]:
        return {name: self._payload_tensor(payload[name], "root_state", 13) for name in self._apply_objects}

    def _payload_tensor(self, payload: dict[str, Any], key: str, size: int) -> torch.Tensor:
        tensor = torch.as_tensor(payload[key], device=self.device, dtype=torch.float32).reshape(1, -1)
        if tensor.shape[1] != size:
            raise ValueError(f"field {key!r} expected {size} values, received {tensor.shape[1]}")
        if not torch.isfinite(tensor).all():
            raise ValueError(f"field {key!r} contains non-finite values")
        return tensor

    def _apply_scene_states(
        self,
        robot_states: dict[str, tuple[torch.Tensor, torch.Tensor, torch.Tensor]],
        object_states: dict[str, torch.Tensor],
        frame_joint_names: tuple[str, ...],
    ) -> None:
        for name, (root_state, joint_pos, joint_vel) in robot_states.items():
            visual_robot = self._apply_visual_robots.get(name)
            if visual_robot is not None:
                visual_robot.apply_state(
                    root_state.flatten().tolist(),
                    joint_pos.flatten().tolist(),
                    frame_joint_names,
                )
                continue
            robot = self._apply_robots[name]
            # 镜像侧只写位姿、**不写速度**（对端的 joint_vel/root 速度整组丢弃）。
            #
            # 这一侧的镜像 G1 是 disable_gravity + linear/angular_damping=0 + collision 关闭的
            # 自由体（见 peer_robot 的 spawn 配置），没有重力、阻尼或接触能耗散速度。而本函数
            # 只在收到新帧时才被调用，两帧之间机器人是自由演化的：把对端的瞬时速度硬写进来，
            # 它就会匀速漂到下一帧，然后被下一次硬写拉回原位。对端"看起来静止"时关节仍带高频
            # 速度噪声，这些噪声在对端被 PD、接触和重力约束住、积不出位移，到了镜像侧却被直接
            # 积分成比原抖动更大的位移——结果就是对端静止、镜像持续抖动。
            #
            # 位置每帧硬写就足以完全确定姿态，速度对纯镜像没有物理意义。同一个道理已经
            # 在 kinematic 物体上修过一次（下方 write_root_pose_to_sim 分支，14f087ce8），
            # 当时漏了机器人这条路径（17e71c0a0）。
            #
            # ⚠️ 速度 target 必须跟着一起清零：若留着非零的 velocity target 而实际速度为 0，
            # PD 的阻尼项 damping*(target_vel - 0) 会持续输出力矩推关节，比不改更糟。
            frozen_root = root_state.clone()
            frozen_root[:, 7:] = 0.0
            zero_joint_vel = torch.zeros_like(joint_vel)
            robot.write_root_state_to_sim(frozen_root)
            robot.write_joint_state_to_sim(joint_pos, zero_joint_vel)
            robot.set_joint_position_target(joint_pos)
            robot.set_joint_velocity_target(zero_joint_vel)

        for name, root_state in object_states.items():
            if self._object_is_kinematic[name]:
                # 镜像侧的同步物体 spawn 时就被翻成 kinematic，而 write_root_state_to_sim
                # 会把速度一并写下去。**CPU pipeline**（--device cpu）下这个写入会落到逐
                # body 的 PhysX C++ 调用上，而 PhysX 拒绝给 kinematic 刚体设速度：
                #     PxRigidDynamic::setLinearVelocity: Body must be non-kinematic!
                # 每帧每个物体刷一对 linear/angular，几十秒就撞满 PhysX 的 1000 条错误上限，
                # 于是 "PhysX has reported too many errors, simulation has been stopped"
                # —— 镜像侧整个仿真被掐停，箱子和机器人一起僵住，看着就像同步断了。
                # GPU pipeline（--device cuda:0）走的是 tensor API 批量写入，不经过这条
                # 逐 body 路径，所以同样的代码在 GPU 上没有症状，只有 CPU 上会炸。
                # kinematic 体的运动完全由位姿决定，速度写入本来就没有意义，直接跳过。
                self._apply_objects[name].write_root_pose_to_sim(root_state[:, :7])
            else:
                self._apply_objects[name].write_root_state_to_sim(root_state)

    def _warn_if_stale(self, now: float) -> None:
        timeout = max(0.0, float(self.cfg.stale_timeout_s))
        if timeout <= 0.0:
            return
        baseline = self._last_receive_time if self._last_receive_time is not None else self._subscriber_start_time
        age = now - baseline
        if age < timeout:
            return
        interval = max(0.1, float(self.cfg.stale_log_interval_s))
        if now - self._last_stale_warning_time < interval:
            return
        self._last_stale_warning_time = now
        self._stale_reported = True
        logger.warning(
            "[ZMQ Scene Sync] Stream stale endpoint=%s last_frame_age=%.3fs; holding the last mirrored scene",
            self.cfg.connect_endpoint,
            age,
        )


@configclass
class ZmqEnvResetSyncActionCfg(ActionTermCfg):
    """Configuration for broadcasting a full-environment reset event."""

    class_type: type = None  # set in __post_init__

    role: str = "none"
    """Synchronization role: ``publisher``, ``subscriber``, or ``none``."""

    endpoint: str = ""
    """Publisher binds this endpoint; subscriber connects to it."""

    topic: str = "env_reset"
    """PUB/SUB topic used for reset events."""

    repeat_frames: int = 10
    """Number of action-application frames over which a reset event is repeated."""

    send_hwm: int = 3
    """Publisher high-water mark used by the shared PUB socket."""

    receive_hwm: int = 3
    """Subscriber high-water mark."""

    external_pump: bool = False
    """True = 收发不挂 env.step，由宿主主循环每迭代调一次 pump()（方案 b）。
    注意 repeat_frames 的语义随之从物理步（200Hz，10 帧≈50ms）变为主循环迭代
    （50Hz，10 帧≈200ms）——重发窗口反而更宽。"""

    def __post_init__(self):
        self.class_type = ZmqEnvResetSyncAction


class ZmqEnvResetSyncAction(ActionTerm):
    """Send ID=1 reset events and expose de-duplicated reset requests on ID=2."""

    cfg: ZmqEnvResetSyncActionCfg

    def __init__(self, cfg: ZmqEnvResetSyncActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._action_dim = 0
        self._raw_actions = torch.zeros((self.num_envs, 0), device=self.device)
        self._processed_actions = torch.zeros_like(self._raw_actions)
        self._export_IO_descriptor = False

        self.role = str(cfg.role).strip().lower()
        if self.role not in {"publisher", "subscriber", "none"}:
            logger.error("[ZMQ Env Reset] Unsupported role %r; disabling reset sync", cfg.role)
            self.role = "none"
        self.endpoint = str(cfg.endpoint).strip()
        self.topic = str(cfg.topic).encode("utf-8")
        self._context = None
        self._socket = None

        self._publisher_session = uuid.uuid4().hex
        self._next_reset_sequence = 0
        self._pending_reset_id: str | None = None
        self._pending_repeat_frames = 0
        self._last_received_reset_id: str | None = None
        self._remote_reset_id: str | None = None

        if self.role == "none":
            return
        if zmq is None:
            logger.error("[ZMQ Env Reset] pyzmq is not installed; disabling reset synchronization")
            self.role = "none"
            return
        if not self.endpoint:
            logger.error("[ZMQ Env Reset] Empty endpoint; disabling reset synchronization")
            self.role = "none"
            return

        try:
            if self.role == "publisher":
                self._socket = ZmqPubSocketManager.get_pub_socket(self.endpoint, cfg.send_hwm)
            else:
                self._context = zmq.Context()
                self._socket = self._context.socket(zmq.SUB)
                self._socket.setsockopt(zmq.RCVHWM, max(1, int(cfg.receive_hwm)))
                self._socket.setsockopt(zmq.LINGER, 0)
                self._socket.setsockopt(zmq.SUBSCRIBE, self.topic)
                self._socket.connect(self.endpoint)
                logger.info(
                    "[ZMQ Env Reset] Subscriber connected to %s topic=%s",
                    self.endpoint,
                    self.topic.decode("utf-8"),
                )
        except Exception as exc:
            logger.error(
                "[ZMQ Env Reset] Failed to initialize role=%s endpoint=%s: %s",
                self.role,
                self.endpoint,
                exc,
            )
            self._close_subscriber_resources()
            self._socket = None
            self.role = "none"

    def __del__(self):
        self._close_subscriber_resources()
        try:
            super().__del__()
        except Exception:
            pass

    def _close_subscriber_resources(self) -> None:
        if getattr(self, "role", "none") != "subscriber":
            return
        socket = getattr(self, "_socket", None)
        if socket is not None:
            try:
                socket.close(0)
            except Exception:
                pass
        context = getattr(self, "_context", None)
        if context is not None:
            try:
                context.term()
            except Exception:
                pass

    @property
    def action_dim(self) -> int:
        return self._action_dim

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions = actions
        self._processed_actions = actions

    def reset(self, env_ids=None) -> None:
        """Preserve event IDs and repeat state across ``env.reset()``.

        The subscriber must retain the last processed ID so repeated packets do
        not cause a reset loop. The publisher must retain its queued repeats so
        ID=2 can still receive the event after ID=1 has reset locally.
        """

        self._raw_actions.zero_()
        self._processed_actions.zero_()

    def request_local_reset(self) -> str | None:
        """Queue a reset event on the authoritative ID=1 publisher."""

        if self.role != "publisher":
            return None
        reset_id = f"{self._publisher_session}:{self._next_reset_sequence}"
        self._next_reset_sequence += 1
        self._pending_reset_id = reset_id
        self._pending_repeat_frames = max(1, int(self.cfg.repeat_frames))
        logger.info(
            "[ZMQ Env Reset] Queued reset_id=%s repeats=%d",
            reset_id,
            self._pending_repeat_frames,
        )
        return reset_id

    def consume_remote_reset_request(self) -> str | None:
        """Return and clear the subscriber's pending full-env reset ID."""

        reset_id = self._remote_reset_id
        self._remote_reset_id = None
        return reset_id

    def apply_actions(self):
        if self.cfg.external_pump:
            return
        self.pump()

    def pump(self):
        """执行一轮复位事件收发（挂载模式见 external_pump 说明）。"""

        if self._socket is None:
            return
        if self.role == "publisher":
            self._publish_pending_reset()
        elif self.role == "subscriber":
            self._receive_reset_events()

    def _publish_pending_reset(self) -> None:
        if self._pending_reset_id is None or self._pending_repeat_frames <= 0:
            return
        payload = {
            "version": 1,
            "reset_id": self._pending_reset_id,
            "timestamp_s": time.time(),
        }
        try:
            message = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self._socket.send_multipart([self.topic, message], flags=zmq.NOBLOCK)
            self._pending_repeat_frames -= 1
            if self._pending_repeat_frames <= 0:
                logger.info("[ZMQ Env Reset] Finished publishing reset_id=%s", self._pending_reset_id)
                self._pending_reset_id = None
        except zmq.Again:
            pass
        except zmq.ZMQError as exc:
            logger.warning("[ZMQ Env Reset] Publish failed endpoint=%s: %s", self.endpoint, exc)

    def _receive_reset_events(self) -> None:
        latest_message = None
        while True:
            try:
                parts = self._socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except zmq.ZMQError as exc:
                logger.warning("[ZMQ Env Reset] Receive failed endpoint=%s: %s", self.endpoint, exc)
                return
            if len(parts) == 2 and parts[0] == self.topic:
                latest_message = parts[1]

        if latest_message is None:
            return
        try:
            payload: dict[str, Any] = json.loads(latest_message.decode("utf-8"))
            reset_id = str(payload["reset_id"])
        except Exception as exc:
            logger.warning("[ZMQ Env Reset] Invalid reset event: %s", exc)
            return
        if reset_id == self._last_received_reset_id:
            return
        self._last_received_reset_id = reset_id
        self._remote_reset_id = reset_id
        logger.info("[ZMQ Env Reset] Received reset_id=%s", reset_id)
