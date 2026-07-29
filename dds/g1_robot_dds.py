# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
G1 robot DDS communication class
Handle the state publishing and command receiving of the G1 robot
"""

import numpy as np
import time
from typing import Any, Dict, Optional
# from dds.dds_base import BaseDDSNode, node_manager
from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowCmd_, LowState_
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__IMUState_,
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.utils.crc import CRC


ISAAC_LOWSTATE_SYNC_MAGIC = 0x49534143  # ASCII-ish "ISAC"
SONIC_LOWCMD_SYNC_MAGIC = 0x534E4331  # "SNC1"


class G1RobotDDS(DDSObject):
    """G1 robot DDS communication class - singleton pattern
    
    Features:
    - Publish the state of the G1 robot to DDS (rt/lowstate)
    - Receive the control command of the G1 robot (rt/lowcmd)
    """
    
    def __init__(self,node_name:str="g1_robot"):
        """Initialize the G1 robot DDS node"""
        # avoid duplicate initialization
        if hasattr(self, '_initialized'):
            return
            
        super().__init__()
        self.node_name = node_name
        self.crc = CRC()
        # Windows 上 SDK 的 CRC 走纯 Python 回退,单次 LowCmd 校验实测 1.08ms;
        # C++ 侧以 500Hz 发 lowcmd,逐包校验要吃掉 54% 单线程,再叠加解析/json/
        # shm 写,回调线程连同 GIL 一起被打满,锁步等待循环被饿到 ~0.4Hz
        # (每步 2.5s,Init Done 要 6 分钟)。降为抽样校验(每 50 包验 1 次)后
        # 负载 ~1%,持续性损坏仍能在 0.1s 内被发现。Linux 的 CRC 是 C 库
        # (~0.01ms),保持逐包全验不变。
        self._crc_sample_interval = 1 if getattr(self.crc, "platform", "") == "Linux" else 50
        self._crc_sample_counter = 0
        if self._crc_sample_interval > 1:
            print(
                f"g1_robot_dds [{node_name}] pure-python CRC detected; "
                f"validating 1 in {self._crc_sample_interval} lowcmd packets"
            )
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.torso_imu_state = unitree_hg_msg_dds__IMUState_()
        self._stats_window_start = time.monotonic()
        self._lowstate_publish_count = 0
        self._torso_imu_publish_count = 0
        self._fresh_sample_count = 0
        self._repeated_sample_publish_count = 0
        self._last_sample_seq = None
        self._latest_written_sample_seq = None
        self._latest_written_sample_time = None
        self._latest_written_sim_time_s = None
        self._latest_written_reset_epoch = 0
        self._last_published_reset_epoch = 0
        self._reset_epoch = 0
        self._lowcmd_packet_count = 0
        self._lowcmd_content_update_count = 0
        self._last_lowcmd_signature = None
        self._last_lowcmd_ack_tick = None
        self._last_lowcmd_ack_reset_epoch = None
        self._lowcmd_ack_match_count = 0
        self._lowcmd_ack_stale_count = 0
        self._last_sample_age_ms = 0.0
        self._reset_state_grace_until = 0.0
        self._reset_state_grace_active = False
        self._reset_state_grace_samples = 0
        self._reset_state_grace_max_abs_dq = 0.0
        self._initialized = True
        
        # setup the shared memory
        self.setup_shared_memory(
            input_shm_name="isaac_robot_state",  # read the state of the G1 robot from Isaac Lab
            output_shm_name="dds_robot_cmd",  # output the command to Isaac Lab
            input_size=8192,
            output_size=8192  # output the command to Isaac Lab
        )
        
        print(f"[{self.node_name}] G1 robot DDS node initialized")
    
    def setup_publisher(self) -> bool:
        """Setup the publisher of the G1 robot"""
        try:
            self.publisher = ChannelPublisher("rt/lowstate", LowState_)
            self.publisher.Init()
            self.torso_imu_publisher = ChannelPublisher("rt/secondary_imu", IMUState_)
            self.torso_imu_publisher.Init()
            print(f"[{self.node_name}] State publisher initialized (rt/lowstate)")
            print(f"[{self.node_name}] Torso IMU publisher initialized (rt/secondary_imu)")
            return True
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] State publisher initialization failed: {e}")    
            return False

    def begin_reset_epoch(self, reason: str) -> int:
        """Advance the Isaac-only reset generation carried in LowState.reserve.

        ``tick`` identifies an individual advanced PhysX state.  The reset epoch
        identifies a discontinuous articulation reset, allowing SONIC to clear
        recurrent/history state without changing the standard G1 message type.
        The marker is ignored by real-robot and MuJoCo paths.
        """
        self._reset_epoch = (int(getattr(self, "_reset_epoch", 0)) + 1) & 0xFFFFFFFF
        print(
            f"[{self.node_name}] Isaac reset epoch -> {self._reset_epoch} "
            f"({reason})"
        )
        return self._reset_epoch

    def get_latest_written_state_info(self) -> Dict[str, Any]:
        """Return the state generation that the next Isaac step must consume."""
        return {
            "sample_seq": getattr(self, "_latest_written_sample_seq", None),
            "sample_time_monotonic": getattr(self, "_latest_written_sample_time", None),
            "sim_time_s": getattr(self, "_latest_written_sim_time_s", None),
            "reset_epoch": int(getattr(self, "_latest_written_reset_epoch", 0)),
        }
    
    def setup_subscriber(self) -> bool:
        """Setup the subscriber of the G1 robot"""
        try:
            print(f"[{self.node_name}] Create ChannelSubscriber...")
            self.subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
            self.subscriber.Init(lambda msg: self.dds_subscriber(msg, ""), 32)
            return True
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Command subscriber initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    def begin_reset_state_grace(self, duration_s: float, reason: str) -> None:
        """Suppress non-physical LowState velocity/torque during an Isaac reset.

        Resetting a PhysX articulation writes a discontinuous pose into the
        simulation.  A velocity sample produced around that teleport is not a
        physical motor velocity and must not trip the deploy process's normal
        35 rad/s safety limit.  Position and IMU samples remain live; only dq
        and tau_est are reported as zero for this explicitly requested window.
        """
        duration_s = max(0.0, float(duration_s))
        if duration_s <= 0.0:
            return

        now = time.monotonic()
        new_deadline = now + duration_s
        was_active = self._reset_state_grace_active and now < self._reset_state_grace_until
        if not was_active:
            self._reset_state_grace_samples = 0
            self._reset_state_grace_max_abs_dq = 0.0
        self._reset_state_grace_until = max(self._reset_state_grace_until, new_deadline)
        self._reset_state_grace_active = True
        action = "extended" if was_active else "started"
        print(
            f"[{self.node_name}] Reset LowState grace {action}: "
            f"zeroing published dq/tau for {duration_s:.3f}s ({reason})"
        )
    
    def dds_publisher(self) -> Any:
        """Convert Isaac Lab state to DDS message and publish."""
        try:
            data = self.input_shm.read_data()
            if data is None:
                return

            motor_state = self.low_state.motor_state

            positions = data.get("joint_positions")
            velocities = data.get("joint_velocities")
            torques = data.get("joint_torques")

            if positions is None or velocities is None or torques is None:
                return

            q_array = np.asarray(positions, dtype=np.float32)
            dq_array = np.asarray(velocities, dtype=np.float32)
            tau_array = np.asarray(torques, dtype=np.float32)
            if not (len(q_array) == len(dq_array) == len(tau_array)):
                raise ValueError(
                    "joint state arrays must have equal lengths: "
                    f"q={len(q_array)}, dq={len(dq_array)}, tau={len(tau_array)}"
                )
            if len(q_array) > len(motor_state):
                raise ValueError(
                    f"joint state contains {len(q_array)} motors, LowState supports {len(motor_state)}"
                )
            if not (
                np.isfinite(q_array).all()
                and np.isfinite(dq_array).all()
                and np.isfinite(tau_array).all()
            ):
                raise ValueError("joint state contains NaN or Inf")

            publish_time = time.monotonic()
            if publish_time < self._reset_state_grace_until:
                self._reset_state_grace_active = True
                self._reset_state_grace_samples += 1
                if dq_array.size:
                    self._reset_state_grace_max_abs_dq = max(
                        self._reset_state_grace_max_abs_dq,
                        float(np.max(np.abs(dq_array))),
                    )
                dq_array = np.zeros_like(dq_array)
                tau_array = np.zeros_like(tau_array)
            elif self._reset_state_grace_active:
                print(
                    f"[{self.node_name}] Reset LowState grace complete: "
                    f"samples={self._reset_state_grace_samples}, "
                    f"max_raw_abs_dq={self._reset_state_grace_max_abs_dq:.3f}rad/s; "
                    "restoring live dq/tau"
                )
                self._reset_state_grace_active = False

            for i in range(len(q_array)):
                motor = motor_state[i]
                motor.q = q_array[i]
                motor.dq = dq_array[i]
                motor.tau_est = tau_array[i]

            base_imu = data.get("base_imu_data", data.get("imu_data"))
            if not self._copy_imu_state(self.low_state.imu_state, base_imu, "base/pelvis"):
                return

            torso_imu = data.get("torso_imu_data")
            have_torso_imu = self._copy_imu_state(
                self.torso_imu_state,
                torso_imu,
                "torso",
                required=False,
            )

            sample_seq = data.get("sample_seq")
            if sample_seq is None:
                # Backward-compatible fallback for non-SONIC observation writers.
                sample_seq = int(self.low_state.tick) + 1
            sample_seq = int(sample_seq)
            if self._last_sample_seq != sample_seq:
                self._fresh_sample_count += 1
                self._last_sample_seq = sample_seq
            else:
                self._repeated_sample_publish_count += 1

            # Tick identifies a fresh PhysX sample.  Re-publishing the same
            # shared-memory state at 100 Hz deliberately keeps the same tick so
            # diagnostics and future synchronized consumers can distinguish it
            # from a newly advanced simulation state.
            self.low_state.tick = sample_seq & 0xFFFFFFFF
            # Isaac/Sonic synchronized-control extension.  Standard Unitree
            # fields remain untouched; consumers opt in by checking the magic.
            published_reset_epoch = int(
                data.get("reset_epoch", getattr(self, "_reset_epoch", 0))
            ) & 0xFFFFFFFF
            self._last_published_reset_epoch = published_reset_epoch
            self.low_state.reserve[0] = published_reset_epoch
            self.low_state.reserve[1] = 1  # protocol version
            self.low_state.reserve[2] = 0
            self.low_state.reserve[3] = ISAAC_LOWSTATE_SYNC_MAGIC
            self.low_state.crc = self.crc.Crc(self.low_state)

            # Publish the secondary IMU first and LowState last.  In the
            # synchronized bridge the unique LowState tick is the commit marker
            # that wakes the C++ policy; sending torso data first minimizes the
            # chance that one policy step combines a new pelvis sample with the
            # previous torso sample.
            if have_torso_imu:
                self.torso_imu_publisher.Write(self.torso_imu_state)
                self._torso_imu_publish_count += 1
            self.publisher.Write(self.low_state)
            self._lowstate_publish_count += 1

            sample_time = data.get("sample_time_monotonic")
            if sample_time is not None:
                self._last_sample_age_ms = max(0.0, (time.monotonic() - float(sample_time)) * 1000.0)
            self._report_publish_stats()

        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error processing publish data: {e}")

    @staticmethod
    def _copy_imu_state(message, imu_data, label: str, required: bool = True) -> bool:
        if imu_data is None:
            if required:
                print(f"g1_robot_dds: missing required {label} IMU sample")
            return False

        imu_array = np.asarray(imu_data, dtype=np.float32)
        if imu_array.size < 13:
            raise ValueError(f"{label} IMU sample has {imu_array.size} values; expected at least 13")
        if not np.isfinite(imu_array[:13]).all():
            raise ValueError(f"{label} IMU sample contains NaN or Inf")

        quaternion_wxyz = imu_array[3:7]
        quaternion_norm = float(np.linalg.norm(quaternion_wxyz))
        if quaternion_norm < 1.0e-6:
            raise ValueError(f"{label} IMU quaternion has zero norm")
        quaternion_wxyz = quaternion_wxyz / quaternion_norm

        # Unitree HG IMUState and SONIC both use quaternion order [w, x, y, z].
        message.quaternion[:] = quaternion_wxyz
        message.accelerometer[:] = imu_array[7:10]
        message.gyroscope[:] = imu_array[10:13]
        return True

    def _report_publish_stats(self) -> None:
        now = time.monotonic()
        elapsed = now - self._stats_window_start
        if elapsed < 5.0:
            return

        lowstate_hz = self._lowstate_publish_count / elapsed
        torso_imu_hz = self._torso_imu_publish_count / elapsed
        fresh_sample_hz = self._fresh_sample_count / elapsed
        repeated_publish_hz = self._repeated_sample_publish_count / elapsed
        lowcmd_packet_hz = self._lowcmd_packet_count / elapsed
        lowcmd_content_hz = self._lowcmd_content_update_count / elapsed
        ack_match_hz = self._lowcmd_ack_match_count / elapsed
        ack_stale_hz = self._lowcmd_ack_stale_count / elapsed
        print(
            f"[{self.node_name}] DDS publish: lowstate={lowstate_hz:.1f}Hz, "
            f"secondary_imu={torso_imu_hz:.1f}Hz, "
            f"fresh_physx={fresh_sample_hz:.1f}Hz, repeats={repeated_publish_hz:.1f}Hz, "
            f"lowcmd_packets={lowcmd_packet_hz:.1f}Hz, "
            f"lowcmd_changes={lowcmd_content_hz:.1f}Hz, "
            f"ack_match={ack_match_hz:.1f}Hz, ack_stale={ack_stale_hz:.1f}Hz, "
            f"sample_age={self._last_sample_age_ms:.2f}ms, "
            f"sample_seq={self._last_sample_seq}, ack_tick={self._last_lowcmd_ack_tick}, "
            f"reset_epoch={getattr(self, '_last_published_reset_epoch', 0)}/"
            f"{self._last_lowcmd_ack_reset_epoch}"
        )
        self._stats_window_start = now
        self._lowstate_publish_count = 0
        self._torso_imu_publish_count = 0
        self._fresh_sample_count = 0
        self._repeated_sample_publish_count = 0
        self._lowcmd_packet_count = 0
        self._lowcmd_content_update_count = 0
        self._lowcmd_ack_match_count = 0
        self._lowcmd_ack_stale_count = 0

    
    def dds_subscriber(self, msg: LowCmd_,datatype:str=None) -> Dict[str, Any]:
        """Process the subscribe data: convert the DDS command to the Isaac Lab format
        
        Return data format:
        {
            "mode_pr": int,
            "mode_machine": int,
            "motor_cmd": {
                "modes": [29 motor enable modes],
                "positions": [29 joint position commands],
                "velocities": [29 joint velocity commands],
                "torques": [29 joint torque commands],
                "kp": [29 position gains],
                "kd": [29 speed gains]
            }
        }
        """
        try:
            # verify the CRC (sampled on Windows - see __init__ for why).
            # getattr defaults keep instances built without __init__ (tests use
            # __new__ + manual attributes) on the old validate-every-packet path.
            crc_interval = getattr(self, "_crc_sample_interval", 1)
            self._crc_sample_counter = getattr(self, "_crc_sample_counter", 0) + 1
            if self._crc_sample_counter >= crc_interval:
                self._crc_sample_counter = 0
                if self.crc.Crc(msg) != msg.crc:
                    print(f"g1_robot_dds [{self.node_name}] Warning: CRC verification failed!")
                    return {}
            
            # extract the command data
            num_cmd_motors = len(msg.motor_cmd)
            modes = [int(msg.motor_cmd[i].mode) for i in range(num_cmd_motors)]
            positions = [float(msg.motor_cmd[i].q) for i in range(num_cmd_motors)]
            velocities = [float(msg.motor_cmd[i].dq) for i in range(num_cmd_motors)]
            torques = [float(msg.motor_cmd[i].tau) for i in range(num_cmd_motors)]
            kp = [float(msg.motor_cmd[i].kp) for i in range(num_cmd_motors)]
            kd = [float(msg.motor_cmd[i].kd) for i in range(num_cmd_motors)]
            reserve = [int(value) for value in msg.reserve]
            signature = (
                int(msg.mode_pr),
                int(msg.mode_machine),
                tuple(reserve),
                tuple(modes),
                tuple(positions),
                tuple(velocities),
                tuple(torques),
                tuple(kp),
                tuple(kd),
            )
            self._lowcmd_packet_count += 1
            if signature != self._last_lowcmd_signature:
                self._lowcmd_content_update_count += 1
                self._last_lowcmd_signature = signature

            sync_magic = reserve[3] if len(reserve) >= 4 else 0
            if sync_magic == SONIC_LOWCMD_SYNC_MAGIC:
                self._last_lowcmd_ack_tick = reserve[0]
                self._last_lowcmd_ack_reset_epoch = reserve[1]
                if (
                    self._last_sample_seq is not None
                    and reserve[0] == (int(self._last_sample_seq) & 0xFFFFFFFF)
                    and reserve[1]
                    == (int(getattr(self, "_last_published_reset_epoch", 0)) & 0xFFFFFFFF)
                ):
                    self._lowcmd_ack_match_count += 1
                else:
                    self._lowcmd_ack_stale_count += 1

            cmd_data = {
                "mode_pr": int(msg.mode_pr),
                "mode_machine": int(msg.mode_machine),
                "receive_time_monotonic": time.monotonic(),
                "reserve": reserve,
                "ack_state_tick": reserve[0] if sync_magic == SONIC_LOWCMD_SYNC_MAGIC else None,
                "ack_reset_epoch": reserve[1] if sync_magic == SONIC_LOWCMD_SYNC_MAGIC else None,
                "control_step_count": reserve[2] if sync_magic == SONIC_LOWCMD_SYNC_MAGIC else None,
                "sync_magic": sync_magic,
                "motor_cmd": {
                    "modes": modes,
                    "positions": positions,
                    "velocities": velocities,
                    "torques": torques,
                    "kp": kp,
                    "kd": kd,
                }
            }
            self.output_shm.write_data(cmd_data)
            
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error processing subscribe data: {e}")
            return {}
    
    def get_robot_command(self) -> Optional[Dict[str, Any]]:
        """Get the robot control command
        
        Returns:
            Dict: the robot control command, return None if there is no new command
        """
        if self.output_shm:
            return self.output_shm.read_data()
        return None
    
    def write_robot_state(
        self,
        joint_positions,
        joint_velocities,
        joint_torques,
        imu_data=None,
        *,
        base_imu_data=None,
        torso_imu_data=None,
        sample_seq=None,
        sim_time_s=None,
    ):
        """Write the robot state to the shared memory
        
        Args:
            joint_positions: the joint position list or torch.Tensor
            joint_velocities: the joint velocity list or torch.Tensor
            joint_torques: the joint torque list or torch.Tensor
            imu_data: legacy base IMU argument kept for other tasks
            base_imu_data: pelvis/base IMU in [pos, quat(wxyz), accel, gyro] layout
            torso_imu_data: torso IMU in [pos, quat(wxyz), accel, gyro] layout
        """
        if self.input_shm is None:
            return
        try:
            if base_imu_data is None:
                base_imu_data = imu_data

            normalized_sample_seq = int(sample_seq) if sample_seq is not None else None
            write_time = time.monotonic()
            self._latest_written_sample_seq = normalized_sample_seq
            self._latest_written_sample_time = write_time
            self._latest_written_sim_time_s = (
                float(sim_time_s) if sim_time_s is not None else None
            )
            self._latest_written_reset_epoch = int(
                getattr(self, "_reset_epoch", 0)
            ) & 0xFFFFFFFF

            state_data = {
                "joint_positions": joint_positions.tolist() if hasattr(joint_positions, 'tolist') else joint_positions,
                "joint_velocities": joint_velocities.tolist() if hasattr(joint_velocities, 'tolist') else joint_velocities,
                "joint_torques": joint_torques.tolist() if hasattr(joint_torques, 'tolist') else joint_torques,
                "base_imu_data": base_imu_data.tolist() if hasattr(base_imu_data, 'tolist') else base_imu_data,
                "torso_imu_data": torso_imu_data.tolist() if hasattr(torso_imu_data, 'tolist') else torso_imu_data,
                "sample_time_monotonic": write_time,
                "sample_seq": normalized_sample_seq,
                "sim_time_s": float(sim_time_s) if sim_time_s is not None else None,
                "reset_epoch": self._latest_written_reset_epoch,
            }
            self.input_shm.write_data(state_data)
            # lock-step latency path: wake the publish loop so the fresh
            # sample leaves immediately instead of waiting for the next
            # scheduled slot (no-op unless enabled via the DDS manager)
            registered_name = getattr(self, "_dds_registered_name", None)
            if registered_name is not None:
                from dds.dds_master import dds_manager
                dds_manager.notify_fresh_sample(registered_name)
        except Exception as e:
            print(f"g1_robot_dds [{self.node_name}] Error writing robot state: {e}")
