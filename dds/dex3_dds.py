# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Dex3 DDS command subscriber and simulated-state publisher."""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_


class Dex3DDS(DDSObject):
    """Bridge both seven-motor Dex3 hands between DDS and Isaac Lab."""

    def __init__(self, node_name: str = "dex3", topic_prefix: str = "rt", shm_suffix: str = ""):
        """topic_prefix/shm_suffix 语义与 G1RobotDDS 相同：host 第二实例传
        ("rt/r2", "_r2") → rt/r2/dex3/{left,right}/{cmd,state} + 独立 shm 段
        （同名 shm 会被静默 attach 共享，两副手互相执行对方命令）。"""
        if hasattr(self, "_initialized"):
            return

        super().__init__()
        self.node_name = node_name
        self.topic_prefix = str(topic_prefix).rstrip("/") or "rt"
        self.shm_suffix = str(shm_suffix)
        self.left_hand_state = unitree_hg_msg_dds__HandState_()
        self.right_hand_state = unitree_hg_msg_dds__HandState_()

        self.left_state_publisher = None
        self.right_state_publisher = None
        self.left_cmd_subscriber = None
        self.right_cmd_subscriber = None

        self.existing_data = {"left_hand_cmd": {}, "right_hand_cmd": {}}
        self._command_lock = threading.Lock()
        self.setup_shared_memory(
            input_shm_name=f"isaac_dex3_state{self.shm_suffix}",
            input_size=4096,
            output_shm_name=f"isaac_dex3_cmd{self.shm_suffix}",
            output_size=4096,
        )
        self._initialized = True
        print(
            f"[{self.node_name}] Hand DDS node initialized "
            f"(topics={self.topic_prefix}/dex3/*, shm=isaac_dex3_state{self.shm_suffix})"
        )

    def setup_publisher(self) -> bool:
        try:
            self.left_state_publisher = ChannelPublisher(
                f"{self.topic_prefix}/dex3/left/state", HandState_
            )
            self.left_state_publisher.Init()
            self.right_state_publisher = ChannelPublisher(
                f"{self.topic_prefix}/dex3/right/state", HandState_
            )
            self.right_state_publisher.Init()
            print(f"[{self.node_name}] Hand state publisher initialized")
            return True
        except Exception as exc:
            print(
                f"dex3_dds [{self.node_name}] "
                f"Hand state publisher initialization failed: {exc}"
            )
            return False

    def setup_subscriber(self) -> bool:
        try:
            self.left_cmd_subscriber = ChannelSubscriber(
                f"{self.topic_prefix}/dex3/left/cmd", HandCmd_
            )
            self.left_cmd_subscriber.Init(
                lambda message: self.dds_subscriber(message, "left"), 32
            )
            self.right_cmd_subscriber = ChannelSubscriber(
                f"{self.topic_prefix}/dex3/right/cmd", HandCmd_
            )
            self.right_cmd_subscriber.Init(
                lambda message: self.dds_subscriber(message, "right"), 32
            )
            print(f"[{self.node_name}] Hand command subscriber initialized")
            return True
        except Exception as exc:
            print(
                f"dex3_dds [{self.node_name}] "
                f"Hand command subscriber initialization failed: {exc}"
            )
            return False

    def dds_subscriber(self, msg: HandCmd_, datatype: str = None) -> None:
        """Store one complete hand command with a precise receive timestamp."""

        try:
            if datatype not in ("left", "right"):
                raise ValueError(f"invalid hand side: {datatype!r}")
            command = self.process_hand_command(msg, datatype)
            if command and self.output_shm:
                # DDS callbacks can execute concurrently. Publish both side
                # snapshots atomically so the action provider never observes
                # a half-updated payload.
                with self._command_lock:
                    self.existing_data[f"{datatype}_hand_cmd"] = command
                    self.output_shm.write_data(self.existing_data)
        except Exception as exc:
            print(
                f"dex3_dds [{self.node_name}] "
                f"Error handling {datatype} hand command: {exc}"
            )

    def process_hand_command(
        self, msg: HandCmd_, datatype: str = None
    ) -> Dict[str, Any]:
        """Convert HandCmd into a JSON/shared-memory-safe dictionary."""

        try:
            motors = msg.motor_cmd
            return {
                "modes": [int(motor.mode) for motor in motors],
                "positions": [float(motor.q) for motor in motors],
                "velocities": [float(motor.dq) for motor in motors],
                "torques": [float(motor.tau) for motor in motors],
                "kp": [float(motor.kp) for motor in motors],
                "kd": [float(motor.kd) for motor in motors],
                # SharedMemoryManager's metadata timestamp has one-second
                # resolution; control safety needs a monotonic subsecond age.
                "receive_time_monotonic": time.monotonic(),
            }
        except Exception as exc:
            print(
                f"dex3_dds [{self.node_name}] "
                f"Error processing {datatype} hand command data: {exc}"
            )
            return {}

    def dds_publisher(self) -> Any:
        """Publish the latest simulated states for both hands."""

        try:
            data = self.input_shm.read_data() or {}
            if "left_hand" in data:
                self._update_hand_state(self.left_hand_state, data["left_hand"])
                if self.left_state_publisher:
                    self.left_state_publisher.Write(self.left_hand_state)
            if "right_hand" in data:
                self._update_hand_state(self.right_hand_state, data["right_hand"])
                if self.right_state_publisher:
                    self.right_state_publisher.Write(self.right_hand_state)
        except Exception as exc:
            print(
                f"dex3_dds [{self.node_name}] Error processing publish data: {exc}"
            )
        return None

    def _update_hand_state(self, hand_state, hand_data: Dict[str, Any]) -> None:
        try:
            positions = hand_data["positions"]
            velocities = hand_data["velocities"]
            torques = hand_data["torques"]
            motor_count = min(
                7,
                len(hand_state.motor_state),
                len(positions),
                len(velocities),
                len(torques),
            )
            for motor_index in range(motor_count):
                motor_state = hand_state.motor_state[motor_index]
                motor_state.q = float(positions[motor_index])
                motor_state.dq = float(velocities[motor_index])
                motor_state.tau_est = float(torques[motor_index])
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            print(
                f"dex3_dds [{self.node_name}] Error updating hand state: {exc}"
            )

    def get_hand_commands(self) -> Optional[Dict[str, Any]]:
        if self.output_shm:
            return self.output_shm.read_data()
        return None

    def get_left_hand_command(self) -> Optional[Dict[str, Any]]:
        commands = self.get_hand_commands()
        if commands:
            return commands.get("left_hand_cmd")
        return None

    def get_right_hand_command(self) -> Optional[Dict[str, Any]]:
        commands = self.get_hand_commands()
        if commands:
            return commands.get("right_hand_cmd")
        return None

    def publish_hand_states(
        self,
        left_hand_data: Dict[str, Any],
        right_hand_data: Dict[str, Any],
    ) -> None:
        try:
            if self.input_shm:
                self.input_shm.write_data(
                    {
                        "left_hand": left_hand_data,
                        "right_hand": right_hand_data,
                    }
                )
        except Exception as exc:
            print(
                f"dex3_dds [{self.node_name}] Error publishing hand states: {exc}"
            )

    @staticmethod
    def _to_list(values):
        return values.tolist() if hasattr(values, "tolist") else list(values)

    def write_hand_states(
        self,
        left_positions,
        left_velocities,
        left_torques,
        right_positions,
        right_velocities,
        right_torques,
    ) -> None:
        """Write actual q/dq/tau for both hands to the publisher buffer."""

        self.publish_hand_states(
            {
                "positions": self._to_list(left_positions),
                "velocities": self._to_list(left_velocities),
                "torques": self._to_list(left_torques),
            },
            {
                "positions": self._to_list(right_positions),
                "velocities": self._to_list(right_velocities),
                "torques": self._to_list(right_torques),
            },
        )

    def write_single_hand_state(
        self,
        hand_side: str,
        positions,
        velocities,
        torques,
    ) -> None:
        """Update one side while preserving the latest state of the other."""

        if hand_side not in ("left", "right"):
            print(f"dex3_dds [{self.node_name}] Invalid hand side: {hand_side}")
            return
        hand_data = {
            "positions": self._to_list(positions),
            "velocities": self._to_list(velocities),
            "torques": self._to_list(torques),
        }
        existing = self.input_shm.read_data() if self.input_shm else {}
        existing = existing or {}
        zero_state = {
            "positions": [0.0] * 7,
            "velocities": [0.0] * 7,
            "torques": [0.0] * 7,
        }
        if hand_side == "left":
            self.publish_hand_states(
                hand_data,
                existing.get("right_hand", zero_state),
            )
        else:
            self.publish_hand_states(
                existing.get("left_hand", zero_state),
                hand_data,
            )
