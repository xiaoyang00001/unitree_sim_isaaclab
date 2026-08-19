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


HAND_COMMAND_SIDES = ("left", "right")
HAND_COMMAND_FIELDS = ("modes", "positions", "velocities", "torques", "kp", "kd")
HAND_COMMAND_SHM_HEARTBEAT_S = 0.05
HAND_COMMAND_STATS_INTERVAL_S = 5.0


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
        # SONIC sends each HandCmd topic at 500 Hz, although the command
        # normally changes only when a new policy result is available.  Keep
        # the fully decoded payload and receive time separately: duplicate
        # packets can refresh liveness without rebuilding two JSON hand
        # payloads or contending on shared memory.
        self._latest_hand_commands = {side: None for side in HAND_COMMAND_SIDES}
        self._last_hand_command_fingerprints = {
            side: None for side in HAND_COMMAND_SIDES
        }
        self._last_hand_command_receive_times = {
            side: None for side in HAND_COMMAND_SIDES
        }
        self._handcmd_stats_window_start = time.monotonic()
        self._handcmd_packet_counts = {side: 0 for side in HAND_COMMAND_SIDES}
        self._handcmd_change_counts = {side: 0 for side in HAND_COMMAND_SIDES}
        self._handcmd_duplicate_counts = {side: 0 for side in HAND_COMMAND_SIDES}
        self._handcmd_out_of_order_counts = {
            side: 0 for side in HAND_COMMAND_SIDES
        }
        self._handcmd_shm_write_count = 0
        self._handcmd_shm_last_attempt_time = 0.0
        self._handcmd_shm_timestamp = None
        self._handcmd_shm_dirty = False
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

    @staticmethod
    def _hand_command_fingerprint(msg: HandCmd_) -> tuple:
        """Return an exact value fingerprint for the complete wire command."""

        return (
            tuple(msg.reserve),
            tuple(
                (
                    motor.mode,
                    motor.q,
                    motor.dq,
                    motor.tau,
                    motor.kp,
                    motor.kd,
                    motor.reserve,
                )
                for motor in msg.motor_cmd
            ),
        )

    def _snapshot_hand_commands_locked(self) -> Dict[str, Any]:
        """Materialize a caller-owned snapshot while ``_command_lock`` is held."""

        snapshot: Dict[str, Any] = {}
        for side in HAND_COMMAND_SIDES:
            command = self._latest_hand_commands[side]
            if command is None:
                snapshot[f"{side}_hand_cmd"] = {}
                continue
            side_snapshot = {
                field_name: list(command[field_name])
                for field_name in HAND_COMMAND_FIELDS
            }
            side_snapshot["receive_time_monotonic"] = (
                self._last_hand_command_receive_times[side]
            )
            snapshot[f"{side}_hand_cmd"] = side_snapshot
        return snapshot

    def _collect_hand_command_stats_locked(self, now: float) -> Optional[str]:
        elapsed = now - self._handcmd_stats_window_start
        if elapsed < HAND_COMMAND_STATS_INTERVAL_S:
            return None

        side_stats = []
        for side in HAND_COMMAND_SIDES:
            side_stats.append(
                f"{side}={self._handcmd_packet_counts[side] / elapsed:.1f}Hz "
                f"(changes={self._handcmd_change_counts[side] / elapsed:.1f}, "
                f"duplicates={self._handcmd_duplicate_counts[side] / elapsed:.1f}, "
                f"out_of_order={self._handcmd_out_of_order_counts[side]})"
            )
        result = (
            f"[{self.node_name}] HandCmd receive: {', '.join(side_stats)}, "
            f"shm_writes={self._handcmd_shm_write_count / elapsed:.1f}Hz"
        )
        self._handcmd_stats_window_start = now
        for side in HAND_COMMAND_SIDES:
            self._handcmd_packet_counts[side] = 0
            self._handcmd_change_counts[side] = 0
            self._handcmd_duplicate_counts[side] = 0
            self._handcmd_out_of_order_counts[side] = 0
        self._handcmd_shm_write_count = 0
        return result

    def dds_subscriber(self, msg: HandCmd_, datatype: str = None) -> None:
        """Store one complete hand command with a precise receive timestamp."""

        try:
            if datatype not in HAND_COMMAND_SIDES:
                raise ValueError(f"invalid hand side: {datatype!r}")
            receive_time = time.monotonic()
            fingerprint = self._hand_command_fingerprint(msg)
            report_line = None
            with self._command_lock:
                self._handcmd_packet_counts[datatype] += 1
                previous_receive_time = self._last_hand_command_receive_times[datatype]
                if (
                    previous_receive_time is not None
                    and receive_time < previous_receive_time
                ):
                    # A delayed callback must not roll a newer same-side
                    # command or its liveness timestamp backwards.
                    self._handcmd_out_of_order_counts[datatype] += 1
                    report_line = self._collect_hand_command_stats_locked(receive_time)
                else:
                    is_duplicate = (
                        self._latest_hand_commands[datatype] is not None
                        and fingerprint
                        == self._last_hand_command_fingerprints[datatype]
                    )
                    if is_duplicate:
                        self._handcmd_duplicate_counts[datatype] += 1
                    else:
                        command = self.process_hand_command(
                            msg,
                            datatype,
                            receive_time_monotonic=receive_time,
                        )
                        if not command:
                            return
                        self._latest_hand_commands[datatype] = command
                        self._last_hand_command_fingerprints[datatype] = fingerprint
                        self._handcmd_change_counts[datatype] += 1

                    # A repeated packet is still a live command.  Store the
                    # timestamp separately so published snapshots remain
                    # immutable and concurrent readers cannot see half an
                    # update.
                    self._last_hand_command_receive_times[datatype] = receive_time

                    output_shm = getattr(self, "output_shm", None)
                    heartbeat_due = (
                        receive_time - self._handcmd_shm_last_attempt_time
                        >= HAND_COMMAND_SHM_HEARTBEAT_S
                    )
                    if output_shm is not None and (
                        not is_duplicate
                        or self._handcmd_shm_dirty
                        or heartbeat_due
                    ):
                        snapshot = self._snapshot_hand_commands_locked()
                        self.existing_data = snapshot
                        self._handcmd_shm_last_attempt_time = receive_time
                        write_succeeded = output_shm.write_data(snapshot) is not False
                        self._handcmd_shm_dirty = not write_succeeded
                        if write_succeeded:
                            self._handcmd_shm_write_count += 1
                            # Preserve SharedMemoryManager.read_data()'s public
                            # top-level metadata for in-process cache readers.
                            self._handcmd_shm_timestamp = int(time.time()) & 0xFFFFFFFF

                    report_line = self._collect_hand_command_stats_locked(receive_time)

            if report_line:
                print(report_line)
        except Exception as exc:
            print(
                f"dex3_dds [{self.node_name}] "
                f"Error handling {datatype} hand command: {exc}"
            )

    def process_hand_command(
        self,
        msg: HandCmd_,
        datatype: str = None,
        *,
        receive_time_monotonic: Optional[float] = None,
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
                "receive_time_monotonic": (
                    time.monotonic()
                    if receive_time_monotonic is None
                    else float(receive_time_monotonic)
                ),
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
        # Action providers run in this process, so use the decoded cache and
        # avoid JSON-decoding shared memory on every simulation step.  Return
        # caller-owned lists to preserve the old read_data() ownership model.
        with self._command_lock:
            if any(
                self._latest_hand_commands[side] is not None
                for side in HAND_COMMAND_SIDES
            ):
                snapshot = self._snapshot_hand_commands_locked()
                if self._handcmd_shm_timestamp is not None:
                    snapshot["_timestamp"] = self._handcmd_shm_timestamp
                return snapshot
        output_shm = getattr(self, "output_shm", None)
        if output_shm:
            return output_shm.read_data()
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
