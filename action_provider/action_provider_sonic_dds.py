"""Direct Gear SONIC DDS control for the floating-base G1 29-DoF robot.

This provider owns the 29 G1 body joints.  The 14 Dex3 joints are deliberately
held at their configured default positions until the OpenXR hand-control phase
is implemented.
"""

from __future__ import annotations

import time
from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from dds.dds_master import dds_manager
from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER


DEX3_JOINT_MARKERS = ("left_hand_", "right_hand_")


class SonicDDSActionProvider(ActionProvider):
    """Map the complete 29-motor SONIC LowCmd to Isaac Lab joint targets."""

    def __init__(self, env, args_cli):
        super().__init__("sonic_dds")

        self.env = env
        self.robot = env.scene["robot"]
        self.device = env.device
        self.robot_dds = dds_manager.get_object("g129")
        if self.robot_dds is None:
            raise RuntimeError("G1 DDS object 'g129' is not registered")

        self.command_timeout_s = max(0.01, float(getattr(args_cli, "sonic_lowcmd_timeout", 0.10)))
        self.ramp_duration_s = max(0.0, float(getattr(args_cli, "sonic_ramp_seconds", 2.0)))
        self.max_target_step = max(0.0, float(getattr(args_cli, "sonic_max_target_step", 0.10)))

        joint_names = list(self.robot.data.joint_names)
        joint_to_index = {name: index for index, name in enumerate(joint_names)}

        missing = [name for name in G1_29DOF_DDS_JOINT_ORDER if name not in joint_to_index]
        if missing:
            raise ValueError(f"G1 29-DoF USD is missing SONIC joints: {missing}")

        body_indices = [joint_to_index[name] for name in G1_29DOF_DDS_JOINT_ORDER]
        if len(set(body_indices)) != len(G1_29DOF_DDS_JOINT_ORDER):
            raise ValueError("SONIC 29-DoF mapping contains duplicate articulation indices")

        dex3_indices = [
            index for index, name in enumerate(joint_names) if name.startswith(DEX3_JOINT_MARKERS)
        ]
        if len(dex3_indices) != 14:
            raise ValueError(f"Expected 14 Dex3 joints in the wholebody USD, found {len(dex3_indices)}")

        self._body_indices = torch.tensor(body_indices, dtype=torch.long, device=self.device)
        self._dex3_indices = torch.tensor(dex3_indices, dtype=torch.long, device=self.device)
        self._default_action = self.robot.data.default_joint_pos[0].clone()
        self._last_action = self._default_action.clone()
        self._incoming_positions = torch.empty(29, dtype=torch.float32, device=self.device)

        self._first_valid_command_time: Optional[float] = None
        self._last_valid_command_time: Optional[float] = None
        self._last_timeout_warning_time = 0.0

        print("[sonic_dds] Control ownership: G1 body=SONIC DDS (29), Dex3=default hold (14), root=PhysX")
        for motor_index, (name, articulation_index) in enumerate(zip(G1_29DOF_DDS_JOINT_ORDER, body_indices)):
            print(f"[sonic_dds] motor[{motor_index:02d}] -> joint[{articulation_index:02d}] {name}")
        print("[sonic_dds] Body mapping 29/29, Dex3 hold mapping 14/14")

    def get_action(self, env) -> Optional[torch.Tensor]:
        del env
        now = time.monotonic()
        command = self.robot_dds.get_robot_command()

        if not command or "motor_cmd" not in command:
            return self._hold_action(now, "waiting for the first rt/lowcmd")

        motor_cmd = command["motor_cmd"]
        positions = motor_cmd.get("positions", [])
        if len(positions) < 29:
            return self._hold_action(now, f"rt/lowcmd contains only {len(positions)} motors")

        received_at = command.get("receive_time_monotonic")
        if received_at is not None and now - float(received_at) > self.command_timeout_s:
            return self._hold_action(now, "rt/lowcmd timed out")

        self._incoming_positions.copy_(torch.as_tensor(positions[:29], dtype=torch.float32, device=self.device))
        if not bool(torch.isfinite(self._incoming_positions).all()):
            return self._hold_action(now, "rt/lowcmd contains NaN or Inf")

        target = self._default_action.clone()
        target.index_copy_(0, self._body_indices, self._incoming_positions)
        # Phase 1: the Dex3 joints are present in the asset but remain in their
        # configured open/default pose.  Do not accept hand commands yet.
        target.index_copy_(0, self._dex3_indices, self._default_action.index_select(0, self._dex3_indices))

        if self._first_valid_command_time is None:
            self._first_valid_command_time = now

        if self.ramp_duration_s > 0.0:
            alpha = min(1.0, (now - self._first_valid_command_time) / self.ramp_duration_s)
            target = torch.lerp(self._default_action, target, alpha)

        if self.max_target_step > 0.0:
            delta = torch.clamp(target - self._last_action, -self.max_target_step, self.max_target_step)
            target = self._last_action + delta

        self._last_action.copy_(target)
        self._last_valid_command_time = now
        return target.unsqueeze(0)

    def _hold_action(self, now: float, reason: str) -> torch.Tensor:
        if now - self._last_timeout_warning_time >= 1.0:
            print(f"[sonic_dds] HOLD: {reason}")
            self._last_timeout_warning_time = now
        return self._last_action.unsqueeze(0)

    def cleanup(self):
        # DDS lifecycle belongs to dds_manager; this provider only borrows it.
        self.is_running = False
