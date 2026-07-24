"""Direct Gear SONIC DDS control for floating-base G1 SONIC tasks.

This provider owns the 29 G1 body joints.  A task may use either the exact
29-DoF training articulation or the 29+14-DoF Dex3 articulation.  For every
body motor it consumes the complete Unitree LowCmd tuple
``mode, q, dq, tau, kp, kd``.  The 14 Dex3 joints, when present, remain at their
configured defaults until the later OpenXR hand-control phase.
"""

from __future__ import annotations

import time
from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from dds.dds_master import dds_manager
from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER


DEX3_JOINT_MARKERS = ("left_hand_", "right_hand_")
SONIC_BRIDGE_CONTROL_MODE = 0xA2
SONIC_ACTION_TERMS = ("joint_pos", "joint_vel", "joint_effort")
SONIC_ACTION_FIELDS_PER_JOINT = 3


class SonicDDSActionProvider(ActionProvider):
    """Map the complete 29-motor SONIC LowCmd to Isaac Lab commands."""

    def __init__(self, env, args_cli):
        super().__init__("sonic_dds")

        self.env = env
        self.robot = env.scene["robot"]
        self.device = env.device
        self.robot_dds = dds_manager.get_object("g129")
        if self.robot_dds is None:
            raise RuntimeError("G1 DDS object 'g129' is not registered")

        self.command_timeout_s = max(0.01, float(getattr(args_cli, "sonic_lowcmd_timeout", 0.10)))
        self.ramp_duration_s = max(0.0, float(getattr(args_cli, "sonic_ramp_seconds", 0.0)))
        self.max_target_step = max(0.0, float(getattr(args_cli, "sonic_max_target_step", 0.0)))
        self.metrics_interval_s = max(1.0, float(getattr(args_cli, "stats_interval", 10.0)))

        joint_names = list(self.robot.data.joint_names)
        self._num_joints = len(joint_names)
        if self.robot.data.default_joint_pos.shape[0] != 1:
            raise ValueError("SONIC DDS control currently requires exactly one Isaac environment")

        action_manager = getattr(env, "action_manager", None)
        expected_action_dim = SONIC_ACTION_FIELDS_PER_JOINT * self._num_joints
        if action_manager is not None:
            active_terms = tuple(action_manager.active_terms)
            if active_terms != SONIC_ACTION_TERMS:
                raise ValueError(
                    "SONIC action terms must be ordered as "
                    f"{SONIC_ACTION_TERMS}, got {active_terms}"
                )
            if action_manager.total_action_dim != expected_action_dim:
                raise ValueError(
                    "SONIC action dimension must contain q/dq/tau for every joint: "
                    f"expected {expected_action_dim}, got {action_manager.total_action_dim}"
                )

        joint_to_index = {name: index for index, name in enumerate(joint_names)}
        missing = [name for name in G1_29DOF_DDS_JOINT_ORDER if name not in joint_to_index]
        if missing:
            raise ValueError(f"G1 articulation is missing SONIC body joints: {missing}")

        body_indices = [joint_to_index[name] for name in G1_29DOF_DDS_JOINT_ORDER]
        if len(set(body_indices)) != len(G1_29DOF_DDS_JOINT_ORDER):
            raise ValueError("SONIC 29-DoF mapping contains duplicate articulation indices")

        dex3_indices = [
            index for index, name in enumerate(joint_names) if name.startswith(DEX3_JOINT_MARKERS)
        ]
        if len(dex3_indices) not in (0, 14):
            raise ValueError(
                "SONIC tasks require either no articulated Dex3 joints or exactly 14; "
                f"found {len(dex3_indices)}"
            )

        self._body_indices = torch.tensor(body_indices, dtype=torch.long, device=self.device)
        self._dex3_indices = torch.tensor(dex3_indices, dtype=torch.long, device=self.device)
        self._default_position_target = self.robot.data.default_joint_pos[0].clone()
        self._default_velocity_target = self.robot.data.default_joint_vel[0].clone()
        self._default_effort_target = torch.zeros_like(self._default_position_target)
        self._default_kp = self.robot.data.default_joint_stiffness[0].clone()
        self._default_kd = self.robot.data.default_joint_damping[0].clone()
        self._default_root_state = self.robot.data.default_root_state.clone()

        self._last_position_target = self._default_position_target.clone()
        self._last_velocity_target = self._default_velocity_target.clone()
        self._last_effort_target = self._default_effort_target.clone()
        self._last_kp = self._default_kp.clone()
        self._last_kd = self._default_kd.clone()
        self._last_applied_kp: Optional[torch.Tensor] = None
        self._last_applied_kd: Optional[torch.Tensor] = None
        self._gain_update_count = 0

        self._incoming_modes = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_positions = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_velocities = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_torques = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_kp = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_kd = torch.empty(29, dtype=torch.float32, device=self.device)

        self._first_valid_command_time: Optional[float] = None
        self._last_valid_command_time: Optional[float] = None
        self._last_timeout_warning_time = 0.0
        self._control_started = False
        self._full_lowcmd_logged = False
        self._damping_only_active = False
        self._fall_recovery_started_time: Optional[float] = None
        self._fall_recovery_hold_until = 0.0
        self._fall_recovery_waiting_for_control = False
        self._fall_recovery_blend_start_time: Optional[float] = None
        self._fall_recovery_blend_duration_s = 0.0
        self._fall_recovery_count = 0
        self._metrics_last_report_time = time.monotonic()
        self._metrics_total_samples = 0
        self._reset_metrics_window()

        hand_description = "default hold (14)" if dex3_indices else "not articulated (0)"
        print(
            "[sonic_dds] Control ownership: "
            f"articulation={self._num_joints}, G1 body=SONIC DDS (29), "
            f"Dex3={hand_description}, root=PhysX"
        )
        print(
            "[sonic_dds] LowCmd execution: mode/q/dq/tau/kp/kd enabled; "
            f"environment action={expected_action_dim} (q/dq/tau field-major)"
        )
        print("[sonic_dds] Startup hold: pinning the default root state until the SONIC CONTROL marker")
        for motor_index, (name, articulation_index) in enumerate(
            zip(G1_29DOF_DDS_JOINT_ORDER, body_indices)
        ):
            print(f"[sonic_dds] motor[{motor_index:02d}] -> joint[{articulation_index:02d}] {name}")
        print(
            f"[sonic_dds] Body mapping 29/29, Dex3 hold mapping "
            f"{len(dex3_indices)}/{len(dex3_indices)}"
        )

    @property
    def control_started(self) -> bool:
        return self._control_started

    @property
    def fall_recovery_active(self) -> bool:
        return (
            self._fall_recovery_waiting_for_control
            or self._fall_recovery_blend_start_time is not None
        )

    def begin_fall_recovery(
        self,
        *,
        hold_duration_s: float,
        blend_duration_s: float,
        reason: str,
    ) -> None:
        """Discard pre-reset targets and prepare a safe post-reset SONIC hand-off."""
        now = time.monotonic()
        self._report_metrics(now, force=True)
        self._fall_recovery_count += 1
        self._fall_recovery_started_time = now
        self._fall_recovery_hold_until = now + max(0.0, float(hold_duration_s))
        self._fall_recovery_blend_duration_s = max(0.0, float(blend_duration_s))
        self._fall_recovery_blend_start_time = None
        self._fall_recovery_waiting_for_control = self._control_started
        self._restore_default_command()
        self._last_valid_command_time = None
        self._metrics_last_report_time = now
        self._reset_metrics_window()

        if self._control_started:
            print(
                f"[sonic_dds] FALL RECOVERY #{self._fall_recovery_count}: {reason}; "
                f"holding the default standing pose for {max(0.0, float(hold_duration_s)):.2f}s, "
                "discarding pre-reset LowCmd, then waiting for a fresh CONTROL packet"
            )
        else:
            print(
                f"[sonic_dds] RESET #{self._fall_recovery_count}: {reason}; "
                "SONIC control has not started, keeping the normal startup hold"
            )

    def get_action(self, env) -> Optional[torch.Tensor]:
        del env
        now = time.monotonic()

        if self._fall_recovery_waiting_for_control and now < self._fall_recovery_hold_until:
            remaining_s = self._fall_recovery_hold_until - now
            return self._hold_action(
                now,
                f"post-reset standing stabilization ({remaining_s:.2f}s remaining)",
            )

        command = self.robot_dds.get_robot_command()
        if not command or "motor_cmd" not in command:
            return self._hold_action(now, "waiting for the first rt/lowcmd")

        motor_cmd = command["motor_cmd"]
        command_fields = (
            ("modes", self._incoming_modes),
            ("positions", self._incoming_positions),
            ("velocities", self._incoming_velocities),
            ("torques", self._incoming_torques),
            ("kp", self._incoming_kp),
            ("kd", self._incoming_kd),
        )
        for field_name, destination in command_fields:
            values = motor_cmd.get(field_name, [])
            if len(values) < 29:
                return self._hold_action(
                    now,
                    f"rt/lowcmd contains only {len(values)} {field_name} values",
                )
            try:
                destination.copy_(
                    torch.as_tensor(values[:29], dtype=torch.float32, device=self.device)
                )
            except (TypeError, ValueError, RuntimeError) as exc:
                return self._hold_action(
                    now,
                    f"rt/lowcmd contains invalid {field_name} data: {exc}",
                )

        incoming_tensors = tuple(destination for _, destination in command_fields)
        if not all(bool(torch.isfinite(values).all()) for values in incoming_tensors):
            return self._hold_action(now, "rt/lowcmd contains NaN or Inf")
        if not bool(torch.all(self._incoming_modes == torch.round(self._incoming_modes))):
            return self._hold_action(now, "rt/lowcmd contains a non-integer motor mode")
        if not bool(torch.all((self._incoming_modes == 0.0) | (self._incoming_modes == 1.0))):
            return self._hold_action(now, "rt/lowcmd contains an unsupported motor mode")
        if bool(torch.any(self._incoming_kp < 0.0)) or bool(torch.any(self._incoming_kd < 0.0)):
            return self._hold_action(now, "rt/lowcmd contains a negative kp or kd")

        received_at = command.get("receive_time_monotonic")
        received_at_value = None
        if received_at is not None:
            try:
                received_at_value = float(received_at)
            except (TypeError, ValueError):
                return self._hold_action(now, "rt/lowcmd contains an invalid receive timestamp")
        if received_at_value is not None and now - received_at_value > self.command_timeout_s:
            return self._hold_action(now, "rt/lowcmd timed out")

        if self._fall_recovery_waiting_for_control:
            if received_at_value is None:
                return self._hold_action(now, "post-reset rt/lowcmd has no freshness timestamp")
            if (
                self._fall_recovery_started_time is not None
                and received_at_value <= self._fall_recovery_started_time
            ):
                return self._hold_action(now, "discarding pre-reset rt/lowcmd")

        try:
            bridge_mode = int(command.get("mode_machine", -1))
        except (TypeError, ValueError):
            return self._hold_action(now, "rt/lowcmd contains an invalid mode_machine value")

        # Outside the explicit fall-recovery transaction, root ownership is a
        # one-way transition. Before the first A2 marker, INIT/WAIT commands are
        # applied while the configured root state is pinned. Once A2 has
        # released the floating base, A0/A1 must never teleport it to spawn.
        if bridge_mode != SONIC_BRIDGE_CONTROL_MODE and self._control_started:
            if self._fall_recovery_waiting_for_control:
                reason = (
                    "post-reset waiting for fresh SONIC CONTROL marker "
                    f"(mode=0x{bridge_mode & 0xFF:02x})"
                )
            else:
                reason = (
                    "ignoring non-CONTROL marker after root release "
                    f"(mode=0x{bridge_mode & 0xFF:02x})"
                )
            return self._hold_action(now, reason)

        motor_enabled = self._incoming_modes > 0.5
        body_kp = torch.where(motor_enabled, self._incoming_kp, 0.0)
        body_kd = torch.where(motor_enabled, self._incoming_kd, 0.0)
        body_tau = torch.where(motor_enabled, self._incoming_torques, 0.0)

        position_target = self._default_position_target.clone()
        velocity_target = self._default_velocity_target.clone()
        effort_target = self._default_effort_target.clone()
        kp_target = self._default_kp.clone()
        kd_target = self._default_kd.clone()
        position_target.index_copy_(0, self._body_indices, self._incoming_positions)
        velocity_target.index_copy_(0, self._body_indices, self._incoming_velocities)
        effort_target.index_copy_(0, self._body_indices, body_tau)
        kp_target.index_copy_(0, self._body_indices, body_kp)
        kd_target.index_copy_(0, self._body_indices, body_kd)

        # Phase 1: Dex3 is articulated but receives no hand command yet.  The
        # default clones above keep q/dq/tau/kp/kd at the configured hand hold.
        if self._dex3_indices.numel() > 0:
            position_target.index_copy_(
                0,
                self._dex3_indices,
                self._default_position_target.index_select(0, self._dex3_indices),
            )

        if bridge_mode != SONIC_BRIDGE_CONTROL_MODE:
            self._pin_initial_root_state()
            if now - self._last_timeout_warning_time >= 1.0:
                print(
                    "[sonic_dds] STARTUP HOLD: waiting for SONIC CONTROL marker "
                    f"(mode=0x{bridge_mode & 0xFF:02x})"
                )
                self._last_timeout_warning_time = now
        elif not self._control_started:
            self._control_started = True
            self._first_valid_command_time = now
            self._metrics_last_report_time = now
            self._metrics_total_samples = 0
            self._reset_metrics_window()
            print("[sonic_dds] CONTROL marker received: releasing the floating base to PhysX")
        elif self._fall_recovery_waiting_for_control:
            self._fall_recovery_waiting_for_control = False
            self._fall_recovery_blend_start_time = now
            self._restore_default_command()
            print(
                "[sonic_dds] FALL RECOVERY: fresh CONTROL packet received; "
                f"releasing the root and blending SONIC targets over "
                f"{self._fall_recovery_blend_duration_s:.2f}s"
            )

        if self._control_started and self.ramp_duration_s > 0.0:
            alpha = min(1.0, (now - self._first_valid_command_time) / self.ramp_duration_s)
            position_target = torch.lerp(self._default_position_target, position_target, alpha)
            velocity_target = torch.lerp(self._default_velocity_target, velocity_target, alpha)
            effort_target = torch.lerp(self._default_effort_target, effort_target, alpha)
            kp_target = torch.lerp(self._default_kp, kp_target, alpha)
            kd_target = torch.lerp(self._default_kd, kd_target, alpha)

        if self._fall_recovery_blend_start_time is not None:
            if self._fall_recovery_blend_duration_s <= 0.0:
                recovery_alpha = 1.0
            else:
                recovery_alpha = min(
                    1.0,
                    (now - self._fall_recovery_blend_start_time)
                    / self._fall_recovery_blend_duration_s,
                )
            position_target = torch.lerp(
                self._default_position_target, position_target, recovery_alpha
            )
            velocity_target = torch.lerp(
                self._default_velocity_target, velocity_target, recovery_alpha
            )
            effort_target = torch.lerp(
                self._default_effort_target, effort_target, recovery_alpha
            )
            kp_target = torch.lerp(self._default_kp, kp_target, recovery_alpha)
            kd_target = torch.lerp(self._default_kd, kd_target, recovery_alpha)
            if recovery_alpha >= 1.0:
                self._fall_recovery_blend_start_time = None
                print("[sonic_dds] FALL RECOVERY complete: normal SONIC control restored")

        if self.max_target_step > 0.0:
            delta = torch.clamp(
                position_target - self._last_position_target,
                -self.max_target_step,
                self.max_target_step,
            )
            position_target = self._last_position_target + delta

        # Motor enable is a hard safety gate, not a quantity to interpolate.
        # Re-apply it after startup/fall-recovery blending so a mode=0 motor
        # cannot temporarily inherit non-zero default gains during the blend.
        if bool(torch.any(~motor_enabled)):
            body_effort = effort_target.index_select(0, self._body_indices)
            body_kp_target = kp_target.index_select(0, self._body_indices)
            body_kd_target = kd_target.index_select(0, self._body_indices)
            effort_target.index_copy_(
                0,
                self._body_indices,
                torch.where(motor_enabled, body_effort, 0.0),
            )
            kp_target.index_copy_(
                0,
                self._body_indices,
                torch.where(motor_enabled, body_kp_target, 0.0),
            )
            kd_target.index_copy_(
                0,
                self._body_indices,
                torch.where(motor_enabled, body_kd_target, 0.0),
            )

        damping_only = (
            bool(torch.all(torch.abs(body_kp) <= 1.0e-6))
            and bool(torch.all(torch.abs(body_tau) <= 1.0e-6))
            and bool(torch.any(body_kd > 1.0e-6))
        )
        if damping_only and not self._damping_only_active:
            print(
                "[sonic_dds] Applying SONIC damping-only LowCmd "
                "(q is inactive because kp=0; dq/tau/kd remain effective)"
            )
        elif self._damping_only_active and not damping_only:
            print("[sonic_dds] Leaving damping-only LowCmd; active PD control restored")
        self._damping_only_active = damping_only

        self._apply_joint_gains(kp_target, kd_target)
        self._last_position_target.copy_(position_target)
        self._last_velocity_target.copy_(velocity_target)
        self._last_effort_target.copy_(effort_target)
        self._last_kp.copy_(kp_target)
        self._last_kd.copy_(kd_target)
        self._last_valid_command_time = now

        if not self._full_lowcmd_logged:
            enabled_count = int(torch.count_nonzero(motor_enabled).item())
            print(
                "[sonic_dds] First complete LowCmd applied: "
                f"enabled={enabled_count}/29, "
                f"max|dq_target|={float(torch.abs(self._incoming_velocities).max().item()):.4f}, "
                f"max|tau_ff|={float(torch.abs(body_tau).max().item()):.4f}, "
                f"kp=[{float(body_kp.min().item()):.4f}, {float(body_kp.max().item()):.4f}], "
                f"kd=[{float(body_kd.min().item()):.4f}, {float(body_kd.max().item()):.4f}]"
            )
            self._full_lowcmd_logged = True

        if self._control_started:
            self._update_metrics(
                now,
                position_target,
                velocity_target,
                effort_target,
                kp_target,
                kd_target,
            )
        return self._pack_action(position_target, velocity_target, effort_target)

    def _pack_action(
        self,
        position_target: torch.Tensor,
        velocity_target: torch.Tensor,
        effort_target: torch.Tensor,
    ) -> torch.Tensor:
        """Return the field-major tensor expected by the three action terms."""
        return torch.cat((position_target, velocity_target, effort_target), dim=0).unsqueeze(0)

    def _restore_default_command(self) -> None:
        self._last_position_target.copy_(self._default_position_target)
        self._last_velocity_target.copy_(self._default_velocity_target)
        self._last_effort_target.copy_(self._default_effort_target)
        self._last_kp.copy_(self._default_kp)
        self._last_kd.copy_(self._default_kd)
        self._damping_only_active = False

    def _apply_joint_gains(self, kp: torch.Tensor, kd: torch.Tensor) -> None:
        """Write LowCmd gains to PhysX and keep Isaac Lab's actuator buffers in sync."""
        gains_unchanged = (
            self._last_applied_kp is not None
            and self._last_applied_kd is not None
            and torch.equal(kp, self._last_applied_kp)
            and torch.equal(kd, self._last_applied_kd)
        )
        if gains_unchanged:
            return

        kp_batch = kp.unsqueeze(0)
        kd_batch = kd.unsqueeze(0)
        for actuator in self.robot.actuators.values():
            joint_indices = actuator.joint_indices
            actuator.stiffness.copy_(kp_batch[:, joint_indices])
            actuator.damping.copy_(kd_batch[:, joint_indices])

        self.robot.write_joint_stiffness_to_sim(kp_batch)
        self.robot.write_joint_damping_to_sim(kd_batch)
        self._last_applied_kp = kp.clone()
        self._last_applied_kd = kd.clone()
        self._gain_update_count += 1

    def _reset_metrics_window(self) -> None:
        self._metrics_samples = 0
        self._metrics_joint_abs_sum = torch.zeros(
            len(G1_29DOF_DDS_JOINT_ORDER), dtype=torch.float32, device=self.device
        )
        self._metrics_joint_max = torch.zeros(
            len(G1_29DOF_DDS_JOINT_ORDER), dtype=torch.float32, device=self.device
        )
        self._metrics_pd_target_abs_sum = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_pd_target_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_tilt_sum = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_tilt_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_base_z_min = torch.full((), float("inf"), dtype=torch.float32, device=self.device)
        self._metrics_base_z_last = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_joint_speed_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_command_dq_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_command_tau_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_kp_min = torch.full((), float("inf"), dtype=torch.float32, device=self.device)
        self._metrics_kp_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_kd_min = torch.full((), float("inf"), dtype=torch.float32, device=self.device)
        self._metrics_kd_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_torque_ratio_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_torque_saturated_count = 0
        self._metrics_torque_value_count = 0

    def _update_metrics(
        self,
        now: float,
        position_target: torch.Tensor,
        velocity_target: torch.Tensor,
        effort_target: torch.Tensor,
        kp_target: torch.Tensor,
        kd_target: torch.Tensor,
    ) -> None:
        actual_body_pos = self.robot.data.joint_pos[0].index_select(0, self._body_indices)
        target_body_pos = position_target.index_select(0, self._body_indices)
        body_vel = self.robot.data.joint_vel[0].index_select(0, self._body_indices)
        body_command_vel = velocity_target.index_select(0, self._body_indices)
        body_command_tau = effort_target.index_select(0, self._body_indices)
        body_kp = kp_target.index_select(0, self._body_indices)
        body_kd = kd_target.index_select(0, self._body_indices)
        # This is displacement from the controller's virtual PD equilibrium,
        # not reference-motion tracking error.  Non-zero support displacement
        # is expected, especially at the ankles.
        pd_target_abs = torch.abs(actual_body_pos - target_body_pos)

        root_quat = self.robot.data.root_quat_w[0]
        root_quat = root_quat / torch.clamp(torch.linalg.vector_norm(root_quat), min=1.0e-6)
        vertical_cosine = torch.clamp(
            1.0 - 2.0 * (root_quat[1] * root_quat[1] + root_quat[2] * root_quat[2]),
            -1.0,
            1.0,
        )
        tilt_deg = torch.rad2deg(torch.acos(vertical_cosine))
        base_z = self.robot.data.root_pos_w[0, 2]

        self._metrics_samples += 1
        self._metrics_total_samples += 1
        self._metrics_joint_abs_sum += pd_target_abs
        self._metrics_joint_max = torch.maximum(self._metrics_joint_max, pd_target_abs)
        self._metrics_pd_target_abs_sum += pd_target_abs.sum()
        self._metrics_pd_target_max = torch.maximum(self._metrics_pd_target_max, pd_target_abs.max())
        self._metrics_tilt_sum += tilt_deg
        self._metrics_tilt_max = torch.maximum(self._metrics_tilt_max, tilt_deg)
        self._metrics_base_z_min = torch.minimum(self._metrics_base_z_min, base_z)
        self._metrics_base_z_last.copy_(base_z)
        self._metrics_joint_speed_max = torch.maximum(
            self._metrics_joint_speed_max,
            torch.abs(body_vel).max(),
        )
        self._metrics_command_dq_max = torch.maximum(
            self._metrics_command_dq_max,
            torch.abs(body_command_vel).max(),
        )
        self._metrics_command_tau_max = torch.maximum(
            self._metrics_command_tau_max,
            torch.abs(body_command_tau).max(),
        )
        self._metrics_kp_min = torch.minimum(self._metrics_kp_min, body_kp.min())
        self._metrics_kp_max = torch.maximum(self._metrics_kp_max, body_kp.max())
        self._metrics_kd_min = torch.minimum(self._metrics_kd_min, body_kd.min())
        self._metrics_kd_max = torch.maximum(self._metrics_kd_max, body_kd.max())

        computed_torque = getattr(self.robot.data, "computed_torque", None)
        effort_limits = getattr(self.robot.data, "joint_effort_limits", None)
        if computed_torque is not None and effort_limits is not None:
            body_computed_torque = computed_torque[0].index_select(0, self._body_indices)
            body_effort_limits = effort_limits[0].index_select(0, self._body_indices)
            valid_limits = torch.isfinite(body_effort_limits) & (body_effort_limits > 0.0)
            if bool(torch.any(valid_limits)):
                torque_ratio = torch.abs(body_computed_torque[valid_limits]) / body_effort_limits[valid_limits]
                self._metrics_torque_ratio_max = torch.maximum(
                    self._metrics_torque_ratio_max,
                    torque_ratio.max(),
                )
                self._metrics_torque_saturated_count += int(
                    torch.count_nonzero(torque_ratio >= 0.98).item()
                )
                self._metrics_torque_value_count += int(torque_ratio.numel())

        if now - self._metrics_last_report_time >= self.metrics_interval_s:
            self._report_metrics(now)

    def _report_metrics(self, now: float, *, force: bool = False) -> None:
        if self._metrics_samples == 0:
            return
        if not force and now - self._metrics_last_report_time < self.metrics_interval_s:
            return

        joint_value_count = self._metrics_samples * len(G1_29DOF_DDS_JOINT_ORDER)
        pd_target_mae = float((self._metrics_pd_target_abs_sum / joint_value_count).item())
        pd_target_max = float(self._metrics_pd_target_max.item())
        tilt_mean = float((self._metrics_tilt_sum / self._metrics_samples).item())
        tilt_max = float(self._metrics_tilt_max.item())
        base_z_min = float(self._metrics_base_z_min.item())
        base_z_last = float(self._metrics_base_z_last.item())
        joint_speed_max = float(self._metrics_joint_speed_max.item())
        command_dq_max = float(self._metrics_command_dq_max.item())
        command_tau_max = float(self._metrics_command_tau_max.item())
        kp_min = float(self._metrics_kp_min.item())
        kp_max = float(self._metrics_kp_max.item())
        kd_min = float(self._metrics_kd_min.item())
        kd_max = float(self._metrics_kd_max.item())
        torque_ratio_max = float(self._metrics_torque_ratio_max.item())
        if self._metrics_torque_value_count:
            torque_saturation_pct = (
                100.0 * self._metrics_torque_saturated_count / self._metrics_torque_value_count
            )
        else:
            torque_saturation_pct = 0.0

        per_joint_mae = self._metrics_joint_abs_sum / self._metrics_samples
        top_count = min(5, len(G1_29DOF_DDS_JOINT_ORDER))
        top_values, top_indices = torch.topk(per_joint_mae, k=top_count)
        top_values_list = top_values.cpu().tolist()
        top_indices_list = top_indices.cpu().tolist()
        per_joint_max_list = self._metrics_joint_max.index_select(0, top_indices).cpu().tolist()

        print(
            "[sonic_dds][metrics] "
            f"samples={self._metrics_samples}, total={self._metrics_total_samples}, "
            f"tilt_mean={tilt_mean:.2f}deg, tilt_max={tilt_max:.2f}deg, "
            f"base_z={base_z_last:.3f}m, base_z_min={base_z_min:.3f}m, "
            f"pd_target_mae={pd_target_mae:.4f}rad, pd_target_max={pd_target_max:.4f}rad, "
            f"max_abs_dq={joint_speed_max:.2f}rad/s, "
            f"cmd_max_abs_dq={command_dq_max:.4f}rad/s, "
            f"cmd_max_abs_tau={command_tau_max:.4f}Nm, "
            f"kp=[{kp_min:.3f},{kp_max:.3f}], kd=[{kd_min:.3f},{kd_max:.3f}], "
            f"torque_ratio_max={torque_ratio_max:.3f}, "
            f"torque_sat={torque_saturation_pct:.2f}%, gain_updates={self._gain_update_count}"
        )
        top_joint_text = ", ".join(
            f"{G1_29DOF_DDS_JOINT_ORDER[index]}="
            f"{mean_error:.4f}/{max_error:.4f}rad(mean/max)"
            for index, mean_error, max_error in zip(
                top_indices_list,
                top_values_list,
                per_joint_max_list,
            )
        )
        print(f"[sonic_dds][joint_metrics] top_pd_target_errors: {top_joint_text}")
        self._metrics_last_report_time = now
        self._reset_metrics_window()

    def _hold_action(self, now: float, reason: str) -> torch.Tensor:
        # Keep startup/fall-reset root pinning behavior.  On an ordinary DDS
        # timeout preserve the last position and gains, but remove stale motion
        # and feed-forward commands: continuing a non-zero dq/tau indefinitely
        # would be unsafe when the publisher has disappeared.
        if not self._control_started or self._fall_recovery_waiting_for_control:
            self._pin_initial_root_state()

        self._apply_joint_gains(self._last_kp, self._last_kd)
        if now - self._last_timeout_warning_time >= 1.0:
            print(f"[sonic_dds] HOLD: {reason}")
            self._last_timeout_warning_time = now
        return self._pack_action(
            self._last_position_target,
            self._default_velocity_target,
            self._default_effort_target,
        )

    def _pin_initial_root_state(self) -> None:
        self.robot.write_root_state_to_sim(self._default_root_state)

    def cleanup(self):
        # DDS lifecycle belongs to dds_manager; this provider only borrows it.
        self._report_metrics(time.monotonic(), force=True)
        self.is_running = False
