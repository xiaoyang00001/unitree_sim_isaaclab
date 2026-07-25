"""Direct Gear SONIC DDS control for floating-base G1 SONIC tasks.

This provider owns the 29 G1 body joints and, when present, the 14 articulated
Dex3 joints.  It consumes the complete Unitree command tuple
``mode, q, dq, tau, kp, kd`` for both LowCmd and the two Dex3 HandCmd topics.
Body and hand freshness are checked independently so a stale hand packet can
never pause or destabilize the locomotion loop.
"""

from __future__ import annotations

import time
import math
from typing import Optional

import torch

from action_provider.action_base import ActionProvider
from dds.dds_master import dds_manager
from robots.g1_joint_order import G1_29DOF_DDS_JOINT_ORDER
from robots.g1_sonic_urdf import DEX3_HAND_JOINT_NAMES


DEX3_JOINT_MARKERS = ("left_hand_", "right_hand_")
DEX3_JOINTS_PER_HAND = 7
DEX3_SIDES = ("left", "right")
DEX3_COMMAND_FIELDS = ("modes", "positions", "velocities", "torques", "kp", "kd")
SONIC_BRIDGE_CONTROL_MODE = 0xA2
SONIC_ACTION_TERMS = ("joint_pos", "joint_vel", "joint_effort")
SONIC_ACTION_FIELDS_PER_JOINT = 3
SONIC_LOWCMD_SYNC_MAGIC = 0x534E4331
BODY_GROUP_RANGES = {
    "legs": range(0, 12),
    "waist": range(12, 15),
    "arms": range(15, 29),
}

# Fallback limits use the articulated Unitree URDF order. At runtime the
# imported articulation limits take precedence. These constants keep command
# validation deterministic in unit tests and if an Isaac version does not
# expose joint position limits through ArticulationData.
DEX3_FALLBACK_LOWER_LIMITS = (
    -1.04719755,
    -0.72431163,
    0.0,
    -1.57079632,
    -1.74532925,
    -1.57079632,
    -1.74532925,
    -1.04719755,
    -1.04719755,
    -1.74532925,
    0.0,
    0.0,
    0.0,
    0.0,
)
DEX3_FALLBACK_UPPER_LIMITS = (
    1.04719755,
    1.04719755,
    1.74532925,
    0.0,
    0.0,
    0.0,
    0.0,
    1.04719755,
    0.72431163,
    0.0,
    1.57079632,
    1.74532925,
    1.57079632,
    1.74532925,
)


class SonicDDSActionProvider(ActionProvider):
    """Map complete SONIC body and optional Dex3 commands to Isaac Lab."""

    def __init__(self, env, args_cli):
        super().__init__("sonic_dds")

        self.env = env
        self.robot = env.scene["robot"]
        self.device = env.device
        self.robot_dds = dds_manager.get_object("g129")
        if self.robot_dds is None:
            raise RuntimeError("G1 DDS object 'g129' is not registered")

        self.command_timeout_s = max(0.01, float(getattr(args_cli, "sonic_lowcmd_timeout", 0.10)))
        self.hand_command_timeout_s = max(
            0.01, float(getattr(args_cli, "sonic_handcmd_timeout", 0.20))
        )
        # Matches Dex3Hands::MAX_DELTA_Q in Gear SONIC. This bounds the
        # instantaneous PD error against the actual simulated finger state,
        # rather than adding latency with a wall-clock command filter.
        self.hand_max_target_error = max(
            0.0, float(getattr(args_cli, "sonic_hand_max_target_error", 0.25))
        )
        self.ramp_duration_s = max(0.0, float(getattr(args_cli, "sonic_ramp_seconds", 0.0)))
        self.max_target_step = max(0.0, float(getattr(args_cli, "sonic_max_target_step", 0.0)))
        self.group_max_target_step = {
            "legs": max(0.0, float(getattr(args_cli, "sonic_leg_max_target_step", 0.0))),
            "waist": max(0.0, float(getattr(args_cli, "sonic_waist_max_target_step", 0.0))),
            "arms": max(0.0, float(getattr(args_cli, "sonic_arm_max_target_step", 0.0))),
        }
        self.sync_with_lowstate = bool(
            getattr(args_cli, "sonic_sync_with_lowstate", False)
        )
        self.sync_wait_timeout_s = max(
            0.01, float(getattr(args_cli, "sonic_sync_wait_timeout", 0.25))
        )
        self.sync_poll_interval_s = max(
            0.0002, float(getattr(args_cli, "sonic_sync_poll_interval", 0.001))
        )
        self.metrics_interval_s = max(1.0, float(getattr(args_cli, "stats_interval", 10.0)))
        self._step_dt = float(getattr(env, "step_dt", 0.02))

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

        articulated_hand_names = [
            name for name in joint_names if name.startswith(DEX3_JOINT_MARKERS)
        ]
        if len(articulated_hand_names) not in (0, 14):
            raise ValueError(
                "SONIC tasks require either no articulated Dex3 joints or exactly 14; "
                f"found {len(articulated_hand_names)}"
            )
        if articulated_hand_names:
            missing_hand_names = [
                name for name in DEX3_HAND_JOINT_NAMES if name not in joint_to_index
            ]
            unexpected_hand_names = sorted(
                set(articulated_hand_names) - set(DEX3_HAND_JOINT_NAMES)
            )
            if missing_hand_names or unexpected_hand_names:
                raise ValueError(
                    "Dex3 articulation does not match the Unitree 7+7 DDS mapping: "
                    f"missing={missing_hand_names}, unexpected={unexpected_hand_names}"
                )
            dex3_indices = [joint_to_index[name] for name in DEX3_HAND_JOINT_NAMES]
        else:
            dex3_indices = []

        self._body_indices = torch.tensor(body_indices, dtype=torch.long, device=self.device)
        self._body_group_indices = {
            group_name: torch.tensor(
                [body_indices[index] for index in motor_range],
                dtype=torch.long,
                device=self.device,
            )
            for group_name, motor_range in BODY_GROUP_RANGES.items()
        }
        self._dex3_indices = torch.tensor(dex3_indices, dtype=torch.long, device=self.device)
        self._dex3_side_indices = {
            side: self._dex3_indices[
                side_index * DEX3_JOINTS_PER_HAND : (side_index + 1) * DEX3_JOINTS_PER_HAND
            ]
            for side_index, side in enumerate(DEX3_SIDES)
        }
        self.dex3_dds = None
        if dex3_indices:
            self.dex3_dds = dds_manager.get_object("dex3")
            if self.dex3_dds is None:
                raise RuntimeError(
                    "43-DoF SONIC task requires the Dex3 DDS object; "
                    "sim_main should enable it automatically"
                )
        self._foot_contact_sensor = None
        self._foot_body_indices = torch.empty(0, dtype=torch.long, device=self.device)
        try:
            self._foot_contact_sensor = env.scene["foot_contact"]
            robot_body_names = getattr(
                self.robot.data,
                "body_names",
                getattr(self.robot, "body_names", ()),
            )
            body_name_to_index = {
                name: index for index, name in enumerate(robot_body_names)
            }
            foot_body_indices = [
                body_name_to_index[name]
                for name in self._foot_contact_sensor.body_names
                if name in body_name_to_index
            ]
            self._foot_body_indices = torch.tensor(
                foot_body_indices, dtype=torch.long, device=self.device
            )
        except (KeyError, AttributeError, TypeError):
            self._foot_contact_sensor = None
        self._default_position_target = self.robot.data.default_joint_pos[0].clone()
        self._default_velocity_target = self.robot.data.default_joint_vel[0].clone()
        self._default_effort_target = torch.zeros_like(self._default_position_target)
        self._default_kp = self.robot.data.default_joint_stiffness[0].clone()
        self._default_kd = self.robot.data.default_joint_damping[0].clone()
        self._default_root_state = self.robot.data.default_root_state.clone()

        self._dex3_lower_limits = torch.tensor(
            DEX3_FALLBACK_LOWER_LIMITS,
            dtype=torch.float32,
            device=self.device,
        )
        self._dex3_upper_limits = torch.tensor(
            DEX3_FALLBACK_UPPER_LIMITS,
            dtype=torch.float32,
            device=self.device,
        )
        self._dex3_velocity_limits = torch.full(
            (len(DEX3_HAND_JOINT_NAMES),),
            float("inf"),
            dtype=torch.float32,
            device=self.device,
        )
        self._dex3_effort_limits = torch.full_like(
            self._dex3_velocity_limits, float("inf")
        )
        if dex3_indices:
            self._load_dex3_articulation_limits()

        self._last_position_target = self._default_position_target.clone()
        self._last_velocity_target = self._default_velocity_target.clone()
        self._last_effort_target = self._default_effort_target.clone()
        self._last_kp = self._default_kp.clone()
        self._last_kd = self._default_kd.clone()
        self._last_applied_kp: Optional[torch.Tensor] = None
        self._last_applied_kd: Optional[torch.Tensor] = None
        self._gain_update_count = 0
        self._group_limit_counts = {name: 0 for name in BODY_GROUP_RANGES}

        self._incoming_modes = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_positions = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_velocities = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_torques = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_kp = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_kd = torch.empty(29, dtype=torch.float32, device=self.device)

        self._hand_incoming = {
            side: {
                field_name: torch.empty(
                    DEX3_JOINTS_PER_HAND,
                    dtype=torch.float32,
                    device=self.device,
                )
                for field_name in DEX3_COMMAND_FIELDS
            }
            for side in DEX3_SIDES
        }
        self._hand_last_warning_time = {side: 0.0 for side in DEX3_SIDES}
        self._hand_first_command_logged = {side: False for side in DEX3_SIDES}

        self._first_valid_command_time: Optional[float] = None
        self._last_valid_command_time: Optional[float] = None
        self._last_timeout_warning_time = 0.0
        self._control_started = False
        self._full_lowcmd_logged = False
        self._damping_only_active = False
        self._fall_recovery_started_time: Optional[float] = None
        self._fall_recovery_hold_until = 0.0
        self._fall_recovery_hold_steps_remaining = 0
        self._fall_recovery_waiting_for_control = False
        self._fall_recovery_blend_start_time: Optional[float] = None
        self._fall_recovery_blend_duration_s = 0.0
        self._fall_recovery_blend_steps_total = 0
        self._fall_recovery_blend_step = 0
        self._fall_recovery_count = 0
        self._environment_step_ready = not self.sync_with_lowstate
        self._sync_expected_tick: Optional[int] = None
        self._sync_expected_reset_epoch: Optional[int] = None
        self._sync_last_ack_tick: Optional[int] = None
        self._sync_last_ack_reset_epoch: Optional[int] = None
        self._sync_wait_count = 0
        self._sync_timeout_count = 0
        self._sync_stale_command_count = 0
        self._sync_wait_sum_s = 0.0
        self._sync_wait_max_s = 0.0
        self._metrics_last_report_time = time.monotonic()
        self._metrics_total_samples = 0
        self._reset_metrics_window()

        hand_description = "SONIC Dex3 DDS (14)" if dex3_indices else "not articulated (0)"
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
        print(f"[sonic_dds] Body mapping 29/29, Dex3 mapping {len(dex3_indices)}/14")
        if dex3_indices:
            for hand_index, (name, articulation_index) in enumerate(
                zip(DEX3_HAND_JOINT_NAMES, dex3_indices)
            ):
                print(
                    f"[sonic_dds] dex3[{hand_index:02d}] -> "
                    f"joint[{articulation_index:02d}] {name}"
                )
            print(
                "[sonic_dds] Dex3 execution: mode/q/dq/tau/kp/kd enabled; "
                f"timeout={self.hand_command_timeout_s:.3f}s, "
                f"max_target_error={self.hand_max_target_error:.3f}rad"
            )
        if self.sync_with_lowstate:
            print(
                "[sonic_dds] Isaac lock-step enabled: one unique LowState tick -> "
                "one SONIC command -> one environment step"
            )
        enabled_group_limits = {
            name: value for name, value in self.group_max_target_step.items() if value > 0.0
        }
        if self.max_target_step > 0.0 or enabled_group_limits:
            print(
                "[sonic_dds] Joint-target impulse protection: "
                f"global={self.max_target_step:.4f}rad/step, groups={enabled_group_limits}"
            )
        if self._foot_contact_sensor is not None:
            print(
                "[sonic_dds] Foot diagnostics enabled for bodies: "
                f"{list(self._foot_contact_sensor.body_names)}"
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

    def can_step_environment(self) -> bool:
        """Whether the current action is acknowledged for the latest PhysX state."""
        return self._environment_step_ready

    def _load_dex3_articulation_limits(self) -> None:
        """Cache hand limits in Unitree DDS order from the live articulation."""

        position_limits = getattr(self.robot.data, "joint_pos_limits", None)
        if position_limits is not None:
            selected = position_limits[0].index_select(0, self._dex3_indices)
            if selected.shape == (len(DEX3_HAND_JOINT_NAMES), 2):
                finite = torch.isfinite(selected)
                self._dex3_lower_limits = torch.where(
                    finite[:, 0], selected[:, 0], self._dex3_lower_limits
                ).to(dtype=torch.float32, device=self.device)
                self._dex3_upper_limits = torch.where(
                    finite[:, 1], selected[:, 1], self._dex3_upper_limits
                ).to(dtype=torch.float32, device=self.device)

        velocity_limits = getattr(self.robot.data, "joint_vel_limits", None)
        if velocity_limits is not None:
            selected = velocity_limits[0].index_select(0, self._dex3_indices)
            valid = torch.isfinite(selected) & (selected > 0.0)
            self._dex3_velocity_limits = torch.where(
                valid,
                selected.to(dtype=torch.float32, device=self.device),
                self._dex3_velocity_limits,
            )

        effort_limits = getattr(self.robot.data, "joint_effort_limits", None)
        if effort_limits is not None:
            selected = effort_limits[0].index_select(0, self._dex3_indices)
            valid = torch.isfinite(selected) & (selected > 0.0)
            self._dex3_effort_limits = torch.where(
                valid,
                selected.to(dtype=torch.float32, device=self.device),
                self._dex3_effort_limits,
            )

    def _warn_hand_command(self, side: str, now: float, reason: str) -> None:
        if now - self._hand_last_warning_time[side] >= 1.0:
            print(f"[sonic_dds][dex3:{side}] HOLD: {reason}")
            self._hand_last_warning_time[side] = now

    def _parse_hand_command(
        self,
        side: str,
        command: dict,
        now: float,
    ) -> tuple[dict[str, torch.Tensor], torch.Tensor] | None:
        """Validate one seven-motor HandCmd and decode its Unitree modes."""

        received_at = command.get("receive_time_monotonic")
        try:
            received_at_value = float(received_at)
        except (TypeError, ValueError):
            self._warn_hand_command(side, now, "missing or invalid receive timestamp")
            return None
        command_age = now - received_at_value
        if command_age > self.hand_command_timeout_s:
            self._warn_hand_command(
                side,
                now,
                f"HandCmd timed out (age={command_age * 1000.0:.1f}ms)",
            )
            return None
        if command_age < -1.0:
            self._warn_hand_command(side, now, "HandCmd timestamp is in the future")
            return None
        if (
            self._fall_recovery_waiting_for_control
            and self._fall_recovery_started_time is not None
            and received_at_value <= self._fall_recovery_started_time
        ):
            self._warn_hand_command(side, now, "discarding pre-reset HandCmd")
            return None

        parsed: dict[str, torch.Tensor] = {}
        for field_name in DEX3_COMMAND_FIELDS:
            values = command.get(field_name, [])
            if len(values) != DEX3_JOINTS_PER_HAND:
                self._warn_hand_command(
                    side,
                    now,
                    f"expected 7 {field_name} values, got {len(values)}",
                )
                return None
            destination = self._hand_incoming[side][field_name]
            try:
                destination.copy_(
                    torch.as_tensor(values, dtype=torch.float32, device=self.device)
                )
            except (TypeError, ValueError, RuntimeError) as exc:
                self._warn_hand_command(side, now, f"invalid {field_name}: {exc}")
                return None
            if not bool(torch.isfinite(destination).all()):
                self._warn_hand_command(side, now, f"{field_name} contains NaN or Inf")
                return None
            parsed[field_name] = destination

        modes = parsed["modes"]
        if not bool(torch.all(modes == torch.round(modes))):
            self._warn_hand_command(side, now, "mode contains a non-integer value")
            return None
        mode_int = modes.to(dtype=torch.int64)
        if bool(torch.any((mode_int < 0) | (mode_int > 0xFF))):
            self._warn_hand_command(side, now, "mode is outside uint8 range")
            return None
        expected_motor_ids = torch.arange(
            DEX3_JOINTS_PER_HAND, dtype=torch.int64, device=self.device
        )
        if not bool(torch.equal(mode_int & 0x0F, expected_motor_ids)):
            self._warn_hand_command(side, now, "mode motor IDs do not match slots 0..6")
            return None
        if bool(torch.any(parsed["kp"] < 0.0)) or bool(torch.any(parsed["kd"] < 0.0)):
            self._warn_hand_command(side, now, "negative kp or kd")
            return None

        status = (mode_int >> 4) & 0x07
        timed_out = (mode_int & 0x80) != 0
        motor_enabled = (status != 0) & ~timed_out
        return parsed, motor_enabled

    def _apply_dex3_commands(
        self,
        now: float,
        position_target: torch.Tensor,
        velocity_target: torch.Tensor,
        effort_target: torch.Tensor,
        kp_target: torch.Tensor,
        kd_target: torch.Tensor,
    ) -> torch.Tensor:
        """Apply fresh left/right HandCmd snapshots without blocking the body."""

        hard_disable_mask = torch.zeros(
            len(DEX3_HAND_JOINT_NAMES), dtype=torch.bool, device=self.device
        )
        if self.dex3_dds is None or self._dex3_indices.numel() == 0:
            return hard_disable_mask

        commands = self.dex3_dds.get_hand_commands() or {}
        actual_hand_q = self.robot.data.joint_pos[0].index_select(0, self._dex3_indices)

        for side_index, side in enumerate(DEX3_SIDES):
            side_slice = slice(
                side_index * DEX3_JOINTS_PER_HAND,
                (side_index + 1) * DEX3_JOINTS_PER_HAND,
            )
            side_indices = self._dex3_side_indices[side]
            command = commands.get(f"{side}_hand_cmd")
            if not command:
                self._warn_hand_command(side, now, "waiting for first HandCmd")
                continue

            parsed_result = self._parse_hand_command(side, command, now)
            if parsed_result is None:
                continue
            parsed, motor_enabled = parsed_result

            lower_limits = self._dex3_lower_limits[side_slice]
            upper_limits = self._dex3_upper_limits[side_slice]
            q_target = torch.clamp(parsed["positions"], lower_limits, upper_limits)
            side_actual_q = actual_hand_q[side_slice]
            if self.hand_max_target_error > 0.0:
                q_error = torch.clamp(
                    q_target - side_actual_q,
                    -self.hand_max_target_error,
                    self.hand_max_target_error,
                )
                q_target = side_actual_q + q_error

            velocity_limits = self._dex3_velocity_limits[side_slice]
            dq_target = torch.maximum(
                torch.minimum(parsed["velocities"], velocity_limits),
                -velocity_limits,
            )
            effort_limits = self._dex3_effort_limits[side_slice]
            tau_target = torch.maximum(
                torch.minimum(parsed["torques"], effort_limits),
                -effort_limits,
            )

            zeros = torch.zeros_like(q_target)
            # A disabled/timeout motor is genuinely relaxed. Use actual q as a
            # harmless bookkeeping target and hard-zero all active terms.
            q_target = torch.where(motor_enabled, q_target, side_actual_q)
            dq_target = torch.where(motor_enabled, dq_target, zeros)
            tau_target = torch.where(motor_enabled, tau_target, zeros)
            hand_kp = torch.where(motor_enabled, parsed["kp"], zeros)
            hand_kd = torch.where(motor_enabled, parsed["kd"], zeros)

            position_target.index_copy_(0, side_indices, q_target)
            velocity_target.index_copy_(0, side_indices, dq_target)
            effort_target.index_copy_(0, side_indices, tau_target)
            kp_target.index_copy_(0, side_indices, hand_kp)
            kd_target.index_copy_(0, side_indices, hand_kd)
            hard_disable_mask[side_slice] = ~motor_enabled
            if not self._hand_first_command_logged[side]:
                enabled_count = int(torch.count_nonzero(motor_enabled).item())
                print(
                    f"[sonic_dds][dex3:{side}] First complete HandCmd applied: "
                    f"enabled={enabled_count}/7, "
                    f"q=[{float(q_target.min().item()):.3f}, "
                    f"{float(q_target.max().item()):.3f}], "
                    f"kp=[{float(hand_kp.min().item()):.3f}, "
                    f"{float(hand_kp.max().item()):.3f}], "
                    f"kd=[{float(hand_kd.min().item()):.3f}, "
                    f"{float(hand_kd.max().item()):.3f}]"
                )
                self._hand_first_command_logged[side] = True

        return hard_disable_mask

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
        self._fall_recovery_hold_steps_remaining = int(
            math.ceil(max(0.0, float(hold_duration_s)) / max(self._step_dt, 1.0e-6))
        )
        self._fall_recovery_blend_duration_s = max(0.0, float(blend_duration_s))
        self._fall_recovery_blend_steps_total = int(
            math.ceil(self._fall_recovery_blend_duration_s / max(self._step_dt, 1.0e-6))
        )
        self._fall_recovery_blend_step = 0
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

    def _get_command_for_current_state(self, now: float):
        """Wait for a LowCmd that explicitly acknowledges the latest state tick.

        The wait never advances PhysX.  Repeated DDS packets are allowed, but a
        command is accepted only when both the state tick and reset epoch match.
        This prevents wall-clock SONIC iterations from running ahead of a slow
        Isaac simulation without building an input/action backlog.
        """
        if not self.sync_with_lowstate:
            self._environment_step_ready = True
            return self.robot_dds.get_robot_command()

        get_state_info = getattr(self.robot_dds, "get_latest_written_state_info", None)
        if get_state_info is None:
            self._environment_step_ready = False
            return None

        state_info = get_state_info()
        sample_seq = state_info.get("sample_seq")
        if sample_seq is None:
            self._environment_step_ready = False
            return None

        expected_tick = int(sample_seq) & 0xFFFFFFFF
        expected_reset_epoch = int(state_info.get("reset_epoch", 0)) & 0xFFFFFFFF
        self._sync_expected_tick = expected_tick
        self._sync_expected_reset_epoch = expected_reset_epoch

        wait_start = time.monotonic()
        deadline = wait_start + self.sync_wait_timeout_s
        last_stale_signature = None
        while True:
            command = self.robot_dds.get_robot_command()
            if command:
                try:
                    sync_magic = int(command.get("sync_magic", 0))
                    ack_tick = command.get("ack_state_tick")
                    ack_reset_epoch = command.get("ack_reset_epoch")
                    ack_tick = None if ack_tick is None else int(ack_tick) & 0xFFFFFFFF
                    ack_reset_epoch = (
                        None
                        if ack_reset_epoch is None
                        else int(ack_reset_epoch) & 0xFFFFFFFF
                    )
                except (TypeError, ValueError):
                    sync_magic = 0
                    ack_tick = None
                    ack_reset_epoch = None

                if (
                    sync_magic == SONIC_LOWCMD_SYNC_MAGIC
                    and ack_tick == expected_tick
                    and ack_reset_epoch == expected_reset_epoch
                ):
                    wait_s = time.monotonic() - wait_start
                    self._environment_step_ready = True
                    self._sync_last_ack_tick = ack_tick
                    self._sync_last_ack_reset_epoch = ack_reset_epoch
                    self._sync_wait_count += 1
                    self._sync_wait_sum_s += wait_s
                    self._sync_wait_max_s = max(self._sync_wait_max_s, wait_s)
                    return command

                stale_signature = (sync_magic, ack_tick, ack_reset_epoch)
                if stale_signature != last_stale_signature:
                    self._sync_stale_command_count += 1
                    last_stale_signature = stale_signature

            if time.monotonic() >= deadline:
                wait_s = time.monotonic() - wait_start
                self._environment_step_ready = False
                self._sync_timeout_count += 1
                self._sync_wait_sum_s += wait_s
                self._sync_wait_max_s = max(self._sync_wait_max_s, wait_s)
                return None
            time.sleep(self.sync_poll_interval_s)

    def get_action(self, env) -> Optional[torch.Tensor]:
        del env
        now = time.monotonic()

        command = self._get_command_for_current_state(now)
        if command is None:
            return self._hold_action(now, "waiting for synchronized rt/lowcmd acknowledgement")
        now = time.monotonic()

        recovery_hold_active = False
        if self._fall_recovery_waiting_for_control:
            if self.sync_with_lowstate:
                recovery_hold_active = self._fall_recovery_hold_steps_remaining > 0
            else:
                recovery_hold_active = now < self._fall_recovery_hold_until
        if recovery_hold_active:
            if self.sync_with_lowstate:
                self._fall_recovery_hold_steps_remaining -= 1
                remaining_s = self._fall_recovery_hold_steps_remaining * self._step_dt
            else:
                remaining_s = self._fall_recovery_hold_until - now
            return self._hold_action(
                now,
                f"post-reset standing stabilization ({remaining_s:.2f}s remaining)",
            )

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

        hand_hard_disable_mask = torch.zeros(
            len(DEX3_HAND_JOINT_NAMES), dtype=torch.bool, device=self.device
        )
        if self._dex3_indices.numel() > 0:
            # A missing/stale hand packet holds its last safe q and gains while
            # stale dq/tau are removed. Hand freshness is intentionally
            # independent of rt/lowcmd freshness.
            position_target.index_copy_(
                0,
                self._dex3_indices,
                self._last_position_target.index_select(0, self._dex3_indices),
            )
            kp_target.index_copy_(
                0,
                self._dex3_indices,
                self._last_kp.index_select(0, self._dex3_indices),
            )
            kd_target.index_copy_(
                0,
                self._dex3_indices,
                self._last_kd.index_select(0, self._dex3_indices),
            )
            if bridge_mode == SONIC_BRIDGE_CONTROL_MODE:
                hand_hard_disable_mask = self._apply_dex3_commands(
                    now,
                    position_target,
                    velocity_target,
                    effort_target,
                    kp_target,
                    kd_target,
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
            if self.sync_with_lowstate:
                self._fall_recovery_blend_step += 1
                if self._fall_recovery_blend_steps_total <= 0:
                    recovery_alpha = 1.0
                else:
                    recovery_alpha = min(
                        1.0,
                        self._fall_recovery_blend_step
                        / self._fall_recovery_blend_steps_total,
                    )
            elif self._fall_recovery_blend_duration_s <= 0.0:
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

        for group_name, max_step in self.group_max_target_step.items():
            if max_step <= 0.0:
                continue
            group_indices = self._body_group_indices[group_name]
            group_target = position_target.index_select(0, group_indices)
            group_previous = self._last_position_target.index_select(0, group_indices)
            raw_group_delta = group_target - group_previous
            self._group_limit_counts[group_name] += int(
                torch.count_nonzero(torch.abs(raw_group_delta) > max_step).item()
            )
            group_delta = torch.clamp(
                raw_group_delta,
                -max_step,
                max_step,
            )
            position_target.index_copy_(0, group_indices, group_previous + group_delta)

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

        if self._dex3_indices.numel() > 0 and bool(torch.any(hand_hard_disable_mask)):
            hand_effort = effort_target.index_select(0, self._dex3_indices)
            hand_kp_target = kp_target.index_select(0, self._dex3_indices)
            hand_kd_target = kd_target.index_select(0, self._dex3_indices)
            hand_enabled = ~hand_hard_disable_mask
            effort_target.index_copy_(
                0,
                self._dex3_indices,
                torch.where(hand_enabled, hand_effort, 0.0),
            )
            kp_target.index_copy_(
                0,
                self._dex3_indices,
                torch.where(hand_enabled, hand_kp_target, 0.0),
            )
            kd_target.index_copy_(
                0,
                self._dex3_indices,
                torch.where(hand_enabled, hand_kd_target, 0.0),
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
        self._metrics_roll_abs_sum = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_roll_abs_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_pitch_abs_sum = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_pitch_abs_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_root_linear_speed_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_root_angular_speed_max = torch.zeros((), dtype=torch.float32, device=self.device)
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
        self._metrics_estimated_pd_torque_max = {
            name: torch.zeros((), dtype=torch.float32, device=self.device)
            for name in BODY_GROUP_RANGES
        }
        self._metrics_foot_force_sum = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_foot_force_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_foot_force_value_count = 0
        self._metrics_double_support_count = 0
        self._metrics_foot_slip_sum = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_foot_slip_max = torch.zeros((), dtype=torch.float32, device=self.device)
        self._metrics_foot_slip_value_count = 0
        if hasattr(self, "_group_limit_counts"):
            for group_name in self._group_limit_counts:
                self._group_limit_counts[group_name] = 0
        if hasattr(self, "_sync_wait_count"):
            self._sync_wait_count = 0
            self._sync_timeout_count = 0
            self._sync_stale_command_count = 0
            self._sync_wait_sum_s = 0.0
            self._sync_wait_max_s = 0.0

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
        quat_w, quat_x, quat_y, quat_z = root_quat.unbind()
        roll_rad = torch.atan2(
            2.0 * (quat_w * quat_x + quat_y * quat_z),
            1.0 - 2.0 * (quat_x * quat_x + quat_y * quat_y),
        )
        pitch_rad = torch.asin(
            torch.clamp(
                2.0 * (quat_w * quat_y - quat_z * quat_x),
                -1.0,
                1.0,
            )
        )
        roll_abs_deg = torch.abs(torch.rad2deg(roll_rad))
        pitch_abs_deg = torch.abs(torch.rad2deg(pitch_rad))
        base_z = self.robot.data.root_pos_w[0, 2]

        self._metrics_samples += 1
        self._metrics_total_samples += 1
        self._metrics_joint_abs_sum += pd_target_abs
        self._metrics_joint_max = torch.maximum(self._metrics_joint_max, pd_target_abs)
        self._metrics_pd_target_abs_sum += pd_target_abs.sum()
        self._metrics_pd_target_max = torch.maximum(self._metrics_pd_target_max, pd_target_abs.max())
        self._metrics_tilt_sum += tilt_deg
        self._metrics_tilt_max = torch.maximum(self._metrics_tilt_max, tilt_deg)
        self._metrics_roll_abs_sum += roll_abs_deg
        self._metrics_roll_abs_max = torch.maximum(
            self._metrics_roll_abs_max,
            roll_abs_deg,
        )
        self._metrics_pitch_abs_sum += pitch_abs_deg
        self._metrics_pitch_abs_max = torch.maximum(
            self._metrics_pitch_abs_max,
            pitch_abs_deg,
        )
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

        root_linear_velocity = getattr(self.robot.data, "root_lin_vel_w", None)
        if root_linear_velocity is not None:
            self._metrics_root_linear_speed_max = torch.maximum(
                self._metrics_root_linear_speed_max,
                torch.linalg.vector_norm(root_linear_velocity[0]),
            )
        root_angular_velocity = getattr(self.robot.data, "root_ang_vel_w", None)
        if root_angular_velocity is not None:
            self._metrics_root_angular_speed_max = torch.maximum(
                self._metrics_root_angular_speed_max,
                torch.linalg.vector_norm(root_angular_velocity[0]),
            )

        # Requested generalized torque before PhysX effort saturation.  This
        # makes q/dq/tau/kp/kd contributions observable without silently
        # replacing SONIC's stabilizing gains.  Group maxima guide later A/B
        # limits (especially arms) while preserving raw lower-body authority.
        estimated_pd_torque = (
            body_command_tau
            + body_kp * (target_body_pos - actual_body_pos)
            + body_kd * (body_command_vel - body_vel)
        )
        for group_name, motor_range in BODY_GROUP_RANGES.items():
            group_torque = estimated_pd_torque[motor_range.start : motor_range.stop]
            self._metrics_estimated_pd_torque_max[group_name] = torch.maximum(
                self._metrics_estimated_pd_torque_max[group_name],
                torch.abs(group_torque).max(),
            )

        if self._foot_contact_sensor is not None:
            try:
                foot_forces_w = self._foot_contact_sensor.data.net_forces_w[0]
                foot_force_norm = torch.linalg.vector_norm(foot_forces_w, dim=-1)
                if foot_force_norm.numel() > 0:
                    self._metrics_foot_force_sum += foot_force_norm.sum()
                    self._metrics_foot_force_max = torch.maximum(
                        self._metrics_foot_force_max,
                        foot_force_norm.max(),
                    )
                    self._metrics_foot_force_value_count += int(foot_force_norm.numel())
                    contact_mask = foot_force_norm >= 5.0
                    if contact_mask.numel() >= 2 and bool(torch.all(contact_mask)):
                        self._metrics_double_support_count += 1

                    body_linear_velocity = getattr(self.robot.data, "body_lin_vel_w", None)
                    if (
                        body_linear_velocity is not None
                        and self._foot_body_indices.numel() == foot_force_norm.numel()
                    ):
                        foot_velocity = body_linear_velocity[0].index_select(
                            0,
                            self._foot_body_indices,
                        )
                        foot_slip_speed = torch.linalg.vector_norm(
                            foot_velocity[:, :2],
                            dim=-1,
                        )
                        contact_slip = foot_slip_speed[contact_mask]
                        if contact_slip.numel() > 0:
                            self._metrics_foot_slip_sum += contact_slip.sum()
                            self._metrics_foot_slip_max = torch.maximum(
                                self._metrics_foot_slip_max,
                                contact_slip.max(),
                            )
                            self._metrics_foot_slip_value_count += int(contact_slip.numel())
            except (AttributeError, IndexError, RuntimeError):
                # Diagnostics must never interrupt the control path.  A sensor
                # can be briefly unavailable while an articulation is reset.
                pass

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
        roll_abs_mean = float((self._metrics_roll_abs_sum / self._metrics_samples).item())
        roll_abs_max = float(self._metrics_roll_abs_max.item())
        pitch_abs_mean = float((self._metrics_pitch_abs_sum / self._metrics_samples).item())
        pitch_abs_max = float(self._metrics_pitch_abs_max.item())
        root_linear_speed_max = float(self._metrics_root_linear_speed_max.item())
        root_angular_speed_max = float(self._metrics_root_angular_speed_max.item())
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

        estimated_pd_torque_text = ", ".join(
            f"{group_name}={float(value.item()):.2f}Nm"
            for group_name, value in self._metrics_estimated_pd_torque_max.items()
        )
        if self._metrics_foot_force_value_count:
            foot_force_mean = float(
                (self._metrics_foot_force_sum / self._metrics_foot_force_value_count).item()
            )
        else:
            foot_force_mean = 0.0
        foot_force_max = float(self._metrics_foot_force_max.item())
        double_support_pct = 100.0 * self._metrics_double_support_count / self._metrics_samples
        if self._metrics_foot_slip_value_count:
            foot_slip_mean = float(
                (self._metrics_foot_slip_sum / self._metrics_foot_slip_value_count).item()
            )
        else:
            foot_slip_mean = 0.0
        foot_slip_max = float(self._metrics_foot_slip_max.item())
        sync_wait_samples = self._sync_wait_count + self._sync_timeout_count
        sync_wait_mean_ms = (
            1000.0 * self._sync_wait_sum_s / sync_wait_samples
            if sync_wait_samples
            else 0.0
        )
        group_limit_text = ", ".join(
            f"{group_name}={count}"
            for group_name, count in self._group_limit_counts.items()
        )

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
            f"roll_abs={roll_abs_mean:.2f}/{roll_abs_max:.2f}deg(mean/max), "
            f"pitch_abs={pitch_abs_mean:.2f}/{pitch_abs_max:.2f}deg(mean/max), "
            f"root_speed_max={root_linear_speed_max:.3f}m/s, "
            f"root_omega_max={root_angular_speed_max:.3f}rad/s, "
            f"base_z={base_z_last:.3f}m, base_z_min={base_z_min:.3f}m, "
            f"pd_target_mae={pd_target_mae:.4f}rad, pd_target_max={pd_target_max:.4f}rad, "
            f"max_abs_dq={joint_speed_max:.2f}rad/s, "
            f"cmd_max_abs_dq={command_dq_max:.4f}rad/s, "
            f"cmd_max_abs_tau={command_tau_max:.4f}Nm, "
            f"kp=[{kp_min:.3f},{kp_max:.3f}], kd=[{kd_min:.3f},{kd_max:.3f}], "
            f"torque_ratio_max={torque_ratio_max:.3f}, "
            f"torque_sat={torque_saturation_pct:.2f}%, gain_updates={self._gain_update_count}"
        )
        print(
            "[sonic_dds][balance_metrics] "
            f"estimated_pd_peak=[{estimated_pd_torque_text}], "
            f"foot_force={foot_force_mean:.2f}/{foot_force_max:.2f}N(mean/max), "
            f"double_support={double_support_pct:.1f}%, "
            f"contact_slip={foot_slip_mean:.4f}/{foot_slip_max:.4f}m/s(mean/max), "
            f"q_step_clamps=[{group_limit_text}]"
        )
        if self.sync_with_lowstate:
            print(
                "[sonic_dds][sync] "
                f"expected={self._sync_expected_reset_epoch}:{self._sync_expected_tick}, "
                f"ack={self._sync_last_ack_reset_epoch}:{self._sync_last_ack_tick}, "
                f"matched={self._sync_wait_count}, timeouts={self._sync_timeout_count}, "
                f"stale_variants={self._sync_stale_command_count}, "
                f"wait={sync_wait_mean_ms:.2f}/{self._sync_wait_max_s * 1000.0:.2f}ms(mean/max)"
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
