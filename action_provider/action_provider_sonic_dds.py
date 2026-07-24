"""Direct Gear SONIC DDS control for floating-base G1 29-DoF tasks.

This provider owns the 29 G1 body joints.  A task may use either the exact
29-DoF training articulation or the later 29+14-DoF Dex3 articulation.  When
Dex3 joints exist, they remain at their configured defaults until the OpenXR
hand-control phase.
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
        self.ramp_duration_s = max(0.0, float(getattr(args_cli, "sonic_ramp_seconds", 0.0)))
        self.max_target_step = max(0.0, float(getattr(args_cli, "sonic_max_target_step", 0.0)))
        self.metrics_interval_s = max(1.0, float(getattr(args_cli, "stats_interval", 10.0)))

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
        if len(dex3_indices) not in (0, 14):
            raise ValueError(
                "SONIC tasks require either no articulated Dex3 joints or exactly 14; "
                f"found {len(dex3_indices)}"
            )

        self._body_indices = torch.tensor(body_indices, dtype=torch.long, device=self.device)
        self._dex3_indices = torch.tensor(dex3_indices, dtype=torch.long, device=self.device)
        self._default_action = self.robot.data.default_joint_pos[0].clone()
        self._default_root_state = self.robot.data.default_root_state.clone()
        self._last_action = self._default_action.clone()
        self._incoming_positions = torch.empty(29, dtype=torch.float32, device=self.device)
        self._incoming_kp = torch.empty(29, dtype=torch.float32, device=self.device)

        self._first_valid_command_time: Optional[float] = None
        self._last_valid_command_time: Optional[float] = None
        self._last_timeout_warning_time = 0.0
        self._control_started = False
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
            f"G1 body=SONIC DDS (29), Dex3={hand_description}, root=PhysX"
        )
        print("[sonic_dds] Startup hold: pinning the default root state until the SONIC CONTROL marker")
        for motor_index, (name, articulation_index) in enumerate(zip(G1_29DOF_DDS_JOINT_ORDER, body_indices)):
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
        self._last_action.copy_(self._default_action)
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

        if (
            self._fall_recovery_waiting_for_control
            and now < self._fall_recovery_hold_until
        ):
            remaining_s = self._fall_recovery_hold_until - now
            return self._hold_action(
                now,
                f"post-reset standing stabilization ({remaining_s:.2f}s remaining)",
            )

        command = self.robot_dds.get_robot_command()

        if not command or "motor_cmd" not in command:
            return self._hold_action(now, "waiting for the first rt/lowcmd")

        motor_cmd = command["motor_cmd"]
        positions = motor_cmd.get("positions", [])
        if len(positions) < 29:
            return self._hold_action(now, f"rt/lowcmd contains only {len(positions)} motors")

        kp = motor_cmd.get("kp", [])
        if len(kp) < 29:
            return self._hold_action(now, f"rt/lowcmd contains only {len(kp)} kp values")

        try:
            self._incoming_positions.copy_(
                torch.as_tensor(positions[:29], dtype=torch.float32, device=self.device)
            )
            self._incoming_kp.copy_(
                torch.as_tensor(kp[:29], dtype=torch.float32, device=self.device)
            )
        except (TypeError, ValueError, RuntimeError) as exc:
            return self._hold_action(now, f"rt/lowcmd contains invalid numeric data: {exc}")

        if not bool(torch.isfinite(self._incoming_positions).all()) or not bool(
            torch.isfinite(self._incoming_kp).all()
        ):
            return self._hold_action(now, "rt/lowcmd contains NaN or Inf")

        if bool(torch.all(torch.abs(self._incoming_kp) <= 1.0e-6)):
            # SONIC uses kp=0 for its final damping-only shutdown packet. Isaac's
            # position-action bridge must not reinterpret that packet's q=0 as a
            # request to drive every joint to zero.
            return self._hold_action(now, "received damping-only rt/lowcmd")

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
                return self._hold_action(
                    now,
                    "post-reset rt/lowcmd has no freshness timestamp",
                )
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
        # released the floating base, a restarted or stale publisher sending
        # A0/A1 must never teleport the robot back to its spawn pose.
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
            return self._hold_action(
                now,
                reason,
            )

        target = self._default_action.clone()
        target.index_copy_(0, self._body_indices, self._incoming_positions)
        # Phase 1: the Dex3 joints are present in the asset but remain in their
        # configured open/default pose.  Do not accept hand commands yet.
        if self._dex3_indices.numel() > 0:
            target.index_copy_(
                0,
                self._dex3_indices,
                self._default_action.index_select(0, self._dex3_indices),
            )

        if bridge_mode != SONIC_BRIDGE_CONTROL_MODE:
            self._pin_initial_root_state()
            if now - self._last_timeout_warning_time >= 1.0:
                print(f"[sonic_dds] STARTUP HOLD: waiting for SONIC CONTROL marker (mode=0x{bridge_mode & 0xFF:02x})")
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
            self._last_action.copy_(self._default_action)
            print(
                "[sonic_dds] FALL RECOVERY: fresh CONTROL packet received; "
                f"releasing the root and blending SONIC targets over "
                f"{self._fall_recovery_blend_duration_s:.2f}s"
            )

        if self._control_started and self.ramp_duration_s > 0.0:
            alpha = min(1.0, (now - self._first_valid_command_time) / self.ramp_duration_s)
            target = torch.lerp(self._default_action, target, alpha)

        if self._fall_recovery_blend_start_time is not None:
            if self._fall_recovery_blend_duration_s <= 0.0:
                recovery_alpha = 1.0
            else:
                recovery_alpha = min(
                    1.0,
                    (now - self._fall_recovery_blend_start_time)
                    / self._fall_recovery_blend_duration_s,
                )
            target = torch.lerp(self._default_action, target, recovery_alpha)
            if recovery_alpha >= 1.0:
                self._fall_recovery_blend_start_time = None
                print("[sonic_dds] FALL RECOVERY complete: normal SONIC control restored")

        if self.max_target_step > 0.0:
            delta = torch.clamp(target - self._last_action, -self.max_target_step, self.max_target_step)
            target = self._last_action + delta

        self._last_action.copy_(target)
        self._last_valid_command_time = now
        if self._control_started:
            self._update_metrics(now, target)
        return target.unsqueeze(0)

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

    def _update_metrics(self, now: float, target: torch.Tensor) -> None:
        actual_body_pos = self.robot.data.joint_pos[0].index_select(0, self._body_indices)
        target_body_pos = target.index_select(0, self._body_indices)
        body_vel = self.robot.data.joint_vel[0].index_select(0, self._body_indices)
        # This is the displacement from the policy's virtual PD equilibrium,
        # not a reference-motion tracking error.  A non-zero value is expected
        # whenever the controller needs spring torque to support or balance the
        # robot (especially at the ankles).
        pd_target_abs = torch.abs(actual_body_pos - target_body_pos)

        root_quat = self.robot.data.root_quat_w[0]
        root_quat = root_quat / torch.clamp(torch.linalg.vector_norm(root_quat), min=1.0e-6)
        # The world-Z component of the root-frame Z axis is 1-2(x²+y²).
        # acos therefore gives heading-independent roll/pitch tilt in [0, 180].
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
            f"max_abs_dq={joint_speed_max:.2f}rad/s"
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
        # A floating-base humanoid cannot remain upright under a static joint
        # pose for the several seconds DDS discovery and policy initialization
        # may take. Keep the simulation at its configured initial root state
        # until SONIC has produced its first complete command. During an
        # explicit fall reset, pinning is temporarily re-enabled until a fresh
        # post-reset CONTROL packet arrives; stale publisher markers never
        # trigger this exception on their own.
        if not self._control_started or self._fall_recovery_waiting_for_control:
            self._pin_initial_root_state()

        if now - self._last_timeout_warning_time >= 1.0:
            print(f"[sonic_dds] HOLD: {reason}")
            self._last_timeout_warning_time = now
        return self._last_action.unsqueeze(0)

    def _pin_initial_root_state(self) -> None:
        self.robot.write_root_state_to_sim(self._default_root_state)

    def cleanup(self):
        # DDS lifecycle belongs to dds_manager; this provider only borrows it.
        self._report_metrics(time.monotonic(), force=True)
        self.is_running = False
