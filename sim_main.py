
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
#!/usr/bin/env python3
# main.py
import os

project_root = os.path.dirname(os.path.abspath(__file__))
os.environ["PROJECT_ROOT"] = project_root

import argparse
import contextlib
import gc
import math
import time
import sys
import signal
import threading
import torch
import gymnasium as gym
from pathlib import Path

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="dds", 
                   choices=["dds", "file", "trajectory", "policy", "replay", "dds_wholebody", "sonic_dds"],
                   help="Action source")


parser.add_argument("--robot_type", type=str, default="g129", help="robot type")
parser.add_argument(
    "--teleop_device",
    type=str,
    choices=("none", "motion_controllers"),
    default="none",
    help=(
        "optional teleoperation device; motion_controllers currently enables "
        "only the OpenXR view anchor and right-controller B yaw recenter"
    ),
)
parser.add_argument("--enable_dex1_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--enable_inspire_dds", action="store_true", help="enable inspire hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")
parser.add_argument(
    "--dds-domain",
    dest="dds_domain",
    type=int,
    default=None,
    help="Unitree DDS domain; SONIC defaults to 1 and can also use UNITREE_DDS_DOMAIN",
)
parser.add_argument(
    "--dds-interface",
    dest="dds_interface",
    type=str,
    default=None,
    help="Unitree DDS network interface; SONIC defaults to lo, use 'auto' for SDK auto-selection",
)

parser.add_argument("--file_path", type=str, default="/home/unitree/Code/xr_teleoperate/teleop/utils/data", help="file path (when action_source=file)")
parser.add_argument("--generate_data_dir", type=str, default="./data", help="save data dir")
parser.add_argument("--generate_data", action="store_true", default=False, help="generate data")
parser.add_argument("--rerun_log", action="store_true", default=False, help="rerun log")
parser.add_argument("--replay_data",  action="store_true", default=False, help="replay data")

parser.add_argument("--modify_light",  action="store_true", default=False, help="modify light")
parser.add_argument("--modify_camera",  action="store_true", default=False,    help="modify camera")

# performance analysis parameters
parser.add_argument(
    "--step_hz",
    type=int,
    default=None,
    help="environment control frequency; defaults to 50 Hz for SONIC and 100 Hz otherwise",
)
parser.add_argument("--enable_profiling", action="store_true", default=True, help="enable performance analysis")
parser.add_argument("--profile_interval", type=int, default=500, help="performance analysis report interval (steps)")

parser.add_argument("--model_path", type=str, default="assets/model/policy.onnx", help="model path")
parser.add_argument("--reward_interval", type=int, default=10, help="step interval for reward calculation")
parser.add_argument("--enable_wholebody_dds", action="store_true", default=False, help="enable wh dds")
parser.add_argument("--sonic_lowcmd_timeout", type=float, default=0.10,
                    help="maximum age in seconds of a SONIC rt/lowcmd before holding the last safe target")
parser.add_argument(
    "--sonic_handcmd_timeout",
    "--sonic-handcmd-timeout",
    dest="sonic_handcmd_timeout",
    type=float,
    default=0.20,
    help=(
        "maximum age in seconds of either Dex3 HandCmd; a stale hand holds "
        "its last q/gains and clears dq/tau without pausing body control"
    ),
)
parser.add_argument(
    "--sonic_hand_max_target_error",
    "--sonic-hand-max-target-error",
    dest="sonic_hand_max_target_error",
    type=float,
    default=0.25,
    help=(
        "maximum Dex3 q target error from actual joint state in radians; "
        "0 disables it, 0.25 matches Gear SONIC Dex3Hands"
    ),
)
parser.add_argument("--sonic_ramp_seconds", type=float, default=0.0,
                    help="optional extra blend from the USD default pose to SONIC targets; SONIC already performs its own INIT ramp")
parser.add_argument("--sonic_max_target_step", type=float, default=0.0,
                    help="optional per-control-step joint-target limiter in radians; 0 preserves the raw SONIC policy target")
parser.add_argument(
    "--sonic_leg_max_target_step",
    "--sonic-leg-max-target-step",
    dest="sonic_leg_max_target_step",
    type=float,
    default=0.0,
    help=(
        "optional lower-body q-target step limit in rad per unique PhysX state; "
        "0 disables it (recommended so balance corrections remain fast)"
    ),
)
parser.add_argument(
    "--sonic_waist_max_target_step",
    "--sonic-waist-max-target-step",
    dest="sonic_waist_max_target_step",
    type=float,
    default=0.0,
    help="optional waist q-target step limit in rad per unique PhysX state; 0 disables it",
)
parser.add_argument(
    "--sonic_arm_max_target_step",
    "--sonic-arm-max-target-step",
    dest="sonic_arm_max_target_step",
    type=float,
    default=0.0,
    help=(
        "optional shoulder/elbow/wrist q-target step limit in rad per unique "
        "PhysX state; use for upper-body impulse A/B tests"
    ),
)
sonic_sync_group = parser.add_mutually_exclusive_group()
sonic_sync_group.add_argument(
    "--sonic_sync_with_lowstate",
    "--sonic-sync-with-lowstate",
    dest="sonic_sync_with_lowstate",
    action="store_true",
    help="advance Isaac only after LowCmd acknowledges the latest unique LowState tick",
)
sonic_sync_group.add_argument(
    "--no_sonic_sync_with_lowstate",
    "--no-sonic-sync-with-lowstate",
    dest="sonic_sync_with_lowstate",
    action="store_false",
    help="disable the Isaac/SONIC lock-step handshake for regression A/B tests",
)
parser.set_defaults(sonic_sync_with_lowstate=None)
parser.add_argument(
    "--sonic_sync_wait_timeout",
    "--sonic-sync-wait-timeout",
    dest="sonic_sync_wait_timeout",
    type=float,
    default=0.25,
    help="wall-clock seconds to wait for the matching LowCmd before pausing PhysX",
)
parser.add_argument(
    "--sonic_sync_poll_interval",
    "--sonic-sync-poll-interval",
    dest="sonic_sync_poll_interval",
    type=float,
    default=0.001,
    help="LowCmd acknowledgement polling interval in seconds",
)
parser.add_argument(
    "--lowstate_pub_hz",
    "--lowstate-pub-hz",
    dest="lowstate_pub_hz",
    type=float,
    default=None,
    help=(
        "manual rt/lowstate DDS publish rate override (default: keep 100 Hz; "
        "SONIC lock-step already publishes fresh samples immediately via an "
        "event-driven wake-up, so raising this is normally unnecessary)"
    ),
)
parser.add_argument(
    "--sim_state_export_hz",
    "--sim-state-export-hz",
    dest="sim_state_export_hz",
    type=float,
    default=None,
    help=(
        "full scene-state DDS export rate; SONIC defaults to 5 Hz, other tasks "
        "retain every-loop export, 0 disables export"
    ),
)

fall_reset_group = parser.add_mutually_exclusive_group()
fall_reset_group.add_argument(
    "--auto_reset_on_fall",
    "--auto-reset-on-fall",
    dest="auto_reset_on_fall",
    action="store_true",
    help="automatically reset a fallen SONIC robot to its configured standing pose",
)
fall_reset_group.add_argument(
    "--no_auto_reset_on_fall",
    "--no-auto-reset-on-fall",
    dest="auto_reset_on_fall",
    action="store_false",
    help="disable automatic fall recovery (useful for intentional crawling/lying motions)",
)
parser.set_defaults(auto_reset_on_fall=None)
parser.add_argument(
    "--fall_reset_tilt_deg",
    "--fall-reset-tilt-deg",
    dest="fall_reset_tilt_deg",
    type=float,
    default=60.0,
    help="root roll/pitch tilt in degrees considered fallen",
)
parser.add_argument(
    "--fall_reset_min_base_height",
    "--fall-reset-min-base-height",
    dest="fall_reset_min_base_height",
    type=float,
    default=0.35,
    help="root height in metres at or below which the robot is considered fallen",
)
parser.add_argument(
    "--fall_reset_debounce_seconds",
    "--fall-reset-debounce-seconds",
    dest="fall_reset_debounce_seconds",
    type=float,
    default=0.50,
    help="time the fall condition must remain true before resetting",
)
parser.add_argument(
    "--fall_reset_hold_seconds",
    "--fall-reset-hold-seconds",
    dest="fall_reset_hold_seconds",
    type=float,
    default=1.0,
    help="time to pin the default standing root pose after a fall reset",
)
parser.add_argument(
    "--fall_reset_blend_seconds",
    "--fall-reset-blend-seconds",
    dest="fall_reset_blend_seconds",
    type=float,
    default=1.0,
    help="time to blend from the default pose back to fresh SONIC targets",
)
parser.add_argument(
    "--fall_reset_lowstate_grace_seconds",
    "--fall-reset-lowstate-grace-seconds",
    dest="fall_reset_lowstate_grace_seconds",
    type=float,
    default=1.0,
    help=(
        "before and after an intentional Isaac reset, publish zero joint dq/tau "
        "for this long so the pose teleport cannot trip SONIC's normal velocity "
        "safety check"
    ),
)
parser.add_argument(
    "--fall_reset_cooldown_seconds",
    "--fall-reset-cooldown-seconds",
    dest="fall_reset_cooldown_seconds",
    type=float,
    default=2.0,
    help="additional fall-detection cooldown after hold and blend complete",
)

parser.add_argument("--physics_dt", type=float, default=None, help="physics time step, e.g., 0.005")
parser.add_argument("--render_interval", type=int, default=None, help="render interval steps (>=1)")
parser.add_argument("--camera_write_interval", type=int, default=None, help="camera write interval steps (>=1)")
parser.add_argument(
    "--keep_kit_loop_pacing",
    "--keep-kit-loop-pacing",
    dest="keep_kit_loop_pacing",
    action="store_true",
    default=False,
    help=(
        "keep Isaac's kit loop runner manual-mode pacing (wall-clock locks every "
        "app.update() to rendering_dt); by default sim_main disables it because "
        "RobotController already enforces step_hz and the two limiters stack "
        "additively (GUI main loop drops to ~37 Hz instead of 50 Hz)"
    ),
)


parser.add_argument(
    "--no_render",
    action="store_true",
    default=False,
    help="run headless and disable rendering updates entirely",
)
parser.add_argument(
    "--no_late_render",
    action="store_true",
    default=False,
    help=(
        "restore in-step rendering for SONIC GUI runs (by default the render "
        "is moved after env.step so it overlaps the C++ inference window)"
    ),
)
parser.add_argument(
    "--disable_xr_frame_cap",
    action="store_true",
    default=False,
    help=(
        "diagnostic only: disable the OpenXR runtime frame-rate cap for "
        "SteamVR. Raises the average XR loop rate (21->34 Hz measured) but "
        "destroys frame pacing (irregular frame intervals -> judder/motion "
        "sickness). With the cap kept and the frame cost inside two HMD "
        "slots, the loop locks to an evenly paced refresh/2 cadence "
        "(36 fps @ 72 Hz), which is what actually feels smooth"
    ),
)
parser.add_argument(
    "--late_render_repeat",
    type=int,
    default=1,
    help=(
        "render M back-to-back frames per render window (XR: trades physics "
        "rate for AR head-view frame rate; world state repeats, HMD pose is "
        "fresh each frame)"
    ),
)
parser.add_argument(
    "--late_render_interval",
    type=int,
    default=2,
    help=(
        "render once every N control loops in late-render mode; default 2 "
        "(GUI at 25 Hz, physics/lock-step at 50 Hz). 1 restores full-rate GUI "
        "but the render cost (~8 ms) then rides every 20 ms loop and the "
        "closed loop lands at ~46 Hz instead of 50 (2026-07-28 measurements)"
    ),
)
parser.add_argument("--public_ip",type=str,default="127.0.0.1",help="public ip")
parser.add_argument(
    "--livestream_type",
    type=int,
    choices=(0, 1, 2),
    default=0,
    help="livestream type (0: disabled, 1: WebRTC public network, 2: WebRTC private network)",
)

parser.add_argument("--solver_iterations", type=int, default=None, help="physx solver iteration count (e.g., 4)")
parser.add_argument("--gravity_z", type=float, default=None, help="override gravity z (e.g., -9.8)")
parser.add_argument("--skip_cvtcolor", action="store_true", default=False, help="skip cv2.cvtColor if upstream already BGR")

parser.add_argument("--camera_jpeg", action="store_true", default=True, help="enable JPEG compression for camera frames")
parser.add_argument("--camera_jpeg_quality", type=int, default=85, help="JPEG quality (1-100)")

parser.add_argument("--physx_substeps", type=int, default=None, help="physx substeps per step")
parser.add_argument("--camera_include", type=str, default="front_camera,left_wrist_camera,right_wrist_camera", help="comma-separated camera names to enable")
parser.add_argument("--camera_exclude", type=str, default="world_camera", help="comma-separated camera names to disable")

parser.add_argument("--env_reward_interval", type=int, default=5, help="environment reward compute interval (steps)")
parser.add_argument("--seed", type=int, default=42, help="environment seed")
# add AppLauncher parameters
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

sonic_task_names = {
    "Isaac-G1-29DoF-Sonic",
    "Isaac-G1-29DoF-Dex3-Sonic",
    "Isaac-G1-29DoF-Training-Sonic",
    "Isaac-G1-29DoF-Sonic-Conveyor",
}
sonic_dex3_task_names = {
    "Isaac-G1-29DoF-Sonic",
    "Isaac-G1-29DoF-Dex3-Sonic",
    "Isaac-G1-29DoF-Sonic-Conveyor",
}
is_sonic_task = args_cli.task in sonic_task_names

if args_cli.teleop_device == "motion_controllers":
    if args_cli.task not in sonic_dex3_task_names:
        parser.error(
            "--teleop_device motion_controllers is currently supported only by "
            "Isaac-G1-29DoF-Sonic, Isaac-G1-29DoF-Dex3-Sonic and Isaac-G1-29DoF-Sonic-Conveyor"
        )
    # Follow Isaac Lab's teleoperation runner behavior: selecting an OpenXR
    # device implies XR, while an explicit --xr remains accepted as well.
    args_cli.xr = True

enabled_hand_dds_count = sum(
    bool(value)
    for value in (
        args_cli.enable_dex1_dds,
        args_cli.enable_dex3_dds,
        args_cli.enable_inspire_dds,
    )
)
if enabled_hand_dds_count > 1:
    parser.error("only one of --enable_dex1_dds, --enable_dex3_dds and --enable_inspire_dds may be enabled")
if args_cli.task in sonic_dex3_task_names:
    if args_cli.enable_dex1_dds or args_cli.enable_inspire_dds:
        parser.error("the 43-DoF SONIC task requires Dex3 DDS, not Dex1/Inspire DDS")
    if not args_cli.enable_dex3_dds:
        args_cli.enable_dex3_dds = True
        print("[sonic_dds] Auto-enabling Dex3 command/state DDS for the 43-DoF task")

if args_cli.auto_reset_on_fall is None:
    # The new behavior is enabled by default only for dedicated SONIC bridge
    # tasks. All existing non-SONIC tasks retain their old behavior.
    args_cli.auto_reset_on_fall = is_sonic_task

if args_cli.sonic_sync_with_lowstate is None:
    args_cli.sonic_sync_with_lowstate = is_sonic_task
if args_cli.sim_state_export_hz is None and is_sonic_task:
    args_cli.sim_state_export_hz = 5.0

if args_cli.step_hz is None:
    args_cli.step_hz = 50 if is_sonic_task else 100
elif args_cli.step_hz <= 0:
    parser.error("--step_hz must be positive")

if args_cli.physics_dt is not None and args_cli.physics_dt <= 0.0:
    parser.error("--physics_dt must be positive")
if args_cli.render_interval is not None and args_cli.render_interval <= 0:
    parser.error("--render_interval must be positive")
if args_cli.sonic_sync_wait_timeout <= 0.0:
    parser.error("--sonic_sync_wait_timeout must be positive")
if args_cli.sonic_sync_poll_interval <= 0.0:
    parser.error("--sonic_sync_poll_interval must be positive")
if not math.isfinite(args_cli.sonic_handcmd_timeout) or args_cli.sonic_handcmd_timeout <= 0.0:
    parser.error("--sonic_handcmd_timeout must be a finite positive value")
if (
    not math.isfinite(args_cli.sonic_hand_max_target_error)
    or args_cli.sonic_hand_max_target_error < 0.0
):
    parser.error("--sonic_hand_max_target_error must be a finite non-negative value")
if args_cli.sim_state_export_hz is not None and args_cli.sim_state_export_hz < 0.0:
    parser.error("--sim_state_export_hz must be non-negative")
for option_name in (
    "sonic_max_target_step",
    "sonic_leg_max_target_step",
    "sonic_waist_max_target_step",
    "sonic_arm_max_target_step",
):
    option_value = float(getattr(args_cli, option_name))
    if not math.isfinite(option_value) or option_value < 0.0:
        parser.error(f"--{option_name} must be a finite non-negative value")

if not 0.0 < args_cli.fall_reset_tilt_deg <= 180.0:
    parser.error("--fall_reset_tilt_deg must be in (0, 180]")
if (
    not math.isfinite(args_cli.fall_reset_min_base_height)
    or args_cli.fall_reset_min_base_height < 0.0
):
    parser.error("--fall_reset_min_base_height must be non-negative")
for option_name in (
    "fall_reset_debounce_seconds",
    "fall_reset_hold_seconds",
    "fall_reset_blend_seconds",
    "fall_reset_lowstate_grace_seconds",
    "fall_reset_cooldown_seconds",
):
    option_value = getattr(args_cli, option_name)
    if not math.isfinite(option_value) or option_value < 0.0:
        parser.error(f"--{option_name} must be non-negative")

if args_cli.dds_domain is None:
    raw_dds_domain = os.environ.get("UNITREE_DDS_DOMAIN", "1").strip()
    try:
        args_cli.dds_domain = int(raw_dds_domain)
    except ValueError:
        parser.error(f"UNITREE_DDS_DOMAIN must be an integer, got {raw_dds_domain!r}")
if args_cli.dds_domain < 0:
    parser.error("--dds-domain must be non-negative")

dds_interface_arg = args_cli.dds_interface
if dds_interface_arg is None:
    dds_interface_arg = os.environ.get("UNITREE_DDS_INTERFACE", "").strip() or None
    if dds_interface_arg is None and is_sonic_task:
        dds_interface_arg = "lo"
elif dds_interface_arg.strip().lower() == "auto":
    dds_interface_arg = None
else:
    dds_interface_arg = dds_interface_arg.strip()
    if not dds_interface_arg:
        parser.error("--dds-interface must be a network interface name or 'auto'")

args_cli.dds_interface = dds_interface_arg
os.environ["UNITREE_DDS_DOMAIN"] = str(args_cli.dds_domain)
if args_cli.dds_interface:
    os.environ["UNITREE_DDS_INTERFACE"] = args_cli.dds_interface
else:
    os.environ.pop("UNITREE_DDS_INTERFACE", None)

print(
    "[DDS Config] "
    f"domain={args_cli.dds_domain}, interface={args_cli.dds_interface or 'auto'}"
)

if args_cli.no_render and args_cli.xr:
    parser.error("--no_render cannot be combined with --xr")

from dds.dds_create import create_dds_objects, create_dds_objects_replay

if args_cli.no_render and args_cli.livestream_type != 0:
    parser.error("--no_render cannot be combined with a non-zero --livestream_type")

if args_cli.no_render:
    # AppLauncher selects its Kit experience before the environment is created.
    # Force the headless experience here and keep rendering/livestream disabled.
    args_cli.headless = True
    os.environ["LIVESTREAM"] = "0"
elif args_cli.livestream_type != 0:
    # Livestreaming still needs offscreen rendering, so it is intentionally
    # separate from --no_render and runs through the headless Kit experience.
    args_cli.headless = True
    os.environ["LIVESTREAM"] = str(args_cli.livestream_type)
    os.environ["PUBLIC_IP"] = args_cli.public_ip
else:
    os.environ["LIVESTREAM"] = "0"

if args_cli.xr and args_cli.disable_xr_frame_cap:
    # 诊断开关,默认不启用。SteamVR(Linux/OpenXR) 按头显刷新率给 xrWaitFrame
    # 定节拍:关掉后平均循环频率上升(实测 21→34Hz),但帧间隔失去对齐、
    # 忽快忽慢——头显里表现为抖动/晕(2026-07-28 实测否决默认开启)。
    # 正确姿势是保住帽、把单帧成本压进两个帧槽,循环锁进"刷新率/2"匀速档
    # (72Hz 面板 → 36fps,帧间隔恒定)。
    # quirk 名单必须在 kit 启动参数里就位——OpenXR instance 在扩展加载期创建,
    # 运行期改 carb 设置无效(SteamVR/Linux 的 instance 不可销毁重建)。
    _xr_kit_args = (
        "--/xr/openxr/needsFrameRateCap/disabled=[SteamVR] "
        "--/xr/openxr/needsGraphicsCompletedBeforeEndFrame/disabled=[SteamVR]"
    )
    args_cli.kit_args = (
        f"{args_cli.kit_args} {_xr_kit_args}" if args_cli.kit_args else _xr_kit_args
    )
    print("[sim] ⚠️ XR frame-rate cap disabled (diagnostic mode: higher avg Hz, broken pacing)")

import pinocchio
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)

from dds.reset_pose_dds import *
import tasks
from isaaclab.devices.teleop_device_factory import create_teleop_device
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from robots.g1_sonic_visuals import apply_g1_sonic_visual_materials

from tools.augmentation_utils import (
    update_light,
    batch_augment_cameras_by_name,
)

from tools.data_json_load import sim_state_to_json
from dds.sim_state_dds import *
from action_provider.create_action_provider import create_action_provider
from tools.get_stiffness import get_robot_stiffness_from_env
from tools.get_reward import get_step_reward_value,get_current_rewards

def setup_signal_handlers(shutdown_event):
    """Request an orderly shutdown from SIGINT/SIGTERM."""
    def signal_handler(signum, frame):
        if not shutdown_event.is_set():
            print(f"\nreceived signal {signum}, requesting orderly shutdown...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


class SonicFallResetMonitor:
    """Debounce SONIC fall detection using root tilt and root height."""

    def __init__(
        self,
        *,
        tilt_threshold_deg: float,
        min_base_height_m: float,
        debounce_s: float,
        cooldown_s: float,
    ):
        self.tilt_threshold_deg = float(tilt_threshold_deg)
        self.min_base_height_m = float(min_base_height_m)
        self.debounce_s = float(debounce_s)
        self.cooldown_s = float(cooldown_s)
        self._candidate_since = None
        self._cooldown_until = 0.0
        self._reset_count = 0

    @property
    def reset_count(self) -> int:
        return self._reset_count

    def clear_candidate(self) -> None:
        self._candidate_since = None

    def mark_reset(self, now: float, recovery_duration_s: float) -> None:
        self._candidate_since = None
        self._cooldown_until = now + max(0.0, recovery_duration_s) + self.cooldown_s
        self._reset_count += 1

    def update(self, now: float, root_pos_w: torch.Tensor, root_quat_w: torch.Tensor):
        """Return confirmed fall information, or ``None`` while healthy/debouncing."""
        if now < self._cooldown_until:
            self._candidate_since = None
            return None

        root_pos = root_pos_w.detach().cpu().tolist()
        root_quat = root_quat_w.detach().cpu().tolist()
        base_z = float(root_pos[2]) if len(root_pos) >= 3 else float("nan")

        tilt_deg = float("nan")
        if len(root_quat) >= 4 and all(math.isfinite(float(value)) for value in root_quat[:4]):
            _, qx, qy, qz = (float(value) for value in root_quat[:4])
            norm_sq = sum(float(value) * float(value) for value in root_quat[:4])
            if norm_sq >= 1.0e-12:
                vertical_cosine = 1.0 - 2.0 * (qx * qx + qy * qy) / norm_sq
                vertical_cosine = max(-1.0, min(1.0, vertical_cosine))
                tilt_deg = math.degrees(math.acos(vertical_cosine))

        reasons = []
        if not math.isfinite(base_z) or not math.isfinite(tilt_deg):
            reasons.append("invalid root pose")
        else:
            if tilt_deg >= self.tilt_threshold_deg:
                reasons.append(
                    f"tilt {tilt_deg:.1f}deg >= {self.tilt_threshold_deg:.1f}deg"
                )
            if base_z <= self.min_base_height_m:
                reasons.append(
                    f"base_z {base_z:.3f}m <= {self.min_base_height_m:.3f}m"
                )

        if not reasons:
            self._candidate_since = None
            return None

        if self._candidate_since is None:
            self._candidate_since = now
        if now - self._candidate_since < self.debounce_s:
            return None

        self._candidate_since = None
        return {
            "tilt_deg": tilt_deg,
            "base_z": base_z,
            "reason": "; ".join(reasons),
        }



def main():
    """main function"""
    # import cProfile
    # import pstats
    # import io
    # profiler = cProfile.Profile()
    # profiler.enable()
    image_server = None
    teleop_interface = None
    shutdown_event = threading.Event()
    print("=" * 60)
    print("robot control system started")
    print(f"Task: {args_cli.task}")
    print(f"Action source: {args_cli.action_source}")
    print("=" * 60)

    # parse environment configuration
    try:
        env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
        env_cfg.env_name = args_cli.task
        if args_cli.teleop_device != "none":
            configured_devices = getattr(
                getattr(env_cfg, "teleop_devices", None),
                "devices",
                {},
            )
            if args_cli.teleop_device not in configured_devices:
                raise ValueError(
                    f"teleop device {args_cli.teleop_device!r} is not configured "
                    f"for task {args_cli.task!r}"
                )
        if args_cli.physics_dt is not None:
            env_cfg.sim.dt = float(args_cli.physics_dt)

        # Isaac Lab's environment step reads cfg.sim.render_interval directly.
        # Changing SimulationContext.render_interval after gym.make() does not
        # alter the modulo test inside ManagerBasedRLEnv.step(), which made the
        # previous command-line override ineffective.  Resolve it before the
        # environment is constructed.
        #
        # SONIC GUI 晚渲染:env.step 内的渲染排在 lowstate 发布之前,给锁步
        # 关键路径白垫 ~8ms。默认把渲染从 env.step 挪到控制器里(env.step 完、
        # 观测已发布之后),让 C++ 推理窗口与渲染并行,ack 等待被渲染时间掩盖。
        # 用 --no_late_render 恢复旧行为。
        # XR 也走晚渲染:除藏住 ack 等待外,渲染用的是本步最新物理状态,
        # 头显 motion-to-photon 延迟更低。XR 下渲染间隔强制每圈(循环本身
        # 慢,隔圈会把 AR 帧率再砍半)。
        late_render_active = (
            is_sonic_task
            and not args_cli.no_late_render
            and not args_cli.no_render
            and not getattr(args_cli, "headless", False)
            and not args_cli.replay_data
            and args_cli.render_interval is None
        )
        # ⚠️ late_render 不能在这里把 interval 拨大:GUI 模式下 rendering_dt =
        # dt × interval 会被 SimulationContext.__init__ 的 kit manual-loop 节拍器
        # 当墙钟步长用,1e6 会让 _init_stage 的 app.update() 挂死(实测)。
        # 改为 env 创建完成后再改 env.cfg.sim.render_interval(取模检查现读)。
        if args_cli.no_render:
            resolved_render_interval = 1_000_000
        elif args_cli.render_interval is not None:
            resolved_render_interval = int(args_cli.render_interval)
        else:
            resolved_render_interval = max(1, int(env_cfg.sim.render_interval))
        env_cfg.sim.render_interval = resolved_render_interval

        self_collisions_enabled = None
        articulation_props = getattr(
            getattr(env_cfg.scene.robot, "spawn", None),
            "articulation_props",
            None,
        )
        if articulation_props is not None:
            self_collisions_enabled = articulation_props.enabled_self_collisions

        physics_dt = float(env_cfg.sim.dt)
        decimation = int(env_cfg.decimation)
        if physics_dt <= 0.0 or decimation <= 0:
            raise ValueError(
                f"invalid environment timing: physics_dt={physics_dt}, decimation={decimation}"
            )

        env_step_dt = physics_dt * decimation
        env_step_hz = 1.0 / env_step_dt
        if is_sonic_task and abs(float(args_cli.step_hz) - env_step_hz) > 1.0e-6:
            raise ValueError(
                "SONIC timing mismatch: the released policy requires the wall-clock "
                f"controller ({args_cli.step_hz} Hz) to match the Isaac environment "
                f"({env_step_hz:.6g} Hz from dt={physics_dt} x decimation={decimation})"
            )
        print(
            "[sim] control timing: "
            f"physics_dt={physics_dt:.6f}s, decimation={decimation}, "
            f"env_step_dt={env_step_dt:.6f}s, step_hz={args_cli.step_hz}"
        )
        if late_render_active:
            print(
                "[sim] rendering: late render enabled — env.step runs physics "
                "only; the controller renders once per loop after the LowState "
                "publish (disable with --no_late_render); "
                f"self_collisions={self_collisions_enabled}"
            )
        else:
            render_hz = 1.0 / (physics_dt * resolved_render_interval)
            print(
                "[sim] rendering: "
                f"render_interval={resolved_render_interval} physics steps "
                f"(~{render_hz:.2f} Hz), "
                f"self_collisions={self_collisions_enabled}"
            )
    except Exception as e:
        print(f"Failed to parse environment configuration: {e}")
        return
    
    # create environment
    print("\ncreate environment...")
    try:
        env_cfg.seed = args_cli.seed
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        env.seed(args_cli.seed)
        if is_sonic_task and not args_cli.no_render:
            try:
                material_report = apply_g1_sonic_visual_materials()
                print(
                    "[g1_materials] reference appearance applied: "
                    f"white={material_report.white_links}, "
                    f"dark={material_report.dark_links}, "
                    f"logo={material_report.logo_links}"
                )
                if material_report.unmapped_visual_links:
                    print(
                        "[g1_materials] unmapped visual links retained their "
                        "imported material: "
                        + ", ".join(material_report.unmapped_visual_links)
                    )
                if args_cli.task == "Isaac-G1-29DoF-Sonic-Conveyor":
                    # 对端镜像 G1 也上涂装，避免双机时看到通体白模误判机型
                    apply_g1_sonic_visual_materials("/World/envs/env_0/PeerRobot")
            except Exception as e:
                # Appearance must never prevent the DDS/physics validation from
                # starting.  A missing material asset is therefore reported but
                # does not change the articulation or abort the simulation.
                print(f"[g1_materials] failed to apply reference appearance: {e}")
        try:
            sensors_dict = getattr(env.scene, "sensors", {})
            if sensors_dict:
                print("Sensors in the environment:")
                for name, sensor in sensors_dict.items():
                    print(name, sensor)
                print("="*60)
        except Exception as e:
            print(f"[sim] failed to list sensors: {e}")
        print(f"\ncreate environment success ...")
        try:
            env._reward_interval = max(1, int(args_cli.env_reward_interval))
            env._reward_counter = 0
            env._reward_last = None
            print(f"[env] reward compute interval set to {env._reward_interval} steps")
        except Exception as e:
            print(f"[env] failed to set reward interval: {e}")
        headless_mode = bool(getattr(args_cli, "headless", False))
        try:
            if args_cli.no_render:
                env.sim.render_mode = "offscreen"
                print(
                    "[sim] rendering disabled via --no_render "
                    f"(cfg interval={env.cfg.sim.render_interval})"
                )
            elif headless_mode:
                env.sim.render_mode = "offscreen"
                print(
                    "[sim] headless offscreen rendering every "
                    f"{env.cfg.sim.render_interval} physics steps"
                )
            elif late_render_active:
                # 现在才拨大 interval:SimulationContext 已用正常 rendering_dt
                # 初始化完毕,这里只影响 ManagerBasedEnv.step 的取模检查(现读
                # cfg),env.step 从此不再内嵌渲染,渲染由控制器显式调用。
                env.cfg.sim.render_interval = 1_000_000
                print("[sim] GUI rendering: once per control loop, after env.step")
            else:
                print(
                    "[sim] GUI rendering every "
                    f"{env.cfg.sim.render_interval} physics steps"
                )
        except Exception as e:
            print(f"[sim] failed to configure rendering: {e}")
        if args_cli.camera_write_interval is not None:
            try:
                import tasks.common_observations.camera_state as cam_state
                cam_state._camera_cache['write_interval_steps'] = max(1, int(args_cli.camera_write_interval))
                print(f"[camera] write interval steps set to {cam_state._camera_cache['write_interval_steps']}")
            except Exception as e:
                print(f"[camera] failed to set write interval: {e}")

        try:
            if args_cli.solver_iterations is not None:
                env.sim.physx.solver_iteration_count = int(args_cli.solver_iterations)
                print(f"[sim] solver_iteration_count={env.sim.physx.solver_iteration_count}")
            if args_cli.physx_substeps is not None:
                try:
                    env.sim.physx.substeps = int(args_cli.physx_substeps)
                except Exception:
                    try:
                        env.sim.set_substeps(int(args_cli.physx_substeps))
                    except Exception:
                        pass
                print(f"[sim] physx_substeps set to {args_cli.physx_substeps}")
            if args_cli.gravity_z is not None:
                g = float(args_cli.gravity_z)
                env.sim.physx.gravity = (0.0, 0.0, g)
                print(f"[sim] gravity set to {env.sim.physx.gravity}")
        except Exception as e:
            print(f"[sim] failed to set physx params: {e}")
        if args_cli.skip_cvtcolor:
            os.environ["CAMERA_SKIP_CVTCOLOR"] = "1"
        try:
            import tasks.common_observations.camera_state as cam_state
            enable_jpeg = bool(args_cli.camera_jpeg) or (os.getenv("CAMERA_JPEG") == "1")
            jpeg_quality = int(args_cli.camera_jpeg_quality if args_cli.camera_jpeg else os.getenv("CAMERA_JPEG_QUALITY", args_cli.camera_jpeg_quality))
            cam_state.set_writer_options(enable_jpeg=enable_jpeg, jpeg_quality=jpeg_quality, skip_cvtcolor=args_cli.skip_cvtcolor)
            include = [n.strip() for n in (args_cli.camera_include or "").split(',') if n.strip()]
            exclude = [n.strip() for n in (args_cli.camera_exclude or "").split(',') if n.strip()]
            try:
                cam_state.set_camera_allowlist(include)
            except Exception:
                pass
            try:
                sensors_dict = getattr(env.scene, "sensors", {})
                for name, sensor in sensors_dict.items():
                    lname = name.lower()
                    if "camera" not in lname:
                        continue
                    if exclude and name in exclude:
                        for attr_name, value in [("enabled", False), ("is_enabled", False)]:
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, value)
                                except Exception:
                                    pass
                        for meth in ("set_active", "disable", "pause"):
                            if hasattr(sensor, meth):
                                try:
                                    getattr(sensor, meth)(False)
                                except Exception:
                                    pass
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
                    elif include and name not in include:
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
            except Exception as e:
                print(f"[camera] failed to tune sensors: {e}")
        except Exception as e:
            print(f"[camera] failed to apply writer options: {e}")
    except Exception as e:
        print(f"\nFailed to create environment: {e}")
        return
    
    # get robot stiffness and damping parameters from runtime environment
    print("\n" + "="*60)
    print("🔍 Getting robot stiffness and damping parameters from runtime environment")
    print("="*60)
    
    try:
        stiffness_data = get_robot_stiffness_from_env(env)
        if stiffness_data:
            print("✅ Successfully got robot parameters!")
        else:
            print("⚠️ Failed to get robot parameters, will try again after environment reset")
    except Exception as e:
        print(f"⚠️ Error getting robot parameters: {e}")
    
    print("="*60)
    
    if not getattr(args_cli, "headless", False) and not args_cli.no_render:
        print("\n")
        print("***  Please left-click on the Sim window to activate rendering. ***")
        print("\n")
    else:
        print("\n")
        print("***  Running without GUI; rendering handled offscreen. ***")
        print("\n")
    # reset environment
    if args_cli.modify_light:
        update_light(
            prim_path="/World/light",
            color=(0.75, 0.75, 0.75),
            intensity=500.0,
            # position=(1.0, 2.0, 3.0),
            radius=0.1,
            enabled=True,
            cast_shadows=True
        )
    if args_cli.modify_camera:
        batch_augment_cameras_by_name(
            names=["front_cam"],
            focal_length=3.0,
            horizontal_aperture=22.0,
            vertical_aperture=16.0,
            exposure=0.8,                
            focus_distance=1.2
        )
    env.sim.reset()
    env.reset()

    # isaacsim 的 SimulationContext 会把 kit loop runner 设成 manual mode,
    # 每次 app.update()(即每次 sim.render())都被墙钟定步到 rendering_dt=20ms,
    # 其中约 15ms 是纯睡眠;与 RobotController 的 step_hz deadline 定频串联后,
    # GUI 主循环被压到 ~37Hz。sim_main 自己负责定频,这里解除 Kit 层节拍。
    if not args_cli.keep_kit_loop_pacing:
        try:
            import omni.kit.loop._loop as _omni_loop
            import carb.settings as _carb_settings

            _omni_loop.acquire_loop_interface().set_manual_mode(False)
            _carb_settings.get_settings().set(
                "/app/runLoops/main/rateLimitEnabled", False
            )
            print(
                "[sim] kit loop pacing disabled: app.update() no longer "
                "wall-clock locked to rendering_dt (use --keep_kit_loop_pacing "
                "to restore)"
            )
        except Exception as e:
            print(f"[sim] failed to disable kit loop pacing: {e}")

    if args_cli.teleop_device != "none":
        print("========= create OpenXR teleop device =========")
        try:
            teleop_interface = create_teleop_device(
                args_cli.teleop_device,
                env_cfg.teleop_devices.devices,
            )
            teleop_interface.reset()
            xr_cfg = env_cfg.teleop_devices.devices[args_cli.teleop_device].xr_cfg
            print(
                "[xr] OpenXR view control enabled: "
                f"device={args_cli.teleop_device}, "
                f"position_anchor={xr_cfg.anchor_prim_path}, "
                f"rotation_anchor={xr_cfg.anchor_rotation_prim_path}"
            )
            print(
                "[xr] Release right-controller B to recenter view yaw; "
                "OpenXR commands are not connected to robot actions"
            )
        except Exception as e:
            print(f"Failed to create OpenXR teleop device: {e}")
            env.close()
            return
        print("========= create OpenXR teleop device success =========")
    
    # create simplified control configuration
    try:    
        control_config = ControlConfig(
            step_hz=args_cli.step_hz,
            replay_mode=args_cli.replay_data,
            late_render=late_render_active,
            late_render_interval=(
                1 if args_cli.xr else max(1, int(args_cli.late_render_interval))
            ),
            late_render_repeat=max(1, int(args_cli.late_render_repeat)),
        )
    except Exception as e:
        print(f"Failed to create control configuration: {e}")
        return
    
    # create controller

    if not args_cli.replay_data:
        if is_sonic_task:
            print("========= skip image server: SONIC task has no cameras =========")
        else:
            print("========= create image server =========")
            try:
                teleimager_src = os.path.join(project_root, "teleimager", "src")
                if teleimager_src not in sys.path:
                    sys.path.insert(0, teleimager_src)
                from teleimager.image_server import run_isaacsim_server

                image_server = run_isaacsim_server()
            except Exception as e:
                print(f"Failed to create image server: {e}")
                return
            print("========= create image server success =========")
        print("========= create dds =========")
        try:
            reset_pose_dds,sim_state_dds,dds_manager = create_dds_objects(args_cli,env)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
        # 锁步的关键路径是 env.step 写完新样本后等 rt/lowstate 出门:100Hz 调度
        # 平均白等 5ms(最坏 10ms),直接吃掉每帧 ack 往返预算。SONIC 锁步改为
        # "新样本即发"(事件唤醒发布线程),保活重发节奏保持 100Hz 不变——
        # 单纯拉高发布频率会让序列化抢 GIL,省下的等待又亏在 env.step 里。
        if args_cli.lowstate_pub_hz:
            try:
                dds_manager.set_publish_rate("g129", float(args_cli.lowstate_pub_hz))
                print(f"[sim] rt/lowstate publish rate set to {args_cli.lowstate_pub_hz:.0f} Hz")
            except Exception as e:
                print(f"[sim] failed to set lowstate publish rate: {e}")
        if is_sonic_task and args_cli.sonic_sync_with_lowstate:
            try:
                dds_manager.enable_immediate_publish("g129")
            except Exception as e:
                print(f"[sim] failed to enable immediate lowstate publish: {e}")
        if is_sonic_task:
            # The first env.reset happens before the DDS object is registered,
            # so its observation cannot seed rt/lowstate. Lock-step control
            # needs one real initial PhysX sample before it can request the
            # first matching LowCmd; publish that sample explicitly here.
            try:
                from tasks.common_observations.g1_29dof_state import (
                    get_robot_boy_joint_states,
                )

                get_robot_boy_joint_states(
                    env,
                    enable_dds=True,
                    dds_min_interval_ms=0.0,
                )
                print("[sonic_dds] Initial PhysX state seeded for LowState lock-step")
                if args_cli.task in sonic_dex3_task_names:
                    from tasks.common_observations.dex3_state import (
                        get_robot_dex3_joint_states,
                    )

                    get_robot_dex3_joint_states(
                        env,
                        enable_dds=True,
                        dds_min_interval_ms=0.0,
                    )
                    print("[sonic_dds] Initial PhysX Dex3 state seeded")
            except Exception as e:
                print(f"Failed to seed initial SONIC LowState: {e}")
                return
    else:
        print("========= create dds =========")
        try:
            create_dds_objects_replay(args_cli,env)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
        from tools.data_json_load import get_data_json_list
        print("========= get data json list =========")
        data_idx=0
        data_json_list = get_data_json_list(args_cli.file_path)
        if args_cli.action_source != "replay":
            args_cli.action_source = "replay"
        print("========= get data json list success =========")
    # create action provider
    
    print(f"\ncreate action provider: {args_cli.action_source}...")
    try:
        print(f"args_cli.task: {args_cli.task}")
        if is_sonic_task and args_cli.action_source == "dds":
            print("[sonic_dds] Selecting the dedicated 29-DoF SONIC action source for this task")
            args_cli.action_source = "sonic_dds"
        elif not args_cli.replay_data and ("Wholebody" in args_cli.task or args_cli.enable_wholebody_dds):
            args_cli.action_source = "dds_wholebody"
            args_cli.enable_wholebody_dds = True
            control_config.use_rl_action_mode = True
        action_provider = create_action_provider(env,args_cli)
        if action_provider is None:
            print("action provider creation failed, exiting")
            return
    except Exception as e:
        print(f"Failed to create action provider: {e}")
        return
    
    # set action provider
    print("========= create controller =========")
    controller = RobotController(env, control_config)
    controller.set_action_provider(action_provider)
    print("========= create controller success =========")

    sonic_reset_supported = (
        is_sonic_task
        and args_cli.action_source == "sonic_dds"
        and hasattr(action_provider, "begin_fall_recovery")
        and hasattr(action_provider, "control_started")
        and hasattr(action_provider, "fall_recovery_active")
    )
    fall_reset_enabled = bool(args_cli.auto_reset_on_fall and sonic_reset_supported)
    fall_reset_monitor = None
    sonic_env_ids = None
    if sonic_reset_supported:
        fall_reset_monitor = SonicFallResetMonitor(
            tilt_threshold_deg=args_cli.fall_reset_tilt_deg,
            min_base_height_m=args_cli.fall_reset_min_base_height,
            debounce_s=args_cli.fall_reset_debounce_seconds,
            cooldown_s=args_cli.fall_reset_cooldown_seconds,
        )
        sonic_env_ids = torch.arange(env.num_envs, dtype=torch.int64, device=env.device)

    if fall_reset_enabled:
        print(
            "[fall_reset] enabled: "
            f"tilt>={args_cli.fall_reset_tilt_deg:.1f}deg or "
            f"base_z<={args_cli.fall_reset_min_base_height:.3f}m for "
            f"{args_cli.fall_reset_debounce_seconds:.2f}s; "
            f"hold={args_cli.fall_reset_hold_seconds:.2f}s, "
            f"blend={args_cli.fall_reset_blend_seconds:.2f}s, "
            f"lowstate_grace={args_cli.fall_reset_lowstate_grace_seconds:.2f}s, "
            f"cooldown={args_cli.fall_reset_cooldown_seconds:.2f}s"
        )
    elif args_cli.auto_reset_on_fall:
        print(
            "[fall_reset] requested but unavailable: automatic recovery requires "
            "a SONIC task with the sonic_dds action provider"
        )
    elif sonic_reset_supported:
        print("[fall_reset] automatic detection disabled; safe manual reset recovery remains available")

    # ZMQ 双机场景同步（Isaac-G1-29DoF-Sonic-Conveyor 任务才有这两个 term）。
    # 复位编排：ID=1 是复位权威，本机整环境复位后广播 reset_id；镜像端（ID=2）
    # 在主循环里消费该事件并跟随复位，scene_state 帧按 reset_id 门控丢弃复位前旧帧。
    def _get_scene_sync_term(term_name: str):
        try:
            return env.action_manager.get_term(term_name)
        except (AttributeError, KeyError, ValueError):
            return None

    scene_sync_term = _get_scene_sync_term("scene_state_sync")
    env_reset_sync_term = _get_scene_sync_term("env_reset_sync")

    def broadcast_sync_reset() -> None:
        """权威端（复位事件 term 为 publisher 角色）广播一次整环境复位。"""
        if env_reset_sync_term is None:
            return
        sync_reset_id = env_reset_sync_term.request_local_reset()
        if sync_reset_id and scene_sync_term is not None:
            scene_sync_term.set_publisher_reset_id(sync_reset_id)

    def trigger_robot_reset(event_name: str, reason: str) -> bool:
        """Reset state and controller history as one operation for SONIC tasks."""
        if not sonic_reset_supported:
            env_cfg.event_manager.trigger(event_name, env)
            broadcast_sync_reset()
            return False

        robot_dds = dds_manager.get_object("g129")
        if robot_dds is not None and hasattr(robot_dds, "begin_reset_epoch"):
            robot_dds.begin_reset_epoch(reason)
        if robot_dds is not None and hasattr(robot_dds, "begin_reset_state_grace"):
            robot_dds.begin_reset_state_grace(
                args_cli.fall_reset_lowstate_grace_seconds,
                reason,
            )
        action_provider.begin_fall_recovery(
            hold_duration_s=args_cli.fall_reset_hold_seconds,
            blend_duration_s=args_cli.fall_reset_blend_seconds,
            reason=reason,
        )
        # The task event writes the default root/joint state and clears joint
        # targets. env.reset then clears action/observation/history buffers and
        # forwards the new state before the next control step.
        env_cfg.event_manager.trigger(event_name, env)
        env.reset(env_ids=sonic_env_ids)
        # env.reset can itself take longer than the pre-reset grace window on a
        # CPU-loaded Isaac instance.  Re-arm the window after the reset so the
        # first live PhysX samples (where teleport-derived dq is most likely)
        # are always suppressed, independent of reset duration.
        if robot_dds is not None and hasattr(robot_dds, "begin_reset_state_grace"):
            robot_dds.begin_reset_state_grace(
                args_cli.fall_reset_lowstate_grace_seconds,
                f"{reason}: post-reset stabilization",
            )
        fall_reset_monitor.mark_reset(
            time.monotonic(),
            args_cli.fall_reset_hold_seconds + args_cli.fall_reset_blend_seconds,
        )
        print(
            f"[fall_reset] reset #{fall_reset_monitor.reset_count} complete: "
            "default standing state restored; waiting for safe SONIC re-entry"
        )
        broadcast_sync_reset()
        return True

    def consume_remote_sync_reset() -> bool:
        """镜像端消费权威端的整环境复位事件；无事件或非镜像端返回 False。"""
        if env_reset_sync_term is None:
            return False
        sync_reset_id = env_reset_sync_term.consume_remote_reset_request()
        if not sync_reset_id:
            return False
        if scene_sync_term is not None:
            scene_sync_term.expect_reset_id(sync_reset_id)
        trigger_robot_reset("reset_all_self", f"remote scene-sync reset {sync_reset_id}")
        return True
    
    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")


    # The signal handler only flips this event. Resource teardown is centralized
    # in the finally block below, outside signal-handler context.
    setup_signal_handlers(shutdown_event)
        
    print(
        "[DDS Config] Runtime "
        f"domain={args_cli.dds_domain}, interface={args_cli.dds_interface or 'auto'}"
    )
    try:
        # start controller - start asynchronous components
        print("========= start controller =========")
        controller.start()
        print("========= start controller success =========")
        
        # main loop - execute in main thread to support rendering
        monotonic = time.monotonic
        last_stats_time = monotonic()
        loop_start_time = last_stats_time
        loop_count = 0
        last_loop_time = last_stats_time
        recent_loop_times = []  # for calculating moving average frequency
        sim_state_update_count = 0
        sim_state_work_s = 0.0
        if args_cli.sim_state_export_hz is None:
            sim_state_export_enabled = True
            sim_state_export_interval_s = 0.0  # legacy every-loop behavior
        else:
            sim_state_export_enabled = args_cli.sim_state_export_hz > 0.0
            sim_state_export_interval_s = (
                1.0 / args_cli.sim_state_export_hz
                if sim_state_export_enabled
                else 0.0
            )
        next_sim_state_export_time = last_stats_time
        print(
            "[sim-state] export "
            + (
                "disabled"
                if not sim_state_export_enabled
                else (
                    "every loop"
                    if sim_state_export_interval_s <= 0.0
                    else f"at {args_cli.sim_state_export_hz:.2f} Hz"
                )
            )
        )
        
        
        reward_interval = max(1, args_cli.reward_interval)

        # use torch.inference_mode() and exception suppression
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while (
                simulation_app.is_running()
                and controller.is_running
                and not shutdown_event.is_set()
            ):
                current_time = monotonic()
                loop_count += 1
                reset_performed = False
                if not args_cli.replay_data:
                    export_due = sim_state_export_enabled and (
                        sim_state_export_interval_s <= 0.0
                        or current_time >= next_sim_state_export_time
                    )
                    if export_due:
                        sim_state_work_start = monotonic()
                        try:
                            env_state = env.scene.get_state()
                            env_state_json = sim_state_to_json(env_state)
                            sim_state = {
                                "init_state": env_state_json,
                                "task_name": args_cli.task,
                            }
                        except Exception as e:
                            print(f"Failed to get env state: {e}")
                            raise e
                        try:
                            sim_state_dds.write_sim_state_data(sim_state)
                        except Exception as e:
                            print(f"Failed to write sim state: {e}")
                            raise e
                        sim_state_update_count += 1
                        sim_state_work_s += monotonic() - sim_state_work_start
                        if sim_state_export_interval_s > 0.0:
                            next_sim_state_export_time = current_time + sim_state_export_interval_s

                    try:
                        reset_pose_cmd = reset_pose_dds.get_reset_pose_command()
                    except Exception as e:
                        print(f"Failed to get reset pose command: {e}")
                        raise e
                    # Compute current reward values manually if needed for debugging
                    try:
                        if (loop_count % reward_interval) == 0:
                            pass
                            # current_reward = get_step_reward_value(env)
                    except Exception as e:
                        print(f"奖励计算失败: {e}")
                        pass
                    
                    if reset_pose_cmd is not None:
                        try:
                            reset_category = reset_pose_cmd.get("reset_category")
                            if (args_cli.enable_wholebody_dds and (reset_category == '1' or reset_category == '2')) or (not args_cli.enable_wholebody_dds and reset_category == '1'):
                                print("reset object")
                                trigger_robot_reset("reset_object_self", "manual reset object command")
                                reset_performed = True
                                reset_pose_dds.write_reset_pose_command(-1)
                            elif reset_category == '2' and not args_cli.enable_wholebody_dds:
                                print("reset all")
                                trigger_robot_reset("reset_all_self", "manual reset all command")
                                reset_performed = True
                                reset_pose_dds.write_reset_pose_command(-1)
                        except Exception as e:
                            print(f"Failed to write reset pose command: {e}")
                            raise e

                    # 方案 b：同步收发挂主循环——deploy 断连/锁步暂停时镜像仍活着
                    if scene_sync_term is not None and getattr(scene_sync_term.cfg, "external_pump", False):
                        scene_sync_term.pump()
                    if env_reset_sync_term is not None and getattr(env_reset_sync_term.cfg, "external_pump", False):
                        env_reset_sync_term.pump()

                    # 镜像端（ID=2）：跟随权威端的整环境复位事件
                    if consume_remote_sync_reset():
                        reset_performed = True
                else:
                    if action_provider.get_start_loop() and data_idx<len(data_json_list):
                        print(f"data_idx: {data_idx}")
                        try:
                            sim_state,task_name = action_provider.load_data(data_json_list[data_idx])
                            if task_name!=args_cli.task:
                                raise ValueError(f" The {task_name} in the dataset is different from the {args_cli.task} being executed .")
                        except Exception as e:
                            print(f"Failed to load data: {e}")
                            raise e
                        try:
                            env.reset_to(sim_state, torch.tensor([0], device=env.device), is_relative=True)
                            env.sim.reset()
                            time.sleep(1)
                            action_provider.start_replay()
                            data_idx+=1
                        except Exception as e:
                            print(f"Failed to start replay: {e}")
                            raise e
                # print(f"env_state: {env_state}")
                # calculate instantaneous loop time
                if reset_performed:
                    current_time = monotonic()
                    last_loop_time = current_time
                    recent_loop_times.clear()
                else:
                    loop_dt = current_time - last_loop_time
                    last_loop_time = current_time
                    recent_loop_times.append(loop_dt)

                    # keep recent 100 loop times
                    if len(recent_loop_times) > 100:
                        recent_loop_times.pop(0)

                if fall_reset_enabled and not reset_performed:
                    if not bool(action_provider.control_started):
                        fall_reset_monitor.clear_candidate()
                    else:
                        robot = env.scene["robot"]
                        fall_info = fall_reset_monitor.update(
                            time.monotonic(),
                            robot.data.root_pos_w[0],
                            robot.data.root_quat_w[0],
                        )
                        if fall_info is not None:
                            print(
                                "[fall_reset] confirmed fall: "
                                f"{fall_info['reason']} "
                                f"(tilt={fall_info['tilt_deg']:.1f}deg, "
                                f"base_z={fall_info['base_z']:.3f}m)"
                            )
                            trigger_robot_reset("reset_all_self", "automatic fall detection")
                            reset_performed = True
                            last_loop_time = monotonic()
                            recent_loop_times.clear()
                
                # execute control step (in main thread, support rendering)
                controller.step()

                # print statistics and loop frequency periodically
                stats_now = monotonic()
                if stats_now - last_stats_time >= args_cli.stats_interval:
                    # calculate while loop execution frequency
                    elapsed_time = stats_now - loop_start_time
                    loop_frequency = loop_count / elapsed_time if elapsed_time > 0 else 0
                    
                    # calculate moving average frequency (based on recent loop times)
                    if recent_loop_times:
                        avg_loop_time = sum(recent_loop_times) / len(recent_loop_times)
                        moving_avg_frequency = 1.0 / avg_loop_time if avg_loop_time > 0 else 0
                        min_loop_time = min(recent_loop_times)
                        max_loop_time = max(recent_loop_times)
                        max_freq = 1.0 / min_loop_time if min_loop_time > 0 else 0
                        min_freq = 1.0 / max_loop_time if max_loop_time > 0 else 0
                    else:
                        moving_avg_frequency = 0
                        min_freq = max_freq = 0
                    
                    print(f"\n=== While loop execution frequency statistics ===")
                    print(f"loop execution count: {loop_count}")
                    print(f"running time: {elapsed_time:.2f} seconds")
                    print(f"overall average frequency: {loop_frequency:.2f} Hz")
                    print(f"moving average frequency: {moving_avg_frequency:.2f} Hz (last {len(recent_loop_times)} times)")
                    print(f"frequency range: {min_freq:.2f} - {max_freq:.2f} Hz")
                    print(f"average loop time: {(elapsed_time/loop_count*1000):.2f} ms")
                    if recent_loop_times:
                        print(f"recent loop time: {(avg_loop_time*1000):.2f} ms")
                    stats_window_s = stats_now - last_stats_time
                    sim_state_rate = (
                        sim_state_update_count / stats_window_s if stats_window_s > 0.0 else 0.0
                    )
                    sim_state_mean_ms = (
                        1000.0 * sim_state_work_s / sim_state_update_count
                        if sim_state_update_count > 0
                        else 0.0
                    )
                    print(
                        "sim-state export: "
                        f"{sim_state_rate:.2f} Hz, mean work {sim_state_mean_ms:.3f} ms"
                    )
                    print(f"=============================")
                    
                    # print_stats(controller)
                    last_stats_time = stats_now
                    sim_state_update_count = 0
                    sim_state_work_s = 0.0
       
                # check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
                # rate_limiter.sleep(env)
    except KeyboardInterrupt:
        print("\nuser interrupted program")
    
    except Exception as e:
        print(f"\nprogram exception: {e}")
    
    finally:
        # clean up resources
        print("\nclean up resources...")
        controller.cleanup()
        if not args_cli.replay_data:
            dds_manager.cleanup()
        if image_server is not None:
            image_server.stop()
        # Isaac Lab's current OpenXRDevice has no public close() method.  Its
        # destructor is an idempotent cleanup hook that explicitly releases
        # the XR message-bus and button subscriptions.  Invoke it while the
        # SimulationContext still exists; merely dropping our local reference
        # is insufficient because those subscriptions also retain callbacks to
        # the device.
        if teleop_interface is not None:
            teleop_cleanup = getattr(teleop_interface, "__del__", None)
            if callable(teleop_cleanup):
                teleop_cleanup()
            teleop_interface = None
            gc.collect()
        env.close()
        print("cleanup completed")
    # profiler.disable()
    # s = io.StringIO()
    # ps = pstats.Stats(profiler, stream=s).strip_dirs().sort_stats("time")
    # ps.print_stats(30)  

    # print(s.getvalue())

if __name__ == "__main__":
    try:
        main()
    finally:
        print("Closing simulation application...")
        try:
            simulation_app.close()
        except Exception as e:
            print(f"Failed to close simulation application: {e}")
        print("Program exit completed")

# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint   --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129

# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-RedBlock-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint     --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Inspire-Joint     --enable_inspire_dds --robot_type g129




# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody  --robot_type g129 --enable_dex1_dds 
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Dex3-Wholebody  --robot_type g129 --enable_dex3_dds 
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Inspire-Wholebody  --robot_type g129 --enable_inspire_dds 

# Gear SONIC -> DDS -> Isaac Lab floating-base G1-29DoF body validation.
# python sim_main.py --task Isaac-G1-29DoF-Sonic --robot_type g129 --action_source sonic_dds --device cpu --no_render
# OpenXR view anchoring and right-controller B yaw recenter (robot control remains SONIC DDS).
# python sim_main.py --task Isaac-G1-29DoF-Sonic --robot_type g129 --action_source sonic_dds --device cpu --teleop_device motion_controllers --xr


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint --enable_inspire_dds --robot_type h1_2
