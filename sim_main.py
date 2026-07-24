
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
#!/usr/bin/env python3
# main.py
import os

project_root = os.path.dirname(os.path.abspath(__file__))
os.environ["PROJECT_ROOT"] = project_root

import argparse
import contextlib
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
parser.add_argument("--sonic_ramp_seconds", type=float, default=0.0,
                    help="optional extra blend from the USD default pose to SONIC targets; SONIC already performs its own INIT ramp")
parser.add_argument("--sonic_max_target_step", type=float, default=0.0,
                    help="optional per-control-step joint-target limiter in radians; 0 preserves the raw SONIC policy target")

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
    "--no_render",
    action="store_true",
    default=False,
    help="run headless and disable rendering updates entirely",
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
}
is_sonic_task = args_cli.task in sonic_task_names

if args_cli.auto_reset_on_fall is None:
    # The new behavior is enabled by default only for dedicated SONIC bridge
    # tasks. All existing non-SONIC tasks retain their old behavior.
    args_cli.auto_reset_on_fall = is_sonic_task

if args_cli.step_hz is None:
    args_cli.step_hz = 50 if is_sonic_task else 100
elif args_cli.step_hz <= 0:
    parser.error("--step_hz must be positive")

if args_cli.physics_dt is not None and args_cli.physics_dt <= 0.0:
    parser.error("--physics_dt must be positive")
if args_cli.render_interval is not None and args_cli.render_interval <= 0:
    parser.error("--render_interval must be positive")

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

if args_cli.enable_dex3_dds and args_cli.enable_dex1_dds and args_cli.enable_inspire_dds:
    print("Error: enable_dex3_dds and enable_dex1_dds and enable_inspire_dds cannot be enabled at the same time")
    print("Please select one of the options")
    sys.exit(1)


import pinocchio                 
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)

from dds.reset_pose_dds import *
import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

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
        if args_cli.physics_dt is not None:
            env_cfg.sim.dt = float(args_cli.physics_dt)

        # Isaac Lab's environment step reads cfg.sim.render_interval directly.
        # Changing SimulationContext.render_interval after gym.make() does not
        # alter the modulo test inside ManagerBasedRLEnv.step(), which made the
        # previous command-line override ineffective.  Resolve it before the
        # environment is constructed.
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
    
    # create simplified control configuration
    try:    
        control_config = ControlConfig(
            step_hz=args_cli.step_hz,
            replay_mode=args_cli.replay_data
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

    def trigger_robot_reset(event_name: str, reason: str) -> bool:
        """Reset state and controller history as one operation for SONIC tasks."""
        if not sonic_reset_supported:
            env_cfg.event_manager.trigger(event_name, env)
            return False

        robot_dds = dds_manager.get_object("g129")
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
                        # sim_state = json.dumps(sim_state)
                        sim_state_dds.write_sim_state_data(sim_state)
                    except Exception as e:
                        print(f"Failed to write sim state: {e}")
                        raise e
                    sim_state_update_count += 1
                    sim_state_work_s += monotonic() - sim_state_work_start

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


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint --enable_inspire_dds --robot_type h1_2
