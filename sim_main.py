
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
import getpass
import math
import stat
import tempfile
import time
import sys
import signal
import threading
import torch
import gymnasium as gym
from pathlib import Path

# Windows: import h5py 必须抢在 kit 起来之前。h5py 的 hdf5.dll/z.dll 没有
# delvewheel 改名隔离,而 GUI/XR 的扩展集会先加载同名 DLL(Windows 同名 DLL
# 先到先得),之后 isaaclab 在 kit 扩展里 import h5py 就报
# "DLL load failed while importing _errors"。趁进程还干净先 import,
# 正确的 DLL 被钉进内存,kit 后面加载什么都不影响(模块已在 sys.modules)。
# headless 扩展集小、搜索空间干净,从来不触发——所以烟测测不出来,
# 只有 GUI/XR/enable_cameras 会撞。
if os.name == "nt":
    try:
        import h5py  # noqa: F401
    except Exception as _h5py_preload_error:
        print(f"[sim] WARNING: h5py preload failed: {_h5py_preload_error}")

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
    "--full_kit",
    "--full-kit",
    dest="full_kit",
    action="store_true",
    default=False,
    help=(
        "use Isaac Lab's stock GUI experience instead of the trimmed SONIC one. "
        "默认精简版去掉了资产浏览器/示例机器人/合成数据链路等本工程用不到的扩展, "
        "并关掉每帧重绘的地面网格与选中轮廓(渲染 8.33→7.05 ms)。需要 Stage 树、"
        "Property 面板这些编辑器功能时用本开关回退"
    ),
)
parser.add_argument(
    "--hide_ui",
    "--hide-ui",
    dest="hide_ui",
    action="store_true",
    default=False,
    help=(
        "hide every Kit panel and keep only the viewport (--/app/window/hideUi=1). "
        "精简 experience 下本来就是默认行为,这里是给 --full_kit 用的显式开关: "
        "再省 ~0.7 ms/帧(conveyor 场景 7.05→6.35 ms)"
    ),
)
parser.add_argument(
    "--show_ui",
    "--show-ui",
    dest="show_ui",
    action="store_true",
    default=False,
    help=(
        "在精简 experience 下强行显示 Kit 面板。默认不显示是因为精简版里 Stage/"
        "Console 这些停靠容器已被去掉,残留的 Content 浏览器会变成浮动窗口"
        "**挡住 viewport**(实测)。要编辑器请优先用 --full_kit"
    ),
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
    default=None,
    help=(
        "render once every N control loops in late-render mode. Default 1 "
        "(GUI 画面与物理同频 50 fps): 实测非 AR 闭环下 A+E+R 约 13-14 ms, "
        "20 ms 预算里还剩 5-7 ms 余量,每圈渲染不掉主循环。设 2 可把渲染成本 "
        "再摊薄一半(画面 25 fps),留给场景更重、余量吃紧的情况。XR 下默认同样 "
        "每圈,且**不建议改**:实测设 2 无法锁住匀速档——非渲染圈由 step_hz "
        "deadline 网格定拍、渲染圈由 xrWaitFrame 帧槽网格定拍,两套时钟不整除 "
        "会打拍(50Hz 对 72Hz:每对循环相位漂 ~12ms),画面仍在 2/3 帧槽间交替。 "
        "保留此旋钮只为不静默覆盖用户输入与留档该负结论"
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

# ⚠️ --xr_runtime 必须定义在 add_app_launcher_args() **之后**,别挪回上面。
# 该函数内部会先跑一次 parser.parse_known_args() 探测(app_launcher.py:258),
# 而 --xr 是在这次探测**之后**才注册的。若此时 --xr_runtime 已存在,argparse 的
# 缩写匹配会把命令行里的 --xr 解析成 --xr_runtime 的缩写,于是它去吃下一个 token
# 当参数值,报 "argument --xr_runtime: expected one argument"——等于把所有现有的
# --xr 命令行全打断(2026-07-29 实测踩过)。定义在探测之后:探测期 --xr_runtime
# 尚不存在会被当未知参数忽略,--xr 正常精确匹配;下面的 parse_args() 时两者都在,
# 精确匹配优先于缩写,互不干扰。tests/test_xr_runtime_cli.py 锁死这个行为。
parser.add_argument(
    "--xr_runtime",
    choices=("auto", "steamvr", "cloudxr"),
    default="auto",
    help=(
        "选择 --xr 使用哪个 OpenXR runtime。"
        "cloudxr:用 ~/.cloudxr 的外部 CloudXR runtime——**这是消除 AR 卡顿的正解**,"
        "它的头显端客户端自带重投影/ATW,而 SteamVR+NOLO 这条链全程无重投影"
        "(见 doc/xr_ar_judder_zh.md)。要求 runtime 已在跑,否则直接报错退出而**不是**"
        "静默降级——静默跑回 SteamVR 是本方案最容易浪费半天的失败模式。"
        "steamvr:显式清掉 XR_RUNTIME_JSON,强制走 active_runtime.json。"
        "auto(默认):不干预环境,只在启动日志里报告实际会用哪个 runtime"
    ),
)
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


# ---------------------------------------------------------------------------
# OpenXR runtime 选择
#
# AR 卡顿的根因是 SteamVR + NOLO XrLink 这条链全程没有重投影/ATW(证据与机制见
# doc/xr_ar_judder_zh.md)。CloudXR 的头显端客户端自带重投影,是目前唯一能根治的
# 通路,且本机 runtime 已装好、Isaac Sim 已实连过。
#
# 两个必须同时满足的条件,少一个就会**静默**跑回 SteamVR:
#   1. 进程环境里有 XR_RUNTIME_JSON(以及 NV_CXR_RUNTIME_DIR——libopenxr_cloudxr.so
#      靠它定位 ipc_cloudxr 这个 unix socket 才连得上服务进程);
#   2. kit 设置 xr/system/openxr/runtime=custom + activeRuntimeJSON。因为
#      isaaclab.python.xr.openxr.kit 里硬写了 runtime="system",而命令行 --/ 设置
#      优先级高于 .kit 文件。这条不依赖"system 模式是否尊重 XR_RUNTIME_JSON"。
# ⚠️ 绝不能写 runtime=cloudxr:那个值指向 Isaac Sim 内置的 CloudXR 5.0.0 并会自己
#    拉起内置 service,与外部 6.2.0 抢同一套 IPC 与 49100/48322 端口。
# ---------------------------------------------------------------------------
CLOUDXR_RUN_DIR_DEFAULT = os.path.expanduser("~/.cloudxr/run")


def _append_kit_args(extra: str) -> None:
    args_cli.kit_args = f"{args_cli.kit_args} {extra}" if args_cli.kit_args else extra


def _set_openxr_runtime_kit_args(runtime_json: str | None) -> None:
    """把 kit 的 OpenXR runtime 选择钉死。

    ⚠️ 这些是 ``/persistent/`` 设置——它们会被写进 Isaac Sim 的 user.config.json
    **跨进程残留**,而且**优先级高于** experience 文件里的
    ``persistent.xr.system.openxr.runtime = "system"``(实测:跑过一次 CloudXR 之后,
    下一次普通 --xr 启动仍会去连 CloudXR,报 "Failed to connect to monado service
    process" 然后 xrCreateInstance 失败)。所以每种模式都必须**显式**写回自己要的值,
    不能靠"不设置"来表达"用默认"。
    """
    if runtime_json:
        _append_kit_args(
            "--/persistent/xr/system/openxr/runtime=custom "
            f"--/persistent/xr/system/openxr/activeRuntimeJSON={runtime_json}"
        )
    else:
        # 一并清空 activeRuntimeJSON:system 模式不认 JSON 路径,残留值会让 kit 每次
        # 都打 "runtime type 'system' selected, but JSON path provided" 的告警。
        _append_kit_args(
            "--/persistent/xr/system/openxr/runtime=system "
            "--/persistent/xr/system/openxr/activeRuntimeJSON="
        )


def _load_cloudxr_env(run_dir: str) -> dict:
    """读 CloudXR 的运行期环境快照,返回 {KEY: value}。

    该文件由 runtime 启动时写出,每行形如 ``export KEY=value``(实测无引号、无续行)。
    不做 shell 解析,避免把 source 的副作用带进来。
    """
    env_path = os.path.join(run_dir, "cloudxr.env")
    parsed = {}
    with open(env_path, "r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("export "):
                line = line[len("export "):]
            key, sep, value = line.partition("=")
            if sep:
                parsed[key.strip()] = value.strip().strip('"').strip("'")
    return parsed


if args_cli.xr_runtime == "cloudxr":
    if not args_cli.xr:
        parser.error(
            "--xr_runtime cloudxr 需要 XR 模式;加 --xr 或 "
            "--teleop_device motion_controllers"
        )
    _cxr_run_dir = os.environ.get("CLOUDXR_RUN_DIR", CLOUDXR_RUN_DIR_DEFAULT)
    # 就绪判据与 IsaacLab fork 的 start_ubuntu_isaaclab_sonic.sh 保持一致。
    # 这里刻意用 parser.error 硬失败,而不是本工程惯用的 try/except+print 降级:
    # 降级会让人对着 SteamVR 测半天卡顿,还以为在测 CloudXR。
    if not os.path.exists(os.path.join(_cxr_run_dir, "runtime_started")):
        parser.error(
            f"CloudXR runtime 未就绪:{_cxr_run_dir}/runtime_started 不存在。"
            "先启动 runtime:python -m isaacteleop.cloudxr --accept-eula --host-client"
        )
    _cxr_ipc = os.path.join(_cxr_run_dir, "ipc_cloudxr")
    if not (os.path.exists(_cxr_ipc) and stat.S_ISSOCK(os.stat(_cxr_ipc).st_mode)):
        parser.error(
            f"CloudXR runtime 未就绪:{_cxr_run_dir}/ipc_cloudxr 不是 unix socket。"
            "runtime 可能已死或残留孤儿进程占着 49100(ss -ltnp | grep 49100 查)"
        )
    try:
        _cxr_env = _load_cloudxr_env(_cxr_run_dir)
    except OSError as e:
        parser.error(f"读取 CloudXR 环境快照失败:{e}")
    _cxr_runtime_json = _cxr_env.get("XR_RUNTIME_JSON")
    if not _cxr_runtime_json or not os.path.isfile(_cxr_runtime_json):
        parser.error(
            f"CloudXR 环境快照里的 XR_RUNTIME_JSON 无效:{_cxr_runtime_json!r}"
        )
    os.environ.update(_cxr_env)
    _set_openxr_runtime_kit_args(_cxr_runtime_json)
    print(f"[xr] OpenXR runtime = CloudXR ({_cxr_runtime_json})")
    print(
        "[xr] 头显端连 https://<本机IP>:48322/client/ ;"
        "重投影由客户端负责,这是消卡顿的关键"
    )
elif args_cli.xr_runtime == "steamvr":
    # 显式清掉,免得"上一条命令 source 过 cloudxr.env"这种残留把人绕晕。
    if os.environ.pop("XR_RUNTIME_JSON", None):
        print("[xr] cleared inherited XR_RUNTIME_JSON")
    _set_openxr_runtime_kit_args(None)
    print("[xr] OpenXR runtime = SteamVR (~/.config/openxr/1/active_runtime.json)")
    print("[xr] ⚠️ 该链无重投影,应用帧率低于面板刷新率必然卡顿——见 doc/xr_ar_judder_zh.md")
elif args_cli.xr:
    # auto:跟随环境,但必须把"实际会用哪个"喊出来——漏 source 时静默走 SteamVR 是
    # 这条路最容易浪费半天的失败模式,一行日志就能挡住。
    # 注意 auto 并非"什么都不做":kit 那侧的 runtime 选择必须显式写回,否则会继承
    # 上一次 CloudXR 运行残留在 user.config.json 里的 custom(见
    # _set_openxr_runtime_kit_args 的说明)。
    _inherited = os.environ.get("XR_RUNTIME_JSON")
    if _inherited:
        # 用户自己 source 过 cloudxr.env。把 kit 也对齐到同一个 runtime,
        # 免得"环境变量指 CloudXR、kit 走 system"两头不一致。
        _set_openxr_runtime_kit_args(_inherited)
        print(f"[xr] OpenXR runtime = 继承自环境 XR_RUNTIME_JSON={_inherited}")
        if "cloudxr" not in _inherited.lower():
            print("[xr] ⚠️ 不是 CloudXR;要消卡顿请加 --xr_runtime cloudxr")
    else:
        _set_openxr_runtime_kit_args(None)
        print(
            "[xr] OpenXR runtime = 系统默认(active_runtime.json,本机为 SteamVR)。"
            "⚠️ 该链无重投影必卡顿,消卡顿请加 --xr_runtime cloudxr"
        )


def _resolve_slim_experience() -> str | None:
    """把本工程的精简 kit 模板解析成可用的 experience 文件,返回其路径。

    为什么要"解析"而不是直接用:kit 里的 ``${app}`` 指 experience 文件所在目录。
    模板放在本工程 ``apps/`` 下,``${app}/../source`` 会指到本工程而不是 Isaac Lab,
    扩展目录就全找不着了。这里按实际安装位置(可编辑安装,随机器而异)把占位符
    换成绝对路径,产物写进 /tmp——与 SONIC URDF 的做法一致。
    """
    import isaaclab

    template = os.path.join(project_root, "apps", "isaaclab.sonic.kit")
    if not os.path.isfile(template):
        print(f"[sim] slim kit 模板缺失,回退默认 experience: {template}")
        return None

    # namespace package 形态下 __file__ 会是 None,拿不到就老实回退默认 experience
    isaaclab_init = getattr(isaaclab, "__file__", None)
    if not isaaclab_init:
        print("[sim] 无法定位 isaaclab 包位置,回退默认 experience")
        return None

    isaaclab_source = Path(isaaclab_init).resolve().parents[2]
    isaaclab_apps = isaaclab_source.parent / "apps"
    if not isaaclab_source.is_dir() or not isaaclab_apps.is_dir():
        print(
            "[sim] 无法定位 Isaac Lab 的 source/apps 目录,回退默认 experience "
            f"(source={isaaclab_source}, apps={isaaclab_apps})"
        )
        return None

    with open(template, "r", encoding="utf-8") as fp:
        content = fp.read()
    # 必须用正斜杠:kit 是 TOML,Windows 路径里的反斜杠会被当成转义序列
    # (D:\Isaac 的 \I 就是无效转义),整个 experience 会解析失败、扩展目录全丢。
    # kit 在 Windows 上同样认正斜杠,模板里内建的 ${exe-path}/exts 也是这么写的。
    content = content.replace("@ISAACLAB_APPS@", isaaclab_apps.as_posix())
    content = content.replace("@ISAACLAB_SOURCE@", isaaclab_source.as_posix())

    # 后缀在 POSIX 上沿用 uid(保持既有 /tmp 产物路径不变);Windows 没有
    # os.getuid,退回登录名。
    getuid = getattr(os, "getuid", None)
    user_suffix = str(getuid()) if getuid is not None else getpass.getuser()
    out_dir = os.path.join(
        tempfile.gettempdir(), f"unitree_sim_isaaclab_kit_{user_suffix}"
    )
    os.makedirs(out_dir, exist_ok=True)
    resolved = os.path.join(out_dir, "isaaclab.sonic.kit")
    with open(resolved, "w", encoding="utf-8") as fp:
        fp.write(content)
    return resolved


# 精简 experience:去掉编辑器 UI 窗口/示例机器人/合成数据链路等本工程用不到的
# 扩展,并关掉每帧重绘的地面网格与选中轮廓。实测 conveyor 场景渲染 8.33→7.05ms;
# 再叠 --hide_ui 到 6.35ms(-24%)。GUI 闭环 20ms 预算里这 1-2ms 是能不能每圈渲染
# (画面 50fps)的关键。要用 Isaac Sim 的编辑器面板时加 --full_kit 回退。
# ⚠️ XR 与 enable_cameras 各有专属 experience(openxr / rendering kit),精简版
# 没有 OpenXR 与合成数据那套扩展,这两种模式下必须让 AppLauncher 自己选。
if (
    is_sonic_task
    and not args_cli.full_kit
    and not args_cli.headless
    and not args_cli.no_render
    and not args_cli.xr
    and not args_cli.enable_cameras
    and not args_cli.experience
):
    _slim_experience = _resolve_slim_experience()
    if _slim_experience:
        args_cli.experience = _slim_experience
        print(f"[sim] slim kit experience enabled: {_slim_experience}")
        print("      (编辑器面板/菜单已精简,需要完整 GUI 时加 --full_kit)")
        # 精简版缺 Stage/Console 等停靠容器,残留的 Content 浏览器会浮起来盖住
        # viewport(实测画面直接看不见),所以默认只留 viewport;鼠标照样能飞相机。
        if not args_cli.show_ui:
            args_cli.hide_ui = True
        # viewport 左上角的 FPS/Frame time HUD 由 omni.kit.viewport.window 自带的
        # ViewportFPS layer 画,hideUi 并不隐藏它——但精简版用的是独立 app name,
        # 那份 user.config.json 里 renderFPS 默认 False,于是"看不到帧率"。显式打开。
        # ⚠️ 必须写 /persistent/... 这一支:同名的 /app/viewport/defaults/... 优先级更低,
        # persistent 键一旦存在就盖住它。
        _hud_args = (
            "--/persistent/app/viewport/Viewport/Viewport0/hud/renderFPS/visible=true"
        )
        args_cli.kit_args = (
            f"{args_cli.kit_args} {_hud_args}" if args_cli.kit_args else _hud_args
        )

if args_cli.hide_ui:
    # 隐藏 kit 的全部面板只留 viewport:省掉每帧 imgui 绘制,实测再降 ~0.7ms。
    _hide_ui_arg = "--/app/window/hideUi=1"
    args_cli.kit_args = (
        f"{args_cli.kit_args} {_hide_ui_arg}" if args_cli.kit_args else _hide_ui_arg
    )
    print("[sim] kit UI panels hidden (viewport only)")

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
        # 头显 motion-to-photon 延迟更低。
        late_render_active = (
            is_sonic_task
            and not args_cli.no_late_render
            and not args_cli.no_render
            and not getattr(args_cli, "headless", False)
            and not args_cli.replay_data
            and args_cli.render_interval is None
        )
        # 渲染间隔默认每圈(非 AR 画面=物理=50fps;XR 每圈撞 xrWaitFrame,全环
        # 单时钟才能锁相)。此前 XR 下硬编码为 1、静默丢弃用户输入,现改为接受
        # 显式覆盖——但 XR 下改它救不了抖动:2026-07-29 实测 interval=2 仍抖,
        # 因为非渲染圈走 step_hz deadline 网格、渲染圈走帧槽网格,两套刚性时钟
        # 不整除必打拍。AR 卡顿的真因是全链无重投影(见 doc/xr_ar_judder_*.md)。
        late_render_interval = (
            1
            if args_cli.late_render_interval is None
            else max(1, int(args_cli.late_render_interval))
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
                f"only; the controller renders every {late_render_interval} "
                "control loop(s) after the LowState publish (disable with "
                "--no_late_render); "
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
            late_render_interval=late_render_interval,
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
                    render_frames, render_work_s = controller.pop_render_stats()
                    if render_frames > 0 and stats_window_s > 0.0:
                        # 晚渲染:GUI 画面帧率 = 本窗口 sim.render() 次数 / 窗口时长,
                        # 与主循环 Hz 不同(隔圈渲染时约为其 1/late_render_interval)。
                        print(
                            "GUI render: "
                            f"{render_frames / stats_window_s:.2f} fps, "
                            f"mean {1000.0 * render_work_s / render_frames:.2f} ms/frame"
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
