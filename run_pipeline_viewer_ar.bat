@echo off
REM ---------------------------------------------------------------------------
REM Pipeline viewer AR launcher - mirror scene + OpenXR headset view.
REM
REM MUST be run from the desktop session (double-click), NOT over ssh:
REM the OpenXR runtime (SteamVR) lives in the interactive session.
REM
REM Before launching:
REM   1. Start NOLO Link (AR glasses) or ALVR Dashboard with the Pico client
REM      connected; wait until SteamVR reports the headset ready.
REM   2. Double-click this script.
REM
REM The XR anchor follows a MIRROR robot (the local Robot is an off-stage
REM ghost). Pick which operator view this machine serves:
REM   set ISAACLAB_XR_ANCHOR_ROBOT_ID=1   (default; follow robot_1 mirror)
REM   set ISAACLAB_XR_ANCHOR_ROBOT_ID=2   (follow robot_2 mirror)
REM Right-controller B (release) recenters the view yaw; it does NOT reset the
REM environment. Robot control stays 100% on the Ubuntu-side deploys.
REM
REM XR framerate caveats: XR loads a different kit pipeline (no slim kit),
REM frame pacing is dictated by the compositor; the non-AR ledger does not
REM apply. A stale XRLink from an earlier session eats ~245% CPU - check
REM Get-Process XRLink if AR feels much worse than usual.
REM ---------------------------------------------------------------------------

if not defined PIPELINE_HOST_IP set "PIPELINE_HOST_IP=192.168.50.68"
set "ISAACLAB_LOCAL_ROBOT_ID=0"
set "ISAACLAB_SCENE_SYNC_PEER_IP=%PIPELINE_HOST_IP%"
if not defined ISAACLAB_XR_ANCHOR_ROBOT_ID set "ISAACLAB_XR_ANCHOR_ROBOT_ID=1"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\pipeline_viewer_ar.log"

if not defined SIM_DDS_IFACE (
    if /i "%COMPUTERNAME%"=="DESKTOP-CMSDIPM" (
        set "SIM_DDS_IFACE=192.168.50.127"
    ) else (
        set "SIM_DDS_IFACE=auto"
    )
)

call "%~dp0run_win.bat" ^
    --task Isaac-G1-29DoF-Sonic-Conveyor ^
    --robot_type g129 ^
    --action_source dds ^
    --device cpu ^
    --teleop_device motion_controllers ^
    --dds-interface %SIM_DDS_IFACE% ^
    --stats_interval 10 ^
    %*

pause
