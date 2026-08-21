@echo off
REM ---------------------------------------------------------------------------
REM Pipeline viewer AR launcher - Office scene, operator #1 view.
REM
REM Office-scene counterpart of run_pipeline_viewer_ar.bat: the local process is
REM a pure mirror (ID=0), OpenXR follows PeerRobot (global robot_1), and the peer uses the full
REM articulation asset so the G1 morphology matches the pipeline viewer.  Run
REM from the logged-in desktop session (double-click), not from SSH; SteamVR or
REM NOLO Link must already see the headset.
REM ---------------------------------------------------------------------------

if not defined PIPELINE_HOST_IP set "PIPELINE_HOST_IP=192.168.50.68"
set "ISAACLAB_LOCAL_ROBOT_ID=0"
set "ISAACLAB_SCENE_SYNC=1"
set "ISAACLAB_SCENE_SYNC_PEER_IP=%PIPELINE_HOST_IP%"
set "ISAACLAB_XR_ANCHOR_ROBOT_ID=1"
set "ISAACLAB_PEER_ROBOT_MODE=articulation"
set "UNITREE_DDS_DOMAIN=9"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\pipeline_viewer_ar_robot1_office.log"

if not defined SIM_DDS_IFACE (
    if /i "%COMPUTERNAME%"=="DESKTOP-CMSDIPM" (
        set "SIM_DDS_IFACE=192.168.50.127"
    ) else (
        set "SIM_DDS_IFACE=auto"
    )
)

call "%~dp0run_win.bat" ^
    --task Isaac-G1-29DoF-Sonic-Conveyor-Office ^
    --robot_type g129 ^
    --action_source dds ^
    --device cpu ^
    --teleop_device motion_controllers ^
    --xr_runtime steamvr ^
    --dds-interface %SIM_DDS_IFACE% ^
    --stats_interval 10 ^
    %*

pause
