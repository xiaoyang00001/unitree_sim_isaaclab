@echo off
REM ---------------------------------------------------------------------------
REM Pipeline viewer AR launcher - Office scene, operator #2 view.
REM
REM This is the Office-scene counterpart of run_pipeline_viewer_ar_robot2.bat:
REM the local process is a pure mirror (ID=0), OpenXR follows PeerRobot2, and
REM the peer uses the full articulation asset so the G1 morphology matches the
REM pipeline viewer.  Run it from the logged-in desktop session (double-click),
REM not from an SSH shell; SteamVR/NOLO Link must already see the headset.
REM
REM The upstream host must publish robot_2 for this anchor to move.  With a
REM one-robot host, PeerRobot2 stays at its spawn pose until the second host
REM robot/deploy is enabled.  This does not affect robot_1 scene reception.
REM ---------------------------------------------------------------------------

if not defined PIPELINE_HOST_IP set "PIPELINE_HOST_IP=192.168.50.68"
set "ISAACLAB_LOCAL_ROBOT_ID=0"
set "ISAACLAB_SCENE_SYNC=1"
set "ISAACLAB_SCENE_SYNC_PEER_IP=%PIPELINE_HOST_IP%"
set "ISAACLAB_XR_ANCHOR_ROBOT_ID=2"
set "ISAACLAB_PEER_ROBOT_MODE=articulation"
set "UNITREE_DDS_DOMAIN=9"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\pipeline_viewer_ar_robot2_office.log"

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
