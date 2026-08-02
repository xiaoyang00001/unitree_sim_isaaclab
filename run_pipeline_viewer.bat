@echo off
REM ---------------------------------------------------------------------------
REM Pipeline viewer launcher (non-AR) - pure mirror of the Ubuntu host scene.
REM
REM Identity: ISAACLAB_LOCAL_ROBOT_ID=0 (viewer). Receives robot_1 + robot_2 +
REM objects from the host's ZMQ PUB, publishes nothing, holds a ghost robot
REM off-stage. Action source auto-switches to "hold" (no DDS lock-step), the
REM main loop runs at full step_hz.
REM
REM Overridable:
REM   PIPELINE_HOST_IP        - Ubuntu host IP (default 192.168.50.68)
REM   ISAACLAB_XR_ANCHOR_ROBOT_ID - which mirror the XR anchor follows (1/2)
REM   SIM_LOG                 - defaults to D:\Isaac\pipeline_viewer.log
REM ---------------------------------------------------------------------------

if not defined PIPELINE_HOST_IP set "PIPELINE_HOST_IP=192.168.50.68"
set "ISAACLAB_LOCAL_ROBOT_ID=0"
set "ISAACLAB_SCENE_SYNC_PEER_IP=%PIPELINE_HOST_IP%"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\pipeline_viewer.log"

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
    --dds-interface %SIM_DDS_IFACE% ^
    --stats_interval 10 ^
    %*
