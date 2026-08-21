@echo off
setlocal

REM Local-only Cafe AR launcher for win129.
REM The Ubuntu authority host is 192.168.1.131 and this workstation serves
REM operator #1, so the OpenXR anchor follows the robot_1 mirror.
REM Start NOLO Link/ALVR and wait for SteamVR before double-clicking this file.

set "PIPELINE_HOST_IP=192.168.1.131"
set "ISAACLAB_XR_ANCHOR_ROBOT_ID=1"
set "ISAACLAB_PEER_ROBOT_MODE=articulation"
set "ISAACLAB_CAFE_KITCHEN_MODE=proxy_background"
set "ISAACLAB_SCENE_SYNC_PORT_BASE=17555"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\cafe_viewer_ar_win129.log"

call "%~dp0run_cafe_viewer_ar.bat" %*

endlocal
