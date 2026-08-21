@echo off
REM Cafe AR operator #2: keep the shared AR viewer and follow PeerRobot2.

set "ISAACLAB_XR_ANCHOR_ROBOT_ID=2"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\cafe_viewer_ar_robot2.log"

call "%~dp0run_cafe_viewer_ar.bat" %*
