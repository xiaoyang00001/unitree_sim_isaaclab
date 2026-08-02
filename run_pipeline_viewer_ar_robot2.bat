@echo off
REM ---------------------------------------------------------------------------
REM Pipeline viewer AR launcher - operator #2 view (XR anchor follows robot_2).
REM
REM Thin wrapper over run_pipeline_viewer_ar.bat: identical in every way except
REM the XR anchor follows the robot_2 mirror (PeerRobot2) instead of robot_1.
REM Per-machine usage: operator #1's machine double-clicks
REM run_pipeline_viewer_ar.bat, operator #2's machine double-clicks this one.
REM Same rules apply: desktop session only (not ssh), headset link + SteamVR
REM ready before launching.
REM ---------------------------------------------------------------------------

set "ISAACLAB_XR_ANCHOR_ROBOT_ID=2"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\pipeline_viewer_ar_robot2.log"

call "%~dp0run_pipeline_viewer_ar.bat" %*
