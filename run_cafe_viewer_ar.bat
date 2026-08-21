@echo off
REM Cafe AR viewer: identical to the validated Conveyor AR viewer except for
REM the Cafe task. The visible mirrors remain full-fidelity articulations.

set "ISAACLAB_PEER_ROBOT_MODE=articulation"
if defined CAFE_HOST_IP set "PIPELINE_HOST_IP=%CAFE_HOST_IP%"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\cafe_viewer_ar.log"

call "%~dp0run_pipeline_viewer_ar.bat" ^
    --task Isaac-G1-29DoF-Sonic-Cafe ^
    %*
