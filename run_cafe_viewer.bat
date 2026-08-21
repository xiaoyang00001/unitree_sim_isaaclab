@echo off
REM Cafe viewer: reuse the validated Conveyor viewer topology and runtime.
REM Only the task and log name differ; force the full-fidelity peer mode so a
REM stale shell variable cannot select the diagnostic geometry mode.

set "ISAACLAB_PEER_ROBOT_MODE=articulation"
if defined CAFE_HOST_IP set "PIPELINE_HOST_IP=%CAFE_HOST_IP%"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\cafe_viewer.log"

call "%~dp0run_pipeline_viewer.bat" ^
    --task Isaac-G1-29DoF-Sonic-Cafe ^
    %*
