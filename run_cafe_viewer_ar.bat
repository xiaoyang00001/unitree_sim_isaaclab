@echo off
REM Cafe AR viewer: identical to the validated Conveyor AR viewer except for
REM the Cafe task. The visible mirrors remain full-fidelity articulations.

set "ISAACLAB_PEER_ROBOT_MODE=articulation"
if defined CAFE_HOST_IP set "PIPELINE_HOST_IP=%CAFE_HOST_IP%"
if not defined SIM_LOG set "SIM_LOG=D:\Isaac\cafe_viewer_ar.log"
set "PYTHONUNBUFFERED=1"

REM KitchenRoom is an external Lightwheel asset and is not part of this Git
REM worktree.  Both deployed Windows viewers keep the verified copy below.
if not defined LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR (
    if exist "D:\Isaac\Lightwheel_OpenSource\Locomotion\KitchenRoom\KitchenRoom.usd" (
        set "LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR=D:\Isaac\Lightwheel_OpenSource"
    )
)
if not defined LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR (
    echo [sonic_cafe] LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR is not set.
    echo [sonic_cafe] Expected D:\Isaac\Lightwheel_OpenSource\Locomotion\KitchenRoom\KitchenRoom.usd
    exit /b 1
)
if not exist "%LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR%\Locomotion\KitchenRoom\KitchenRoom.usd" (
    echo [sonic_cafe] KitchenRoom.usd not found under: %LIGHTWHEEL_OPEN_SOURCE_ROOT_DIR%
    exit /b 1
)

call "%~dp0run_pipeline_viewer_ar.bat" ^
    --task Isaac-G1-29DoF-Sonic-Cafe ^
    %*
