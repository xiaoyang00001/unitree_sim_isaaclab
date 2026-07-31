@echo off
REM ---------------------------------------------------------------------------
REM One-shot build of the collision-free mirror robot USD that
REM Isaac-G1-29DoF-Sonic-Conveyor needs. Run once per machine.
REM
REM Shipping note (82df796): the product IS committed to git (win2-built,
REM     self-contained, no absolute paths), so a fresh machine only needs
REM     git pull. This script is the REBUILD path: run it with --force after
REM     the SONIC URDF or the IsaacLab/UrdfConverter version changes, then
REM     commit the regenerated scene_assets\peer_robot\ back to the repo.
REM     tools/build_peer_robot_usd.py converts the runtime-synthesised SONIC
REM     URDF and deactivates every collisions scope.
REM
REM Why it has to exist before the conveyor task runs:
REM     G129SonicConveyorEnvCfg.__post_init__ raises RuntimeError when the file
REM     is missing. The URDF-direct fallback keeps its collision bodies, so the
REM     mirror robot gets kicked out by the ground and then drifts upwards at a
REM     constant speed under zero gravity. Every other task only prints the
REM     import-time warning and runs fine without this asset.
REM
REM Environment probing (GR00T_WBC_ROOT / SIM_PYTHON) is a copy of run_win.bat -
REM keep the two in sync. Override either from the environment before calling.
REM
REM Usage - launches Isaac Sim headless, expect ~2 min:
REM     build_peer_win.bat              no-op if the product already exists
REM     build_peer_win.bat --force      rebuild anyway
REM ---------------------------------------------------------------------------

chcp 65001 >nul
set "PYTHONUTF8=1"

cd /d "%~dp0"

set "PEER_USD=%~dp0tasks\g1_tasks\g1_29dof_sonic_conveyor\scene_assets\peer_robot\g1_43dof_peer.usd"

if not defined GR00T_WBC_ROOT (
    if exist "D:\Isaac\groot_assets\gear_sonic\data" (
        set "GR00T_WBC_ROOT=D:\Isaac\groot_assets"
    ) else (
        set "GR00T_WBC_ROOT=D:\reboot\GR00T-WholeBodyControl"
    )
)
if not defined SIM_PYTHON (
    if exist "C:\Users\admin\miniconda3\envs\env_isaaclab\python.exe" (
        set "SIM_PYTHON=C:\Users\admin\miniconda3\envs\env_isaaclab\python.exe"
    ) else (
        set "SIM_PYTHON=C:\Users\nolovr\miniconda3\envs\env_isaaclab\python.exe"
    )
)

if not exist "%SIM_PYTHON%" (
    echo [build_peer] python not found: %SIM_PYTHON%
    echo [build_peer] set SIM_PYTHON to your env_isaaclab python.exe
    exit /b 1
)
if not exist "%GR00T_WBC_ROOT%\gear_sonic\data" (
    echo [build_peer] GR00T assets not found under: %GR00T_WBC_ROOT%
    echo [build_peer] expected %GR00T_WBC_ROOT%\gear_sonic\data\...
    exit /b 1
)

if exist "%PEER_USD%" if /i not "%~1"=="--force" (
    echo [build_peer] already built: %PEER_USD%
    echo [build_peer] pass --force to rebuild
    exit /b 0
)

echo [build_peer] GR00T_WBC_ROOT=%GR00T_WBC_ROOT%
echo [build_peer] launching Isaac Sim headless, this takes a couple of minutes...
"%SIM_PYTHON%" tools\build_peer_robot_usd.py
set "RC=%ERRORLEVEL%"

if not exist "%PEER_USD%" (
    echo [build_peer] FAILED - product still missing: %PEER_USD%
    exit /b 1
)
echo [build_peer] OK: %PEER_USD%
exit /b %RC%
