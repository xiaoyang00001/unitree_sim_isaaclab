@echo off
REM ---------------------------------------------------------------------------
REM Windows launcher for unitree_sim_isaaclab.
REM
REM Why this wrapper is needed (both settings are mandatory, not conveniences):
REM
REM   chcp 65001 + PYTHONUTF8=1
REM       The project logs contain CJK text and emoji. On a Chinese Windows the
REM       console code page is GBK, so print() raises UnicodeEncodeError and
REM       kills the process. chcp fixes the display, PYTHONUTF8 fixes the crash.
REM
REM   GR00T_WBC_ROOT
REM       tasks/__init__.py imports every task package, and the SONIC task
REM       synthesises its URDF at import time. Without this the run dies during
REM       "import tasks" no matter which --task you asked for. The hard-coded
REM       fallback in the source points at another machine's Linux path.
REM
REM Defaults below are probed per machine (win2 = D:\Isaac + user admin, the
REM newer box = D:\reboot + user nolovr) so the same script runs on both.
REM Override any variable from the environment before calling this script.
REM
REM   SIM_LOG
REM       If set, stdout/stderr are redirected to this file. Needed whenever
REM       the sim is started from the desktop session (double-click/schtasks):
REM       that console window is unreachable over ssh, and frame attribution
REM       needs the [Performance] lines. Plain > redirection on purpose - a
REM       PowerShell Tee pipeline would re-quote %* and swallow stdin, which
REM       the keyboard-driven path needs.
REM
REM Usage:
REM   run_win.bat --device cpu --enable_cameras --robot_type g129 ^
REM       --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds
REM ---------------------------------------------------------------------------

chcp 65001 >nul
set "PYTHONUTF8=1"

if not defined UNITREE_SKIP_LOWSTATE_CRC set "UNITREE_SKIP_LOWSTATE_CRC=1"

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

cd /d "%~dp0"

if not exist "%SIM_PYTHON%" (
    echo [run_win] python not found: %SIM_PYTHON%
    echo [run_win] set SIM_PYTHON to your env_isaaclab python.exe
    exit /b 1
)
if not exist "%GR00T_WBC_ROOT%\gear_sonic\data" (
    echo [run_win] GR00T assets not found under: %GR00T_WBC_ROOT%
    echo [run_win] expected %GR00T_WBC_ROOT%\gear_sonic\data\...
    exit /b 1
)

if defined SIM_LOG (
    echo [run_win] logging to %SIM_LOG%
    "%SIM_PYTHON%" sim_main.py %* > "%SIM_LOG%" 2>&1
) else (
    "%SIM_PYTHON%" sim_main.py %*
)
