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
REM Override either variable from the environment before calling this script.
REM
REM Usage:
REM   run_win.bat --device cpu --enable_cameras --robot_type g129 ^
REM       --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds
REM ---------------------------------------------------------------------------

chcp 65001 >nul
set "PYTHONUTF8=1"

if not defined GR00T_WBC_ROOT set "GR00T_WBC_ROOT=D:\Isaac\groot_assets"
if not defined SIM_PYTHON set "SIM_PYTHON=C:\Users\admin\miniconda3\envs\env_isaaclab\python.exe"

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

"%SIM_PYTHON%" sim_main.py %*
