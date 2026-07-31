@echo off
REM ---------------------------------------------------------------------------
REM AR / XR launcher for unitree_sim_isaaclab on win2.
REM
REM MUST be run from the desktop session (double-click), NOT over ssh:
REM the OpenXR runtime (SteamVR) lives in the interactive session and an
REM ssh-spawned process cannot reach it.
REM
REM Before launching:
REM   1. Start the headset link first - NOLO Link (AR glasses) or ALVR Dashboard
REM      with the Pico client connected.  Either brings up SteamVR, which is the
REM      registered OpenXR runtime.
REM   2. Wait until SteamVR reports the headset ready.
REM   3. Double-click this script.
REM
REM ??? XR mode is a DIFFERENT pipeline from the non-AR one: it loads
REM isaaclab.python.xr.openxr.kit (not the slim kit) and its frame pacing is
REM dictated by the compositor / xrWaitFrame, so the non-AR frame-time ledger
REM in CLAUDE.md does not apply here.  See doc/xr_ar_judder_zh.md for judder.
REM
REM ??? A STALE XRLink left over from an earlier session eats ~245% CPU and will
REM wreck frame times even in AR.  If AR feels far worse than usual, check for a
REM duplicate:  Get-Process XRLink
REM (Do NOT kill SteamVR here - unlike the non-AR case, AR needs it.)
REM
REM Overridable:
REM   SIM_DDS_IFACE  - this machine's IP on the DDS network (default below)
REM   GR00T_WBC_ROOT / SIM_PYTHON - see run_win.bat
REM ---------------------------------------------------------------------------

if not defined SIM_DDS_IFACE set "SIM_DDS_IFACE=192.168.1.130"

call "%~dp0run_win.bat" ^
    --task Isaac-G1-29DoF-Sonic ^
    --robot_type g129 ^
    --action_source sonic_dds ^
    --device cpu ^
    --teleop_device motion_controllers ^
    --dds-interface %SIM_DDS_IFACE% ^
    %*

pause
