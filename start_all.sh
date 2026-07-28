#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR=/home/nolovr/Documents/unitree_sim_isaaclab
STEAMVR_DIR=/home/nolovr/.local/share/Steam/steamapps/common/SteamVR
STEAMVR_CONFIG=/home/nolovr/.local/share/Steam/config/steamvr.vrsettings
NOLO_LOG=/home/nolovr/Downloads/nolo_driver_deploy/log/nolo-link-driver.log
NOLO_CONFIG=/home/nolovr/Downloads/nolo_driver_deploy/XrLinkConfig.json
GROOT_DIR=/home/nolovr/GR00T-WholeBodyControl/gear_sonic_deploy
GROOT_ROOT=/home/nolovr/GR00T-WholeBodyControl
STATE_DIR=/home/nolovr/.local/state/unitree-xr
STREAM_HOST=${STREAM_HOST:-192.168.1.131}
export STREAM_HOST

ISAAC_UNIT=unitree-isaac.service
GROOT_UNIT=unitree-groot.service
PICO_UNIT=unitree-pico-manager.service
MANAGED_UNITS=("$GROOT_UNIT" "$ISAAC_UNIT" "$PICO_UNIT")
ISAAC_ARGS=()

info() { printf '\033[1;34m[启动]\033[0m %s\n' "$*"; }
ok() { printf '\033[1;32m[完成]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[提示]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[失败]\033[0m %s\n' "$*" >&2; exit 1; }

require_file() {
  [[ -e "$1" ]] || die "缺少文件：$1"
}

unit_active() {
  systemctl --user is-active --quiet "$1" 2>/dev/null
}

stop_unit() {
  systemctl --user stop "$1" 2>/dev/null || true
  systemctl --user reset-failed "$1" 2>/dev/null || true
}

stop_stale_workloads() {
  local pattern
  local patterns=(
    'python.*sim_main.py.*--task Isaac-G1-29DoF'
    'g1_deploy_onnx_ref.*--input-type zmq_manager'
    'gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5556'
  )

  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
}

start_transient() {
  local unit=$1
  local workdir=$2
  local command=$3

  stop_unit "$unit"
  systemd-run --user --quiet --collect \
    --unit="$unit" \
    --service-type=exec \
    --property=KillMode=control-group \
    --property=TimeoutStopSec=20 \
    --working-directory="$workdir" \
    /bin/bash -lc "$command"
}

stop_steamvr() {
  local name

  # vrmonitor 是主控进程；先结束它，让 vrserver/vrcompositor 自行清理退出。
  for name in vrmonitor vrstartup vrdashboard vrwebhelper; do
    pkill -TERM -x "$name" 2>/dev/null || true
  done

  for _ in $(seq 1 15); do
    pgrep -x vrserver >/dev/null || return 0
    sleep 1
  done

  warn "SteamVR 未及时退出，清理残留子进程。"
  for name in vrcompositor vrserver; do
    pkill -TERM -x "$name" 2>/dev/null || true
  done
  sleep 3
  for name in vrstartup vrdashboard vrwebhelper vrmonitor vrcompositor vrserver; do
    pkill -KILL -x "$name" 2>/dev/null || true
  done
}

enable_nolo_driver() {
  local tmp
  mkdir -p "$STATE_DIR"
  if [[ ! -e "$STATE_DIR/steamvr.vrsettings.before-one-click" ]]; then
    cp "$STEAMVR_CONFIG" "$STATE_DIR/steamvr.vrsettings.before-one-click"
  fi
  tmp=$(mktemp "$STATE_DIR/steamvr.vrsettings.XXXXXX")
  jq '.driver_nolo.enable = true | .driver_nolo.blocked_by_safe_mode = false' \
    "$STEAMVR_CONFIG" >"$tmp"
  chmod --reference="$STEAMVR_CONFIG" "$tmp"
  mv "$tmp" "$STEAMVR_CONFIG"
}

start_steamvr() {
  local vr_pid deadline

  vr_pid=$(pgrep -n -x vrserver || true)
  if [[ -n "$vr_pid" ]] \
    && pgrep -x vrcompositor >/dev/null \
    && grep -F "PID=$vr_pid]" "$NOLO_LOG" 2>/dev/null \
      | grep -F 'CNvEncoder is successfully initialized.' >/dev/null; then
    ok "复用已就绪的 SteamVR/NOLO（vrserver PID $vr_pid）。"
    return 0
  fi

  info "重新启动 SteamVR，并启用 NOLO 驱动。"
  stop_steamvr
  enable_nolo_driver

  (
    cd "$STEAMVR_DIR"
    nohup env \
      DISPLAY="${DISPLAY:-:1}" \
      XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
      DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/1000/bus}" \
      /usr/bin/steam steam://rungameid/250820 \
      >"$STATE_DIR/steamvr-launch.log" 2>&1 </dev/null &
  )

  deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    vr_pid=$(pgrep -n -x vrserver || true)
    if [[ -n "$vr_pid" ]] \
      && pgrep -x vrcompositor >/dev/null \
      && grep -F "PID=$vr_pid]" "$NOLO_LOG" 2>/dev/null \
        | grep -F 'CNvEncoder is successfully initialized.' >/dev/null; then
      ok "SteamVR 与 NOLO/NVENC 已就绪（vrserver PID $vr_pid）。"
      return 0
    fi
    sleep 1
  done

  die "SteamVR 在 90 秒内未完成初始化。查看：$STATE_DIR/steamvr-launch.log"
}

start_pico_manager() {
  info "启动 PICO Manager。"
  start_transient "$PICO_UNIT" "$GROOT_ROOT" \
    'export PYTHONUNBUFFERED=1; source .venv_teleop/bin/activate && exec python gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5556'
  ok "PICO Manager 已交给 systemd 托管。"
}

pico_connected() {
  local vr_pid=$1
  awk -v tag="PID=$vr_pid]" '
    index($0, tag) {
      if ($0 ~ /Connected to .*refreshRate=/) {
        connected = 1
      } else if ($0 ~ /Listener::Disconnect|client is not connected/) {
        connected = 0
      }
    }
    END { exit connected ? 0 : 1 }
  ' "$NOLO_LOG" 2>/dev/null
}

isaac_uses_xr() {
  local arg
  for arg in "${ISAAC_ARGS[@]}"; do
    [[ "$arg" == '--xr' ]] && return 0
  done
  return 1
}

wait_for_pico() {
  local deadline vr_pid
  isaac_uses_xr || return 0

  info "等待 PICO 串流端连接（${STREAM_HOST}）。现在可以打开头显端。"
  deadline=$((SECONDS + 300))
  while (( SECONDS < deadline )); do
    vr_pid=$(pgrep -n -x vrserver || true)
    [[ -n "$vr_pid" ]] && pgrep -x vrcompositor >/dev/null \
      || die '等待 PICO 时 SteamVR 已退出。请重新执行启动命令。'
    if pico_connected "$vr_pid"; then
      ok 'PICO 串流已连接，继续启动 Isaac XR。'
      return 0
    fi
    sleep 1
  done

  die "等待 PICO 串流连接超时（${STREAM_HOST}）。Isaac XR 未启动，SteamVR 保持运行。"
}

start_isaac() {
  local deadline isaac_pid=0 isaac_command isaac_cmdline stable_since=0
  info "启动 Isaac Lab。"
  printf -v isaac_command '%q ' \
    /usr/bin/env \
    "DISPLAY=${DISPLAY:-:1}" \
    "XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
    "DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/1000/bus}" \
    PYTHONUNBUFFERED=1 \
    "ISAACLAB_LOCAL_ROBOT_ID=${ISAACLAB_LOCAL_ROBOT_ID:-1}" \
    "ISAACLAB_SCENE_SYNC=${ISAACLAB_SCENE_SYNC:-0}" \
    "GR00T_WBC_ROOT=${GR00T_WBC_ROOT:-/home/nolovr/GR00T-WholeBodyControl}" \
    "UNITREE_DDS_DOMAIN=${UNITREE_DDS_DOMAIN:-1}" \
    "UNITREE_DDS_INTERFACE=${UNITREE_DDS_INTERFACE:-lo}" \
    "$PROJECT_DIR/start_isaac_xr.sh" \
    "${ISAAC_ARGS[@]}"
  start_transient "$ISAAC_UNIT" "$PROJECT_DIR" "exec $isaac_command"

  deadline=$((SECONDS + 180))
  while (( SECONDS < deadline )); do
    unit_active "$ISAAC_UNIT" \
      || die "Isaac Lab 启动失败。使用 ./start_all.sh logs 查看日志。"
    if isaac_uses_xr && ! pgrep -x vrcompositor >/dev/null; then
      stop_unit "$ISAAC_UNIT"
      die 'Isaac XR 启动期间 vrcompositor 退出；已停止 Isaac，避免 SteamVR 进入连续崩溃。'
    fi
    isaac_pid=$(systemctl --user show "$ISAAC_UNIT" -p MainPID --value 2>/dev/null || echo 0)
    isaac_cmdline=''
    if [[ "$isaac_pid" =~ ^[1-9][0-9]*$ && -r "/proc/$isaac_pid/cmdline" ]]; then
      isaac_cmdline=$(tr '\0' ' ' <"/proc/$isaac_pid/cmdline")
    fi
    if [[ "$isaac_cmdline" == *"python sim_main.py "* ]]; then
      if (( stable_since == 0 )); then
        stable_since=$SECONDS
      elif (( SECONDS - stable_since >= 20 )); then
        ok "Isaac Lab 已稳定运行（PID $isaac_pid）。"
        return 0
      fi
    else
      stable_since=0
    fi
    sleep 2
  done

  die "Isaac Lab 在 180 秒内未启动。使用 ./start_all.sh logs 查看日志。"
}

start_groot() {
  local deadline stable_since=0
  info "启动 GR00T 控制部署。"
  start_transient "$GROOT_UNIT" "$GROOT_DIR" \
    "printf '\\n' | /usr/bin/env GROOT_USE_PREBUILT=${GROOT_USE_PREBUILT:-1} TensorRT_ROOT=/home/nolovr/TensorRT LD_LIBRARY_PATH=/home/nolovr/.local/onnxruntime/lib:/home/nolovr/Downloads/nolo_driver_deploy/bin/linux64:/home/nolovr/TensorRT/lib bash deploy.sh --input-type zmq_manager --zmq-host localhost --dds-domain 1 isaac"

  deadline=$((SECONDS + 300))
  while (( SECONDS < deadline )); do
    if systemctl --user is-failed --quiet "$GROOT_UNIT" 2>/dev/null; then
      die "GR00T 服务启动失败。使用 ./start_all.sh logs 查看日志。"
    fi
    unit_active "$GROOT_UNIT" \
      || die "GR00T 服务已退出。使用 ./start_all.sh logs 查看日志。"
    if pgrep -f 'g1_deploy_onnx_ref.*--input-type zmq_manager' >/dev/null; then
      if (( stable_since == 0 )); then
        stable_since=$SECONDS
      elif (( SECONDS - stable_since >= 20 )); then
        ok "GR00T 已稳定运行。"
        return 0
      fi
    else
      stable_since=0
    fi
    sleep 2
  done

  die "GR00T 在 300 秒内未进入运行状态。使用 ./start_all.sh logs 查看日志。"
}

show_status() {
  local unit state sub pid vr_pid isaac_cmdline=''

  echo 'SteamVR:'
  pgrep -a -x vrserver || echo '  vrserver: 未运行'
  pgrep -a -x vrcompositor || echo '  vrcompositor: 未运行'

  vr_pid=$(pgrep -n -x vrserver || true)
  if [[ -n "$vr_pid" ]] && pico_connected "$vr_pid"; then
    echo "  PICO: 已连接（串流主机 $STREAM_HOST）"
  else
    echo '  PICO: 未连接'
  fi

  echo '托管服务:'
  for unit in "${MANAGED_UNITS[@]}"; do
    state=$(systemctl --user show "$unit" -p ActiveState --value 2>/dev/null || echo inactive)
    sub=$(systemctl --user show "$unit" -p SubState --value 2>/dev/null || echo dead)
    pid=$(systemctl --user show "$unit" -p MainPID --value 2>/dev/null || echo 0)
    printf '  %-29s %-10s %-10s PID=%s\n' "$unit" "$state" "$sub" "$pid"
  done

  if unit_active "$ISAAC_UNIT"; then
    pid=$(systemctl --user show "$ISAAC_UNIT" -p MainPID --value 2>/dev/null || echo 0)
    if [[ "$pid" =~ ^[1-9][0-9]*$ && -r "/proc/$pid/cmdline" ]]; then
      isaac_cmdline=$(tr '\0' ' ' <"/proc/$pid/cmdline")
    fi
    if [[ "$isaac_cmdline" == *"python sim_main.py "* && "$isaac_cmdline" == *" --xr"* ]]; then
      echo '  Isaac AR/XR: 已启动'
    elif [[ "$isaac_cmdline" == *"python sim_main.py "* ]]; then
      echo '  Isaac Lab: 已启动'
    else
      echo '  Isaac Lab: 正在启动'
    fi
  fi
}

stop_all() {
  local unit
  info "停止 GR00T、Isaac、PICO Manager 和 SteamVR。"
  for unit in "${MANAGED_UNITS[@]}"; do
    stop_unit "$unit"
  done
  stop_stale_workloads
  stop_steamvr
  ok "已停止；Steam 客户端保持运行。"
}

show_logs() {
  exec journalctl --user -f -n 80 \
    -u "$ISAAC_UNIT" \
    -u "$GROOT_UNIT" \
    -u "$PICO_UNIT"
}

start_all() {
  ISAAC_ARGS=("$@")
  require_file "$STEAMVR_CONFIG"
  require_file "$NOLO_LOG"
  require_file "$NOLO_CONFIG"
  require_file "$PROJECT_DIR/start_isaac_xr.sh"
  require_file "$GROOT_DIR/deploy.sh"
  require_file "$GROOT_ROOT/.venv_teleop/bin/activate"
  command -v jq >/dev/null || die '缺少 jq。'
  command -v systemd-run >/dev/null || die '缺少 systemd-run。'
  systemctl --user is-system-running >/dev/null 2>&1 \
    || die '用户级 systemd 未运行。'

  mkdir -p "$STATE_DIR"
  STREAM_HOST=$(jq -r '.ip // empty' "$NOLO_CONFIG")
  [[ -n "$STREAM_HOST" ]] || die "NOLO 配置中缺少串流地址：$NOLO_CONFIG"
  export STREAM_HOST
  info '清理上次由一键脚本启动的服务。'
  for unit in "${MANAGED_UNITS[@]}"; do
    stop_unit "$unit"
  done
  stop_stale_workloads
  start_pico_manager
  start_steamvr
  start_groot
  wait_for_pico
  start_isaac
  echo
  ok '全部服务已启动。'
  show_status
  echo
  echo '查看日志：./start_all.sh logs'
  echo '停止全部：./start_all.sh stop'
}

case "${1:-start}" in
  start)
    if (( $# > 0 )); then
      shift
    fi
    start_all "$@"
    ;;
  status) show_status ;;
  stop) stop_all ;;
  logs) show_logs ;;
  --*) start_all "$@" ;;
  *)
    echo "用法：$0 [start [ISAAC参数...]]|status|logs|stop" >&2
    exit 2
    ;;
esac
