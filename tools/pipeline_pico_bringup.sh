#!/bin/bash
# Pico VR 控制 bring-up（pipeline 2..5 机器人，工作包 Pico-①/②）：
#   sim(HOST_MODE 多机器人) + deploy#1(--input-type zmq_manager ← Pico manager)
#   + 默认 deploy#2(keyboard 调试通道)，或 PIPELINE_DUAL_PICO=1 时：
#     deploy#2(--input-type zmq_manager ← Pico manager#2)
#   + robot_3..N（PIPELINE_SONIC_ROBOT_COUNT，默认 2）仅在显式扩容时各走独立 keyboard deploy；
#     外部 GR00T deploy.sh 尚未原生识别 ID=3..5，本脚本显式钉死 rt/rN 前缀与输出端口。
#
# 与 keyboard 版编排的差异：
#   - deploy#1 的输入从 stdin keyboard 换成 ZMQ(localhost:5556)，发车不再靠 ']'，
#     而是 manager 在操作者按 A+B+X+Y 进 PLANNER 模式时发 command(start=True)。
#     manager 起点用 --no_auto_pose：头显数据一到就自动进 POSE 全身跟随太危险，
#     改为操作者显式按键发车。
#   - manager 走 XROBO_TRANSPORT=udp：头显端 GameLink/定制 Unity 发送端直发本机
#     63901/udp（双 Pico 时 #2 用 63902/udp），不需要 XRoboToolkit PC Service。
#   - 双 Pico 的 manager ZMQ PUB 分别为 5556/5566，deploy#2 必须显式
#     --zmq-port 5566；G1_LOCAL_ROBOT_ID=2 只分 DDS/输出侧，不分 ZMQ 输入。
#   - 整场景复位权威固定在 manager#1：左手 X 单键、摇杆回中后持续 2s，
#     manager#1 经 domain=1/lo 发 rt/reset_pose/cmd。manager#2 不创建复位 publisher，
#     防止两个操作者同时触发全局 reset。Ubuntu Kit 窗口 F12 是同一入口的备用键。
#   - 锁步注意：N 路 ack AND 门下全部 deploy 必须并行存活（逐个等 Init 会自锁）；
#     ack 在 Init 后即持续回，(操作者未发车时机器人保持默认站姿，物理照常推进)。
#
# 操作者手册（头显侧）：
#   1. GameLink 目标 IP 指向本机（192.168.50.68），#1 端口 63901、#2 端口 63902。
#      机器人专用强刷版以 APK 内置 JSON 为真源并在每次启动时刷新 files 副本；旧版才
#      手工修改 files/XrLinkConfig.json。安装/覆盖规则及强制回读步骤见部署文档 §1.1。
#   2. 戴上头显、手柄唤醒，确认对应 pico_manager*.log 出现 body 数据（不再刷 waiting）。
#   3. 按 A+B+X+Y（四键同按，瞬按）→ 进 PLANNER：左摇杆=行走方向、右摇杆=转向；
#      A+B 升档(SLOW_WALK→WALK→RUN...)、X+Y 降档。
#   4. A+X 切 POSE 全身跟随（校准帧=进入时刻的身体姿态）；左摇杆按下切 VR_3PT。
#   5. 整场景 reset：#1 左 X 单键长按 2s（摇杆回中）；或聚焦 Ubuntu Kit
#      窗口按 F12。短按 X 无动作，A+X / X+Y / 四键组合不会触发 reset。
#   6. 再按 A+B+X+Y = 急停（manager 退出，deploy 停控）。
set -u
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_SIM_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
LOG_DIR="${PIPELINE_LOG_DIR:-/tmp/pipeline_pico}"
SIM_DIR="${PIPELINE_SIM_DIR:-$DEFAULT_SIM_DIR}"
GR00T_ROOT="${GR00T_WBC_ROOT:-$HOME/GR00T-WholeBodyControl}"
DEPLOY_DIR="$GR00T_ROOT/gear_sonic_deploy"
PY="${PIPELINE_SIM_PY:-$HOME/miniconda3/envs/env_isaaclab/bin/python}"
MGR_PY="$GR00T_ROOT/.venv_teleop/bin/python"
PEER_IP="${ISAACLAB_SCENE_SYNC_PEER_IP:-192.168.50.127}"
DUAL_PICO="${PIPELINE_DUAL_PICO:-0}"
SONIC_ROBOT_COUNT="${PIPELINE_SONIC_ROBOT_COUNT:-2}"
# 生产默认启用真身执行器 6→1 合并；遇到回归可在启动前显式设 0 原路回滚。
# runtime tensor 校验只用于 A/B/诊断，默认关闭，避免正常启动发生 GPU→CPU 同步。
# 使用 ${VAR-default} 而不是 ${VAR:-default}：显式空串也必须被下面的严格校验拒绝。
SONIC_MERGE_ACTUATORS="${PIPELINE_SONIC_MERGE_ACTUATORS-1}"
SONIC_VALIDATE_ACTUATORS="${PIPELINE_SONIC_VALIDATE_ACTUATORS-0}"
# Host 本地画面只是诊断预览；Cafe 已验证每 4 个控制圈渲染一次时，
# GUI 约 12.5 fps，但物理与 ZMQ 发布仍保持 50 Hz。设 1 可回退每圈渲染。
HOST_LATE_RENDER_INTERVAL="${PIPELINE_HOST_LATE_RENDER_INTERVAL:-4}"
# 只降低 Isaac Dex3 HandCmd；LowCmd、ACK 和控制/规划轮询仍保持 500 Hz。
# 遇到兼容性问题可用 PIPELINE_ISAAC_HANDCMD_HZ=500 恢复旧流量。
ISAAC_HANDCMD_HZ="${PIPELINE_ISAAC_HANDCMD_HZ:-100}"
PICO_R1_UDP_PORT=63901
PICO_R2_UDP_PORT=63902
PICO_R1_ZMQ_PORT=5556
PICO_R2_ZMQ_PORT=5566
declare -a DEPLOY_PIDS=()
declare -a MANAGER_PIDS=()

validate_binary_toggle() {
  local name="$1"
  local value="$2"
  case "$value" in
    0|1) return 0 ;;
    *)
      echo "ERROR: $name 只接受字面 0 或 1（当前值: $value）。" >&2
      return 1
      ;;
  esac
}

validate_isaac_handcmd_hz() {
  local value="$1"
  [[ "$value" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] &&
    awk -v hz="$value" 'BEGIN { exit !(hz >= 20 && hz <= 500) }'
}

case "$DUAL_PICO" in
  0|1) ;;
  *)
    echo "ERROR: PIPELINE_DUAL_PICO 只接受 0 或 1（当前值: $DUAL_PICO）。"
    exit 2
    ;;
esac
case "$SONIC_ROBOT_COUNT" in
  2|3|4|5) ;;
  *)
    echo "ERROR: PIPELINE_SONIC_ROBOT_COUNT 只接受 2..5（当前值: $SONIC_ROBOT_COUNT）。"
    exit 2
    ;;
esac
if ! validate_binary_toggle \
    "PIPELINE_SONIC_MERGE_ACTUATORS" "$SONIC_MERGE_ACTUATORS"; then
  exit 2
fi
if ! validate_binary_toggle \
    "PIPELINE_SONIC_VALIDATE_ACTUATORS" "$SONIC_VALIDATE_ACTUATORS"; then
  exit 2
fi
if ! validate_isaac_handcmd_hz "$ISAAC_HANDCMD_HZ"; then
  echo "ERROR: PIPELINE_ISAAC_HANDCMD_HZ 只接受 [20, 500] 内的有限数值（当前值: $ISAAC_HANDCMD_HZ）。"
  exit 2
fi
if ! [[ "$HOST_LATE_RENDER_INTERVAL" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: PIPELINE_HOST_LATE_RENDER_INTERVAL 只接受正整数（当前值: $HOST_LATE_RENDER_INTERVAL）。" >&2
  exit 2
fi

if [ ! -d "$SIM_DIR" ]; then
  echo "ERROR: 仿真仓库不存在: $SIM_DIR（可用 PIPELINE_SIM_DIR 覆盖）。"
  exit 2
fi
if [ ! -d "$DEPLOY_DIR" ]; then
  echo "ERROR: GR00T deploy 目录不存在: $DEPLOY_DIR（可用 GR00T_WBC_ROOT 覆盖）。"
  exit 2
fi
if [ ! -x "$PY" ]; then
  echo "ERROR: Isaac Python 不可执行: $PY（可用 PIPELINE_SIM_PY 覆盖）。"
  exit 2
fi
if [ ! -x "$MGR_PY" ]; then
  echo "ERROR: Pico manager Python 不可执行: $MGR_PY。"
  exit 2
fi
if ! (cd "$DEPLOY_DIR" && bash deploy.sh --help 2>&1) | grep -q -- "--isaac-handcmd-hz"; then
  echo "ERROR: GR00T deploy.sh 不支持 --isaac-handcmd-hz。"
  echo "      请更新外部仓库，至少包含提交 0f4e0b4，再启动流水线。"
  exit 2
fi

mkdir -p "$LOG_DIR"

echo "== SONIC actuator production settings =="
echo "PIPELINE_SONIC_MERGE_ACTUATORS=$SONIC_MERGE_ACTUATORS (0=回滚原始 6 组, 1=43 关节单组)"
echo "PIPELINE_SONIC_VALIDATE_ACTUATORS=$SONIC_VALIDATE_ACTUATORS (0=正常运行, 1=startup tensor 门禁)"
echo "PIPELINE_HOST_LATE_RENDER_INTERVAL=$HOST_LATE_RENDER_INTERVAL (Host 本地预览间隔，不节流 ZMQ 发布)"

# 渲染形态：默认 GUI（--hide_ui）——需要一个能用的 X 会话，DISPLAY 优先取调用方
# 环境、否则 :0（PIPELINE_DISPLAY 可强制指定）。无桌面/纯 ssh 的机器用
# PIPELINE_HEADLESS=1 切 --no_render（headless experience，不碰窗口子系统；host
# 本地无画面，物理/锁步/viewer 均不受影响）。不预检硬跑的下场：kit 起到 ~5s 在
# RTX 插件初始化段错误（日志指纹 = "GLFW initialization failed" ×3 → SIGSEGV）。
# ⚠️ --device cpu 只切 PhysX 后端、不关渲染，与这个崩溃无关，别为此去掉它。
RENDER_ARG="--hide_ui"
DISPLAY_TARGET="${PIPELINE_DISPLAY:-${DISPLAY:-:0}}"
if [ "${PIPELINE_HEADLESS:-0}" = "1" ]; then
  RENDER_ARG="--no_render"
elif ! command -v xset >/dev/null 2>&1; then
  echo "WARN: 没有 xset（x11-xserver-utils），无法预检 X 可用性。"
  echo "      若 sim 10s 内段错误且日志有 'GLFW initialization failed'，"
  echo "      用 PIPELINE_HEADLESS=1 重跑或装 xset 后再试。"
elif ! timeout 3 xset -display "$DISPLAY_TARGET" q >/dev/null 2>&1; then
  echo "ERROR: X display $DISPLAY_TARGET 不可用（ssh 无桌面会话 / 显示号不对）。"
  echo "  修法A: 在机器本地桌面终端跑，或 PIPELINE_DISPLAY=<实际显示> 指过去"
  echo "  修法B: PIPELINE_HEADLESS=1 bash $0   # 无本地画面，其余功能不受影响"
  exit 1
fi

wait_for() {  # wait_for <pattern> <file> <timeout_s> <label>
  local waited=0
  until grep -q "$1" "$2" 2>/dev/null; do
    sleep 3
    waited=$((waited + 3))
    if [ "$waited" -ge "$3" ]; then
      echo "TIMEOUT waiting for $4 ($1)"
      return 1
    fi
  done
  echo "OK: $4"
}

latest_physics_steps() {  # latest_physics_steps <host_log>
  # 一行可能混有其他统计字段，日志也可能已有多轮 bring-up 输出；只取全文最后一个
  # 完整的 physics_steps=<整数>，避免 tail 行边界/颜色码影响字段切割。
  awk '
    {
      rest = $0
      while (match(rest, /physics_steps=[0-9]+/)) {
        value = substr(rest, RSTART + 14, RLENGTH - 14)
        rest = substr(rest, RSTART + RLENGTH)
      }
    }
    END {
      if (value != "") print value
    }
  ' "$1" 2>/dev/null
}

check_process_alive() {  # check_process_alive <pid> <label> <log_file>
  local pid="$1"
  local label="$2"
  local log_file="$3"
  if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: $label 已退出（pid=${pid:-missing}）。日志尾部："
    echo "----------------------------------------------------------------------"
    tail -30 "$log_file" 2>/dev/null || true
    echo "----------------------------------------------------------------------"
    return 1
  fi
}

check_tracked_processes_alive() {
  local robot_id
  local deploy_pid
  local manager_id
  local manager_count=1
  local manager_pid
  local manager_log
  check_process_alive "$SIM_PID" "host sim" "$LOG_DIR/host_dual.log" || return 1
  for robot_id in $(seq 1 "$SONIC_ROBOT_COUNT"); do
    deploy_pid="${DEPLOY_PIDS[$robot_id]:-}"
    check_process_alive "$deploy_pid" "deploy#$robot_id" \
      "$LOG_DIR/deploy_r$robot_id.log" || return 1
  done
  if [ "$DUAL_PICO" = "1" ]; then
    manager_count=2
  fi
  for manager_id in $(seq 1 "$manager_count"); do
    manager_pid="${MANAGER_PIDS[$manager_id]:-}"
    manager_log="$LOG_DIR/pico_manager.log"
    if [ "$manager_id" = "2" ]; then
      manager_log="$LOG_DIR/pico_manager_r2.log"
    fi
    check_process_alive "$manager_pid" "pico manager#$manager_id" "$manager_log" || return 1
  done
}

PHYSICS_STEPS_RESULT=""
wait_for_physics_sample() {  # wait_for_physics_sample <timeout_s>
  local waited=0
  local current=""
  while [ "$waited" -lt "$1" ]; do
    check_tracked_processes_alive || return 1
    current=$(latest_physics_steps "$LOG_DIR/host_dual.log")
    if [[ "$current" =~ ^[0-9]+$ ]]; then
      PHYSICS_STEPS_RESULT="$current"
      echo "OK: physics_steps baseline=$current"
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  echo "TIMEOUT: host 日志未出现可解析的 physics_steps=<整数>。"
  return 1
}

wait_for_physics_progress() {  # wait_for_physics_progress <baseline> <timeout_s> <label>
  local baseline="$1"
  local timeout_s="$2"
  local label="$3"
  local waited=0
  local current=""
  while [ "$waited" -lt "$timeout_s" ]; do
    check_tracked_processes_alive || return 1
    current=$(latest_physics_steps "$LOG_DIR/host_dual.log")
    if [[ "$current" =~ ^[0-9]+$ ]] && [ "$current" -gt "$baseline" ]; then
      PHYSICS_STEPS_RESULT="$current"
      echo "OK: $label physics_steps $baseline -> $current"
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  echo "TIMEOUT: $label 未前进（physics_steps=${current:-missing}, baseline=$baseline）。"
  return 1
}

echo "== stop old processes (hard) =="
pkill -INT -f "sim_mai[n].py" 2>/dev/null
sleep 5
pkill -9 -f "g1_deploy_onnx_re[f]" 2>/dev/null
pkill -9 -f "deploy.s[h]" 2>/dev/null
pkill -9 -f "pico_manager_thread_serve[r]" 2>/dev/null
pkill -f "tail -f.*dk_r[1-5]" 2>/dev/null
sleep 4
pgrep -f "sim_mai[n]|g1_deploy_onnx_re[f]" >/dev/null && { pkill -9 -f "sim_mai[n]|g1_deploy_onnx_re[f]"; sleep 3; }
echo "residual deploys: $(pgrep -c -f 'g1_deploy_onnx_re[f]' 2>/dev/null || echo 0)"

echo "== start host sim ($SONIC_ROBOT_COUNT SONIC robots) =="
cd "$SIM_DIR" || exit 1
# 不在包装脚本硬传 --sim-state-export-hz：sim_main 必须等 Env 创建后确认
# ZMQ PUB socket 真正就绪才关闭重复 SimState；bind 失败时保留 5 Hz 回退。
env DISPLAY="$DISPLAY_TARGET" GR00T_WBC_ROOT="$GR00T_ROOT" PYTHONUNBUFFERED=1 \
    UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo \
    ISAACLAB_LOCAL_ROBOT_ID=1 ISAACLAB_HOST_BOTH_ROBOTS=1 \
    ISAACLAB_SONIC_ROBOT_COUNT="$SONIC_ROBOT_COUNT" \
    ISAACLAB_SONIC_MERGE_ACTUATORS="$SONIC_MERGE_ACTUATORS" \
    ISAACLAB_SONIC_VALIDATE_ACTUATORS="$SONIC_VALIDATE_ACTUATORS" \
    ISAACLAB_SCENE_SYNC_PEER_IP="$PEER_IP" \
    UNITREE_SKIP_LOWSTATE_CRC=1 UNITREE_LOWCMD_CRC_SAMPLE_INTERVAL=50 \
    "$PY" sim_main.py --task Isaac-G1-29DoF-Sonic-Conveyor --robot_type g129 \
    --action_source sonic_dds --device cpu $RENDER_ARG --stats_interval 10 \
    --profile_interval 25 --lowstate-pub-hz 55 \
    --handstate-pub-hz 10 --late-render-interval "$HOST_LATE_RENDER_INTERVAL" \
    > "$LOG_DIR/host_dual.log" 2>&1 &
SIM_PID=$!

# 早崩检测：kit 渲染/驱动层的崩溃集中在启动头几秒——与其让 bash 甩一行段错误,
# 不如当场把日志指纹和判读打出来。
sleep 10
if ! kill -0 "$SIM_PID" 2>/dev/null; then
  echo "ERROR: sim 启动 10s 内就退出了（多为渲染/驱动层崩溃）。host_dual.log 尾部："
  echo "----------------------------------------------------------------------"
  tail -20 "$LOG_DIR/host_dual.log"
  echo "----------------------------------------------------------------------"
  echo "判读："
  echo "  · 有 'GLFW initialization failed' → 拿不到 X：本地桌面跑 / PIPELINE_HEADLESS=1"
  echo "  · 无 GLFW 报错、栈在 librtx/carb → 驱动/Vulkan 层："
  echo "      vulkaninfo --summary | grep -i device   # 掉成 llvmpipe = 用户态驱动错配"
  echo "      nvidia-smi --query-gpu=name,driver_version --format=csv"
  exit 1
fi

# N 路 ack AND 门：所有 deploy 必须并行存活（.trt 缓存只读共享，build 错峰启动）。
echo "== start deploy#1 (rt/*, input=zmq_manager) =="
echo "" > "$LOG_DIR/dk_r1"
(
  cd "$DEPLOY_DIR" || exit 1
  exec bash deploy.sh --disable-crc-check \
    --input-type zmq_manager --zmq-port "$PICO_R1_ZMQ_PORT" \
    --isaac-handcmd-hz "$ISAAC_HANDCMD_HZ" \
    isaac < <(tail -f "$LOG_DIR/dk_r1")
) > "$LOG_DIR/deploy_r1.log" 2>&1 &
DEPLOY_PIDS[1]=$!
echo "deploy#1 pid=${DEPLOY_PIDS[1]}"
sleep 20

if [ "$DUAL_PICO" = "1" ]; then
  echo "== start deploy#2 (rt/r2/*, input=zmq_manager :$PICO_R2_ZMQ_PORT) =="
  # deploy.sh 启动前会 read 一次确认：只送一个空行接受默认 Y；该换行被确认提示
  # 消费后，运行期 stdin 只剩 EOF，没有任何键控字节。不能保留 dk_r2 键管道，
  # 否则脚本仍能替操作者发车。
  (
    cd "$DEPLOY_DIR" || exit 1
    exec env G1_LOCAL_ROBOT_ID=2 bash deploy.sh \
      --disable-crc-check --input-type zmq_manager --zmq-port "$PICO_R2_ZMQ_PORT" \
      --isaac-handcmd-hz "$ISAAC_HANDCMD_HZ" \
      isaac <<< ""
  ) > "$LOG_DIR/deploy_r2.log" 2>&1 &
else
  echo "== start deploy#2 (rt/r2/*, input=keyboard) =="
  echo "" > "$LOG_DIR/dk_r2"
  (
    cd "$DEPLOY_DIR" || exit 1
    exec env G1_LOCAL_ROBOT_ID=2 bash deploy.sh \
      --disable-crc-check --input-type keyboard \
      --isaac-handcmd-hz "$ISAAC_HANDCMD_HZ" \
      isaac < <(tail -f "$LOG_DIR/dk_r2")
  ) > "$LOG_DIR/deploy_r2.log" 2>&1 &
fi
DEPLOY_PIDS[2]=$!
echo "deploy#2 pid=${DEPLOY_PIDS[2]}"

# robot_3..N：当前 GR00T deploy.sh 只有 ID=2 特判；只传 G1_LOCAL_ROBOT_ID=3
# 会静默落回 rt/* 并与 robot_1 串台。因此 prefix、输入端口和 debug 输出全部显式隔离。
for ROBOT_ID in $(seq 3 "$SONIC_ROBOT_COUNT"); do
  ROBOT_ZMQ_IN_PORT=$((5556 + (ROBOT_ID - 1) * 10))
  ROBOT_DEBUG_PORT=$((5557 + (ROBOT_ID - 1) * 10))
  echo "== start deploy#$ROBOT_ID (rt/r$ROBOT_ID/*, input=keyboard) =="
  echo "" > "$LOG_DIR/dk_r$ROBOT_ID"
  (
    cd "$DEPLOY_DIR" || exit 1
    exec env G1_LOCAL_ROBOT_ID="$ROBOT_ID" SONIC_DDS_TOPIC_PREFIX="rt/r$ROBOT_ID" \
      bash deploy.sh --disable-crc-check --input-type keyboard \
      --zmq-port "$ROBOT_ZMQ_IN_PORT" \
      --zmq-out-port "$ROBOT_DEBUG_PORT" --zmq-out-topic "g1_${ROBOT_ID}_debug" \
      --udp-out-port "$ROBOT_DEBUG_PORT" --udp-out-topic "g1_${ROBOT_ID}_debug" \
      --isaac-handcmd-hz "$ISAAC_HANDCMD_HZ" \
      isaac < <(tail -f "$LOG_DIR/dk_r$ROBOT_ID")
  ) > "$LOG_DIR/deploy_r$ROBOT_ID.log" 2>&1 &
  DEPLOY_PIDS[$ROBOT_ID]=$!
  echo "deploy#$ROBOT_ID pid=${DEPLOY_PIDS[$ROBOT_ID]}"
  sleep 10
done

# manager 拿到第一帧头显数据后才 bind ZMQ PUB；deploy 的 SUB 是 connect 语义，
# command 有 1Hz keepalive 兜 slow-joiner，所以 manager/deploy 谁先启动都能接上。
: > "$LOG_DIR/pico_manager.log"
if [ "$DUAL_PICO" = "1" ]; then
  : > "$LOG_DIR/pico_manager_r2.log"
fi
if [ "$DUAL_PICO" = "1" ]; then
  echo "== start pico manager#1 (udp :$PICO_R1_UDP_PORT, PUB :$PICO_R1_ZMQ_PORT, scene-reset authority) =="
  # 双 Pico 模式显式钉死两个 UDP 端口，防调用环境残留 XROBO_UDP_PORT 污染隔离。
  (
    cd "$GR00T_ROOT" || exit 1
    exec env XROBO_TRANSPORT=udp XROBO_UDP_PORT="$PICO_R1_UDP_PORT" \
      UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo PYTHONUNBUFFERED=1 \
      "$MGR_PY" gear_sonic/scripts/pico_manager_thread_server.py \
      --manager --no_auto_pose --enable_isaac_scene_reset --port "$PICO_R1_ZMQ_PORT"
  ) > "$LOG_DIR/pico_manager.log" 2>&1 &
  MANAGER_PIDS[1]=$!
  echo "pico manager#1 pid=${MANAGER_PIDS[1]}"

  echo "== start pico manager#2 (udp :$PICO_R2_UDP_PORT, PUB :$PICO_R2_ZMQ_PORT) =="
  (
    cd "$GR00T_ROOT" || exit 1
    exec env XROBO_TRANSPORT=udp XROBO_UDP_PORT="$PICO_R2_UDP_PORT" \
      PYTHONUNBUFFERED=1 "$MGR_PY" gear_sonic/scripts/pico_manager_thread_server.py \
      --manager --no_auto_pose --port "$PICO_R2_ZMQ_PORT"
  ) > "$LOG_DIR/pico_manager_r2.log" 2>&1 &
  MANAGER_PIDS[2]=$!
  echo "pico manager#2 pid=${MANAGER_PIDS[2]}"
else
  echo "== start pico manager (udp transport, PUB :$PICO_R1_ZMQ_PORT, scene-reset authority) =="
  # 单 Pico 保留原行为：UDP 默认 63901，也允许调用方沿用 XROBO_UDP_PORT 覆盖。
  (
    cd "$GR00T_ROOT" || exit 1
    exec env XROBO_TRANSPORT=udp \
      UNITREE_DDS_DOMAIN=1 UNITREE_DDS_INTERFACE=lo PYTHONUNBUFFERED=1 "$MGR_PY" \
      gear_sonic/scripts/pico_manager_thread_server.py --manager --no_auto_pose \
      --enable_isaac_scene_reset --port "$PICO_R1_ZMQ_PORT"
  ) > "$LOG_DIR/pico_manager.log" 2>&1 &
  MANAGER_PIDS[1]=$!
  echo "pico manager#1 pid=${MANAGER_PIDS[1]}"
fi

# 不等头显首帧/ZMQ bind，但至少确认两个 UDP receiver 已成功启动；否则依赖缺失、
# 端口占用等早退不能伪装成 BRINGUP_DONE。单 Pico允许外部覆盖端口，所以只认通用标记。
if [ "$DUAL_PICO" = "1" ]; then
  wait_for "UDP receiver listening on .*:$PICO_R1_UDP_PORT" "$LOG_DIR/pico_manager.log" \
    60 "pico manager#1 UDP :$PICO_R1_UDP_PORT" || exit 1
  wait_for "UDP receiver listening on .*:$PICO_R2_UDP_PORT" "$LOG_DIR/pico_manager_r2.log" \
    60 "pico manager#2 UDP :$PICO_R2_UDP_PORT" || exit 1
else
  wait_for "UDP receiver listening on" "$LOG_DIR/pico_manager.log" \
    60 "pico manager UDP receiver" || exit 1
fi

wait_for "Init Done" "$LOG_DIR/deploy_r1.log" 300 "deploy#1 Init Done" || exit 1
wait_for "Init Done" "$LOG_DIR/deploy_r2.log" 300 "deploy#2 Init Done" || exit 1
grep -m1 "DDS topics" "$LOG_DIR/deploy_r1.log" || true
grep -m1 "DDS topics" "$LOG_DIR/deploy_r2.log" || true
for ROBOT_ID in $(seq 3 "$SONIC_ROBOT_COUNT"); do
  wait_for "Init Done" "$LOG_DIR/deploy_r$ROBOT_ID.log" 300 \
    "deploy#$ROBOT_ID Init Done" || exit 1
  grep -m1 "DDS topics" "$LOG_DIR/deploy_r$ROBOT_ID.log" || true
done

if [ "$DUAL_PICO" = "1" ]; then
  echo "== wait sim STARTUP HOLD (dual Pico: wait for operators) =="
else
  echo "== wait sim STARTUP HOLD, start channel #2 (keyboard) =="
fi
wait_for "STARTUP HOLD" "$LOG_DIR/host_dual.log" 300 "sim STARTUP HOLD" || exit 1
if [ "$DUAL_PICO" = "0" ]; then
  sleep 2
  printf ']' >> "$LOG_DIR/dk_r2"
  wait_for "sonic_dds:r2.*CONTROL marker received" "$LOG_DIR/host_dual.log" 60 \
    "channel#2 CONTROL" || exit 1
fi
# robot_3..N 固定为 keyboard 调试通道：逐台发车并预激活 planner，之后可向
# $LOG_DIR/dk_rN 写 w/s/a/d/j/l 等键，只会控制对应 rt/rN/* 机器人。
for ROBOT_ID in $(seq 3 "$SONIC_ROBOT_COUNT"); do
  sleep 2
  printf ']' >> "$LOG_DIR/dk_r$ROBOT_ID"
  wait_for "sonic_dds:r$ROBOT_ID.*CONTROL marker received" "$LOG_DIR/host_dual.log" 60 \
    "channel#$ROBOT_ID CONTROL" || exit 1
  sleep 2
  printf '\n' >> "$LOG_DIR/dk_r$ROBOT_ID"
  sleep 2
  printf '2' >> "$LOG_DIR/dk_r$ROBOT_ID"
done
if [ "$DUAL_PICO" = "0" ]; then
  # 通道 #2 planner 预激活（行走靠往 dk_r2 里发 w/s/a/d）。这一步也放在最终
  # 存活/进度门禁之前，避免模型在 planner 切换时退出却仍打印 DONE。
  sleep 3
  printf '\n' >> "$LOG_DIR/dk_r2"
  sleep 3
  printf '2' >> "$LOG_DIR/dk_r2"
fi

# marker 只证明某一时刻收到了控制事件；deploy 随后 OOM/崩溃时，N 路 ack AND 会把
# host 永久锁住。最终成功门槛因此同时要求所有真实 deploy shell 仍存活，并连续看到
# 两次新的 PhysX 进度（不是重复读取同一条历史统计）。任一步失败都必须先于 DONE 退出。
check_tracked_processes_alive || exit 1
echo "OK: host sim + deploy#1..#$SONIC_ROBOT_COUNT + Pico manager 均存活"
wait_for_physics_sample 60 || exit 1
PHYSICS_BASELINE="$PHYSICS_STEPS_RESULT"
wait_for_physics_progress "$PHYSICS_BASELINE" 60 "physics progress probe#1" || exit 1
PHYSICS_PROBE_1="$PHYSICS_STEPS_RESULT"
wait_for_physics_progress "$PHYSICS_PROBE_1" 60 "physics progress probe#2" || exit 1
echo "OK: host physics confirmed advancing twice ($PHYSICS_BASELINE -> $PHYSICS_PROBE_1 -> $PHYSICS_STEPS_RESULT)"

# 帧率三件套之三：提优先级（sim + 全部 deploy）
sudo -n renice -n -10 -p $(pgrep -f "sim_mai[n].py" | head -1) \
    $(pgrep -f "g1_deploy_onnx_re[f]" | tr '\n' ' ') 2>/dev/null || true
if [ "$DUAL_PICO" = "1" ]; then
  if [ "$SONIC_ROBOT_COUNT" -ge 3 ]; then
    echo "BRINGUP_DONE (dual Pico: robot#1/#2 等操作者发车，robot#3..#$SONIC_ROBOT_COUNT 已开 keyboard)"
  else
    echo "BRINGUP_DONE (dual Pico: robot#1/#2 等操作者发车)"
  fi
  echo "logs: $LOG_DIR/host_dual.log + deploy_r1..r$SONIC_ROBOT_COUNT.log + pico_manager*.log"
else
  if [ "$SONIC_ROBOT_COUNT" -ge 3 ]; then
    echo "BRINGUP_DONE (robot#1 等操作者头显发车，robot#2..#$SONIC_ROBOT_COUNT 已开 keyboard)"
  else
    echo "BRINGUP_DONE (robot#1 等操作者头显发车，robot#2 已开 keyboard)"
  fi
  echo "logs: $LOG_DIR/host_dual.log + deploy_r1..r$SONIC_ROBOT_COUNT.log + pico_manager.log"
fi
